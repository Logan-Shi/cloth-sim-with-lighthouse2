#include "bvh.h"
#include "watch.h"
#include <iostream>
#include <fstream>
#include <algorithm>
#include <bitset>
#include <thrust/sort.h>
#include <thrust/execution_policy.h>

#include "Utilities.h"

using namespace std;
extern inline void copyFromCPUtoGPU(void** dst, void* src, int size);
extern inline void copyFromGPUtoCPU(void** dst, void* src, int size);

// Expands a 10-bit integer into 30 bits
// by inserting 2 zeros after each bit.
__device__ unsigned int d_expandBits(unsigned int v)
{
	v = (v * 0x00010001u) & 0xFF0000FFu;
	v = (v * 0x00000101u) & 0x0F00F00Fu;
	v = (v * 0x00000011u) & 0xC30C30C3u;
	v = (v * 0x00000005u) & 0x49249249u;
	return v;
}

// Calculates a 30-bit Morton code for the
// given 3D point located within the unit cube [0,1].
__device__ unsigned int d_morton3D(glm::vec3 p)
{
	float x = p.x, float y = p.y, float z = p.z;
	x = min(max(x * 1024.0f, 0.0f), 1023.0f);
	y = min(max(y * 1024.0f, 0.0f), 1023.0f);
	z = min(max(z * 1024.0f, 0.0f), 1023.0f);
	unsigned int xx = d_expandBits((unsigned int)x);
	unsigned int yy = d_expandBits((unsigned int)y);
	unsigned int zz = d_expandBits((unsigned int)z);
	return xx * 4 + yy * 2 + zz;
}


__global__ void get_bb(int num, int m, Primitive* d_primitives, BBox* d_bb)
{
	unsigned int index = blockIdx.x * blockDim.x + threadIdx.x;
	if (index >= num + 1)
		return;
	int div = m / num;
	int res = m % num;
	if (index == num + 1)
	{
		BBox tem_bbox;
		for (int i = m - res; i < m; i++)
		{
			tem_bbox.expand(d_primitives[i].d_get_expand_bbox());
		}
		d_bb[index] = tem_bbox;
	}
	else
	{
		BBox tem_bbox;
		for (int i = 0; i < div; i++)  //use shared to replace
		{
			tem_bbox.expand(d_primitives[i * num + index].d_get_expand_bbox());
		}
		d_bb[index].expand(tem_bbox);
	}
}

__global__ void compute_morton_bbox(int num, Primitive* d_primitives, BBox bb, MortonCode* mortons, BBox* bboxes)
{
	unsigned int index = blockIdx.x * blockDim.x + threadIdx.x;
	if (index >= num)
		return;
	BBox tem_bbox = d_primitives[index].d_get_expand_bbox();
	bboxes[index] = tem_bbox;
	mortons[index] = d_morton3D(bb.getUnitcubePosOf(tem_bbox.centroid()));
}

// Expands a 10-bit integer into 30 bits
// by inserting 2 zeros after each bit.
unsigned int BVHAccel::expandBits(unsigned int v)
{
	v = (v * 0x00010001u) & 0xFF0000FFu;
	v = (v * 0x00000101u) & 0x0F00F00Fu;
	v = (v * 0x00000011u) & 0xC30C30C3u;
	v = (v * 0x00000005u) & 0x49249249u;
	return v;
}

// Calculates a 30-bit Morton code for the
// given 3D point located within the unit cube [0,1].
unsigned int BVHAccel::morton3D(float x, float y, float z)
{
	x = min(max(x * 1024.0f, 0.0f), 1023.0f);
	y = min(max(y * 1024.0f, 0.0f), 1023.0f);
	z = min(max(z * 1024.0f, 0.0f), 1023.0f);
	unsigned int xx = expandBits((unsigned int)x);
	unsigned int yy = expandBits((unsigned int)y);
	unsigned int zz = expandBits((unsigned int)z);
	return xx * 4 + yy * 2 + zz;
}

/**
* a wrapper to calculate morton code from
* the position of an object inside the
* unit cube.
*/
unsigned int BVHAccel::morton3D(glm::vec3 pos)
{
	return morton3D(pos.x, pos.y, pos.z);
}

/**
* comparer used to sort primitives acoording
* to their morton code.
*/


BBox BVHAccel::computet_root_bbox(Primitive* d_tem_primitives)
{
	const unsigned int num_threads = 128;
	vector<BBox> c_bb(num_threads + 1);
	BBox* d_bb;

	copyFromCPUtoGPU((void**)&d_bb, &c_bb[0], sizeof(BBox) * c_bb.size());
	get_bb << <1, c_bb.size() >> > (num_threads, _primitives.size(), d_tem_primitives, d_bb);

	BBox* cc_bb, bb;
	copyFromGPUtoCPU((void**)&cc_bb, d_bb, sizeof(BBox) * c_bb.size());
	for (int i = 0; i < c_bb.size(); i++)
	{
		bb.expand(cc_bb[i]);
	}

	cudaFree(d_bb);

	return bb;
}

void BVHAccel::compute_bbox_and_morton()
{
	Primitive* d_tem_primitives;
	MortonCode* d_tem_morton_codes;
	BBox* d_tem_bboxes;
	_morton_codes.resize(_primitives.size());
	_bboxes.resize(_primitives.size());

	copyFromCPUtoGPU((void**)&d_tem_primitives, &_primitives[0], sizeof(Primitive) * _primitives.size());
	copyFromCPUtoGPU((void**)&d_tem_morton_codes, &_morton_codes[0], sizeof(MortonCode) * _morton_codes.size());
	copyFromCPUtoGPU((void**)&d_tem_bboxes, &_bboxes[0], sizeof(BBox) * _bboxes.size());

	BBox bb = computet_root_bbox(d_tem_primitives);

	unsigned int numThreads, numBlocks;
	unsigned int blockSize = 512;
	unsigned int n = _primitives.size();
	numThreads = min(blockSize, n);
	numBlocks = (n % numThreads != 0) ? (n / numThreads + 1) : (n / numThreads);

	compute_morton_bbox << <numBlocks, numThreads >> > (n, d_tem_primitives, bb, d_tem_morton_codes, d_tem_bboxes);

	cudaMemcpy(&_morton_codes[0], d_tem_morton_codes, sizeof(MortonCode) * _morton_codes.size(), cudaMemcpyDeviceToHost);
	cudaMemcpy(&_bboxes[0], d_tem_bboxes, sizeof(BBox) * _bboxes.size(), cudaMemcpyDeviceToHost);

	cudaFree(d_tem_primitives);
	cudaFree(d_tem_morton_codes);
	cudaFree(d_tem_bboxes);
}

__global__ void init_nodes(BRTreeNode* _nodes, const unsigned int num)
{
	unsigned int index = blockIdx.x * blockDim.x + threadIdx.x;
	if (index >= num)
		return;

	BRTreeNode node;
	node.setIdx(index);
	node.bbox = BBox();

	_nodes[index] = node;
}

void BVHAccel::init()
{
	d_bvh = new D_BVH();

	auto size = _sorted_primitives.size();
	numInternalNode = size - 1;
	numLeafNode = size;

	//whether to set h_vertices = NULL before send to gpu?
	copyFromCPUtoGPU((void**)&d_bvh->d_primitives, &_sorted_primitives[0], sizeof(Primitive) * _sorted_primitives.size());
	copyFromCPUtoGPU((void**)&d_sorted_morton_code, &_sorted_morton_codes[0], sizeof(MortonCode) * _sorted_morton_codes.size());
	copyFromCPUtoGPU((void**)&d_bboxes, &_sorted_bboxes[0], sizeof(BBox) * _sorted_bboxes.size());

	//initialize d_leaf_nodes and d_internal_nodes: with a parallel way? ?????
	cudaMalloc((void**)&d_bvh->d_leaf_nodes, numLeafNode * sizeof(BRTreeNode));
	cudaMalloc((void**)&d_bvh->d_internal_nodes, numInternalNode * sizeof(BRTreeNode));

	int threadPerBlock = DEFAULT_THREAD_PER_BLOCK;
	int numBlock = (numLeafNode + DEFAULT_THREAD_PER_BLOCK - 1) / threadPerBlock;
	init_nodes << <numBlock, threadPerBlock >> > (d_bvh->d_leaf_nodes, numLeafNode);


	threadPerBlock = DEFAULT_THREAD_PER_BLOCK;
	numBlock = (numInternalNode + DEFAULT_THREAD_PER_BLOCK - 1) / threadPerBlock;
	init_nodes << <numBlock, threadPerBlock >> > (d_bvh->d_internal_nodes, numInternalNode);
}

void BVHAccel::build()
{
	//build the bvh
	int threadPerBlock = DEFAULT_THREAD_PER_BLOCK;
	int numBlock = (numInternalNode + DEFAULT_THREAD_PER_BLOCK - 1) / threadPerBlock;
	processInternalNode << <numBlock, threadPerBlock >> > (d_sorted_morton_code, numInternalNode,
		d_bvh->d_leaf_nodes, d_bvh->d_internal_nodes);

	//calculate bounding box
	threadPerBlock = DEFAULT_THREAD_PER_BLOCK;
	numBlock = (numLeafNode + DEFAULT_THREAD_PER_BLOCK - 1) / threadPerBlock;
	calculateBoudingBox << <numBlock, threadPerBlock >> > (d_bboxes, numLeafNode,
		d_bvh->d_leaf_nodes, d_bvh->d_internal_nodes);
}

void BVHAccel::init_primitives(Mesh& body)
{
	//prepare primitives
	obj_vertices.resize(body.vertices.size());
	for (int i = 0; i < body.vertices.size(); i++)
	{
		obj_vertices[i] = glm::vec3(body.vertices[i]);
	}

	safe_cuda(cudaMalloc((void**)&d_obj_vertices, sizeof(glm::vec3) * obj_vertices.size()));
	safe_cuda(cudaMemcpy(d_obj_vertices, &obj_vertices[0], sizeof(glm::vec3) * obj_vertices.size(), cudaMemcpyHostToDevice));

	//create primitives
	glm::vec3* h_obj_vertices = &obj_vertices[0];
	_primitives.resize(body.vertex_indices.size() / 3);

	for (int i = 0; i < _primitives.size(); i++)
	{
		Primitive tem_pri(h_obj_vertices, d_obj_vertices, body.vertex_indices[i * 3 + 0],
			body.vertex_indices[i * 3 + 1],
			body.vertex_indices[i * 3 + 2]);
		_primitives[i] = tem_pri;
	}
}

BVHAccel::BVHAccel(Mesh& body, size_t max_leaf_size) :

	d_bboxes(nullptr),
#ifdef _DEBUG
	h_leaf_nodes(nullptr),
	h_internal_nodes(nullptr),
#endif
	d_sorted_morton_code(nullptr)
{
	init_primitives(body);

	// edge case
	if (_primitives.empty()) {
		return;
	}

	compute_bbox_and_morton();


	// remove duplicates
	vector<unsigned int> indices;
	indices_sort(_morton_codes, indices);
	remove_redundant(_morton_codes, indices);

	filter(_morton_codes, indices, _sorted_morton_codes);
	filter(_primitives, indices, _sorted_primitives);
	filter(_bboxes, indices, _sorted_bboxes);

	// init	GPU data, including d_bboxes,d_primitives, d_sorted_morton_code,d_leaf_nodes, d_internal_nodes 
	init();

	// build the brt tree
	build();
}

BVHAccel::~BVHAccel()
{
	cudaFree(d_bboxes);
	cudaFree(d_sorted_morton_code);
	cudaFree(d_obj_vertices);

	// Free d_bvh here cause it has pointer points to gpu memory
	// and we need to pass the value several times and make sure the 
	// resource not freed, so we can't free it in its own destructor. 
	// \BVHAccel controls the lifetieme of \d_bvh, if the destructor 
	// of \BVHAccel called, which means we can free all the resources in 
	// gpu and cpu(Obviously, this violates the "new" and "free" pair priciple)

	d_bvh->free_memory();
}