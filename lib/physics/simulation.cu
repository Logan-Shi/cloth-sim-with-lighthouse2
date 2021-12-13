#include "simulator.h"

#include <iostream>
#include <fstream>
#include "watch.h"

using namespace std;
vector<glm::vec3> test(10000);
__global__ void compute_face_normal(glm::vec3* g_pos_in, unsigned int* cloth_index, const unsigned int cloth_index_size, glm::vec3* cloth_face);   //update cloth face normal
__global__ void verlet(glm::vec3 * g_pos_in, glm::vec3 * g_pos_old_in, glm::vec3 * g_pos_out, glm::vec3 * g_pos_old_out,
						unsigned int* CSR_R_STR, s_spring* CSR_C_STR, unsigned int* CSR_R_BD, s_spring* CSR_C_BD,
						D_BVH bvh, glm::vec3* d_collision_force,
						const unsigned int NUM_VERTICES);  //verlet intergration
__global__ void update_vbo_pos(glm::vec4* pos_vbo, glm::vec3* pos_cur, const unsigned int NUM_VERTICES);
__global__ void compute_vbo_normal(glm::vec3* normals, unsigned int* CSR_R, unsigned int* CSR_C_adjface_to_vertex, glm::vec3* face_normal, const unsigned int NUM_VERTICES);

Simulator::Simulator()
{
	
}

Simulator::~Simulator()
{
	cudaFree(x_cur[0]);
	cudaFree(x_cur[1]);
	cudaFree(x_last[0]);
	cudaFree(x_last[1]);
	cudaFree(d_collision_force);
	cudaFree(d_CSR_R);
	cudaFree(d_CSR_C_adjface_to_vertex);
	cudaFree(d_face_normals);

	cudaFree(CSR_R_structure);
	cudaFree(CSR_R_bend);
	cudaFree(CSR_C_structure);
	cudaFree(CSR_C_bend);

	delete cuda_bvh;
}

Simulator::Simulator(Mesh& sim_cloth, Mesh& body) :readID(0), writeID(1)
{
	init_cloth(sim_cloth);
	init_spring(sim_cloth);
	build_bvh(body);
}

void Simulator::init_cloth(Mesh& sim_cloth)
{
	//set heap size, the default is 8M
	size_t heap_size = 256 * 1024 * 1024;  
	cudaDeviceSetLimit(cudaLimitMallocHeapSize, heap_size);

	// Send the cloth's vertices to GPU
	const unsigned int vertices_bytes = sizeof(glm::vec3) * sim_cloth.vertices.size();
	safe_cuda(cudaMalloc((void**)&x_cur[0], vertices_bytes));			 // cloth vertices
	safe_cuda(cudaMalloc((void**)&x_cur[1], vertices_bytes));			 // cloth vertices
	safe_cuda(cudaMalloc((void**)&x_last[0], vertices_bytes));	 // cloth old vertices
	safe_cuda(cudaMalloc((void**)&x_last[1], vertices_bytes));	 // cloth old vertices
	safe_cuda(cudaMalloc((void**)&d_collision_force, sizeof(glm::vec3) * sim_cloth.vertices.size()));  //collision response force
	safe_cuda(cudaMemset(d_collision_force, 0, sizeof(glm::vec3) * sim_cloth.vertices.size()));    //initilize to 0

	x_cur_in = x_cur[readID];
	x_cur_out = x_cur[writeID];
	x_last_in = x_last[readID];
	x_last_out = x_last[writeID];

	vector<glm::vec3> tem_vertices(sim_cloth.vertices.size());
	for (int i=0;i< sim_cloth.vertices.size();i++)
	{
		tem_vertices[i] = glm::vec3(sim_cloth.vertices[i]);   // glm::vec4 -> glm::vec3
	}

	safe_cuda(cudaMemcpy(x_cur[0], &tem_vertices[0], vertices_bytes, cudaMemcpyHostToDevice));
	safe_cuda(cudaMemcpy(x_last[0], &tem_vertices[0], vertices_bytes, cudaMemcpyHostToDevice));


	//cudaMemcpy(&test[0], x_cur[0], sizeof(glm::vec3) * sim_cloth.vertices.size(), cudaMemcpyDeviceToHost);
	//cudaMemcpy(&test, x_cur[1], sizeof(glm::vec3), cudaMemcpyDeviceToHost);
	//cudaMemcpy(&test, d_collision_force, sizeof(glm::vec3)+1, cudaMemcpyDeviceToHost);
	//����normal��������ݣ�ÿ�����ڽӵ�������� + ÿ�����3���������
	vector<unsigned int> TEM_CSR_R;
	vector<unsigned int> TEM_CSR_C_adjface;
	get_vertex_adjface(sim_cloth, TEM_CSR_R, TEM_CSR_C_adjface);

	safe_cuda(cudaMalloc((void**)&d_CSR_R, sizeof(unsigned int) * TEM_CSR_R.size()));
	safe_cuda(cudaMalloc((void**)&d_CSR_C_adjface_to_vertex, sizeof(unsigned int) * TEM_CSR_C_adjface.size()));
	safe_cuda(cudaMemcpy(d_CSR_R, &TEM_CSR_R[0], sizeof(unsigned int) * TEM_CSR_R.size(), cudaMemcpyHostToDevice));
	safe_cuda(cudaMemcpy(d_CSR_C_adjface_to_vertex, &TEM_CSR_C_adjface[0], sizeof(unsigned int) * TEM_CSR_C_adjface.size(), cudaMemcpyHostToDevice));
	
	safe_cuda(cudaMalloc((void**)&d_face_normals, sizeof(glm::vec3) * sim_cloth.faces.size()));    //face normal
}

void Simulator::init_spring(Mesh& sim_cloth)
{
	cout << "build springs" << endl;
	// Construct structure and bend springs in GPU
	Springs springs(&sim_cloth);
	
	vector<unsigned int> TEM_CSR_R_structure, TEM_CSR_R_bend;
	vector<s_spring> TEM_CSR_C_structure, TEM_CSR_C_bend;

	springs.CSR_structure_spring(&sim_cloth, TEM_CSR_R_structure, TEM_CSR_C_structure);
	springs.CSR_bend_spring(&sim_cloth, TEM_CSR_R_bend, TEM_CSR_C_bend);

	safe_cuda(cudaMalloc((void**)&CSR_R_structure, TEM_CSR_R_structure.size() * sizeof(unsigned int)));
	safe_cuda(cudaMalloc((void**)&CSR_R_bend, TEM_CSR_R_bend.size() * sizeof(unsigned int)));
	safe_cuda(cudaMalloc((void**)&CSR_C_structure, TEM_CSR_C_structure.size() * sizeof(s_spring)));
	safe_cuda(cudaMalloc((void**)&CSR_C_bend, TEM_CSR_C_bend.size() * sizeof(s_spring)));

	safe_cuda(cudaMemcpy(CSR_R_structure, &TEM_CSR_R_structure[0], TEM_CSR_R_structure.size() * sizeof(unsigned int), cudaMemcpyHostToDevice));
	safe_cuda(cudaMemcpy(CSR_R_bend, &TEM_CSR_R_bend[0], TEM_CSR_R_bend.size() * sizeof(unsigned int), cudaMemcpyHostToDevice));
	safe_cuda(cudaMemcpy(CSR_C_structure, &TEM_CSR_C_structure[0], TEM_CSR_C_structure.size() * sizeof(s_spring), cudaMemcpyHostToDevice));
	safe_cuda(cudaMemcpy(CSR_C_bend, &TEM_CSR_C_bend[0], TEM_CSR_C_bend.size() * sizeof(s_spring), cudaMemcpyHostToDevice));
	
	cout << "springs build successfully!" << endl;
}

void Simulator::build_bvh(Mesh& body)
{
	stop_watch watch;
	watch.start();
	Mesh bvh_body = body;   // for bvh consttruction
	bvh_body.vertex_extend(0.007);

	watch.start();
	cuda_bvh = new BVHAccel(bvh_body);
	watch.stop();
	cout << "bvh build done free time elapsed: " << watch.elapsed() << "us" << endl;
}


void Simulator::simulate(Mesh* sim_cloth)
{
	//cuda kernel compute .........
	/*
	for (int i = 0; i<1000; i++)
	{
		cuda_verlet(sim_cloth->vertices.size());
		//cuda_update_vbo(sim_cloth);     // update array buffer for opengl
		cudaMemcpy(&test[0], x_cur_out, sizeof(glm::vec3) * sim_cloth->vertices.size(), cudaMemcpyDeviceToHost);
		swap_buffer();
	}
	*/
	cuda_verlet(sim_cloth->vertices.size());
	//	cuda_update_vbo(sim_cloth);     // update array buffer for opengl
	//test.resize(sim_cloth->vertices.size());
	cudaMemcpy(&sim_cloth->onestep_vertices[0], x_cur_out, sizeof(glm::vec3) * sim_cloth->vertices.size(), cudaMemcpyDeviceToHost);
	swap_buffer();
}

void Simulator::get_vertex_adjface(Mesh& sim_cloth, vector<unsigned int>& CSR_R, vector<unsigned int>& CSR_C_adjface)
{
	vector<vector<unsigned int>> adjaceny(sim_cloth.vertices.size());
	for(int i=0;i<sim_cloth.faces.size();i++)
	{
		unsigned int f[3];
		for(int j=0;j<3;j++)
		{
			f[j] = sim_cloth.faces[i].vertex_index[j];
			adjaceny[f[j]].push_back(i);
		}
	}

	// i-th vertex adjacent face start_index = CSR_R[i], end_index = CSR_R[i+1]
	// then you can acess CSR_C_adjface[start_index->end_index]
	unsigned int start_idx = 0;
	for(int i=0;i<adjaceny.size();i++)
	{
		CSR_R.push_back(start_idx);
		start_idx += adjaceny[i].size();

		for(int j=0;j<adjaceny[i].size();j++)
		{
			CSR_C_adjface.push_back(adjaceny[i][j]);
		}
	}

	CSR_R.push_back(start_idx);
}

void Simulator::cuda_verlet(const unsigned int numParticles)
{
	unsigned int numThreads, numBlocks;
	
	computeGridSize(numParticles, 512, numBlocks, numThreads);
	verlet <<< numBlocks, numThreads >>>(x_cur_in,x_last_in, x_cur_out, x_last_out,
										CSR_R_structure, CSR_C_structure, CSR_R_bend, CSR_C_bend,
										*cuda_bvh->d_bvh, d_collision_force,
										numParticles);

	// stop the CPU until the kernel has been executed
	safe_cuda(cudaDeviceSynchronize());
}

void Simulator::computeGridSize(unsigned int n, unsigned int blockSize, unsigned int &numBlocks, unsigned int &numThreads)
{
	numThreads = min(blockSize, n);
	numBlocks = (n % numThreads != 0) ? (n / numThreads + 1) : (n / numThreads);
}

void Simulator::swap_buffer()
{
	swap(readID, writeID);

	x_cur_in = x_cur[readID];
	x_cur_out = x_cur[writeID];
	x_last_in = x_last[readID];
	x_last_out = x_last[writeID];
}

void Simulator::update_vertex(glm::vec3 new_value, const unsigned int idx)
{
	safe_cuda(cudaMemcpy(&x_cur_in[idx], &new_value[0], sizeof(glm::vec3), cudaMemcpyHostToDevice));
	safe_cuda(cudaMemcpy(&x_last_in[idx], &new_value[0], sizeof(glm::vec3), cudaMemcpyHostToDevice));
}

