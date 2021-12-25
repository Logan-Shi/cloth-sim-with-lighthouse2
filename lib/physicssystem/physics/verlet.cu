#include "spring.h"
#include <cuda.h>
#include <device_functions.h>
#include <device_launch_parameters.h>
#include <thrust/host_vector.h>
#include <thrust/device_vector.h>

//physics parameter
__constant__ double spring_structure = 1000.0;
__constant__ double spring_bend = 2.0;
__constant__ float damp = -0.02f;
__constant__ float mass = 0.3;
__constant__ float dt = 1 / 100.0f;

__constant__ float gravit_x = 0.0f;   // in y dir
__constant__ float gravit_y = -0.00981f;   // in y dir
__constant__ float gravit_z = 0.0f;   // in y dir
__constant__ int perm[256] = { 151,160,137,91,90,15,
 131,13,201,95,96,53,194,233,7,225,140,36,103,30,69,142,8,99,37,240,21,10,23,
 190, 6,148,247,120,234,75,0,26,197,62,94,252,219,203,117,35,11,32,57,177,33,
 88,237,149,56,87,174,20,125,136,171,168, 68,175,74,165,71,134,139,48,27,166,
 77,146,158,231,83,111,229,122,60,211,133,230,220,105,92,41,55,46,245,40,244,
 102,143,54, 65,25,63,161, 1,216,80,73,209,76,132,187,208, 89,18,169,200,196,
 135,130,116,188,159,86,164,100,109,198,173,186, 3,64,52,217,226,250,124,123,
 5,202,38,147,118,126,255,82,85,212,207,206,59,227,47,16,58,17,182,189,28,42,
 223,183,170,213,119,248,152, 2,44,154,163, 70,221,153,101,155,167, 43,172,9,
 129,22,39,253, 19,98,108,110,79,113,224,232,178,185, 112,104,218,246,97,228,
 251,34,242,193,238,210,144,12,191,179,162,241, 81,51,145,235,249,14,239,107,
 49,192,214, 31,181,199,106,157,184, 84,204,176,115,121,50,45,127, 4,150,254,
 138,236,205,93,222,114,67,29,24,72,243,141,128,195,78,66,215,61,156,180 }; // perlin noise index & slope

// random parameters
__constant__ float alpha_wind = 0.5;
__constant__ float beta_wind = 0.5;
__constant__ float lambda_wind = 0;
__constant__ float split_time_wind = 300000.0;
// direction parameters
__constant__ float thetav_wind = 90.0;
__constant__ float thetal_wind = 90.0;
__constant__ float degree_to_rad = 3.1415926 / 180.0;
// air parameters
__constant__ float C_lift = 5;
__constant__ float C_drag = 5;
__constant__ float strong_wind = 0.1;

__constant__ float critical_length = 0.03;
__constant__ float repul_stiffness = 200;

__constant__ float slope_index = 1000;
__constant__ float reflect_index = 0.1;
__constant__ float cut_time = 0;


__device__ float perlin(float x) {
	// ÕûÊýx1ºÍx2µÄ×ø±ê
	int x1 = floor(x) + 1;
	int x2 = x1 + 1;

	// x1ºÍx2µÄÌÝ¶ÈÖµ
	float grad1 = perm[x1 % 255] * 2.0 - 255.0;
	float grad2 = perm[x2 % 255] * 2.0 - 255.0;
	// x1ºÍx2Ö¸ÏòxµÄ·½ÏòÏòÁ¿
	float vec1 = x - x1;
	float vec2 = x - x2;

	// xµ½x1µÄ¾àÀë¼´vec1£¬ÀûÓÃ¹«Ê½3¼ÆËãÆ½»¬²ÎÊý
	float t = 3 * pow(vec1, 2) - 2 * pow(vec1, 3);

	// ÌÝ¶ÈÖµÓë·½ÏòÏòÁ¿µÄ³Ë»ý
	float product1 = grad1 * vec1;
	float product2 = grad2 * vec2;

	// ²åÖµ
	return product1 + t * (product2 - product1);
}
__device__ glm::vec3 perlin_noise(glm::vec3 point)
{
	glm::vec3 output_noise;
	float x, y, z;
	float noise_length;
	x = perlin(abs(point.x) * 10000 + 0.1);
	y = perlin(abs(point.y) * 10000 + 0.1);
	z = perlin(abs(point.z) * 10000 + 0.1);
	noise_length = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2)) + 0.1;
	x = x / noise_length;
	y = y / noise_length;
	z = z / noise_length;
	output_noise = glm::vec3(x, y, z);

	return output_noise;
}
__device__ glm::vec3 compute_wind_force_lift(unsigned int idx, glm::vec3* g_pos_in, unsigned int* CSR_R, s_spring* CSR_C_SPRING, glm::vec3 vel, glm::vec3 pos, float sim_time)
{
	float now_wind = (float)strong_wind * sin(sim_time / 10.0) + 0.2;
	glm::vec3 vec_wind = now_wind * glm::vec3(cos(thetav_wind * degree_to_rad) * sin(thetal_wind * degree_to_rad), cos(thetav_wind * degree_to_rad) * cos(thetal_wind * degree_to_rad), sin(thetav_wind * degree_to_rad));
	glm::vec3 relative_wind = vec_wind - vel;
	int first_neigh = CSR_R[idx];
	glm::vec3 final_normal = glm::vec3(0.0);
	glm::vec3 F_lift = glm::vec3(0.0);
	glm::vec3 F_drag = glm::vec3(0.0);
	float final_S = 0;
	float gap = 0.0001;
	for (int k = first_neigh; k < CSR_R[idx + 1]; k++)
	{
		if (k <= CSR_R[idx + 1] - 2)
		{
			int index_neigh1 = CSR_C_SPRING[k].end;
			int index_neigh2 = CSR_C_SPRING[k + 1].end;
			volatile auto pos1 = g_pos_in[index_neigh1];
			volatile auto pos2 = g_pos_in[index_neigh2];
			glm::vec3 p1 = glm::vec3(pos1.x, pos1.y, pos1.z);
			glm::vec3 p2 = glm::vec3(pos2.x, pos2.y, pos2.z);
			glm::vec3 temp_normal = glm::cross(p1 - pos, p2 - pos);
			float temp_S = 0.5 * glm::length(temp_normal);
			final_normal += temp_normal;
			final_S += temp_S;
		}
		else
		{
			int index_neigh1 = CSR_C_SPRING[k].end;
			int index_neigh2 = CSR_C_SPRING[first_neigh].end;
			volatile auto pos1 = g_pos_in[index_neigh1];
			volatile auto pos2 = g_pos_in[index_neigh2];
			glm::vec3 p1 = glm::vec3(pos1.x, pos1.y, pos1.z);
			glm::vec3 p2 = glm::vec3(pos2.x, pos2.y, pos2.z);
			glm::vec3 temp_normal = glm::cross(p1 - pos, p2 - pos);
			float temp_S = 0.5 * glm::length(temp_normal);
			final_normal += temp_normal;
			final_S += temp_S;
		}
	}
	final_normal = final_normal / (glm::length(final_normal) + gap);
	float costheta = glm::dot(final_normal, glm::normalize(relative_wind));
	F_drag = -C_drag * final_S * sqrt(1 - costheta * costheta) * relative_wind;
	F_lift = C_lift * final_S * costheta * glm::cross(glm::cross(final_normal, relative_wind), relative_wind);

	return F_lift;
}
__device__ glm::vec3 compute_wind_force_drag(unsigned int idx, glm::vec3* g_pos_in, unsigned int* CSR_R, s_spring* CSR_C_SPRING, glm::vec3 vel, glm::vec3 pos, float sim_time)
{
	float now_wind = (float)strong_wind * sin(sim_time / 10.0) + 0.2;
	glm::vec3 vec_wind = now_wind * glm::vec3(cos(thetav_wind * degree_to_rad) * sin(thetal_wind * degree_to_rad), cos(thetav_wind * degree_to_rad) * cos(thetal_wind * degree_to_rad), sin(thetav_wind * degree_to_rad));
	glm::vec3 relative_wind = vec_wind - vel;
	int first_neigh = CSR_R[idx];
	glm::vec3 final_normal = glm::vec3(0.0);
	glm::vec3 F_lift = glm::vec3(0.0);
	glm::vec3 F_drag = glm::vec3(0.0);
	float final_S = 0;
	float gap = 0.0001;
	for (int k = first_neigh; k < CSR_R[idx + 1]; k++)
	{
		if (k <= CSR_R[idx + 1] - 2)
		{
			int index_neigh1 = CSR_C_SPRING[k].end;
			int index_neigh2 = CSR_C_SPRING[k + 1].end;
			volatile auto pos1 = g_pos_in[index_neigh1];
			volatile auto pos2 = g_pos_in[index_neigh2];
			glm::vec3 p1 = glm::vec3(pos1.x, pos1.y, pos1.z);
			glm::vec3 p2 = glm::vec3(pos2.x, pos2.y, pos2.z);
			glm::vec3 temp_normal = glm::cross(p1 - pos, p2 - pos);
			float temp_S = 0.5 * glm::length(temp_normal);
			final_normal += temp_normal;
			final_S += temp_S;
		}
		else
		{
			int index_neigh1 = CSR_C_SPRING[k].end;
			int index_neigh2 = CSR_C_SPRING[first_neigh].end;
			volatile auto pos1 = g_pos_in[index_neigh1];
			volatile auto pos2 = g_pos_in[index_neigh2];
			glm::vec3 p1 = glm::vec3(pos1.x, pos1.y, pos1.z);
			glm::vec3 p2 = glm::vec3(pos2.x, pos2.y, pos2.z);
			glm::vec3 temp_normal = glm::cross(p1 - pos, p2 - pos);
			float temp_S = 0.5 * glm::length(temp_normal);
			final_normal += temp_normal;
			final_S += temp_S;
		}
	}
	final_normal = final_normal / (glm::length(final_normal) + gap);
	float costheta = glm::dot(final_normal, glm::normalize(relative_wind));
	F_drag = -C_drag * final_S * sqrt(1 - costheta * costheta) * relative_wind;
	F_lift = C_lift * final_S * costheta * glm::cross(glm::cross(final_normal, relative_wind), relative_wind);

	return F_drag;
}
__device__ glm::vec3 compute_wind_force2_lift(unsigned int idx, glm::vec3* g_pos_in, unsigned int* CSR_R, s_spring* CSR_C_SPRING, glm::vec3 vel, glm::vec3 pos, glm::vec3 vec_wind)
{
	glm::vec3 relative_wind = vec_wind - vel;
	int first_neigh = CSR_R[idx];
	glm::vec3 final_normal = glm::vec3(0.0);
	glm::vec3 F_lift = glm::vec3(0.0);
	glm::vec3 F_drag = glm::vec3(0.0);
	float final_S = 0;
	float gap = 0.0001;
	for (int k = first_neigh; k < CSR_R[idx + 1]; k++)
	{
		if (k <= CSR_R[idx + 1] - 2)
		{
			int index_neigh1 = CSR_C_SPRING[k].end;
			int index_neigh2 = CSR_C_SPRING[k + 1].end;
			volatile auto pos1 = g_pos_in[index_neigh1];
			volatile auto pos2 = g_pos_in[index_neigh2];
			glm::vec3 p1 = glm::vec3(pos1.x, pos1.y, pos1.z);
			glm::vec3 p2 = glm::vec3(pos2.x, pos2.y, pos2.z);
			glm::vec3 temp_normal = glm::cross(p1 - pos, p2 - pos);
			float temp_S = 0.5 * glm::length(temp_normal);
			final_normal += temp_normal;
			final_S += temp_S;
		}
		else
		{
			int index_neigh1 = CSR_C_SPRING[k].end;
			int index_neigh2 = CSR_C_SPRING[first_neigh].end;
			volatile auto pos1 = g_pos_in[index_neigh1];
			volatile auto pos2 = g_pos_in[index_neigh2];
			glm::vec3 p1 = glm::vec3(pos1.x, pos1.y, pos1.z);
			glm::vec3 p2 = glm::vec3(pos2.x, pos2.y, pos2.z);
			glm::vec3 temp_normal = glm::cross(p1 - pos, p2 - pos);
			float temp_S = 0.5 * glm::length(temp_normal);
			final_normal += temp_normal;
			final_S += temp_S;
		}
	}
	final_normal = final_normal / (glm::length(final_normal) + gap);
	float costheta = glm::dot(final_normal, glm::normalize(relative_wind));
	F_drag = -C_drag * final_S * sqrt(1 - costheta * costheta) * relative_wind;
	F_lift = C_lift * final_S * costheta * glm::cross(glm::cross(final_normal, relative_wind), relative_wind);

	return F_lift;
}
__device__ glm::vec3 compute_wind_force2_drag(unsigned int idx, glm::vec3* g_pos_in, unsigned int* CSR_R, s_spring* CSR_C_SPRING, glm::vec3 vel, glm::vec3 pos, glm::vec3 vec_wind)
{
	glm::vec3 relative_wind = vec_wind - vel;
	int first_neigh = CSR_R[idx];
	glm::vec3 final_normal = glm::vec3(0.0);
	glm::vec3 F_lift = glm::vec3(0.0);
	glm::vec3 F_drag = glm::vec3(0.0);
	float final_S = 0;
	float gap = 0.0001;
	for (int k = first_neigh; k < CSR_R[idx + 1]; k++)
	{
		if (k <= CSR_R[idx + 1] - 2)
		{
			int index_neigh1 = CSR_C_SPRING[k].end;
			int index_neigh2 = CSR_C_SPRING[k + 1].end;
			volatile auto pos1 = g_pos_in[index_neigh1];
			volatile auto pos2 = g_pos_in[index_neigh2];
			glm::vec3 p1 = glm::vec3(pos1.x, pos1.y, pos1.z);
			glm::vec3 p2 = glm::vec3(pos2.x, pos2.y, pos2.z);
			glm::vec3 temp_normal = glm::cross(p1 - pos, p2 - pos);
			float temp_S = 0.5 * glm::length(temp_normal);
			final_normal += temp_normal;
			final_S += temp_S;
		}
		else
		{
			int index_neigh1 = CSR_C_SPRING[k].end;
			int index_neigh2 = CSR_C_SPRING[first_neigh].end;
			volatile auto pos1 = g_pos_in[index_neigh1];
			volatile auto pos2 = g_pos_in[index_neigh2];
			glm::vec3 p1 = glm::vec3(pos1.x, pos1.y, pos1.z);
			glm::vec3 p2 = glm::vec3(pos2.x, pos2.y, pos2.z);
			glm::vec3 temp_normal = glm::cross(p1 - pos, p2 - pos);
			float temp_S = 0.5 * glm::length(temp_normal);
			final_normal += temp_normal;
			final_S += temp_S;
		}
	}
	final_normal = final_normal / (glm::length(final_normal) + gap);
	float costheta = glm::dot(final_normal, glm::normalize(relative_wind));
	F_drag = -C_drag * final_S * sqrt(1 - costheta * costheta) * relative_wind;
	F_lift = C_lift * final_S * costheta * glm::cross(glm::cross(final_normal, relative_wind), relative_wind);

	return F_drag;
}
__device__ glm::vec3 wind_velocity(glm::vec3 F_lift, glm::vec3 F_drag, float sim_time)
{
	glm::vec3 vec_wind = strong_wind * glm::vec3(cos(thetav_wind * degree_to_rad) * sin(thetal_wind * degree_to_rad), cos(thetav_wind * degree_to_rad) * cos(thetal_wind * degree_to_rad), sin(thetav_wind * degree_to_rad));
	vec_wind += alpha_wind * perlin_noise(F_lift) + beta_wind * perlin_noise(F_drag);
	/*
	if (sim_time <= split_time_wind)
	{
		vec_wind += alpha_wind * sim_time * perlin_noise(F_lift) + beta_wind * sim_time * perlin_noise(F_drag);
	}
	else
	{
		glm::vec3 sum_force = alpha_wind * F_lift + beta_wind * F_drag;
		float cx = cos(sum_force.x);
		float cy = cos(sum_force.y);
		float cz = cos(sum_force.z);
		vec_wind += lambda_wind * glm::vec3(cx, cy, cz);
	}


	return vec_wind;
	*/
	return vec_wind;
}
__device__ glm::vec3 compute_selfcollide_force(unsigned int idx, glm::vec3* g_pos_in, unsigned int num, unsigned int* CSR_R1, s_spring* CSR_C_SPRING1, unsigned int* CSR_R2, s_spring* CSR_C_SPRING2)
{
	float dis = 1;
	int first_neigh1 = CSR_R1[idx];
	int last_neigh1 = CSR_R1[idx + 1];
	int first_neigh2 = CSR_R2[idx];
	int last_neigh2 = CSR_R2[idx + 1];
	float temp_stiffness = repul_stiffness;
	float all_neigh[15] = {0};
	for (int i = first_neigh1; i < last_neigh1; i++)
	{
		//all_neigh.push_back(CSR_C_SPRING1[i].end);
		all_neigh[i - first_neigh1] = CSR_C_SPRING1[i].end;
	}
	for (int i = first_neigh2; i < last_neigh2; i++)
	{
		//all_neigh.push_back(CSR_C_SPRING2[i].end);
		all_neigh[last_neigh1 - first_neigh1 + i - first_neigh2] = CSR_C_SPRING2[i].end;
	}
	
	glm::vec3 repul_force = glm::vec3(0.0);
	glm::vec3 pos2;
	volatile glm::vec3 posData = g_pos_in[idx];
	glm::vec3 pos = glm::vec3(posData.x, posData.y, posData.z);
	for (int i = 0; i < num; i++)
	{
		for (int k=0; k < last_neigh2-first_neigh2+last_neigh1-first_neigh1; k++)
		{
			if (all_neigh[k] == i)
			{
				temp_stiffness = 0;
				break;
			}
		}
		volatile glm::vec3 posData = g_pos_in[i];
		pos2 = glm::vec3(posData.x, posData.y, posData.z);
		pos2 = pos2 - pos;
		dis = glm::length(pos2);
		if (dis > 0 && dis < critical_length)
		{
			repul_force -= temp_stiffness * (critical_length - dis) * glm::normalize(pos2);
		}
		temp_stiffness = repul_stiffness;

	}

	return repul_force;
}

__device__ void collision_response_projection(D_BVH bvh,
	glm::vec3& force, glm::vec3& pos, glm::vec3& pos_old,
	int idx, glm::vec3* collision_force, glm::vec3& vel, float sim_time)
{
	glm::vec3 pos1;
	glm::vec3 pos2;
	glm::vec3 dir;
	int idx_pri;
	bool inter = bvh.intersect(pos, idx_pri);
	if (inter)
	{
		float dist;
		float d, d1, d2;
		float mid;
		float gap = 0.00001;
		glm::vec3 normal;
		if (bvh.primitive_intersect(idx_pri, pos, dist, normal))  // check the point inside the primitive or not
		{
			float k = 1.0;
			dist = k * glm::abs(dist);    // //collision response with penalty force

			pos1 = pos + dist * normal;
			d = glm::dot(normal, pos1);
			d2 = glm::dot(normal, pos) - d;
			d1 = glm::dot(normal, pos_old) - d;
			mid = (abs(d2) + 0.00001) / (abs(d1) + abs(d2) + 0.00001);
			pos2 = mid * pos_old + (1 - mid) * pos;
			dir = (slope_index * (pos1 - pos2) + dist * normal) / (glm::length(slope_index * (pos1 - pos2) + dist * normal) + gap);
			if (sim_time > cut_time)
			{
				pos = pos2 + reflect_index * glm::length(vel) * dir * dt;
				pos_old = pos2;
			}
			else
			{
				pos = pos + dist * normal;
				pos_old = pos;
			}

			collision_force[idx] = normal;
		}
		else
			collision_force[idx] = glm::vec3(0.0);
	}
	else
		collision_force[idx] = glm::vec3(0.0);
}

__device__ glm::vec3 compute_spring_force(int index, glm::vec3* g_pos_in, glm::vec3* g_pos_old_in,
	unsigned int* CSR_R, s_spring* CSR_C_SPRING,
	glm::vec3 pos, glm::vec3 vel, float k_spring)
{
	glm::vec3 force(0.0);
	int first_neigh = CSR_R[index];
	int time = 0;
	for (int k = first_neigh; k < CSR_R[index + 1]; k++)
	{
		float ks = k_spring;
		float kd = -0.5;

		int index_neigh = CSR_C_SPRING[k].end;
		volatile auto pos_neighData = g_pos_in[index_neigh];
		volatile auto pos_lastData = g_pos_old_in[index_neigh];
		glm::vec3 p2 = glm::vec3(pos_neighData.x, pos_neighData.y, pos_neighData.z);
		glm::vec3 p2_last = glm::vec3(pos_lastData.x, pos_lastData.y, pos_lastData.z);

		glm::vec3 v2 = (p2 - p2_last) / dt;
		glm::vec3 deltaP = pos - p2;

		deltaP += glm::vec3(FLT_EPSILON);    //avoid 0

		glm::vec3 deltaV = vel - v2;
		float dist = glm::length(deltaP);


		float original_length = CSR_C_SPRING[k].original;
		float leftTerm = -ks * (dist - original_length);
		float  rightTerm = kd * (glm::dot(deltaV, deltaP) / dist);
		glm::vec3 springForce = (leftTerm + rightTerm) * glm::normalize(deltaP);

		force += springForce;
	}
	return force;
}


__global__ void compute_face_normal(glm::vec3* g_pos_in, unsigned int* cloth_index, const unsigned int cloth_index_size, glm::vec3* cloth_face)
{
	unsigned int index = blockIdx.x * blockDim.x + threadIdx.x;
	unsigned int max_thread = cloth_index_size / 3;
	if (index >= max_thread)
		return;

	unsigned int f_index[3];
	for (int i = 0; i < 3; i++)
		f_index[i] = index * 3 + i;

	glm::vec3 vertex[3];
	for (int i = 0; i < 3; i++)
		vertex[i] = g_pos_in[cloth_index[f_index[i]]];  //find the fucking bug!

	glm::vec3 pos[3];
	for (int i = 0; i < 3; i++)
		pos[i] = glm::vec3(vertex[i].x, vertex[i].y, vertex[i].z);

	glm::vec3 side1, side2, normal;
	side1 = pos[1] - pos[0];
	side2 = pos[2] - pos[0];
	normal = glm::normalize(glm::cross(side1, side2));

	cloth_face[index] = normal;
}


__global__ void update_vbo_pos(glm::vec4* pos_vbo, glm::vec3* pos_cur, const unsigned int NUM_VERTICES)
{
	unsigned int index = blockIdx.x * blockDim.x + threadIdx.x;
	if (index >= NUM_VERTICES)
		return;

	auto pos = pos_cur[index];
	pos_vbo[index] = glm::vec4(pos.x, pos.y, pos.z, 1.0);
}

__global__ void verlet(glm::vec3* g_pos_in, glm::vec3* g_pos_old_in, glm::vec3* g_pos_out, glm::vec3* g_pos_old_out,
	unsigned int* CSR_R_STR, s_spring* CSR_C_STR, unsigned int* CSR_R_BD, s_spring* CSR_C_BD,
	D_BVH bvh, glm::vec3* collision_force,
	const unsigned int NUM_VERTICES, float* gpu_time, float sim_time, glm::vec3* detect_force)
{
	unsigned int index = blockIdx.x * blockDim.x + threadIdx.x;
	if (index >= NUM_VERTICES)
		return;

	volatile glm::vec3 posData = g_pos_in[index];
	volatile glm::vec3 posOldData = g_pos_old_in[index];


	glm::vec3 pos = glm::vec3(posData.x, posData.y, posData.z);
	glm::vec3 pos_old = glm::vec3(posOldData.x, posOldData.y, posOldData.z);
	glm::vec3 vel = (pos - pos_old) / dt;

	glm::vec3 gravity(gravit_x, gravit_y, gravit_z);
	glm::vec3 force = gravity * mass + vel * damp;
	glm::vec3 F_lift;
	glm::vec3 F_drag;
	force += compute_spring_force(index, g_pos_in, g_pos_old_in, CSR_R_STR, CSR_C_STR, pos, vel, spring_structure); // Compute structure spring force
	force += compute_spring_force(index, g_pos_in, g_pos_old_in, CSR_R_BD, CSR_C_BD, pos, vel, spring_bend); // Compute bend spring force
	// wind_begin
	F_lift = compute_wind_force_lift(index, g_pos_in, CSR_R_STR, CSR_C_STR, vel, pos, sim_time);
	F_drag = compute_wind_force_drag(index, g_pos_in, CSR_R_STR, CSR_C_STR, vel, pos, sim_time);
	glm::vec3 vec_wind = wind_velocity(F_lift, F_drag, sim_time);
	F_lift = compute_wind_force2_lift(index, g_pos_in, CSR_R_STR, CSR_C_STR, vel, pos, vec_wind);
	F_drag = compute_wind_force2_drag(index, g_pos_in, CSR_R_STR, CSR_C_STR, vel, pos, vec_wind);
	force = force + F_lift + F_drag;
	// wind_end
	force += compute_selfcollide_force(index, g_pos_in, NUM_VERTICES, CSR_R_STR, CSR_C_STR, CSR_R_BD, CSR_C_BD);
	glm::vec3 inelastic_force = glm::dot(collision_force[index], force) * collision_force[index];       //collision response force, if intersected, keep tangential
	force -= inelastic_force;
	glm::vec3 acc = force / mass;
	glm::vec3 tmp = pos;
	pos = pos + pos - pos_old + acc * dt * dt;
	pos_old = tmp;
	collision_response_projection(bvh, force, pos, pos_old, index, collision_force, vel, sim_time);
	//if (sim_time>100)
	//	collision_response_projection(cloth_bvh, force, pos, pos_old, index, collision_force);

	g_pos_out[index] = pos;
	g_pos_old_out[index] = pos_old;
	*gpu_time = sim_time;
	*detect_force = F_lift;

}
__global__ void compute_vbo_normal(glm::vec3* normals, unsigned int* CSR_R, unsigned int* CSR_C_adjface_to_vertex, glm::vec3* face_normal, const unsigned int NUM_VERTICES)
{

	unsigned int index = blockIdx.x * blockDim.x + threadIdx.x;
	if (index >= NUM_VERTICES)
		return;

	//compute point normal
	glm::vec3 normal(0.0);
	int first_face_index = CSR_R[index];
	for (int i = first_face_index; i < CSR_R[index + 1]; i++)
	{
		int findex = CSR_C_adjface_to_vertex[i];
		glm::vec3 fnormal = face_normal[findex];
		normal += fnormal;
	}
	normal = glm::normalize(normal);

	normals[index] = normal;
}