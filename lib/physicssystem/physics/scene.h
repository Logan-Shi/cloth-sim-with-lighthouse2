#pragma once
#include "Mesh.h"

// Singleton
class Scene
{
public:
	static Scene* getInstance();
	~Scene(); 
	
	void add_cloth(Mesh object);   // mesh(cloth) to be simulated
	void add_body(Mesh object);    // mesh(body) to be collided
	void init_simulation();               // construct simualtion
	void render();
	vector<glm::vec4> get_vertices() { return output_vertices; };

private:
	Scene();  //initial

private:
	static Scene* pscene;       //pscene points to the Scene(singleton)
	enum attributes { position, texture, normal };

	Mesh cloth;
	Mesh body;
	Mesh* pcloth;
	Mesh* pbody;

private:
	static void RenderGPU_CUDA();

private:
	vector<glm::vec4> output_vertices;
};