#include "rendersystem.h"
#include "scene.h"
Scene* main_scene = nullptr;

//  +-----------------------------------------------------------------------------+
//  |  HostPhysics::HostPhysics                                                   |
//  |  Constructor.                                                               |
//  +-----------------------------------------------------------------------------+
HostPhysics::HostPhysics()
{
}

//  +-----------------------------------------------------------------------------+
//  |  HostPhysics::~HostPhysics                                                  |
//  |  Destructor.                                                                |
//  +-----------------------------------------------------------------------------+
HostPhysics::~HostPhysics()
{
}

void HostPhysics::InitPhysics(const HostMesh* cloth, const HostMesh* obs)
{
	main_scene = Scene::getInstance(); //initialize opengl 

	Mesh cloth_("../_shareddata/Cloth/dress-victor.obj", SINGLE_LAYER_NOB);
	
	Mesh body_("../_shareddata/male.obj");
	
	main_scene->add_cloth(cloth_);
	main_scene->add_body(body_);

	main_scene->init_simulation();
}

void HostPhysics::UpdatePhysics(const float dt)
{
	main_scene->render();
	vector<float4> tmp;
	for (int i = 0; i < main_scene->pcloth->vertex_indices.size(); i++)
	{
		int index = main_scene->pcloth->vertex_indices[i];
		glm::vec4 veci = glm::vec4(main_scene->pcloth->onestep_vertices[index],1.0);
		float4 vec = make_float4(veci.x, veci.y, veci.z, veci.w);
		tmp.push_back(vec);
	}
	vertices = tmp;
}