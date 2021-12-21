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

void HostPhysics::InitPhysics()
{
	main_scene = Scene::getInstance(); //initialize opengl 

	Mesh cloth_("../_shareddata/Cloth/robe.obj", SINGLE_LAYER_NOB);
	cloth_.translate(0, 2.1, 0);
	Mesh body_("../_shareddata/Cloth/table.obj");
	main_scene->add_cloth(cloth_);
	main_scene->add_body(body_);

	Mesh cloth_2("../_shareddata/Cloth/dress-victor.obj", SINGLE_LAYER_NOB);
	cloth_2.translate(0, 2.1, 0);
	Mesh body_2("../_shareddata/Cloth/table.obj");
	main_scene->add_cloth(cloth_2);
	main_scene->add_body(body_2);

	main_scene->init_simulation();
	vertices.resize(2);
}

void HostPhysics::UpdatePhysics(const float dt)
{
	main_scene->render();
	vector<float4> tmp;
	for (int j = 0; j < main_scene->pcloth.size(); j++)
	{
		for (int i = 0; i < main_scene->pcloth[j]->vertex_indices.size(); i++)
		{
			int index = main_scene->pcloth[j]->vertex_indices[i];
			glm::vec4 veci = glm::vec4(main_scene->pcloth[j]->onestep_vertices[index], 1.0);
			float4 vec = make_float4(veci.x, veci.y, veci.z, veci.w);
			tmp.push_back(vec);
		}
		vertices[j] = tmp;
	}
}