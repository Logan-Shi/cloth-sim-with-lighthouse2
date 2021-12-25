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

	Mesh table_cloth("../_shareddata/Cloth/table_cloth.obj", SINGLE_LAYER_NOB);
	//table_cloth.scale(3.5);
	//table_cloth.rotation(90, Y);
	//table_cloth.translate(0.2, 1.29, 0);
	Mesh table("../_shareddata/Cloth/table.obj");
	main_scene->add_cloth(table_cloth);
	table.scale(1.1);
	table.translate(0,0.81,0);
	main_scene->add_body(table);
	
	Mesh curtain_1("../_shareddata/Cloth/curtain.obj", SINGLE_LAYER_NOB);
	Mesh holder_1("../_shareddata/Cloth/holder1.obj");
	main_scene->add_cloth(curtain_1);
	main_scene->add_body(holder_1);

	Mesh curtain_2("../_shareddata/Cloth/curtain2.obj", SINGLE_LAYER_NOB);
	Mesh holder_2("../_shareddata/Cloth/holder2.obj");
	main_scene->add_cloth(curtain_2);
	main_scene->add_body(holder_2);

	//Mesh curtain_3("../_shareddata/Cloth/curtain3.obj", SINGLE_LAYER_NOB);
	//Mesh holder_3("../_shareddata/Cloth/holder3.obj");
	//main_scene->add_cloth(curtain_3);
	//main_scene->add_body(holder_3);

	//Mesh curtain_4("../_shareddata/Cloth/curtain4.obj", SINGLE_LAYER_NOB);
	//Mesh holder_4("../_shareddata/Cloth/holder4.obj");
	//main_scene->add_cloth(curtain_4);
	//main_scene->add_body(holder_4);

	main_scene->init_simulation();
	vertices.resize(3);
}

void HostPhysics::UpdatePhysics(const float dt)
{
	main_scene->render();
	for (int j = 0; j < main_scene->pcloth.size(); j++)
	{
		vector<float4> tmp;
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