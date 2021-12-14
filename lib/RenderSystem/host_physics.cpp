#include "rendersystem.h"
#include "host_physics.h"
#include "../physics/scene.h"

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

	Mesh cloth_("../_shareddata/Cloth/Cloth.obj", SINGLE_LAYER_NOB);
	
	Mesh body_("../_shareddata/male.obj");
	
	main_scene->add_cloth(cloth_);
	main_scene->add_body(body_);

	main_scene->init_simulation();
}

void HostPhysics::UpdatePhysics(const float dt)
{
	main_scene->render();
}