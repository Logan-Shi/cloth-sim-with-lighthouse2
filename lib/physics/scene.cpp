#include "scene.h"
#include "simulator.h"

using namespace std;

// OPENGL场景的各种参数declaration
Scene* Scene::pscene = nullptr;
Simulator* simulation = nullptr;

Scene* Scene::getInstance()
{
	if (pscene == nullptr)
		pscene = new Scene;

	return pscene;
}

Scene::Scene()
{
}

Scene::~Scene()
{
	delete simulation;
}

void Scene::add_cloth(Mesh& object)
{
	cloth = &object;
}

void Scene::add_body(Mesh& object)
{
	body = &object;
}

void Scene::render()
{
	RenderGPU_CUDA();
}

void Scene::init_simulation()
{
	if (cloth && body)
	{
		simulation = new Simulator(*cloth, *body);
	}
}

void Scene::RenderGPU_CUDA()
{
	if (simulation)
	{
 		simulation->simulate(pscene->cloth);
	}
}