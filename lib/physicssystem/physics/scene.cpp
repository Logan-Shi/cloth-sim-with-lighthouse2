#include "scene.h"
#include "simulator.h"

using namespace std;

Scene* Scene::pscene = nullptr;
static Simulator* simulation = nullptr;

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

void Scene::add_cloth(Mesh object)
{
	cloth = object;
	pcloth = &cloth;
}

void Scene::add_body(Mesh object)
{
	body = object;
	pbody = &body;
}

void Scene::render()
{
	RenderGPU_CUDA();
}

void Scene::init_simulation()
{
	if (pcloth && pbody)
	{
		simulation = new Simulator(*pcloth, *pbody);
	}
}

void Scene::RenderGPU_CUDA()
{
	if (simulation)
	{
 		simulation->simulate(pscene->pcloth);
	}
}