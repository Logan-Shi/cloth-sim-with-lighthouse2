#include "scene.h"
#include "simulator.h"

using namespace std;

Scene* Scene::pscene = nullptr;
static vector<Simulator*> simulation;

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
	for each (auto var in simulation)
	{
		delete var;
	}
}

void Scene::add_cloth(Mesh object)
{
	cloth.push_back(object);
}

void Scene::add_body(Mesh object)
{
	body.push_back(object);
}

void Scene::render()
{
	RenderGPU_CUDA();
}

void Scene::init_simulation()
{
	pcloth.resize(cloth.size());
	pbody.resize(cloth.size());
	simulation.resize(cloth.size());
	for (int i = 0; i < cloth.size(); i++)
	{
		pcloth[i] = &cloth[i];
		pbody[i] = &body[i];
		if (pcloth[i] && pbody[i])
		{
			Simulator* lsimulation = new Simulator(*pcloth[i], *pbody[i]);
			simulation[i] = lsimulation;
		}
	}
}

void Scene::RenderGPU_CUDA()
{
	for (int i = 0; i < simulation.size(); i++)
	{
		simulation[i]->simulate(pscene->pcloth[i]);
	}
}