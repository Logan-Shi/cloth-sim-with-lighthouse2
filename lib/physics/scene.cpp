
#include "scene.h"

using namespace std;

// OPENGL场景的各种参数declaration
Scene* Scene::pscene = nullptr;
static int current_width;
static int current_height ;

static int num_screenshot = 0;
bool SAVE_OBJ = false;
vector<glm::vec4> gap_vertices;
vector<glm::vec4> old_vertices;
vector<glm::vec4> new_vertices;
float oldx = 0;
float oldy = 0;
float oldz = 0;
float newx = 0;
float newy = 0;
float newz = 0;

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
	if (pscene->simulation)
	{
 		pscene->simulation->simulate(pscene->cloth);
	}
}