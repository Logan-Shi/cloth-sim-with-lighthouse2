
#include "scene.h"
#include "wglew.h"
#include <iostream>
#include <FreeImage.h>
#include <fstream> 

#include "Utilities.h"
#include "Ray.h"

using namespace std;


// OPENGL场景的各种参数declaration
Scene* Scene::pscene = nullptr;
int Scene::oldX = 0, Scene::oldY = 0;
float Scene::rX = 15, Scene::rY = 0;
int Scene::state = 1;
float Scene::dist = -23;
float Scene::dy = 0;
GLint Scene::viewport[4];
GLfloat Scene::modelview[16];
GLfloat Scene::projection[16];
glm::vec3 Scene::Up = glm::vec3(0, 1, 0),
		  Scene::Right = glm::vec3(0, 0, 0), 
		  Scene::viewDir= glm::vec3(0, 0, 0);
int Scene::selected_index = -1;
static int current_width;
static int current_height ;
bool Scene::start_sim = false;

static int num_screenshot = 0;
GLenum GL_MODE = GL_LINE_LOOP;
bool SAVE_OBJ = false;

Scene* Scene::getInstance(int argc, char** argv)
{
	if (pscene == nullptr)
		pscene = new Scene(argc,argv);

	return pscene;
}

Scene::Scene(int argc, char** argv)
{
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
	glutInitWindowSize(width,height);
	glutCreateWindow("visg_sim");
	
	GLenum err = glewInit();
	if (err != GLEW_OK) {
		fprintf(stderr, "%s\n", glewGetErrorString(err));
		return;
	}
	wglSwapIntervalEXT(0);  // disable Vertical synchronization
}

Scene::~Scene()
{
	delete simulation;
}

void Scene::add_cloth(Mesh& object)
{
	cloth = &object;
	add(object);
}

void Scene::add_body(Mesh& object)
{
	body = &object;
	add(object);
}

void Scene::add(Mesh& object)
{
	//add VAOs and Buffers
	VAO_Buffer tem_vao;

	glGenVertexArrays(1, &tem_vao.vao);
	glGenBuffers(1, &tem_vao.array_buffer);
	glGenBuffers(1, &tem_vao.index_buffer);
	tem_vao.texture = object.g_textureID;
	tem_vao.index_size = object.vertex_indices.size();
	check_GL_error();

	glBindVertexArray(tem_vao.vao);
	glBindBuffer(GL_ARRAY_BUFFER, tem_vao.array_buffer);

	glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec4)*object.vertices.size() + sizeof(glm::vec2)*object.tex.size() + sizeof(glm::vec3)*object.normals.size(), NULL, GL_STATIC_DRAW);
	glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(glm::vec4)*object.vertices.size(), &object.vertices[0]);
	glBufferSubData(GL_ARRAY_BUFFER, sizeof(glm::vec4)*object.vertices.size(), sizeof(glm::vec2)*object.tex.size(), &object.tex[0]);
	glBufferSubData(GL_ARRAY_BUFFER, sizeof(glm::vec4)*object.vertices.size() + sizeof(glm::vec2)*object.tex.size(), sizeof(glm::vec3)*object.normals.size(), &object.normals[0]);
	check_GL_error();

	glVertexAttribPointer(position, 4, GL_FLOAT, GL_FALSE, sizeof(glm::vec4), 0);
	glVertexAttribPointer(texture, 2, GL_FLOAT, GL_FALSE, sizeof(glm::vec2), (const GLvoid*)(sizeof(glm::vec4)*object.vertices.size()));
	glVertexAttribPointer(normal, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (const GLvoid*)(sizeof(glm::vec4)*object.vertices.size() + sizeof(glm::vec2)*object.tex.size()));

	glEnableVertexAttribArray(position);
	glEnableVertexAttribArray(texture);
	glEnableVertexAttribArray(normal);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, tem_vao.index_buffer);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(GLuint)*object.vertex_indices.size(), &object.vertex_indices[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	glBindVertexArray(0);

	obj_vaos.push_back(tem_vao); // add new vao to the scene
	object.vbo = tem_vao;
}

void Scene::init_simulation()
{
	if (cloth && body)
	{
		simulation = new Simulator(*cloth, *body);
	}
	else
	{
		cout << "check your cloth and body, make sure both ***not*** null!" << endl;
	}
}

void Scene::RenderGPU_CUDA()
{
	if (pscene->simulation && start_sim)
	{
		pscene->simulation->simulate(pscene->cloth);
	}
	
	for (auto vao : pscene->obj_vaos)
		pscene->RenderBuffer(vao);
}