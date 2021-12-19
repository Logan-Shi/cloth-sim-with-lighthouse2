#pragma once
#include <vector>
#include <string>
#include <glm.hpp>

using namespace std;

struct Face
{
	unsigned int vertex_index[3];
	unsigned int normal_index[3];
};

class ObjLoader
{
public:
	ObjLoader(const string file);

public:
	string obj_file;

	vector<glm::vec4> vertices; 
	vector<glm::vec3> onestep_vertices;
	vector<glm::vec3> normals;
	vector<Face> faces;

	vector<pair<string,unsigned int>> vertex_object;  //for vertices region division 
	vector<pair<string,unsigned int>> face_group;
};


