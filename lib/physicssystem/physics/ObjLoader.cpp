
#include <iostream>
#include <fstream>
#include <sstream>

#include <Windows.h>

#include "ObjLoader.h"

using namespace std;

ObjLoader::ObjLoader(const string file):obj_file(file)
{
	ifstream input(file);
	if(!input)
	{
		cout <<"error: unable to open input file: " << file << endl;
		exit(-1);
	}

	while (!input.eof())
	{
		string buffer;
		getline(input, buffer);
		stringstream line(buffer);
		string	f0 ,f1, f2, f3;
		line >> f0 >> f1 >> f2 >> f3;

		if (f1 == "object")
		{
			vertex_object.push_back(make_pair(f2,0));
		}
		else if (f2 == "vertices")
		{
			vertex_object.back().second = atoi(f1.c_str());
		}
		else if(f0 == "v")
		{
			glm::vec4 ver(atof(f1.c_str()),atof(f2.c_str()),atof(f3.c_str()),1.0); 
			vertices.push_back(ver);

		}
		else if(f0 == "vn")
		{
			glm::vec3 nor(atof(f1.c_str()),atof(f2.c_str()),atof(f3.c_str())); 
			normals.push_back(nor);
		}
		else if(f0 == "g")   //read each group
		{
			face_group.push_back(make_pair(f1,0));
		}
		else if(f2 == "polygons")
		{
			stringstream tem_line(buffer);
			string	f00 ,f11, f22, f33,f44;
			tem_line >> f00 >> f11 >> f22 >> f33 >> f44;
			face_group.back().second = atoi(f44.c_str());
		}
		else if(f0 == "f")
		{
			Face tem;
			int sPos = 0;
			int ePos = sPos;
			string temp;
			ePos = f1.find_first_of("/");
			if(ePos == string::npos)  //处理不同格式的face, f  1 2 3
			{
				tem.vertex_index[0] = atoi(f1.c_str()) - 1;
				tem.vertex_index[1] = atoi(f2.c_str()) - 1;
				tem.vertex_index[2] = atoi(f3.c_str()) - 1;

				tem.normal_index[0] = atoi(f1.c_str()) - 1;
				tem.normal_index[1] = atoi(f2.c_str()) - 1;
				tem.normal_index[2] = atoi(f3.c_str()) - 1;

			}
			else     //处理不同格式的face, f  1/1/1 2/2/2 3/3/3
			{
				if(ePos != string::npos)  {
					temp = f1.substr(sPos, ePos - sPos);
					tem.vertex_index[0] = atoi(temp.c_str()) - 1;

					sPos = ePos+1;
					ePos = f1.find("/", sPos);
					temp = f1.substr(sPos, ePos - sPos);

					sPos = ePos+1;
					ePos = f1.length();
					temp = f1.substr(sPos, ePos - sPos);
					tem.normal_index[0] = atoi(temp.c_str()) - 1;
				}

				sPos = 0;
				ePos = f2.find_first_of("/");
				if(ePos != string::npos)  {
					temp = f2.substr(sPos, ePos - sPos);
					tem.vertex_index[1] = atoi(temp.c_str()) - 1;

					sPos = ePos + 1;
					ePos = f2.find("/", sPos+1);
					temp = f2.substr(sPos, ePos - sPos);

					sPos = ePos + 1;
					ePos = f2.length();
					temp = f2.substr(sPos, ePos - sPos);
					tem.normal_index[1] = atoi(temp.c_str()) - 1;
				}

				sPos = 0;
				ePos = f3.find_first_of("/");
				if(ePos != string::npos)  {
					temp = f3.substr(sPos, ePos - sPos);
					tem.vertex_index[2] = atoi(temp.c_str()) - 1;

					sPos = ePos + 1;
					ePos = f3.find("/", sPos+1);
					temp = f3.substr(sPos, ePos - sPos);

					sPos = ePos + 1;
					ePos = f3.length();
					temp = f3.substr(sPos, ePos - sPos);
					tem.normal_index[2] = atoi(temp.c_str()) - 1;
				}
			}

			faces.push_back(tem);
		}
	}

	cout << "vertices size" << vertices.size() << endl;
	onestep_vertices.resize(vertices.size());
}

