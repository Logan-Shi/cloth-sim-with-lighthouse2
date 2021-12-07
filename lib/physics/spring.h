#pragma once
#include "Mesh.h"
#include <set>
#include <vector>
#include <map>

//simplified spring for gpu
struct s_spring      
{
	int end;  //һ�˵ĵ������
	float original;   //����ԭ��
};

class Springs
{
public:
	~Springs();
	Springs(Mesh* spring_obj);  //������������boundary and cloth���ɾ���ϵ��һ��
	void draw(Mesh* spring_obj);
	void CSR_structure_spring(Mesh* spring_obj, vector<unsigned int>& CSR_R, vector<s_spring>& CSR_C);
	void CSR_bend_spring(Mesh* spring_obj, vector<unsigned int>& CSR_R, vector<s_spring>& CSR_C);

private:
	vector<pair<unsigned int,unsigned int>> cloth_boundary_springs;   //ֻ����pair(1,2)
	vector<pair<unsigned int,unsigned int>> boundary_boundary_springs;   //Ӧ���Ѿ�����pair(1,2) && pair(2,1)
	set<pair<unsigned int,unsigned int>> boundary;
	vector<vector<unsigned int>> neigh1;   //�洢ÿ���������һ��������Ϣ(�洢�������),�� structure spring
	vector<vector<unsigned int>> neigh2;   //�洢ÿ��������ж���������Ϣ(�洢�������),�� bend spring
	map<pair<unsigned int, unsigned int>,float> bend_spring_length;  //�洢�������������ζԽǶ�������+�����ľ���

private:
	void create_neigh(Mesh* spring_obj);
	bool exist(const vector<unsigned int>& array, const unsigned int val);
	void get_cloth_boundary_spring(Mesh* spring_obj);
	void get_boundary_boundary_spring(Mesh* spring_obj);
	void CSR_spring(const Mesh* spring_obj, const vector<vector<unsigned int>>& neigh, vector<unsigned int>& CSR_R, vector<s_spring>& CSR_C);
};