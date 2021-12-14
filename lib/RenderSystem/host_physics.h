/* host_physics.h - Copyright 2019/2021 Logan Shi

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

	   http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/

#pragma once

namespace lighthouse2
{
	//  +-----------------------------------------------------------------------------+
	//  |  HostPhysics                                                                |
	//  +-----------------------------------------------------------------------------+
	class HostMesh;
	class HostPhysics
	{
	public:
		// constructor / destructor
		HostPhysics();
		~HostPhysics();
		// methods
		void InitPhysics(const HostMesh* cloth_vertices, const HostMesh* obs_vertices);
		void UpdatePhysics(const float dt);
		vector<float4> GetVertices() { return vertices; };
	private:
		vector<float4> vertices;
	};
} //namespace lighthouse2

// EOF