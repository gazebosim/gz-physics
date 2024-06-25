/*
 * Copyright (C) 2018 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include "CustomConeMeshShape.hh"

#include <memory>
#include <string>

#include <gz/common/Console.hh>
#include <gz/common/MeshManager.hh>
#include <gz/math/Cone.hh>

namespace gz {
namespace physics {
namespace dartsim {

/////////////////////////////////////////////////
const gz::common::Mesh* MakeCustomConeMesh(
    const gz::math::Coned &_cone,
    int _meshRings,
    int _meshSegments)
{
  common::MeshManager *meshMgr = common::MeshManager::Instance();
  std::string coneMeshName = std::string("cone_mesh")
    + "_" + std::to_string(_cone.Radius())
    + "_" + std::to_string(_cone.Length());
  meshMgr->CreateCone(
    coneMeshName,
    _cone.Radius(),
    _cone.Length(),
    _meshRings, _meshSegments);
  return meshMgr->MeshByName(coneMeshName);
}

/////////////////////////////////////////////////
CustomConeMeshShape::CustomConeMeshShape(
    const gz::math::Coned &_cone,
    int _meshRings,
    int _meshSegments)
  : CustomMeshShape(*MakeCustomConeMesh(_cone, _meshRings, _meshSegments),
                    Eigen::Vector3d(1, 1, 1)),
    cone(_cone)
{
}

}
}
}
