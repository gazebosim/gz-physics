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

#ifndef IGNITION_PHYSICS_MESH_DETAIL_MESHSHAPE_HH_
#define IGNITION_PHYSICS_MESH_DETAIL_MESHSHAPE_HH_

#include <ignition/physics/mesh/MeshShape.hh>

namespace ignition
{
namespace physics
{
namespace mesh
{
  /////////////////////////////////////////////////
  template <typename PolicyT, typename FeaturesT>
  auto GetMeshShapeProperties::MeshShape<PolicyT, FeaturesT>::GetSize() const
  -> Dimensions
  {
    return this->template Interface<GetMeshShapeProperties>()
        ->GetMeshSize(this->identity);
  }

  /////////////////////////////////////////////////
  template <typename PolicyT, typename FeaturesT>
  auto GetMeshShapeProperties::MeshShape<PolicyT, FeaturesT>::GetScale() const
  -> Dimensions
  {
    return this->template Interface<GetMeshShapeProperties>()
        ->GetMeshScale(this->identity);
  }

  /////////////////////////////////////////////////
  template <typename PolicyT, typename FeaturesT>
  void SetMeshShapeProperties::MeshShape<PolicyT, FeaturesT>::SetScale(
      const Dimensions &_dimensions)
  {
    this->template Interface<SetMeshShapeProperties>()
        ->SetMeshShapeScale(this->identity, _dimensions);
  }

  /////////////////////////////////////////////////
  template <typename PolicyT, typename FeaturesT>
  auto AttachMeshShapeFeature::Link<PolicyT, FeaturesT>::AttachMeshShape(
      const std::string &_name,
      const ignition::common::Mesh &_mesh,
      const PoseType &_pose) -> ShapePtrType
  {
    return ShapePtrType(this->pimpl,
          this->template Interface<AttachMeshShapeFeature>()
              ->AttachMeshShape(this->identity, _name, _mesh, _pose));
  }
}
}
}

#endif // IGNITION_PHYSICS_MESH_DETAIL_MESHSHAPE_HH_
