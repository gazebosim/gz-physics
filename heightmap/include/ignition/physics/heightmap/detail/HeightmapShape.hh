/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#ifndef IGNITION_PHYSICS_HEIGHTMAP_DETAIL_HEIGHTMAPSHAPE_HH_
#define IGNITION_PHYSICS_HEIGHTMAP_DETAIL_HEIGHTMAPSHAPE_HH_

#include <string>

#include <ignition/physics/heightmap/HeightmapShape.hh>

namespace ignition
{
namespace physics
{
namespace heightmap
{
  /////////////////////////////////////////////////
  template <typename PolicyT, typename FeaturesT>
  auto GetHeightmapShapeProperties::HeightmapShape<PolicyT, FeaturesT>::
      GetSize() const -> Dimensions
  {
    return this->template Interface<GetHeightmapShapeProperties>()
        ->GetHeightmapShapeSize(this->identity);
  }

  /////////////////////////////////////////////////
  template <typename PolicyT, typename FeaturesT>
  auto AttachHeightmapShapeFeature::Link<PolicyT, FeaturesT>::
      AttachHeightmapShape(
      const std::string &_name,
      const common::HeightmapData &_heightmapData,
      const PoseType &_pose,
      const Dimensions &_size,
      int _subSampling) -> ShapePtrType
  {
    return ShapePtrType(this->pimpl,
          this->template Interface<AttachHeightmapShapeFeature>()
              ->AttachHeightmapShape(this->identity, _name, _heightmapData,
              _pose, _size, _subSampling));
  }
}
}
}

#endif  // IGNITION_PHYSICS_MESH_DETAIL_MESHSHAPE_HH_
