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

#ifndef IGNITION_PHYSICS_DARTSIM_SRC_CUSTOMHEIGHTMAPSHAPE_HH_
#define IGNITION_PHYSICS_DARTSIM_SRC_CUSTOMHEIGHTMAPSHAPE_HH_

#include <dart/dynamics/HeightmapShape.hpp>
#include <ignition/common/HeightmapData.hh>

namespace ignition {
namespace physics {
namespace dartsim {

/// \brief This class creates a custom derivative of dartsim's HeightmapShape
/// class which allows an ignition::common::Heightmap to be converted into a
/// HeightmapShape that can be used by dartsim.
/// Using float precision because Bullet's collision detector doesn't support
/// double. common::HeightmapData also holds floats.
class CustomHeightmapShape : public dart::dynamics::HeightmapShape<float>
{
  /// \brief Constructor
  /// \param[in] _input Holds heightmap data.
  /// \param[in] _size Heightmap size in meters.
  /// \param[in] _subSampling How much to subsample.
  public: CustomHeightmapShape(
      const common::HeightmapData &_input,
      const Eigen::Vector3d &_size,
      const int _subSampling);
};
}
}
}

#endif  // IGNITION_PHYSICS_DARTSIM_SRC_CUSTOMHEIGHTMAPSHAPE_HH_
