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

#ifndef GZ_PHYSICS_DARTSIM_SRC_CUSTOMMESHSHAPE_HH_
#define GZ_PHYSICS_DARTSIM_SRC_CUSTOMMESHSHAPE_HH_

#include <dart/dynamics/MeshShape.hpp>
#include <gz/common/Mesh.hh>

namespace gz {
namespace physics {
namespace dartsim {

/// \brief This class creates a custom derivative of dartsim's MeshShape class
/// which allows an gz::common::Mesh to be converted into a MeshShape that
/// can be used by dartsim.
class CustomMeshShape : public dart::dynamics::MeshShape
{
  public: CustomMeshShape(
      const gz::common::Mesh &_input,
      const Eigen::Vector3d &_scale);
};

}
}
}

#endif  // GZ_PHYSICS_DARTSIM_SRC_CUSTOMMESHSHAPE_HH_
