/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#include <ignition/math/AxisAlignedBox.hh>
#include <ignition/math/Pose3.hh>

#include "ignition/physics/tpelib/Export.hh"

namespace ignition {
namespace physics {
namespace tpelib {

  /// \brief Transform an axis aligned box by pose
  /// \param[in] _box Axis aligned box to be transformed
  /// \param[in] _pose Transform to be applied
  /// \return New axis aligned box that surrounds the transformed version of
  /// the old box
  IGNITION_PHYSICS_TPELIB_VISIBLE
  math::AxisAlignedBox transformAxisAlignedBox(
      const math::AxisAlignedBox &_box, const math::Pose3d &_pose);
}
}
}



