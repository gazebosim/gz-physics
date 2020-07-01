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

#include "Utils.hh"

namespace ignition {
namespace physics {
namespace tpelib {

//////////////////////////////////////////////////
math::AxisAlignedBox transformAxisAlignedBox(
    const math::AxisAlignedBox &_box, const math::Pose3d &_pose)
{
  if (_box == math::AxisAlignedBox())
    return _box;

  // Transform the original axis aligned box by applying the pose
  // transformation to all 8 corners. This results in a rotated box, which
  // we need to fit a new axis aligned box to. We can achieve this by creating
  // an empty axis aligned box and merging it with each corner of the rotated
  // box. The result is a larger axis aligned box that surrounds the rotated box

  // old min and max of original axis aligned box
  math::Vector3d oldMin = _box.Min();
  math::Vector3d oldMax = _box.Max();

  // empty min max of new axis aligned box
  // we will merge the empty min and max values with each of the
  //  transformed corners
  math::Vector3d newMin(math::MAX_D, math::MAX_D, math::MAX_D);
  math::Vector3d newMax(math::LOW_D, math::LOW_D, math::LOW_D);

  // min min min
  // transform corner
  math::Vector3d corner = oldMin;
  auto v = _pose.Rot() * corner + _pose.Pos();
  // merge min/max with transformed corner
  newMin.Min(v);
  newMax.Max(v);

  // min min max
  corner.Z() = oldMax.Z();
  v = _pose.Rot() * corner + _pose.Pos();
  newMin.Min(v);
  newMax.Max(v);

  // min max max
  corner.Y() = oldMax.Y();
  v = _pose.Rot() * corner + _pose.Pos();
  newMin.Min(v);
  newMax.Max(v);

  // min max min
  corner.Z() = oldMin.Z();
  v = _pose.Rot() * corner + _pose.Pos();
  newMin.Min(v);
  newMax.Max(v);

  // max max min
  corner.X() = oldMax.X();
  v = _pose.Rot() * corner + _pose.Pos();
  newMin.Min(v);
  newMax.Max(v);

  // max max max
  corner.Z() = oldMax.Z();
  v = _pose.Rot() * corner + _pose.Pos();
  newMin.Min(v);
  newMax.Max(v);

  // max min max
  corner.Y() = oldMin.Y();
  v = _pose.Rot() * corner + _pose.Pos();
  newMin.Min(v);
  newMax.Max(v);

  // max min min
  corner.Z() = oldMin.Z();
  v = _pose.Rot() * corner + _pose.Pos();
  newMin.Min(v);
  newMax.Max(v);

  return math::AxisAlignedBox(newMin, newMax);
}

}
}
}
