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

#include <gtest/gtest.h>

#include "Utils.hh"

using namespace ignition;
using namespace physics;
using namespace tpelib;

/////////////////////////////////////////////////
TEST(Utils, TransformAxisAlignedBox)
{
  math::Pose3d p(2, 3, 4, 0, IGN_PI * 0.5, 0);
  math::AxisAlignedBox box;
  math::AxisAlignedBox boxTransformed = transformAxisAlignedBox(box, p);
  EXPECT_EQ(math::AxisAlignedBox(), boxTransformed);

  // box2 centered at [0, 0, 0] with size [2, 4, 6]
  math::AxisAlignedBox box2(
      math::Vector3d(-1, -2, -3), math::Vector3d(1, 2, 3));

  // translate box2
  math::Pose3d poseNoRot(2, 2, 2, 0, 0, 0, 0);
  math::AxisAlignedBox box2Transformed =
      transformAxisAlignedBox(box2, poseNoRot);
  EXPECT_EQ(math::Vector3d(2, 2, 2), box2Transformed.Center());
  EXPECT_EQ(box2.Size(), box2Transformed.Size());
  EXPECT_EQ(math::AxisAlignedBox(math::Vector3d(1, 0, -1),
      math::Vector3d(3, 4, 5)), box2Transformed);

  // translate and rotate box2
  math::Pose3d pose(2, 2, 2, 0, IGN_PI * 0.5, 0);
  math::AxisAlignedBox box2TransformedRot =
      transformAxisAlignedBox(box2, pose);
  EXPECT_EQ(math::Vector3d(2, 2, 2), box2TransformedRot.Center());
  EXPECT_EQ((pose.Rot() * box2.Size()).Abs(), box2TransformedRot.Size());
  EXPECT_EQ(math::AxisAlignedBox(math::Vector3d(-1, 0, 1),
      math::Vector3d(5, 4, 3)), box2TransformedRot);
}
