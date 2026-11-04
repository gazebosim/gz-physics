/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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

#include <memory>

#include "CustomHeightmapShape.hh"

using namespace gz;
using namespace physics;
using namespace dartsim;

/////////////////////////////////////////////////
// The bounding box of the heightmap is its size, not one cell more.
TEST(CustomHeightmapShape, BoundingBoxIsSize)
{
  heightmap::detail::SampledHeightmap sampled;
  sampled.vertSize = 5;
  sampled.heights.assign(sampled.vertSize * sampled.vertSize, 0);
  sampled.heights.back() = 2;

  const CustomHeightmapShape shape(sampled, Eigen::Vector3d(8, 6, 2));

  // The 5 vertices are 4 cells apart.
  EXPECT_TRUE(shape.getScale().isApprox(Eigen::Vector3f(2, 1.5, 1), 1e-6));

  const auto &box = shape.getBoundingBox();
  EXPECT_TRUE(box.getMin().isApprox(Eigen::Vector3d(-4, -3, 0), 1e-6));
  EXPECT_TRUE(box.getMax().isApprox(Eigen::Vector3d(4, 3, 2), 1e-6));

  // A clone keeps the heights, the scale and the bounding box.
  const auto clone =
      std::dynamic_pointer_cast<CustomHeightmapShape>(shape.clone());
  ASSERT_NE(nullptr, clone);
  EXPECT_EQ(shape.getHeightField(), clone->getHeightField());
  EXPECT_EQ(shape.getScale(), clone->getScale());
  EXPECT_EQ(box.getMin(), clone->getBoundingBox().getMin());
  EXPECT_EQ(box.getMax(), clone->getBoundingBox().getMax());
}
