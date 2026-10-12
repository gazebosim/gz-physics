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

#include <cstddef>

#include <gz/common/geospatial/ImageHeightmap.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/eigen3/Conversions.hh>
#include <sdf/Root.hh>
#include <sdf/World.hh>

#include <gz/plugin/Loader.hh>
#include <gz/physics/FeatureList.hh>
#include <gz/physics/ForwardStep.hh>
#include <gz/physics/GetBoundingBox.hh>
#include <gz/physics/GetEntities.hh>
#include <gz/physics/RequestEngine.hh>
#include <gz/physics/heightmap/HeightmapShape.hh>
#include <gz/physics/sdf/ConstructWorld.hh>

#include "test/Resources.hh"
#include "test/common_test/Worlds.hh"

// The behavior shared with other engines is tested in
// test/common_test/heightmap_features.cc.

using namespace gz;

struct TestFeatures : public physics::FeatureList<
    physics::ForwardStep,
    physics::GetModelFromWorld,
    physics::GetLinkFromModel,
    physics::GetShapeFromLink,
    physics::GetShapeBoundingBox,
    physics::heightmap::AttachHeightmapShapeFeature,
    physics::heightmap::GetHeightmapShapeProperties,
    physics::sdf::ConstructSdfWorld>
{
};

class Heightmap_TEST : public ::testing::Test
{
  protected: void SetUp() override
  {
    plugin::Loader loader;
    loader.LoadLib(mujoco_plugin_LIB);
    this->engine = physics::RequestEngine3d<TestFeatures>::From(
        loader.Instantiate("gz::physics::mujoco::Plugin"));
    ASSERT_NE(nullptr, this->engine);

    sdf::Root root;
    ASSERT_TRUE(root.Load(common_test::worlds::kHeightmapSdf).empty());
    this->world = this->engine->ConstructWorld(*root.WorldByIndex(0));
    ASSERT_NE(nullptr, this->world);
    this->terrain = this->world->GetModel("terrain")->GetLink("link");
    ASSERT_NE(nullptr, this->terrain);

    ASSERT_EQ(0, this->ramp.Load(physics::test::resources::kHeightmapRampPng));
  }

  protected: physics::Engine3dPtr<TestFeatures> engine;
  protected: physics::World3dPtr<TestFeatures> world;
  protected: physics::Link3dPtr<TestFeatures> terrain;
  protected: common::ImageHeightmap ramp;
};

/////////////////////////////////////////////////
TEST_F(Heightmap_TEST, BoundingBox)
{
  common::ImageHeightmap data;
  ASSERT_EQ(0, data.Load(physics::test::resources::kHeightmapBowlPng));

  const math::Vector3d size(129, 129, 10);
  ASSERT_NE(nullptr, this->terrain->AttachHeightmapShape("heightmap", data,
      Eigen::Isometry3d::Identity(), math::eigen3::convert(size)));

  // The bowl's rim is at the full height. The box also contains the solid
  // base that MuJoCo adds below the lowest point.
  const auto box =
      this->terrain->GetShape("heightmap")->GetAxisAlignedBoundingBox();
  EXPECT_NEAR(-size.X() / 2, box.min().x(), 1e-6);
  EXPECT_NEAR(size.Y() / 2, box.max().y(), 1e-6);
  EXPECT_NEAR(size.Z(), box.max().z(), 1e-3);
  EXPECT_GT(0.0, box.min().z());
}

/////////////////////////////////////////////////
// A heightmap that can't be attached leaves the world as it was.
TEST_F(Heightmap_TEST, FailedAttachLeavesWorldIntact)
{
  const Eigen::Vector3d size(34, 34, 2);
  ASSERT_NE(nullptr, this->terrain->AttachHeightmapShape("heightmap",
      this->ramp, Eigen::Isometry3d::Identity(), size));

  // Invalid size
  EXPECT_EQ(nullptr, this->terrain->AttachHeightmapShape("flat", this->ramp,
      Eigen::Isometry3d::Identity(), Eigen::Vector3d(0, 34, 2)));
  // The names are taken
  EXPECT_EQ(nullptr, this->terrain->AttachHeightmapShape("heightmap",
      this->ramp, Eigen::Isometry3d::Identity(), size));

  EXPECT_EQ(1u, this->terrain->GetShapeCount());
  auto shape = this->terrain->GetShape("heightmap");
  ASSERT_NE(nullptr, shape);
  EXPECT_NEAR(size.x() / 2, shape->GetAxisAlignedBoundingBox().max().x(),
              1e-6);

  physics::ForwardStep::Input input;
  physics::ForwardStep::State state;
  physics::ForwardStep::Output output;
  for (std::size_t i = 0; i < 10; ++i)
  {
    this->world->Step(output, state, input);
  }
}

/////////////////////////////////////////////////
// MuJoCo only supports height fields on static bodies.
TEST_F(Heightmap_TEST, AttachToNonStaticLinkFails)
{
  auto link = this->world->GetModel("sphere_low")->GetLink("link");
  ASSERT_NE(nullptr, link);
  EXPECT_EQ(nullptr, link->AttachHeightmapShape("heightmap", this->ramp,
      Eigen::Isometry3d::Identity(), Eigen::Vector3d(34, 34, 2)));
  EXPECT_EQ(1u, link->GetShapeCount());
}
