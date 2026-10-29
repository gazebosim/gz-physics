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

#include <algorithm>
#include <cstddef>
#include <string>
#include <utility>
#include <vector>

#include <mujoco/mujoco.h>

#include <gz/common/geospatial/HeightmapData.hh>
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
#include <gz/physics/Joint.hh>
#include <gz/physics/RequestEngine.hh>
#include <gz/physics/heightmap/HeightmapShape.hh>
#include <gz/physics/sdf/ConstructWorld.hh>

#include "Base.hh"
#include "test/Resources.hh"
#include "test/common_test/Worlds.hh"

// The behavior shared with other engines is tested in
// test/common_test/heightmap_features.cc.

using namespace gz;

struct TestFeatures : public physics::FeatureList<
    physics::ForwardStep,
    physics::GetModelFromWorld,
    physics::GetLinkFromModel,
    physics::GetJointFromModel,
    physics::GetBasicJointState,
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

/////////////////////////////////////////////////
/// \brief Heightmap data of any width whose samples are low in the first
/// half of the grid (-y) and high in the second half.
class TwoLevelHeightmap : public common::HeightmapData
{
  public: TwoLevelHeightmap(unsigned int _width, float _low, float _high)
      : width(_width), low(_low), high(_high)
  {
  }

  public: void FillHeightMap(int, unsigned int _vertSize,
      const math::Vector3d &, const math::Vector3d &, bool,
      std::vector<float> &_heights) const override
  {
    ++this->fillCount;
    _heights.assign(static_cast<std::size_t>(_vertSize) * _vertSize,
                    this->low);
    std::fill(_heights.begin() + _heights.size() / 2, _heights.end(),
              this->high);
  }

  public: unsigned int Height() const override { return this->width; }
  public: unsigned int Width() const override { return this->width; }
  public: float MaxElevation() const override { return this->high; }
  public: float MinElevation() const override { return this->low; }
  public: std::string Filename() const override { return "two_levels"; }

  /// \brief Number of times the heightmap has been sampled.
  public: mutable std::size_t fillCount = 0;

  private: unsigned int width;
  private: float low;
  private: float high;
};

/////////////////////////////////////////////////
// MuJoCo computes nrow * ncol with ints, so it can't index more than 46340
// samples per side. SampleHeightmap rejects heightmaps with more before it
// samples them, which would allocate gigabytes, and the world is left intact.
TEST_F(Heightmap_TEST, TooManySamplesAreRejected)
{
  const Eigen::Vector3d size(34, 34, 2);
  // (width - 1) * subSampling + 1 samples per side: 49153, and 46341, the
  // smallest grid whose number of samples doesn't fit in an int.
  for (const auto &[width, subSampling] :
       {std::pair{4097u, 12}, std::pair{2u, 46340}})
  {
    const TwoLevelHeightmap data(width, 0.0f, 1.0f);
    EXPECT_EQ(nullptr, this->terrain->AttachHeightmapShape("heightmap", data,
        Eigen::Isometry3d::Identity(), size, subSampling));
    EXPECT_EQ(0u, data.fillCount);
  }
  EXPECT_EQ(0u, this->terrain->GetShapeCount());

  ASSERT_NE(nullptr, this->terrain->AttachHeightmapShape("heightmap",
      this->ramp, Eigen::Isometry3d::Identity(), size));
  physics::ForwardStep::Input input;
  physics::ForwardStep::State state;
  physics::ForwardStep::Output output;
  this->world->Step(output, state, input);
}

/////////////////////////////////////////////////
// MuJoCo needs a positive size along z even for a flat heightmap, but a
// small range of heights is the range of the heightmap, not that minimum.
TEST_F(Heightmap_TEST, SmallHeightRangeIsKept)
{
  const float range = 5e-7f;
  const TwoLevelHeightmap data(2, 0.0f, range);
  auto shape = this->terrain->AttachHeightmapShape("small", data,
      Eigen::Isometry3d::Identity(), Eigen::Vector3d(4, 4, 1));
  ASSERT_NE(nullptr, shape);
  EXPECT_NEAR(range, shape->GetSize()[2], 1e-9);
  EXPECT_NEAR(range,
      this->terrain->GetShape("small")->GetAxisAlignedBoundingBox().max().z(),
      1e-9);
}

/////////////////////////////////////////////////
// Attaching a heightmap recompiles the world, which keeps its state.
TEST_F(Heightmap_TEST, AttachPreservesWorldState)
{
  sdf::Root root;
  ASSERT_TRUE(root.LoadSdfString(R"(<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="state">
    <model name="ground">
      <static>true</static>
      <link name="link"/>
    </model>
    <model name="pendulum">
      <pose>0 0 2 0 0 0</pose>
      <link name="arm">
        <inertial>
          <pose>1 0 0 0 0 0</pose>
          <mass>1.0</mass>
          <inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia>
        </inertial>
      </link>
      <joint name="hinge" type="revolute">
        <parent>world</parent>
        <child>arm</child>
        <axis><xyz>0 1 0</xyz></axis>
      </joint>
    </model>
  </world>
</sdf>)").empty());
  auto stateWorld = this->engine->ConstructWorld(*root.WorldByIndex(0));
  ASSERT_NE(nullptr, stateWorld);
  auto hinge = stateWorld->GetModel("pendulum")->GetJoint("hinge");
  ASSERT_NE(nullptr, hinge);

  physics::ForwardStep::Input input;
  physics::ForwardStep::State state;
  physics::ForwardStep::Output output;
  for (std::size_t i = 0; i < 100; ++i)
  {
    stateWorld->Step(output, state, input);
  }

  const auto *worldInfo = static_cast<physics::mujoco::WorldInfo *>(
      stateWorld->FullIdentity().ref.get());
  const double position = hinge->GetPosition(0);
  const double velocity = hinge->GetVelocity(0);
  const double time = worldInfo->mjDataObj->time;
  ASSERT_NE(0.0, position);
  ASSERT_NE(0.0, velocity);
  ASSERT_LT(0.0, time);

  ASSERT_NE(nullptr, stateWorld->GetModel("ground")->GetLink("link")
      ->AttachHeightmapShape("heightmap", this->ramp,
          Eigen::Isometry3d::Identity(), Eigen::Vector3d(34, 34, 2)));

  EXPECT_EQ(position, hinge->GetPosition(0));
  EXPECT_EQ(velocity, hinge->GetVelocity(0));
  EXPECT_EQ(time, worldInfo->mjDataObj->time);
}
