/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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
#include <dart/dynamics/BodyNode.hpp>

#include <gz/common/Console.hh>
#include <gz/common/testing/TestPaths.hh>

#include <gz/math/eigen3.hh>
#include <gz/plugin/Loader.hh>

#include <gz/physics/FrameSemantics.hh>
#include <gz/physics/GetEntities.hh>
#include <gz/physics/Joint.hh>
#include <gz/physics/RequestEngine.hh>
#include <gz/physics/RevoluteJoint.hh>
#include <gz/physics/AddedMass.hh>

#include <gz/physics/sdf/ConstructLink.hh>
#include <gz/physics/sdf/ConstructModel.hh>
#include <gz/physics/sdf/ConstructWorld.hh>
#include <gz/physics/dartsim/World.hh>

#include <sdf/Link.hh>
#include <sdf/Root.hh>
#include <sdf/World.hh>

#include <test/Utils.hh>

struct TestFeatureList : gz::physics::FeatureList<
    gz::physics::GetEntities,
    gz::physics::GetBasicJointState,
    gz::physics::SetBasicJointState,
    gz::physics::LinkFrameSemantics,
    gz::physics::dartsim::RetrieveWorld,
    gz::physics::sdf::ConstructSdfLink,
    gz::physics::sdf::ConstructSdfModel,
    gz::physics::sdf::ConstructSdfWorld,
    gz::physics::AddedMass
> { };

using World = gz::physics::World3d<TestFeatureList>;
using WorldPtr = gz::physics::World3dPtr<TestFeatureList>;
using ModelPtr = gz::physics::Model3dPtr<TestFeatureList>;
using LinkPtr = gz::physics::Link3dPtr<TestFeatureList>;

/////////////////////////////////////////////////
auto LoadEngine()
{
  gz::plugin::Loader loader;
  loader.LoadLib(dartsim_plugin_LIB);

  gz::plugin::PluginPtr dartsim =
      loader.Instantiate("gz::physics::dartsim::Plugin");

  auto engine =
      gz::physics::RequestEngine3d<TestFeatureList>::From(dartsim);
  return engine;
}

/////////////////////////////////////////////////
WorldPtr LoadWorld(const std::string &_world)
{
  auto engine = LoadEngine();
  EXPECT_NE(nullptr, engine);

  sdf::Root root;
  const sdf::Errors &errors = root.Load(_world);
  EXPECT_EQ(0u, errors.size());
  for (const auto & error : errors) {
    std::cout << error << std::endl;
  }

  EXPECT_EQ(1u, root.WorldCount());
  const sdf::World *sdfWorld = root.WorldByIndex(0);
  EXPECT_NE(nullptr, sdfWorld);

  auto world = engine->ConstructWorld(*sdfWorld);
  EXPECT_NE(nullptr, world);

  return world;
}

/////////////////////////////////////////////////
TEST(AddedMassFeatures, AddedMass)
{
  // Expected spatial inertia matrix. This includes inertia due to the body's
  // mass and added mass. Note that the ordering of the matrix is different
  // than the one used in SDF.
  Eigen::Matrix6d expectedSpatialInertia;
  expectedSpatialInertia <<
    17, 17, 18, 4, 9, 13,
    17, 20, 20, 5, 10, 14,
    18, 20, 22, 6, 11, 15,
    4, 5, 6, 2, 2, 3,
    9, 10, 11, 2, 8, 8,
    13, 14, 15, 3, 8, 13;

  // Expected spatial inertia matrix. This includes inertia due to the body's
  // mass and added mass. Note that the ordering of the matrix is different
  // than the one used in SDF.
  Eigen::Matrix6d expectedZeroSpatialInertia;
  expectedZeroSpatialInertia <<
    1, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0,
    0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 1;

  const auto worldPath =
    gz::common::testing::SourceFile("dartsim", "worlds", "added_mass.sdf");
  const auto world = LoadWorld(worldPath);
  ASSERT_NE(nullptr, world);

  auto dartWorld = world->GetDartsimWorld();
  ASSERT_NE(nullptr, dartWorld);

  ASSERT_EQ(3u, dartWorld->getNumSkeletons());

  {
    const auto skeleton = dartWorld->getSkeleton("body_no_added_mass");
    ASSERT_NE(skeleton, nullptr);

    ASSERT_EQ(1u, skeleton->getNumBodyNodes());
    const dart::dynamics::BodyNode *link = skeleton->getBodyNode("link");
    ASSERT_NE(link, nullptr);

    const Eigen::Matrix6d spatialInertia = link->getSpatialInertia();
    ASSERT_TRUE(expectedZeroSpatialInertia.isApprox(spatialInertia));

    const auto linkAddedMass =
      world->GetModel("body_no_added_mass")->GetLink("link")->GetAddedMass();
    ASSERT_TRUE(Eigen::Matrix6d::Zero().isApprox(
          gz::math::eigen3::convert(linkAddedMass)));
  }

  {
    const auto skeleton = dartWorld->getSkeleton("body_zero_added_mass");
    ASSERT_NE(skeleton, nullptr);

    ASSERT_EQ(1u, skeleton->getNumBodyNodes());
    const dart::dynamics::BodyNode *link = skeleton->getBodyNode("link");
    ASSERT_NE(link, nullptr);

    const Eigen::Matrix6d spatialInertia = link->getSpatialInertia();
    ASSERT_TRUE(expectedZeroSpatialInertia.isApprox(spatialInertia));

    auto linkAddedMass =
      world->GetModel("body_zero_added_mass")->GetLink("link")->GetAddedMass();
    ASSERT_TRUE(Eigen::Matrix6d::Zero().isApprox(
          gz::math::eigen3::convert(linkAddedMass)));
  }

  {
    const auto skeleton = dartWorld->getSkeleton("body_added_mass");
    ASSERT_NE(skeleton, nullptr);

    ASSERT_EQ(1u, skeleton->getNumBodyNodes());
    const dart::dynamics::BodyNode *link = skeleton->getBodyNode("link");
    ASSERT_NE(link, nullptr);

    const Eigen::Matrix6d spatialInertia = link->getSpatialInertia();
    ASSERT_TRUE(expectedSpatialInertia.isApprox(spatialInertia));

    auto linkAddedMass =
      world->GetModel("body_added_mass")->GetLink("link")->GetAddedMass();
    ASSERT_FALSE(Eigen::Matrix6d::Zero().isApprox(
          gz::math::eigen3::convert(linkAddedMass)));
  }
}
