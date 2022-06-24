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

#include <gz/common/Console.hh>

#include "../helpers/TestLibLoader.hh"
#include "../Utils.hh"

#include <gz/physics/FindFeatures.hh>
#include <gz/physics/GetEntities.hh>
#include <gz/physics/RequestEngine.hh>

#include <gz/physics/ForwardStep.hh>
#include <gz/physics/FrameSemantics.hh>
#include <gz/physics/GetBoundingBox.hh>
#include <gz/physics/World.hh>
#include <gz/physics/sdf/ConstructWorld.hh>

#include <sdf/Root.hh>
#include <sdf/World.hh>

// The features that an engine must have to be loaded by this loader.
using Features = gz::physics::FeatureList<
  gz::physics::sdf::ConstructSdfWorld,
  gz::physics::CollisionDetector,
  gz::physics::Solver,
  gz::physics::GetEntities
>;

using TestEnginePtr = gz::physics::Engine3dPtr<Features>;
using TestWorldPtr = gz::physics::World3dPtr<Features>;

class WorldFeaturesTest:
  public gz::physics::TestLibLoader
{
  // Documentation inherited
  public: void SetUp() override
  {
    gz::common::Console::SetVerbosity(4);

    auto plugins = loader.LoadLib(WorldFeaturesTest::GetLibToTest());

    // TODO(ahcorde): We should also run the 3f, 2d, and 2f variants of
    // FindFeatures
    pluginNames = gz::physics::FindFeatures3d<Features>::From(loader);
    if (pluginNames.empty())
    {
      std::cerr << "No plugins with required features found in " << GetLibToTest();
      // TODO(ahcorde): If we update gtest we can use here GTEST_SKIP()
    }
  }
};

// A predicate-formatter for asserting that two vectors are approximately equal.
class AssertVectorApprox
{
  public: explicit AssertVectorApprox(double _tol = 1e-6) : tol(_tol)
  {
  }

  public: ::testing::AssertionResult operator()(
              const char *_mExpr, const char *_nExpr, Eigen::Vector3d _m,
              Eigen::Vector3d _n)
  {
    if (gz::physics::test::Equal(_m, _n, this->tol))
      return ::testing::AssertionSuccess();

    return ::testing::AssertionFailure()
           << _mExpr << " and " << _nExpr << " ([" << _m.transpose()
           << "] and [" << _n.transpose() << "]"
           << ") are not equal";
  }

  private: double tol;
};

/////////////////////////////////////////////////
TEST_F(WorldFeaturesTest, ConstructEmptyWorld)
{
  for (const std::string &name : pluginNames)
  {
    std::cout << "Testing plugin: " << name << std::endl;
    gz::plugin::PluginPtr plugin = loader.Instantiate(name);

    auto engine = gz::physics::RequestEngine3d<Features>::From(plugin);
    ASSERT_NE(nullptr, engine);
    EXPECT_TRUE(engine->GetName().find(PhysicsEngineName(name)) !=
                std::string::npos);

    sdf::Root root;
    const sdf::Errors errors = root.Load(TEST_WORLD_DIR "/empty.sdf");
    EXPECT_TRUE(errors.empty());
    const sdf::World *sdfWorld = root.WorldByIndex(0);

    auto world = engine->ConstructWorld(*sdfWorld);
    EXPECT_EQ("ode", world->GetCollisionDetector());

    world->SetCollisionDetector("banana");
    EXPECT_EQ("ode", world->GetCollisionDetector());

    world->SetCollisionDetector("bullet");
    EXPECT_EQ("bullet", world->GetCollisionDetector());

    world->SetCollisionDetector("fcl");
    EXPECT_EQ("fcl", world->GetCollisionDetector());

    world->SetCollisionDetector("ode");
    EXPECT_EQ("ode", world->GetCollisionDetector());

    world->SetCollisionDetector("dart");
    EXPECT_EQ("dart", world->GetCollisionDetector());
  }
}

//////////////////////////////////////////////////
TEST_F(WorldFeaturesTest, Solver)
{
  for (const std::string &name : pluginNames)
  {
    std::cout << "Testing plugin: " << name << std::endl;
    gz::plugin::PluginPtr plugin = loader.Instantiate(name);

    auto engine = gz::physics::RequestEngine3d<Features>::From(plugin);
    ASSERT_NE(nullptr, engine);
    EXPECT_TRUE(engine->GetName().find(PhysicsEngineName(name)) !=
                std::string::npos);

    sdf::Root root;
    const sdf::Errors errors = root.Load(TEST_WORLD_DIR "/empty.sdf");
    EXPECT_TRUE(errors.empty());
    const sdf::World *sdfWorld = root.WorldByIndex(0);

    auto world = engine->ConstructWorld(*sdfWorld);
    EXPECT_EQ("DantzigBoxedLcpSolver", world->GetSolver());

    world->SetSolver("banana");
    EXPECT_EQ("DantzigBoxedLcpSolver", world->GetSolver());

    world->SetSolver("dantzig");
    EXPECT_EQ("DantzigBoxedLcpSolver", world->GetSolver());

    world->SetSolver("pgs");
    EXPECT_EQ("PgsBoxedLcpSolver", world->GetSolver());
  }
}

// The features that an engine must have to be loaded by this loader.
using FeaturesGravity = gz::physics::FeatureList<
  gz::physics::Gravity,
  gz::physics::ForwardStep,
  gz::physics::LinkFrameSemantics,
  gz::physics::sdf::ConstructSdfWorld,
  gz::physics::GetEntities
>;

class WorldFeaturesGravityTest:
  public gz::physics::TestLibLoader
{
  // Documentation inherited
  public: void SetUp() override
  {
    gz::common::Console::SetVerbosity(4);

    auto plugins = loader.LoadLib(WorldFeaturesTest::GetLibToTest());

    // TODO(ahcorde): We should also run the 3f, 2d, and 2f variants of
    // FindFeatures
    pluginNames = gz::physics::FindFeatures3d<FeaturesGravity>::From(loader);
    if (pluginNames.empty())
    {
      std::cerr << "No plugins with required features found in " << GetLibToTest();
      // TODO(ahcorde): If we update gtest we can use here GTEST_SKIP()
    }
  }
};

//////////////////////////////////////////////////
TEST_F(WorldFeaturesGravityTest, Solver)
{
  for (const std::string &name : pluginNames)
  {
    std::cout << "Testing plugin: " << name << std::endl;
    gz::plugin::PluginPtr plugin = loader.Instantiate(name);

    auto engine = gz::physics::RequestEngine3d<FeaturesGravity>::From(plugin);
    ASSERT_NE(nullptr, engine);
    EXPECT_TRUE(engine->GetName().find(PhysicsEngineName(name)) !=
                std::string::npos);

    sdf::Root root;
    const sdf::Errors errors = root.Load(TEST_WORLD_DIR "/falling.world");
    EXPECT_TRUE(errors.empty());
    const sdf::World *sdfWorld = root.WorldByIndex(0);

    auto world = engine->ConstructWorld(*sdfWorld);
    ASSERT_NE(nullptr, world);

    // Check default gravity value
    AssertVectorApprox vectorPredicate6(1e-6);
    EXPECT_PRED_FORMAT2(vectorPredicate6, Eigen::Vector3d(0, 0, -9.8),
                        world->GetGravity());

    auto model = world->GetModel("sphere");
    ASSERT_NE(nullptr, model);

    auto link = model->GetLink(0);
    ASSERT_NE(nullptr, link);

    // initial link pose
    const Eigen::Vector3d initialLinkPosition(0, 0, 2);
    {
      Eigen::Vector3d pos = link->FrameDataRelativeToWorld().pose.translation();
      EXPECT_PRED_FORMAT2(vectorPredicate6,
                          initialLinkPosition,
                          pos);
    }

    auto linkFrameID = link->GetFrameID();

    std::cerr << "linkFrameID " << linkFrameID.ID() << '\n';

    // Get default gravity in link frame, which is pitched by pi/4
    EXPECT_PRED_FORMAT2(vectorPredicate6,
                        Eigen::Vector3d(6.92964645563, 0, -6.92964645563),
                        world->GetGravity(linkFrameID));

    // set gravity along X axis of linked frame, which is pitched by pi/4
    world->SetGravity(Eigen::Vector3d(1.4142135624, 0, 0), linkFrameID);

    EXPECT_PRED_FORMAT2(vectorPredicate6,
                        Eigen::Vector3d(1, 0, -1),
                        world->GetGravity());

    // test other SetGravity API
    // set gravity along Z axis of linked frame, which is pitched by pi/4
    gz::physics::RelativeForce3d relativeGravity(
        linkFrameID, Eigen::Vector3d(0, 0, 1.4142135624));
    world->SetGravity(relativeGravity);

    EXPECT_PRED_FORMAT2(vectorPredicate6,
                        Eigen::Vector3d(1, 0, 1),
                        world->GetGravity());

    // Confirm that changed gravity direction affects pose of link
    gz::physics::ForwardStep::Input input;
    gz::physics::ForwardStep::State state;
    gz::physics::ForwardStep::Output output;

    const size_t numSteps = 1000;
    for (size_t i = 0; i < numSteps; ++i)
    {
      world->Step(output, state, input);
    }

    AssertVectorApprox vectorPredicate3(1e-3);
    {
      Eigen::Vector3d pos = link->FrameDataRelativeToWorld().pose.translation();
      EXPECT_PRED_FORMAT2(vectorPredicate3,
                          Eigen::Vector3d(0.5, 0, 2.5),
                          pos);
    }
  }
}

int main(int argc, char *argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  WorldFeaturesTest::init(argc, argv);
  WorldFeaturesGravityTest::init(argc, argv);
  return RUN_ALL_TESTS();
}
