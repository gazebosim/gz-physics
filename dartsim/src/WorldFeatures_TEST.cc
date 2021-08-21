/*
 * Copyright (C) 2021 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 3.0 (the "License");
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

#include <ignition/common/Console.hh>
#include <ignition/physics/FindFeatures.hh>
#include <ignition/plugin/Loader.hh>
#include <ignition/physics/RequestEngine.hh>

#include <ignition/physics/ForwardStep.hh>
#include <ignition/physics/FrameSemantics.hh>
#include <ignition/physics/GetBoundingBox.hh>
#include <ignition/physics/World.hh>
#include <ignition/physics/sdf/ConstructWorld.hh>

#include <sdf/Root.hh>
#include <sdf/World.hh>

#include "test/Utils.hh"

struct TestFeatureList : ignition::physics::FeatureList<
    ignition::physics::CollisionDetector,
    ignition::physics::Gravity,
    ignition::physics::LinkFrameSemantics,
    ignition::physics::Solver,
    ignition::physics::ForwardStep,
    ignition::physics::sdf::ConstructSdfWorld,
    ignition::physics::GetEntities
> { };

using namespace ignition;

using TestEnginePtr = physics::Engine3dPtr<TestFeatureList>;
using TestWorldPtr = physics::World3dPtr<TestFeatureList>;

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
    if (ignition::physics::test::Equal(_m, _n, this->tol))
      return ::testing::AssertionSuccess();

    return ::testing::AssertionFailure()
           << _mExpr << " and " << _nExpr << " ([" << _m.transpose()
           << "] and [" << _n.transpose() << "]"
           << ") are not equal";
  }

  private: double tol;
};

//////////////////////////////////////////////////
class WorldFeaturesFixture : public ::testing::Test
{
  protected: void SetUp() override
  {
    ignition::plugin::Loader loader;
    loader.LoadLib(dartsim_plugin_LIB);

    ignition::plugin::PluginPtr dartsim =
        loader.Instantiate("ignition::physics::dartsim::Plugin");

    this->engine =
        ignition::physics::RequestEngine3d<TestFeatureList>::From(dartsim);
    ASSERT_NE(nullptr, this->engine);
  }
  protected: TestEnginePtr engine;
};

//////////////////////////////////////////////////
TestWorldPtr LoadWorld(
    const TestEnginePtr &_engine,
    const std::string &_sdfFile)
{
  sdf::Root root;
  const sdf::Errors errors = root.Load(_sdfFile);
  EXPECT_TRUE(errors.empty());
  const sdf::World *sdfWorld = root.WorldByIndex(0);
  return _engine->ConstructWorld(*sdfWorld);
}

//////////////////////////////////////////////////
TEST_F(WorldFeaturesFixture, CollisionDetector)
{
  auto world = LoadWorld(this->engine, TEST_WORLD_DIR "/empty.sdf");
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

//////////////////////////////////////////////////
TEST_F(WorldFeaturesFixture, Gravity)
{
  auto world = LoadWorld(this->engine, TEST_WORLD_DIR "/falling.world");
  ASSERT_NE(nullptr, world);

  // Check default gravity value
  AssertVectorApprox vectorPredicate10(1e-10);
  EXPECT_PRED_FORMAT2(vectorPredicate10, Eigen::Vector3d(0, 0, -9.8),
                      world->GetGravity());

  auto model = world->GetModel("sphere");
  ASSERT_NE(nullptr, model);

  auto link = model->GetLink(0);
  ASSERT_NE(nullptr, link);

  // initial link pose
  const Eigen::Vector3d initialLinkPosition(0, 0, 2);
  {
    auto pos = link->FrameDataRelativeToWorld().pose.translation();
    EXPECT_PRED_FORMAT2(vectorPredicate10,
                        initialLinkPosition,
                        pos);
  }

  auto linkFrameID = link->GetFrameID();

  // Get default gravity in link frame, which is pitched by pi/4
  EXPECT_PRED_FORMAT2(vectorPredicate10,
                      Eigen::Vector3d(6.92964645563, 0, -6.92964645563),
                      world->GetGravity(linkFrameID));

  // set gravity along X axis of linked frame, which is pitched by pi/4
  world->SetGravity(Eigen::Vector3d(1.4142135624, 0, 0), linkFrameID);

  EXPECT_PRED_FORMAT2(vectorPredicate10,
                      Eigen::Vector3d(1, 0, -1),
                      world->GetGravity());

  // test other SetGravity API
  // set gravity along Z axis of linked frame, which is pitched by pi/4
  physics::RelativeForce3d relativeGravity(
      linkFrameID, Eigen::Vector3d(0, 0, 1.4142135624));
  world->SetGravity(relativeGravity);

  EXPECT_PRED_FORMAT2(vectorPredicate10,
                      Eigen::Vector3d(1, 0, 1),
                      world->GetGravity());

  // Confirm that changed gravity direction affects pose of link
  ignition::physics::ForwardStep::Input input;
  ignition::physics::ForwardStep::State state;
  ignition::physics::ForwardStep::Output output;

  const size_t numSteps = 1000;
  for (size_t i = 0; i < numSteps; ++i)
  {
    world->Step(output, state, input);
  }

  AssertVectorApprox vectorPredicate3(1e-3);
  {
    auto pos = link->FrameDataRelativeToWorld().pose.translation();
    EXPECT_PRED_FORMAT2(vectorPredicate3,
                        Eigen::Vector3d(0.5, 0, 2.5),
                        pos);
  }
}

//////////////////////////////////////////////////
TEST_F(WorldFeaturesFixture, Solver)
{
  auto world = LoadWorld(this->engine, TEST_WORLD_DIR "/empty.sdf");
  EXPECT_EQ("DantzigBoxedLcpSolver", world->GetSolver());

  world->SetSolver("banana");
  EXPECT_EQ("DantzigBoxedLcpSolver", world->GetSolver());

  world->SetSolver("dantzig");
  EXPECT_EQ("DantzigBoxedLcpSolver", world->GetSolver());

  world->SetSolver("pgs");
  EXPECT_EQ("PgsBoxedLcpSolver", world->GetSolver());
}

/////////////////////////////////////////////////
int main(int argc, char *argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
