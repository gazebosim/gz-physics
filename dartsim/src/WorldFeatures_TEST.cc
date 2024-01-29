/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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
#include <gz/physics/FindFeatures.hh>
#include <gz/plugin/Loader.hh>
#include <gz/physics/RequestEngine.hh>

#include <gz/physics/ForwardStep.hh>
#include <gz/physics/FrameSemantics.hh>
#include <gz/physics/GetBoundingBox.hh>
#include <gz/physics/World.hh>
#include <gz/physics/sdf/ConstructWorld.hh>

#include <sdf/Root.hh>
#include <sdf/World.hh>

#include "test/Utils.hh"
#include "test/common_test/Worlds.hh"

struct TestFeatureList : gz::physics::FeatureList<
    gz::physics::CollisionDetector,
    gz::physics::Gravity,
    gz::physics::LinkFrameSemantics,
    gz::physics::Solver,
    gz::physics::ForwardStep,
    gz::physics::sdf::ConstructSdfWorld,
    gz::physics::GetEntities
> { };

using namespace gz;

using TestEnginePtr = physics::Engine3dPtr<TestFeatureList>;
using TestWorldPtr = physics::World3dPtr<TestFeatureList>;
using AssertVectorApprox = physics::test::AssertVectorApprox;

//////////////////////////////////////////////////
class WorldFeaturesFixture : public ::testing::Test
{
  protected: void SetUp() override
  {
    gz::plugin::Loader loader;
    loader.LoadLib(dartsim_plugin_LIB);

    gz::plugin::PluginPtr dartsim =
        loader.Instantiate("gz::physics::dartsim::Plugin");

    this->engine =
        gz::physics::RequestEngine3d<TestFeatureList>::From(dartsim);
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
  const auto world = LoadWorld(this->engine, common_test::worlds::kEmptySdf);
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
TEST_F(WorldFeaturesFixture, Solver)
{
  const auto world = LoadWorld(this->engine, common_test::worlds::kEmptySdf);

  EXPECT_EQ("DantzigBoxedLcpSolver", world->GetSolver());

  world->SetSolver("banana");
  EXPECT_EQ("DantzigBoxedLcpSolver", world->GetSolver());

  world->SetSolver("dantzig");
  EXPECT_EQ("DantzigBoxedLcpSolver", world->GetSolver());

  world->SetSolver("pgs");
  EXPECT_EQ("PgsBoxedLcpSolver", world->GetSolver());
}
