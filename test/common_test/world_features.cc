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
#include <gz/plugin/Loader.hh>

#include "../helpers/TestLibLoader.hh"
#include "../Utils.hh"

#include <gz/physics/FindFeatures.hh>
#include <gz/physics/ForwardStep.hh>
#include <gz/physics/FrameSemantics.hh>
#include <gz/physics/GetEntities.hh>
#include <gz/physics/RequestEngine.hh>

#include <gz/physics/World.hh>
#include <gz/physics/sdf/ConstructWorld.hh>

#include <sdf/Root.hh>


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


template <class T>
class WorldFeaturesTest:
  public testing::Test, public gz::physics::TestLibLoader
{
  // Documentation inherited
  public: void SetUp() override
  {
    gz::common::Console::SetVerbosity(4);

    loader.LoadLib(WorldFeaturesTest::GetLibToTest());

    // TODO(ahcorde): We should also run the 3f, 2d, and 2f variants of
    // FindFeatures
    pluginNames = gz::physics::FindFeatures3d<T>::From(loader);
    if (pluginNames.empty())
    {
      std::cerr << "No plugins with required features found in "
                << GetLibToTest() << std::endl;
      GTEST_SKIP();
    }
  }

  public: std::set<std::string> pluginNames;
  public: gz::plugin::Loader loader;
};

using GravityFeatures = gz::physics::FeatureList<
  gz::physics::GetEngineInfo,
  gz::physics::Gravity,
  gz::physics::sdf::ConstructSdfWorld,
  gz::physics::LinkFrameSemantics,
  gz::physics::GetModelFromWorld,
  gz::physics::GetLinkFromModel,
  gz::physics::ForwardStep
>;

using GravityFeaturesTestTypes =
  ::testing::Types<GravityFeatures>;
TYPED_TEST_SUITE(WorldFeaturesTest,
                 GravityFeatures);

/////////////////////////////////////////////////
TYPED_TEST(WorldFeaturesTest, GravityFeatures)
{
  for (const std::string &name : this->pluginNames)
  {
    std::cout << "Testing plugin: " << name << std::endl;
    gz::plugin::PluginPtr plugin = this->loader.Instantiate(name);

    auto engine = gz::physics::RequestEngine3d<GravityFeatures>::From(plugin);
    ASSERT_NE(nullptr, engine);
    EXPECT_TRUE(engine->GetName().find(this->PhysicsEngineName(name)) !=
                std::string::npos);

    sdf::Root root;
    const sdf::Errors errors = root.Load(
      gz::common::joinPaths(TEST_WORLD_DIR, "falling.world"));
    EXPECT_TRUE(errors.empty()) << errors;
    const sdf::World *sdfWorld = root.WorldByIndex(0);
    EXPECT_NE(nullptr, sdfWorld);

    auto world = engine->ConstructWorld(*root.WorldByIndex(0));
    EXPECT_NE(nullptr, world);

    auto graphErrors = sdfWorld->ValidateGraphs();
    EXPECT_EQ(0u, graphErrors.size()) << graphErrors;

    Eigen::Vector3d gravity = {0, 0, -9.8};

    AssertVectorApprox vectorPredicate(1e-6);
    EXPECT_PRED_FORMAT2(vectorPredicate, gravity,
                        world->GetGravity());

    world->SetGravity({8, 4, 3});
    EXPECT_PRED_FORMAT2(vectorPredicate, Eigen::Vector3d(8, 4, 3),
                        world->GetGravity());

    world->SetGravity(gravity);

    auto model = world->GetModel("sphere");
    ASSERT_NE(nullptr, model);

    auto link = model->GetLink(0);
    ASSERT_NE(nullptr, link);

    AssertVectorApprox vectorPredicate10(1e-10);

    // initial link pose
    const Eigen::Vector3d initialLinkPosition(0, 0, 2);
    {
      Eigen::Vector3d pos = link->FrameDataRelativeToWorld().pose.translation();
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
    gz::physics::RelativeForce3d relativeGravity(
        linkFrameID, Eigen::Vector3d(0, 0, 1.4142135624));
    world->SetGravity(relativeGravity);

    EXPECT_PRED_FORMAT2(vectorPredicate10,
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
  if(!WorldFeaturesTest<GravityFeatures>::init(argc, argv))
    return -1;
  return RUN_ALL_TESTS();
}
