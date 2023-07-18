/*
 * Copyright (C) 2023 Open Source Robotics Foundation
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

#include <chrono>

#include <gz/common/Console.hh>
#include <gz/plugin/Loader.hh>

#include <gz/math/Angle.hh>
#include <gz/math/Helpers.hh>
#include <gz/math/eigen3/Conversions.hh>

#include "TestLibLoader.hh"
#include "../Utils.hh"

#include "gz/physics/FrameSemantics.hh"
#include <gz/physics/FindFeatures.hh>
#include <gz/physics/FixedJoint.hh>
#include <gz/physics/ForwardStep.hh>
#include <gz/physics/FreeGroup.hh>
#include <gz/physics/FreeJoint.hh>
#include <gz/physics/GetEntities.hh>
#include <gz/physics/Joint.hh>
#include <gz/physics/RemoveEntities.hh>
#include <gz/physics/RequestEngine.hh>
#include <gz/physics/RevoluteJoint.hh>
#include <gz/physics/Shape.hh>
#include <gz/physics/sdf/ConstructModel.hh>
#include <gz/physics/sdf/ConstructWorld.hh>

#include <sdf/Root.hh>

template <class T>
class JointMimicFeaturesTest:
  public testing::Test, public gz::physics::TestLibLoader
{
  // Documentation inherited
  public: void SetUp() override
  {
    std::cerr << "SetUp begin" << std::endl;
    gz::common::Console::SetVerbosity(4);

    std::cerr << "JointMimicFeaturesTest::GetLibToTest() "
      << JointMimicFeaturesTest::GetLibToTest() << '\n';

    loader.LoadLib(JointMimicFeaturesTest::GetLibToTest());

    // TODO(ahcorde): We should also run the 3f, 2d, and 2f variants of
    // FindFeatures
    pluginNames = gz::physics::FindFeatures3d<T>::From(loader);
    if (pluginNames.empty())
    {
      std::cerr << "No plugins with required features found in "
                << GetLibToTest() << std::endl;
      GTEST_SKIP();
    }
    for (const std::string &name : this->pluginNames)
    {
      if(this->PhysicsEngineName(name) == "tpe")
      {
        GTEST_SKIP();
      }
    }
    std::cerr << "SetUp end" << std::endl;
  }

  public: std::set<std::string> pluginNames;
  public: gz::plugin::Loader loader;
};

struct JointMimicFeatureList : gz::physics::FeatureList<
    gz::physics::ForwardStep,
    gz::physics::GetBasicJointProperties,
    gz::physics::GetBasicJointState,
    gz::physics::GetEngineInfo,
    gz::physics::GetJointFromModel,
    gz::physics::GetLinkFromModel,
    gz::physics::GetModelFromWorld,
    gz::physics::LinkFrameSemantics,
    gz::physics::SetBasicJointState,
    gz::physics::SetJointVelocityCommandFeature,
    gz::physics::SetMimicConstraintFeature,
    gz::physics::sdf::ConstructSdfWorld>{};

using JointMimicFeatureTest =
    JointMimicFeaturesTest<JointMimicFeatureList>;

// Here, we test mimic constraints on various combinations of
// prismatic and revolute joints using a chain of constraints.
TEST_F(JointMimicFeatureTest, PrismaticRevoluteMimicTest)
{
  std::cerr << "PrismaticRevoluteMimicTest begin" << std::endl;
  // This test contains 5 joints : 3 prismatic and 2 revolute.
  // They are connected as follows :
  // prismatic_joint_1 : Free joint
  // prismatic_joint_2 : Mimics prismatic_joint_1
  // revolute_joint_1 : Mimics prismatic_joint_1
  // revolute_joint_2 : Mimics revolute_joint_1
  // prismatic_joint_3 : Mimics revolute_joint_1
  for (const std::string &name : this->pluginNames)
  {
    std::cerr << "PrismaticRevoluteMimicTest loop "
              << name
              << std::endl;
    if(this->PhysicsEngineName(name) != "bullet-featherstone")
    {
      GTEST_SKIP();
    }

    std::cout << "Testing plugin: " << name << std::endl;
    gz::plugin::PluginPtr plugin = this->loader.Instantiate(name);

    std::cerr << "PrismaticRevoluteMimicTest RequestEngine" << std::endl;
    auto engine =
        gz::physics::RequestEngine3d<JointMimicFeatureList>::From(plugin);
    ASSERT_NE(nullptr, engine);

    std::cerr << "PrismaticRevoluteMimicTest load SDF" << std::endl;
    sdf::Root root;
    const sdf::Errors errors = root.Load(gz::common::joinPaths(TEST_WORLD_DIR,
          "mimic_prismatic_world.sdf"));
    ASSERT_TRUE(errors.empty()) << errors.front();

    std::cerr << "PrismaticRevoluteMimicTest ConstructWorld" << std::endl;
    auto world = engine->ConstructWorld(*root.WorldByIndex(0));

    std::cerr << "PrismaticRevoluteMimicTest GetModel" << std::endl;
    auto model = world->GetModel("prismatic_model");

    std::cerr << "PrismaticRevoluteMimicTest GetJoints" << std::endl;
    auto leaderJoint = model->GetJoint("prismatic_joint_1");
    auto prismaticFollowerJoint1 = model->GetJoint("prismatic_joint_2");
    auto revoluteFollowerJoint1 = model->GetJoint("revolute_joint_1");
    auto revoluteFollowerJoint2 = model->GetJoint("revolute_joint_2");
    auto prismaticFollowerJoint2 = model->GetJoint("prismatic_joint_3");

    // Ensure both joints start from zero angle.
    EXPECT_EQ(leaderJoint->GetPosition(0), 0);
    EXPECT_EQ(prismaticFollowerJoint1->GetPosition(0), 0);
    EXPECT_EQ(revoluteFollowerJoint1->GetPosition(0), 0);
    EXPECT_EQ(revoluteFollowerJoint2->GetPosition(0), 0);
    EXPECT_EQ(prismaticFollowerJoint2->GetPosition(0), 0);

    gz::physics::ForwardStep::Output output;
    gz::physics::ForwardStep::State state;
    gz::physics::ForwardStep::Input input;

    // Case : Without mimic constraint

    // Let the simulation run without mimic constraint.
    // The positions of joints should not be equal.
    for (int i = 0; i < 10; i++)
    {
      std::cerr << "  init Step loop " << i << std::endl;
      world->Step(output, state, input);
      if (i > 5)
      {
        EXPECT_NE(leaderJoint->GetPosition(0),
                  prismaticFollowerJoint1->GetPosition(0));
        EXPECT_NE(leaderJoint->GetPosition(0),
                  revoluteFollowerJoint1->GetPosition(0));
        // This expectation fails, are the revolute joints are too similar?
        // EXPECT_NE(revoluteFollowerJoint1->GetPosition(0),
        //           revoluteFollowerJoint2->GetPosition(0));
        EXPECT_NE(revoluteFollowerJoint1->GetPosition(0),
                  prismaticFollowerJoint2->GetPosition(0));
      }
    }

    auto testMimicFcn = [&](double multiplier, double offset, double reference)
      {
        // Set mimic joint constraints.
        // Leader --> Follower
        // prismatic_joint_1 -> prismatic_joint_2
        // prismatic_joint_1 -> revolute_joint_1
        // revolute_joint_1 --> revolute_joint_2
        // revolute_joint_1 --> prismatic_joint_3
        prismaticFollowerJoint1->SetMimicConstraint(0,
            "prismatic_joint_1", "axis", multiplier, offset, reference);
        revoluteFollowerJoint1->SetMimicConstraint(0,
            "prismatic_joint_1", "axis" , multiplier, offset, reference);
        revoluteFollowerJoint2->SetMimicConstraint(0,
            "revolute_joint_1", "axis", multiplier, offset, reference);
        prismaticFollowerJoint2->SetMimicConstraint(0,
            "revolute_joint_1", "axis", multiplier, offset, reference);

        // Reset positions and run a few iterations so the positions reach
        // nontrivial values.
        leaderJoint->SetPosition(0, 0);
        prismaticFollowerJoint1->SetPosition(0, 0);
        prismaticFollowerJoint2->SetPosition(0, 0);
        revoluteFollowerJoint1->SetPosition(0, 0);
        revoluteFollowerJoint2->SetPosition(0, 0);
        for (int _ = 0; _ < 200; _++)
          world->Step(output, state, input);

        for (int _ = 0; _ < 10; _++)
        {
          world->Step(output, state, input);
          // Check for prismatic -> prismatic mimicking.
          const double positionTolerance = 0.1;
          EXPECT_NEAR(
              multiplier * (leaderJoint->GetPosition(0) - reference) + offset,
              prismaticFollowerJoint1->GetPosition(0),
              positionTolerance)
            << "multiplier [" << multiplier
            << "], reference [" << reference
            << "], offset [" << offset
            << "], leaderPos [" << leaderJoint->GetPosition(0)
            << "], followerPos [" << prismaticFollowerJoint1->GetPosition(0)
            << "]";
          // Check for prismatic -> revolute mimicking.
          EXPECT_NEAR(
              multiplier * (leaderJoint->GetPosition(0) - reference) + offset,
              revoluteFollowerJoint1->GetPosition(0),
              positionTolerance)
            << "multiplier [" << multiplier
            << "], reference [" << reference
            << "], offset [" << offset
            << "], leaderPos [" << leaderJoint->GetPosition(0)
            << "], followerPos [" << revoluteFollowerJoint1->GetPosition(0)
            << "]";
          // Check for revolute -> revolute mimicking.
          EXPECT_NEAR(
              multiplier * (revoluteFollowerJoint1->GetPosition(0) - reference)
                + offset,
              revoluteFollowerJoint2->GetPosition(0),
              positionTolerance)
            << "multiplier [" << multiplier
            << "], reference [" << reference
            << "], offset [" << offset
            << "], leaderPos [" << revoluteFollowerJoint1->GetPosition(0)
            << "], followerPos [" << revoluteFollowerJoint2->GetPosition(0)
            << "]";
          // Check for revolute --> prismatic mimicking.
          EXPECT_NEAR(
              multiplier * (revoluteFollowerJoint1->GetPosition(0) - reference)
                + offset,
              prismaticFollowerJoint2->GetPosition(0),
              positionTolerance)
            << "multiplier [" << multiplier
            << "], reference [" << reference
            << "], offset [" << offset
            << "], leaderPos [" << revoluteFollowerJoint1->GetPosition(0)
            << "], followerPos [" << prismaticFollowerJoint2->GetPosition(0)
            << "]";
        }
      };

    // Testing with different (multiplier, offset, reference) combinations.
    std::cerr << " start calling testMimicFcn" << std::endl;
    testMimicFcn(1, 0, 0);
    testMimicFcn(-1, 0, 0);
    testMimicFcn(1, 0.1, 0);
    testMimicFcn(1, 0.05, 0.05);
    testMimicFcn(-1, 0.2, 0);
    testMimicFcn(-1, 0.05, -0.05);
    testMimicFcn(-2, 0, 0);
    testMimicFcn(2, 0.1, 0);
    std::cerr << "finish calling testMimicFcn" << std::endl;
  }
}

// In this test, we have 2 pendulums of different lengths.
// Originally, their time periods are different, but after applying
// the mimic constraint, they follow the same velocity.
TEST_F(JointMimicFeatureTest, PendulumMimicTest)
{
  for (const std::string &name : this->pluginNames)
  {
    if(this->PhysicsEngineName(name) != "bullet-featherstone")
    {
      GTEST_SKIP();
    }

    std::cout << "Testing plugin: " << name << std::endl;
    gz::plugin::PluginPtr plugin = this->loader.Instantiate(name);

    auto engine =
        gz::physics::RequestEngine3d<JointMimicFeatureList>::From(plugin);
    ASSERT_NE(nullptr, engine);

    sdf::Root root;
    const sdf::Errors errors = root.Load(gz::common::joinPaths(TEST_WORLD_DIR,
          "mimic_pendulum_world.sdf"));
    ASSERT_TRUE(errors.empty()) << errors.front();

    auto world = engine->ConstructWorld(*root.WorldByIndex(0));

    // Test mimic constraint between two revolute joints.
    auto model = world->GetModel("pendulum_with_base");
    auto leaderJoint = model->GetJoint("upper_joint_1");
    auto followerJoint = model->GetJoint("upper_joint_2");

    // Ensure both joints start from zero angle.
    EXPECT_EQ(leaderJoint->GetPosition(0), 0);
    EXPECT_EQ(followerJoint->GetPosition(0), 0);

    gz::physics::ForwardStep::Output output;
    gz::physics::ForwardStep::State state;
    gz::physics::ForwardStep::Input input;

    // Case : Without mimic constraint

    // Let the simulation run without mimic constraint.
    // The positions of joints should not be equal.
    for (int _ = 0; _ < 10; _++)
    {
      world->Step(output, state, input);
      EXPECT_NE(leaderJoint->GetPosition(0), followerJoint->GetPosition(0));
    }

    auto testMimicFcn = [&](double multiplier, double offset, double reference)
      {
        // Set mimic joint constraint.
        followerJoint->SetMimicConstraint(0,
            "upper_joint_1", "axis" , multiplier, offset, reference);
        // Reset positions and run a few iterations so the positions reach
        // nontrivial values.
        leaderJoint->SetPosition(0, 0);
        followerJoint->SetPosition(0, 0);
        for (int _ = 0; _ < 75; _++)
          world->Step(output, state, input);

        const double positionTolerance = 5e-3;
        for (int _ = 0; _ < 10; _++)
        {
          world->Step(output, state, input);
          EXPECT_NEAR(
              multiplier * (leaderJoint->GetPosition(0) - reference) + offset,
              followerJoint->GetPosition(0),
              positionTolerance);
        }
      };

    // Testing with different (multiplier, offset, reference) combinations.
    testMimicFcn(1, 0, 0);
    testMimicFcn(-1, 0, 0);
    testMimicFcn(1, 0.1, 0);
    testMimicFcn(1, 0.2, 0.1);
    testMimicFcn(-1, 0.2, 0);
    testMimicFcn(-2, 0, 0);
    testMimicFcn(2, 0.1, 0);
    testMimicFcn(2, 0.3, -0.1);
  }
}

// In this test, we have a pair of pendulums with different lengths and
// oscillation frequencies that is repeated 3 times. One pendulum pair is
// uncoupled, and they oscillate at different rates, but the other two pairs
// have a mimic constraint so that all the mimicked pendulums oscillate at a
// frequency in between that of the slow and fast pendulums.
// In one mimic pair the fast pendulum follows the slow pendulum,
// and vice versa for the other pair.
TEST_F(JointMimicFeatureTest, PendulumsFastSlowMimicTest)
{
  for (const std::string &name : this->pluginNames)
  {
    if(this->PhysicsEngineName(name) != "bullet-featherstone")
    {
      GTEST_SKIP();
    }

    std::cout << "Testing plugin: " << name << std::endl;
    gz::plugin::PluginPtr plugin = this->loader.Instantiate(name);

    auto engine =
        gz::physics::RequestEngine3d<JointMimicFeatureList>::From(plugin);
    ASSERT_NE(nullptr, engine);

    sdf::Root root;
    const sdf::Errors errors = root.Load(gz::common::joinPaths(TEST_WORLD_DIR,
          "mimic_fast_slow_pendulums_world.sdf"));
    ASSERT_TRUE(errors.empty()) << errors.front();

    auto world = engine->ConstructWorld(*root.WorldByIndex(0));

    // Set mimic parameters so that pendulums oscillate at the same rate
    const double multiplier = 1.0;
    const double offset = 0.0;
    const double reference = 0.0;

    // Get uncoupled pendulum pair
    auto model = world->GetModel("pendulum_with_base");
    auto slowJoint = model->GetJoint("slow_joint");
    auto fastJoint = model->GetJoint("fast_joint");

    // Test mimic constraint with fast joint following slow joint
    const std::string followJointNameFastFollowsSlow = "fast_joint";
    const std::string leaderJointNameFastFollowsSlow = "slow_joint";
    auto modelFastFollowsSlow =
        world->GetModel("pendulum_with_base_mimic_fast_follows_slow");
    auto followJointFastFollowsSlow =
        modelFastFollowsSlow->GetJoint(followJointNameFastFollowsSlow);
    auto leaderJointFastFollowsSlow =
        modelFastFollowsSlow->GetJoint(leaderJointNameFastFollowsSlow);
    followJointFastFollowsSlow->SetMimicConstraint(
        0, leaderJointNameFastFollowsSlow, "axis",
        multiplier, offset, reference);

    // Test mimic constraint with slow joint following fast joint
    const std::string followJointNameSlowFollowsFast = "slow_joint";
    const std::string leaderJointNameSlowFollowsFast = "fast_joint";
    auto modelSlowFollowsFast =
        world->GetModel("pendulum_with_base_mimic_slow_follows_fast");
    auto followJointSlowFollowsFast =
        modelSlowFollowsFast->GetJoint(followJointNameSlowFollowsFast);
    auto leaderJointSlowFollowsFast =
        modelSlowFollowsFast->GetJoint(leaderJointNameSlowFollowsFast);
    followJointSlowFollowsFast->SetMimicConstraint(
        0, leaderJointNameSlowFollowsFast, "axis",
        multiplier, offset, reference);

    // Ensure all joints start from zero angle.
    EXPECT_EQ(fastJoint->GetPosition(0), 0);
    EXPECT_EQ(slowJoint->GetPosition(0), 0);
    EXPECT_EQ(followJointFastFollowsSlow->GetPosition(0), 0);
    EXPECT_EQ(leaderJointFastFollowsSlow->GetPosition(0), 0);
    EXPECT_EQ(followJointSlowFollowsFast->GetPosition(0), 0);
    EXPECT_EQ(leaderJointSlowFollowsFast->GetPosition(0), 0);

    gz::physics::ForwardStep::Output output;
    gz::physics::ForwardStep::State state;
    gz::physics::ForwardStep::Input input;

    // Take two steps to allow the pendulums to start moving
    world->Step(output, state, input);
    world->Step(output, state, input);

    // Step forward less than one half pendulum oscillation and check
    // joint positions and velocities
    for (int _ = 0; _ < 300; _++)
    {
      world->Step(output, state, input);

      // Expect all joint positions are within 0, 90 degrees
      const double deg45 = GZ_DTOR(45);
      EXPECT_NEAR(slowJoint->GetPosition(0), deg45, deg45);
      EXPECT_NEAR(fastJoint->GetPosition(0), deg45, deg45);
      EXPECT_NEAR(followJointFastFollowsSlow->GetPosition(0), deg45, deg45);
      EXPECT_NEAR(leaderJointFastFollowsSlow->GetPosition(0), deg45, deg45);
      EXPECT_NEAR(followJointSlowFollowsFast->GetPosition(0), deg45, deg45);
      EXPECT_NEAR(leaderJointSlowFollowsFast->GetPosition(0), deg45, deg45);

      // Expect all joint velocities are positive
      EXPECT_GT(slowJoint->GetVelocity(0), 0);
      EXPECT_GT(fastJoint->GetVelocity(0), 0);
      EXPECT_GT(followJointFastFollowsSlow->GetVelocity(0), 0);
      EXPECT_GT(leaderJointFastFollowsSlow->GetVelocity(0), 0);
      EXPECT_GT(followJointSlowFollowsFast->GetVelocity(0), 0);
      EXPECT_GT(leaderJointSlowFollowsFast->GetVelocity(0), 0);

      // Expect fast joint position/velocity are greater than other joints
      // position
      EXPECT_GT(fastJoint->GetPosition(0), slowJoint->GetPosition(0));
      EXPECT_GT(fastJoint->GetPosition(0),
                followJointFastFollowsSlow->GetPosition(0));
      EXPECT_GT(fastJoint->GetPosition(0),
                leaderJointFastFollowsSlow->GetPosition(0));
      EXPECT_GT(fastJoint->GetPosition(0),
                followJointSlowFollowsFast->GetPosition(0));
      EXPECT_GT(fastJoint->GetPosition(0),
                leaderJointSlowFollowsFast->GetPosition(0));
      // velocity
      EXPECT_GT(fastJoint->GetVelocity(0), slowJoint->GetVelocity(0));
      EXPECT_GT(fastJoint->GetVelocity(0),
                followJointFastFollowsSlow->GetVelocity(0));
      EXPECT_GT(fastJoint->GetVelocity(0),
                leaderJointFastFollowsSlow->GetVelocity(0));
      EXPECT_GT(fastJoint->GetVelocity(0),
                followJointSlowFollowsFast->GetVelocity(0));
      EXPECT_GT(fastJoint->GetVelocity(0),
                leaderJointSlowFollowsFast->GetVelocity(0));

      // Expect slow joint position/velocity are smaller than mimicked joints
      // position
      EXPECT_LT(slowJoint->GetPosition(0),
                followJointFastFollowsSlow->GetPosition(0));
      EXPECT_LT(slowJoint->GetPosition(0),
                leaderJointFastFollowsSlow->GetPosition(0));
      EXPECT_LT(slowJoint->GetPosition(0),
                followJointSlowFollowsFast->GetPosition(0));
      EXPECT_LT(slowJoint->GetPosition(0),
                leaderJointSlowFollowsFast->GetPosition(0));
      // velocity
      EXPECT_LT(slowJoint->GetVelocity(0),
                followJointFastFollowsSlow->GetVelocity(0));
      EXPECT_LT(slowJoint->GetVelocity(0),
                leaderJointFastFollowsSlow->GetVelocity(0));
      EXPECT_LT(slowJoint->GetVelocity(0),
                followJointSlowFollowsFast->GetVelocity(0));
      EXPECT_LT(slowJoint->GetVelocity(0),
                leaderJointSlowFollowsFast->GetVelocity(0));

      // Expect mimicked joint position/velocity are approximately equal
      const double tolerance = 5e-3;
      // position
      EXPECT_NEAR(followJointFastFollowsSlow->GetVelocity(0),
                  leaderJointFastFollowsSlow->GetVelocity(0), tolerance);
      EXPECT_NEAR(followJointSlowFollowsFast->GetVelocity(0),
                  leaderJointSlowFollowsFast->GetVelocity(0), tolerance);
      EXPECT_NEAR(followJointFastFollowsSlow->GetVelocity(0),
                  followJointSlowFollowsFast->GetVelocity(0), tolerance);
      // velocity
      EXPECT_NEAR(followJointFastFollowsSlow->GetVelocity(0),
                  leaderJointFastFollowsSlow->GetVelocity(0), tolerance);
      EXPECT_NEAR(followJointSlowFollowsFast->GetVelocity(0),
                  leaderJointSlowFollowsFast->GetVelocity(0), tolerance);
      EXPECT_NEAR(followJointFastFollowsSlow->GetVelocity(0),
                  followJointSlowFollowsFast->GetVelocity(0), tolerance);
    }
  }
}

int main(int argc, char *argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  if (!JointMimicFeaturesTest<
      JointMimicFeatureList>::init(argc, argv))
    return -1;
  return RUN_ALL_TESTS();
}
