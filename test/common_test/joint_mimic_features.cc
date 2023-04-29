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
  // This test contains 5 joints : 3 prismatic and 2 revolute.
  // They are connected as follows :
  // prismatic_joint_1 : Free joint
  // prismatic_joint_2 : Mimics prismatic_joint_1
  // revolute_joint_1 : Mimics prismatic_joint_1
  // revolute_joint_2 : Mimics revolute_joint_1
  // prismatic_joint_3 : Mimics revolute_joint_1
  for (const std::string &name : this->pluginNames)
  {
    if(this->PhysicsEngineName(name) != "dartsim")
    {
      GTEST_SKIP();
    }

    std::cout << "Testing plugin: " << name << std::endl;
    gz::plugin::PluginPtr plugin = this->loader.Instantiate(name);

    auto engine = gz::physics::RequestEngine3d<JointMimicFeatureList>::From(plugin);
    ASSERT_NE(nullptr, engine);

    sdf::Root root;
    const sdf::Errors errors = root.Load(gz::common::joinPaths(TEST_WORLD_DIR,
          "mimic_prismatic_world.sdf"));
    ASSERT_TRUE(errors.empty()) << errors.front();

    auto world = engine->ConstructWorld(*root.WorldByIndex(0));

    auto model = world->GetModel("prismatic_model");

    auto parentJoint = model->GetJoint("prismatic_joint_1");
    auto prismaticChildJoint1 = model->GetJoint("prismatic_joint_2");
    auto revoluteChildJoint1 = model->GetJoint("revolute_joint_1");
    auto revoluteChildJoint2 = model->GetJoint("revolute_joint_2");
    auto prismaticChildJoint2 = model->GetJoint("prismatic_joint_3");

    // Ensure both joints start from zero angle.
    EXPECT_EQ(parentJoint->GetPosition(0), 0);
    EXPECT_EQ(prismaticChildJoint1->GetPosition(0), 0);
    EXPECT_EQ(revoluteChildJoint1->GetPosition(0), 0);
    EXPECT_EQ(revoluteChildJoint2->GetPosition(0), 0);
    EXPECT_EQ(prismaticChildJoint2->GetPosition(0), 0);

    gz::physics::ForwardStep::Output output;
    gz::physics::ForwardStep::State state;
    gz::physics::ForwardStep::Input input;

    // Case : Without mimic constraint

    // Let the simulation run without mimic constraint.
    // The positions of joints should not be equal.
    double prismaticJointPrevPos = 0;
    double revoluteJointPrevPos = 0;
    for (int i = 0; i < 10; i++)
    {
      world->Step(output, state, input);
      if (i > 5)
      {
        EXPECT_NE(prismaticJointPrevPos, prismaticChildJoint1->GetPosition(0));
        EXPECT_NE(prismaticJointPrevPos, revoluteChildJoint1->GetPosition(0));
        EXPECT_NE(revoluteJointPrevPos, revoluteChildJoint2->GetPosition(0));
        EXPECT_NE(revoluteJointPrevPos, prismaticChildJoint2->GetPosition(0));
      }

      // Update previous positions.
      prismaticJointPrevPos = parentJoint->GetPosition(0);
      revoluteJointPrevPos = revoluteChildJoint1->GetPosition(0);
    }

    auto testMimicFcn = [&](double multiplier, double offset, double reference)
      {
        // Set mimic joint constraints.
        // Parent --> Child
        // prismatic_joint_1 -> prismatic_joint_2
        // prismatic_joint_1 -> revolute_joint_1
        // revolute_joint_1 --> revolute_joint_2
        // revolute_joint_1 --> prismatic_joint_3
        prismaticChildJoint1->SetMimicConstraint("prismatic_joint_1", multiplier,
            offset, reference);
        revoluteChildJoint1->SetMimicConstraint("prismatic_joint_1", multiplier,
            offset, reference);
        revoluteChildJoint2->SetMimicConstraint("revolute_joint_1", multiplier,
            offset, reference);
        prismaticChildJoint2->SetMimicConstraint("revolute_joint_1", multiplier,
            offset, reference);

        // Reset positions and run a few iterations so the positions reach nontrivial values.
        parentJoint->SetPosition(0, 0);
        prismaticChildJoint1->SetPosition(0, 0);
        prismaticChildJoint2->SetPosition(0, 0);
        revoluteChildJoint1->SetPosition(0, 0);
        revoluteChildJoint2->SetPosition(0, 0);
        for (int _ = 0; _ < 10; _++)
          world->Step(output, state, input);

        // Child joint's position should be equal to that of parent joint in previous timestep,
        // considering the offsets and multipliers.
        prismaticJointPrevPos = parentJoint->GetPosition(0);
        revoluteJointPrevPos = revoluteChildJoint1->GetPosition(0);
        for (int _ = 0; _ < 10; _++)
        {
          world->Step(output, state, input);
          // Check for prismatic -> prismatic mimicking.
          EXPECT_FLOAT_EQ(multiplier * (prismaticJointPrevPos - reference) + offset,
              prismaticChildJoint1->GetPosition(0));
          // Check for prismatic -> revolute mimicking.
          EXPECT_FLOAT_EQ(multiplier * (prismaticJointPrevPos - reference) + offset,
              revoluteChildJoint1->GetPosition(0));
          // Check for revolute -> revolute mimicking.
          EXPECT_FLOAT_EQ(multiplier * (revoluteJointPrevPos - reference) + offset,
              revoluteChildJoint2->GetPosition(0));
          // Check for revolute --> prismatic mimicking.
          EXPECT_FLOAT_EQ(multiplier * (revoluteJointPrevPos - reference) + offset,
              prismaticChildJoint2->GetPosition(0));

          // Update previous positions.
          prismaticJointPrevPos = parentJoint->GetPosition(0);
          revoluteJointPrevPos = revoluteChildJoint1->GetPosition(0);
        }
      };

    // Testing with different (multiplier, offset, reference) combinations.
    testMimicFcn(1, 0, 0);
    testMimicFcn(-1, 0, 0);
    testMimicFcn(1, 0.1, 0);
    testMimicFcn(1, 0.05, 0.05);
    testMimicFcn(-1, 0.2, 0);
    testMimicFcn(-1, 0.05, -0.05);
    testMimicFcn(-2, 0, 0);
    testMimicFcn(2, 0.1, 0);
  }
}

// Here, we test the mimic constraint for a pair of universal joints.
TEST_F(JointMimicFeatureTest, UniversalMimicTest)
{
  for (const std::string &name : this->pluginNames)
  {
    if(this->PhysicsEngineName(name) != "dartsim")
    {
      GTEST_SKIP();
    }

    std::cout << "Testing plugin: " << name << std::endl;
    gz::plugin::PluginPtr plugin = this->loader.Instantiate(name);

    auto engine = gz::physics::RequestEngine3d<JointMimicFeatureList>::From(plugin);
    ASSERT_NE(nullptr, engine);

    sdf::Root root;
    const sdf::Errors errors = root.Load(gz::common::joinPaths(TEST_WORLD_DIR,
          "mimic_universal_world.sdf"));
    ASSERT_TRUE(errors.empty()) << errors.front();

    auto world = engine->ConstructWorld(*root.WorldByIndex(0));

    // Test mimic constraint between two revolute joints.
    auto model = world->GetModel("universal_model");
    auto parentJoint = model->GetJoint("universal_joint_1");
    auto childJoint = model->GetJoint("universal_joint_2");

    // Ensure both joints start from zero angle.
    EXPECT_EQ(parentJoint->GetPosition(0), 0);
    EXPECT_EQ(childJoint->GetPosition(0), 0);

    gz::physics::ForwardStep::Output output;
    gz::physics::ForwardStep::State state;
    gz::physics::ForwardStep::Input input;

    // Case : Without mimic constraint

    // Let the simulation run without mimic constraint.
    // The positions of joints should not be equal.
    double parentJointPrevPosAxis1 = 0;
    double parentJointPrevPosAxis2 = 0;
    for (int _ = 0; _ < 10; _++)
    {
      world->Step(output, state, input);
      EXPECT_NE(parentJointPrevPosAxis1, childJoint->GetPosition(0));
      EXPECT_NE(parentJointPrevPosAxis2, childJoint->GetPosition(1));
      parentJointPrevPosAxis1 = parentJoint->GetPosition(0);
      parentJointPrevPosAxis2 = parentJoint->GetPosition(1);
    }

    auto testMimicFcn = [&](double multiplier, double offset, double reference)
      {
        // Set mimic joint constraint.
        childJoint->SetMimicConstraint("universal_joint_1", multiplier, offset, reference);
        // Reset positions and run a few iterations so the positions reach nontrivial values.
        parentJoint->SetPosition(0, 0);
        parentJoint->SetPosition(1, 0);
        childJoint->SetPosition(0, 0);
        childJoint->SetPosition(1, 0);
        for (int _ = 0; _ < 10; _++)
          world->Step(output, state, input);

        // Child joint's position should be equal to that of parent joint in previous timestep.
        parentJointPrevPosAxis1 = parentJoint->GetPosition(0);
        parentJointPrevPosAxis2 = parentJoint->GetPosition(1);
        for (int _ = 0; _ < 10; _++)
        {
          world->Step(output, state, input);
          EXPECT_FLOAT_EQ(multiplier * (parentJointPrevPosAxis1 - reference) + offset,
              childJoint->GetPosition(0));
          EXPECT_FLOAT_EQ(multiplier * (parentJointPrevPosAxis2 - reference) + offset,
              childJoint->GetPosition(1));
          parentJointPrevPosAxis1 = parentJoint->GetPosition(0);
          parentJointPrevPosAxis2 = parentJoint->GetPosition(1);
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

// In this test, we have 2 pendulums of different lengths.
// Originally, their time periods are different, but after applying
// the mimic constraint, they follow the same velocity, effectively
// violating some laws. (Work done = Change in kinetic energy, for instance)
TEST_F(JointMimicFeatureTest, PendulumMimicTest)
{
  for (const std::string &name : this->pluginNames)
  {
    if(this->PhysicsEngineName(name) != "dartsim")
    {
      GTEST_SKIP();
    }

    std::cout << "Testing plugin: " << name << std::endl;
    gz::plugin::PluginPtr plugin = this->loader.Instantiate(name);

    auto engine = gz::physics::RequestEngine3d<JointMimicFeatureList>::From(plugin);
    ASSERT_NE(nullptr, engine);

    sdf::Root root;
    const sdf::Errors errors = root.Load(gz::common::joinPaths(TEST_WORLD_DIR,
          "mimic_pendulum_world.sdf"));
    ASSERT_TRUE(errors.empty()) << errors.front();

    auto world = engine->ConstructWorld(*root.WorldByIndex(0));

    // Test mimic constraint between two revolute joints.
    auto model = world->GetModel("pendulum_with_base");
    auto parentJoint = model->GetJoint("upper_joint_1");
    auto childJoint = model->GetJoint("upper_joint_2");

    // Ensure both joints start from zero angle.
    EXPECT_EQ(parentJoint->GetPosition(0), 0);
    EXPECT_EQ(childJoint->GetPosition(0), 0);

    gz::physics::ForwardStep::Output output;
    gz::physics::ForwardStep::State state;
    gz::physics::ForwardStep::Input input;

    // Case : Without mimic constraint

    // Let the simulation run without mimic constraint.
    // The positions of joints should not be equal.
    double parentJointPrevPosAxis = 0;
    for (int _ = 0; _ < 10; _++)
    {
      world->Step(output, state, input);
      EXPECT_NE(parentJointPrevPosAxis, childJoint->GetPosition(0));
      parentJointPrevPosAxis = parentJoint->GetPosition(0);
    }

    auto testMimicFcn = [&](double multiplier, double offset, double reference)
      {
        // Set mimic joint constraint.
        childJoint->SetMimicConstraint("upper_joint_1", multiplier, offset, reference);
        // Reset positions and run a few iterations so the positions reach nontrivial values.
        parentJoint->SetPosition(0, 0);
        childJoint->SetPosition(0, 0);
        for (int _ = 0; _ < 10; _++)
          world->Step(output, state, input);

        // Child joint's position should be equal to that of parent joint in previous timestep.
        parentJointPrevPosAxis = parentJoint->GetPosition(0);
        for (int _ = 0; _ < 10; _++)
        {
          world->Step(output, state, input);
          EXPECT_FLOAT_EQ(multiplier * (parentJointPrevPosAxis - reference) + offset,
              childJoint->GetPosition(0));
          parentJointPrevPosAxis = parentJoint->GetPosition(0);
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

int main(int argc, char *argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  if (!JointMimicFeaturesTest<
      JointMimicFeatureList>::init(argc, argv))
    return -1;
  return RUN_ALL_TESTS();
}
