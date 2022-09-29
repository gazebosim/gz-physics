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

#include <chrono>

#include <gz/common/Console.hh>
#include <gz/plugin/Loader.hh>

#include <gz/math/Helpers.hh>
#include <gz/math/eigen3/Conversions.hh>

#include "../helpers/TestLibLoader.hh"
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
class JointFeaturesTest:
  public testing::Test, public gz::physics::TestLibLoader
{
  // Documentation inherited
  public: void SetUp() override
  {
    gz::common::Console::SetVerbosity(4);

    std::cerr << "JointFeaturesTest::GetLibToTest() " << JointFeaturesTest::GetLibToTest() << '\n';

    loader.LoadLib(JointFeaturesTest::GetLibToTest());

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

struct JointFeatureList : gz::physics::FeatureList<
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
    gz::physics::sdf::ConstructSdfWorld
> { };

using JointFeaturesTestTypes =
  ::testing::Types<JointFeatureList>;
TYPED_TEST_SUITE(JointFeaturesTest,
                 JointFeaturesTestTypes);

TYPED_TEST(JointFeaturesTest, JointSetCommand)
{
  for (const std::string &name : this->pluginNames)
  {
    // TODO(ahcorde): reactive this test when test.world is working with bullet
    if(this->PhysicsEngineName(name) == "bullet")
    {
      GTEST_SKIP();
    }

    std::cout << "Testing plugin: " << name << std::endl;
    gz::plugin::PluginPtr plugin = this->loader.Instantiate(name);

    auto engine = gz::physics::RequestEngine3d<JointFeatureList>::From(plugin);
    ASSERT_NE(nullptr, engine);

    sdf::Root root;
    const sdf::Errors errors = root.Load(gz::common::joinPaths(TEST_WORLD_DIR, "test.world"));
    ASSERT_TRUE(errors.empty()) << errors.front();

    const std::string modelName{"double_pendulum_with_base"};
    const std::string jointName{"upper_joint"};

    auto world = engine->ConstructWorld(*root.WorldByIndex(0));

    auto model = world->GetModel(modelName);
    auto joint = model->GetJoint(jointName);

    // EXPECT_EQ(dart::dynamics::Joint::FORCE, dartJoint->getActuatorType());

    // Test joint velocity command
    gz::physics::ForwardStep::Output output;
    gz::physics::ForwardStep::State state;
    gz::physics::ForwardStep::Input input;

    // Expect negative joint velocity after 1 step without joint command
    world->Step(output, state, input);
    EXPECT_LT(joint->GetVelocity(0), 0.0);

    auto base_link = model->GetLink("base");
    ASSERT_NE(nullptr, base_link);

    // Check that invalid velocity commands don't cause collisions to fail
    for (std::size_t i = 0; i < 1000; ++i)
    {
      joint->SetForce(0, std::numeric_limits<double>::quiet_NaN());
      // expect the position of the pendulum to stay above ground
      world->Step(output, state, input);
      auto frameData = base_link->FrameDataRelativeToWorld();
      EXPECT_NEAR(0.0, frameData.pose.translation().z(), 1e-3);
    }

    joint->SetVelocityCommand(0, 1);
    world->Step(output, state, input);
    // Setting a velocity command changes the actuator type to SERVO
    // EXPECT_EQ(dart::dynamics::Joint::SERVO, dartJoint->getActuatorType());

    const std::size_t numSteps = 10;
    world->Step(output, state, input);
    for (std::size_t i = 0; i < numSteps; ++i)
    {
      // Call SetVelocityCommand before each step
      joint->SetVelocityCommand(0, 1);
      world->Step(output, state, input);
      EXPECT_NEAR(1.0, joint->GetVelocity(0), 1e-2);
    }

    if(this->PhysicsEngineName(name) == "dartsim")
    {
      for (std::size_t i = 0; i < numSteps; ++i)
      {
        // expect joint to freeze in subsequent steps without SetVelocityCommand
        world->Step(output, state, input);
        EXPECT_NEAR(0.0, joint->GetVelocity(0), 1e-1);
      }
    }

    // Check that invalid velocity commands don't cause collisions to fail
    for (std::size_t i = 0; i < 1000; ++i)
    {
      joint->SetVelocityCommand(0, std::numeric_limits<double>::quiet_NaN());
      // expect the position of the pendulum to stay above ground
      world->Step(output, state, input);
      auto frameData = base_link->FrameDataRelativeToWorld();
      EXPECT_NEAR(0.0, frameData.pose.translation().z(), 1e-3);
    }
  }
}

struct JointFeaturePositionLimitsList : gz::physics::FeatureList<
    gz::physics::ForwardStep,
    gz::physics::GetBasicJointProperties,
    gz::physics::GetBasicJointState,
    gz::physics::GetEngineInfo,
    gz::physics::GetJointFromModel,
    gz::physics::GetModelFromWorld,
    gz::physics::SetBasicJointState,
    gz::physics::SetJointPositionLimitsFeature,
    gz::physics::SetJointVelocityCommandFeature,
    gz::physics::sdf::ConstructSdfWorld
> { };

template <class T>
class JointFeaturesPositionLimitsTest :
  public JointFeaturesTest<T>{};
using JointFeaturesPositionLimitsTestTypes =
  ::testing::Types<JointFeaturePositionLimitsList>;
TYPED_TEST_SUITE(JointFeaturesPositionLimitsTest,
                 JointFeaturesPositionLimitsTestTypes);

TYPED_TEST(JointFeaturesPositionLimitsTest, JointSetPositionLimitsWithForceControl)
{
  for (const std::string &name : this->pluginNames)
  {
    std::cout << "Testing plugin: " << name << std::endl;
    gz::plugin::PluginPtr plugin = this->loader.Instantiate(name);

    auto engine = gz::physics::RequestEngine3d<JointFeaturePositionLimitsList>::From(plugin);
    ASSERT_NE(nullptr, engine);

    sdf::Root root;
    const sdf::Errors errors = root.Load(gz::common::joinPaths(TEST_WORLD_DIR, "test.world"));
    ASSERT_TRUE(errors.empty()) << errors.front();

    auto world = engine->ConstructWorld(*root.WorldByIndex(0));
    auto model = world->GetModel("simple_joint_test");
    auto joint = model->GetJoint("j1");

    gz::physics::ForwardStep::Output output;
    gz::physics::ForwardStep::State state;
    gz::physics::ForwardStep::Input input;

    world->Step(output, state, input);

    auto pos = joint->GetPosition(0);

    joint->SetMinPosition(0, pos - 0.1);
    joint->SetMaxPosition(0, pos + 0.1);

    for (std::size_t i = 0; i < 100; ++i)
    {
      joint->SetForce(0, 100);
      world->Step(output, state, input);
    }
    EXPECT_NEAR(pos + 0.1, joint->GetPosition(0), 1e-3);

    for (std::size_t i = 0; i < 100; ++i)
    {
      joint->SetForce(0, -100);
      world->Step(output, state, input);
    }
    EXPECT_NEAR(pos - 0.1, joint->GetPosition(0), 1e-3);

    joint->SetMinPosition(0, pos - 0.5);
    joint->SetMaxPosition(0, pos + 0.5);

    for (std::size_t i = 0; i < 300; ++i)
    {
      joint->SetForce(0, 100);
      world->Step(output, state, input);
    }
    EXPECT_NEAR(pos + 0.5, joint->GetPosition(0), 1e-2);

    for (std::size_t i = 0; i < 300; ++i)
    {
      joint->SetForce(0, -100);
      world->Step(output, state, input);
    }
    EXPECT_NEAR(pos - 0.5, joint->GetPosition(0), 1e-2);

    joint->SetMinPosition(0, -gz::math::INF_D);
    joint->SetMaxPosition(0, gz::math::INF_D);
    joint->SetPosition(0, pos);

    for (std::size_t i = 0; i < 300; ++i)
    {
      joint->SetForce(0, 100);
      world->Step(output, state, input);
    }
    EXPECT_LT(pos + 0.5, joint->GetPosition(0));
  }
}

struct JointFeaturePositionLimitsForceControlList : gz::physics::FeatureList<
    gz::physics::ForwardStep,
    gz::physics::GetBasicJointProperties,
    gz::physics::GetBasicJointState,
    gz::physics::GetEngineInfo,
    gz::physics::GetJointFromModel,
    gz::physics::GetModelFromWorld,
    // This feature is not requited but it will force to use dart6.10 which is required
    // to run these tests
    gz::physics::GetShapeFrictionPyramidSlipCompliance,
    gz::physics::SetBasicJointState,
    gz::physics::SetJointEffortLimitsFeature,
    gz::physics::SetJointPositionLimitsFeature,
    gz::physics::SetJointVelocityCommandFeature,
    gz::physics::SetJointVelocityLimitsFeature,
    gz::physics::sdf::ConstructSdfWorld
> { };

template <class T>
class JointFeaturesPositionLimitsForceControlTest :
  public JointFeaturesTest<T>{};
using JointFeaturesPositionLimitsForceControlTestTypes =
  ::testing::Types<JointFeaturePositionLimitsForceControlList>;
TYPED_TEST_SUITE(JointFeaturesPositionLimitsForceControlTest,
                 JointFeaturesPositionLimitsForceControlTestTypes);

///////////// DARTSIM > 6.10
TYPED_TEST(JointFeaturesPositionLimitsForceControlTest, JointSetVelocityLimitsWithForceControl)
{
  for (const std::string &name : this->pluginNames)
  {
    if(this->PhysicsEngineName(name) != "dartsim")
    {
      GTEST_SKIP();
    }

    std::cout << "Testing plugin: " << name << std::endl;
    gz::plugin::PluginPtr plugin = this->loader.Instantiate(name);

    auto engine =
      gz::physics::RequestEngine3d<JointFeaturePositionLimitsForceControlList>::From(plugin);
    ASSERT_NE(nullptr, engine);

    sdf::Root root;
    const sdf::Errors errors = root.Load(gz::common::joinPaths(TEST_WORLD_DIR, "test.world"));
    ASSERT_TRUE(errors.empty()) << errors.front();

    auto world = engine->ConstructWorld(*root.WorldByIndex(0));
    auto model = world->GetModel("simple_joint_test");
    auto joint = model->GetJoint("j1");

    gz::physics::ForwardStep::Output output;
    gz::physics::ForwardStep::State state;
    gz::physics::ForwardStep::Input input;

    world->Step(output, state, input);

    joint->SetMinVelocity(0, -0.25);
    joint->SetMaxVelocity(0, 0.5);

    for (std::size_t i = 0; i < 10; ++i)
    {
      joint->SetForce(0, 1000);
      world->Step(output, state, input);
    }
    EXPECT_NEAR(0.5, joint->GetVelocity(0), 1e-6);

    for (std::size_t i = 0; i < 10; ++i)
    {
      joint->SetForce(0, -1000);
      world->Step(output, state, input);
    }
    EXPECT_NEAR(-0.25, joint->GetVelocity(0), 1e-6);

    // set minimum velocity above zero
    joint->SetMinVelocity(0, 0.25);

    for (std::size_t i = 0; i < 10; ++i)
    {
      joint->SetForce(0, 0);
      world->Step(output, state, input);
    }
    EXPECT_NEAR(0.25, joint->GetVelocity(0), 1e-6);

    for (std::size_t i = 0; i < 10; ++i)
    {
      // make sure the minimum velocity is kept even without velocity commands
      world->Step(output, state, input);
    }
    EXPECT_NEAR(0.25, joint->GetVelocity(0), 1e-6);

    joint->SetMinVelocity(0, -0.25);
    joint->SetPosition(0, 0);
    joint->SetVelocity(0, 0);

    for (std::size_t i = 0; i < 10; ++i)
    {
      joint->SetForce(0, 0);
      world->Step(output, state, input);
    }
    EXPECT_NEAR(0, joint->GetVelocity(0), 1e-6);

    joint->SetMinVelocity(0, -gz::math::INF_D);
    joint->SetMaxVelocity(0, gz::math::INF_D);

    for (std::size_t i = 0; i < 10; ++i)
    {
      joint->SetForce(0, 1000);
      world->Step(output, state, input);
    }
    EXPECT_LT(0.5, joint->GetVelocity(0));
  }
}

TYPED_TEST(JointFeaturesPositionLimitsForceControlTest, JointSetEffortLimitsWithForceControl)
{
  for (const std::string &name : this->pluginNames)
  {
    if(this->PhysicsEngineName(name) != "dartsim")
    {
      GTEST_SKIP();
    }

    std::cout << "Testing plugin: " << name << std::endl;
    gz::plugin::PluginPtr plugin = this->loader.Instantiate(name);

    auto engine =
      gz::physics::RequestEngine3d<JointFeaturePositionLimitsForceControlList>::From(plugin);
    ASSERT_NE(nullptr, engine);

    sdf::Root root;
    const sdf::Errors errors = root.Load(gz::common::joinPaths(TEST_WORLD_DIR, "test.world"));
    ASSERT_TRUE(errors.empty()) << errors.front();

    auto world = engine->ConstructWorld(*root.WorldByIndex(0));
    auto model = world->GetModel("simple_joint_test");
    auto joint = model->GetJoint("j1");

    gz::physics::ForwardStep::Output output;
    gz::physics::ForwardStep::State state;
    gz::physics::ForwardStep::Input input;

    world->Step(output, state, input);

    auto pos = joint->GetPosition(0);

    joint->SetMinEffort(0, -1e-6);
    joint->SetMaxEffort(0, 1e-6);

    for (std::size_t i = 0; i < 100; ++i)
    {
      joint->SetForce(0, 1);
      world->Step(output, state, input);
    }
    EXPECT_NEAR(pos, joint->GetPosition(0), 1e-3);
    EXPECT_NEAR(0, joint->GetVelocity(0), 1e-6);

    for (std::size_t i = 0; i < 100; ++i)
    {
      joint->SetForce(0, -1);
      world->Step(output, state, input);
    }
    EXPECT_NEAR(pos, joint->GetPosition(0), 1e-3);
    EXPECT_NEAR(0, joint->GetVelocity(0), 1e-6);

    joint->SetMinEffort(0, -80);
    joint->SetMaxEffort(0, 80);

    for (std::size_t i = 0; i < 100; ++i)
    {
      joint->SetForce(0, 1);
      world->Step(output, state, input);
    }
    EXPECT_LT(pos, joint->GetPosition(0));
    EXPECT_LT(0, joint->GetVelocity(0));

    joint->SetMinEffort(0, -gz::math::INF_D);
    joint->SetMaxEffort(0, gz::math::INF_D);
    joint->SetPosition(0, 0);
    joint->SetVelocity(0, 0);

    for (std::size_t i = 0; i < 100; ++i)
    {
      joint->SetForce(0, 1);
      world->Step(output, state, input);
    }
    EXPECT_LT(pos, joint->GetPosition(0));
    EXPECT_LT(0, joint->GetVelocity(0));
  }
}

TYPED_TEST(JointFeaturesPositionLimitsForceControlTest, JointSetCombinedLimitsWithForceControl)
{
  for (const std::string &name : this->pluginNames)
  {
    if(this->PhysicsEngineName(name) != "dartsim")
    {
      GTEST_SKIP();
    }

    std::cout << "Testing plugin: " << name << std::endl;
    gz::plugin::PluginPtr plugin = this->loader.Instantiate(name);

    auto engine =
      gz::physics::RequestEngine3d<JointFeaturePositionLimitsForceControlList>::From(plugin);
    ASSERT_NE(nullptr, engine);

    sdf::Root root;
    const sdf::Errors errors = root.Load(gz::common::joinPaths(TEST_WORLD_DIR, "test.world"));
    ASSERT_TRUE(errors.empty()) << errors.front();

    auto world = engine->ConstructWorld(*root.WorldByIndex(0));
    auto model = world->GetModel("simple_joint_test");
    auto joint = model->GetJoint("j1");

    gz::physics::ForwardStep::Output output;
    gz::physics::ForwardStep::State state;
    gz::physics::ForwardStep::Input input;

    world->Step(output, state, input);

    auto pos = joint->GetPosition(0);

    joint->SetMinPosition(0, pos - 0.1);
    joint->SetMaxPosition(0, pos + 0.1);
    joint->SetMinVelocity(0, -0.25);
    joint->SetMaxVelocity(0, 0.5);
    joint->SetMinEffort(0, -1e-6);
    joint->SetMaxEffort(0, 1e-6);

    for (std::size_t i = 0; i < 100; ++i)
    {
      joint->SetForce(0, 100);
      world->Step(output, state, input);
    }
    EXPECT_NEAR(pos, joint->GetPosition(0), 1e-2);
    EXPECT_NEAR(0, joint->GetVelocity(0), 1e-6);

    for (std::size_t i = 0; i < 100; ++i)
    {
      joint->SetForce(0, -100);
      world->Step(output, state, input);
    }
    EXPECT_NEAR(pos, joint->GetPosition(0), 1e-2);
    EXPECT_NEAR(0, joint->GetVelocity(0), 1e-6);

    joint->SetMinEffort(0, -500);
    joint->SetMaxEffort(0, 1000);

    for (std::size_t i = 0; i < 100; ++i)
    {
      joint->SetForce(0, 1000);
      world->Step(output, state, input);
    }
    // 0.05 because we go 0.1 s with max speed 0.5
    EXPECT_NEAR(pos + 0.05, joint->GetPosition(0), 1e-2);
    EXPECT_NEAR(0.5, joint->GetVelocity(0), 1e-6);

    for (std::size_t i = 0; i < 200; ++i)
    {
      joint->SetForce(0, 1000);
      world->Step(output, state, input);
    }
    EXPECT_NEAR(pos + 0.1, joint->GetPosition(0), 1e-2);
    EXPECT_NEAR(0, joint->GetVelocity(0), 1e-6);

    joint->SetPosition(0, pos);
    EXPECT_NEAR(pos, joint->GetPosition(0), 1e-2);

    joint->SetMinVelocity(0, -1);
    joint->SetMaxVelocity(0, 1);

    for (std::size_t i = 0; i < 100; ++i)
    {
      joint->SetForce(0, 1000);
      world->Step(output, state, input);
    }
    EXPECT_NEAR(pos + 0.1, joint->GetPosition(0), 1e-2);
    EXPECT_NEAR(1, joint->GetVelocity(0), 1e-6);

    joint->SetPosition(0, pos);
    EXPECT_NEAR(pos, joint->GetPosition(0), 1e-2);

    joint->SetMinPosition(0, -1e6);
    joint->SetMaxPosition(0, 1e6);

    for (std::size_t i = 0; i < 100; ++i)
    {
      joint->SetForce(0, 1000);
      world->Step(output, state, input);
    }
    EXPECT_LT(pos + 0.1, joint->GetPosition(0));
    EXPECT_NEAR(1, joint->GetVelocity(0), 1e-6);
  }
}

// TODO(anyone): position limits do not work very well with velocity control
// bug https://github.com/dartsim/dart/issues/1583
// resolved in DART 6.11.0
TYPED_TEST(JointFeaturesPositionLimitsForceControlTest, DISABLED_JointSetPositionLimitsWithVelocityControl)
{
  for (const std::string &name : this->pluginNames)
  {
    if(this->PhysicsEngineName(name) != "dartsim")
    {
      GTEST_SKIP();
    }

    std::cout << "Testing plugin: " << name << std::endl;
    gz::plugin::PluginPtr plugin = this->loader.Instantiate(name);

    auto engine =
      gz::physics::RequestEngine3d<JointFeaturePositionLimitsForceControlList>::From(plugin);
    ASSERT_NE(nullptr, engine);

    sdf::Root root;
    const sdf::Errors errors = root.Load(gz::common::joinPaths(TEST_WORLD_DIR, "test.world"));
    ASSERT_TRUE(errors.empty()) << errors.front();

    const std::string modelName{"simple_joint_test"};
    const std::string jointName{"j1"};

    auto world = engine->ConstructWorld(*root.WorldByIndex(0));

    auto model = world->GetModel(modelName);
    auto joint = model->GetJoint(jointName);

    gz::physics::ForwardStep::Output output;
    gz::physics::ForwardStep::State state;
    gz::physics::ForwardStep::Input input;

    world->Step(output, state, input);

    auto pos = joint->GetPosition(0);

    joint->SetMinPosition(0, pos - 0.1);
    joint->SetMaxPosition(0, pos + 0.1);
    for (std::size_t i = 0; i < 1000; ++i)
    {
      joint->SetVelocityCommand(0, 1);
      world->Step(output, state, input);

      if (i % 500 == 499)
      {
        EXPECT_NEAR(pos + 0.1, joint->GetPosition(0), 1e-2);
        EXPECT_NEAR(0, joint->GetVelocity(0), 1e-6);
      }
    }
  }
}

TYPED_TEST(JointFeaturesPositionLimitsForceControlTest, JointSetVelocityLimitsWithVelocityControl)
{
  for (const std::string &name : this->pluginNames)
  {
    if(this->PhysicsEngineName(name) != "dartsim")
    {
      GTEST_SKIP();
    }

    std::cout << "Testing plugin: " << name << std::endl;
    gz::plugin::PluginPtr plugin = this->loader.Instantiate(name);

    auto engine =
      gz::physics::RequestEngine3d<JointFeaturePositionLimitsForceControlList>::From(plugin);
    ASSERT_NE(nullptr, engine);

    sdf::Root root;
    const sdf::Errors errors = root.Load(gz::common::joinPaths(TEST_WORLD_DIR, "test.world"));
    ASSERT_TRUE(errors.empty()) << errors.front();

    auto world = engine->ConstructWorld(*root.WorldByIndex(0));
    auto model = world->GetModel("simple_joint_test");
    auto joint = model->GetJoint("j1");

    gz::physics::ForwardStep::Output output;
    gz::physics::ForwardStep::State state;
    gz::physics::ForwardStep::Input input;

    joint->SetMinVelocity(0, -0.1);
    joint->SetMaxVelocity(0, 0.1);

    for (std::size_t i = 0; i < 100; ++i)
    {
      joint->SetVelocityCommand(0, 1);
      world->Step(output, state, input);
    }
    EXPECT_NEAR(0.1, joint->GetVelocity(0), 1e-6);

    for (std::size_t i = 0; i < 10; ++i)
    {
      joint->SetVelocityCommand(0, 0.1);
      world->Step(output, state, input);
    }
    EXPECT_NEAR(0.1, joint->GetVelocity(0), 1e-6);

    for (std::size_t i = 0; i < 10; ++i)
    {
      joint->SetVelocityCommand(0, -0.025);
      world->Step(output, state, input);
    }
    EXPECT_NEAR(-0.025, joint->GetVelocity(0), 1e-6);

    for (std::size_t i = 0; i < 10; ++i)
    {
      joint->SetVelocityCommand(0, -1);
      world->Step(output, state, input);
    }
    EXPECT_NEAR(-0.1, joint->GetVelocity(0), 1e-6);

    joint->SetMinVelocity(0, -gz::math::INF_D);
    joint->SetMaxVelocity(0, gz::math::INF_D);

    for (std::size_t i = 0; i < 100; ++i)
    {
      joint->SetVelocityCommand(0, 1);
      world->Step(output, state, input);
    }
    EXPECT_NEAR(1, joint->GetVelocity(0), 1e-6);
  }
}

TYPED_TEST(JointFeaturesPositionLimitsForceControlTest, JointSetEffortLimitsWithVelocityControl)
{
  for (const std::string &name : this->pluginNames)
  {
    if(this->PhysicsEngineName(name) != "dartsim")
    {
      GTEST_SKIP();
    }

    std::cout << "Testing plugin: " << name << std::endl;
    gz::plugin::PluginPtr plugin = this->loader.Instantiate(name);

    auto engine =
      gz::physics::RequestEngine3d<JointFeaturePositionLimitsForceControlList>::From(plugin);
    ASSERT_NE(nullptr, engine);

    sdf::Root root;
    const sdf::Errors errors = root.Load(gz::common::joinPaths(TEST_WORLD_DIR, "test.world"));
    ASSERT_TRUE(errors.empty()) << errors.front();

    auto world = engine->ConstructWorld(*root.WorldByIndex(0));
    auto model = world->GetModel("simple_joint_test");
    auto joint = model->GetJoint("j1");

    gz::physics::ForwardStep::Output output;
    gz::physics::ForwardStep::State state;
    gz::physics::ForwardStep::Input input;

    joint->SetMinEffort(0, -1e-6);
    joint->SetMaxEffort(0, 1e-6);

    for (std::size_t i = 0; i < 100; ++i)
    {
      joint->SetVelocityCommand(0, 1);
      world->Step(output, state, input);
    }
    EXPECT_NEAR(0, joint->GetVelocity(0), 1e-6);

    joint->SetMinEffort(0, -80);
    joint->SetMaxEffort(0, 80);

    for (std::size_t i = 0; i < 100; ++i)
    {
      joint->SetVelocityCommand(0, -1);
      world->Step(output, state, input);
    }
    EXPECT_NEAR(-1, joint->GetVelocity(0), 1e-6);

    joint->SetMinEffort(0, -gz::math::INF_D);
    joint->SetMaxEffort(0, gz::math::INF_D);

    for (std::size_t i = 0; i < 10; ++i)
    {
      joint->SetVelocityCommand(0, -100);
      world->Step(output, state, input);
    }
    EXPECT_NEAR(-100, joint->GetVelocity(0), 1e-6);
  }
}

TYPED_TEST(JointFeaturesPositionLimitsForceControlTest, JointSetCombinedLimitsWithVelocityControl)
{
  for (const std::string &name : this->pluginNames)
  {
    if(this->PhysicsEngineName(name) != "dartsim")
    {
      GTEST_SKIP();
    }

    std::cout << "Testing plugin: " << name << std::endl;
    gz::plugin::PluginPtr plugin = this->loader.Instantiate(name);

    auto engine =
      gz::physics::RequestEngine3d<JointFeaturePositionLimitsForceControlList>::From(plugin);
    ASSERT_NE(nullptr, engine);

    sdf::Root root;
    const sdf::Errors errors = root.Load(gz::common::joinPaths(TEST_WORLD_DIR, "test.world"));
    ASSERT_TRUE(errors.empty()) << errors.front();

    auto world = engine->ConstructWorld(*root.WorldByIndex(0));
    auto model = world->GetModel("simple_joint_test");
    auto joint = model->GetJoint("j1");

    // Test joint velocity command
    gz::physics::ForwardStep::Output output;
    gz::physics::ForwardStep::State state;
    gz::physics::ForwardStep::Input input;

    joint->SetMinVelocity(0, -0.5);
    joint->SetMaxVelocity(0, 0.5);
    joint->SetMinEffort(0, -1e-6);
    joint->SetMaxEffort(0, 1e-6);

    for (std::size_t i = 0; i < 1000; ++i)
    {
      joint->SetVelocityCommand(0, 1);
      world->Step(output, state, input);
    }
    EXPECT_NEAR(0, joint->GetVelocity(0), 1e-6);

    joint->SetMinEffort(0, -1e6);
    joint->SetMaxEffort(0, 1e6);

    for (std::size_t i = 0; i < 1000; ++i)
    {
      joint->SetVelocityCommand(0, -1);
      world->Step(output, state, input);
    }
    EXPECT_NEAR(-0.5, joint->GetVelocity(0), 1e-6);
  }
}
///////////// DARTSIM > 6.10 end


struct JointFeatureDetachList : gz::physics::FeatureList<
    gz::physics::DetachJointFeature,
    gz::physics::ForwardStep,
    gz::physics::FreeJointCast,
    gz::physics::GetBasicJointProperties,
    gz::physics::GetBasicJointState,
    gz::physics::GetEngineInfo,
    gz::physics::GetJointFromModel,
    gz::physics::GetLinkFromModel,
    gz::physics::GetModelFromWorld,
    gz::physics::LinkFrameSemantics,
    gz::physics::RevoluteJointCast,
    gz::physics::SetBasicJointState,
    gz::physics::SetJointVelocityCommandFeature,
    gz::physics::sdf::ConstructSdfWorld
> { };

template <class T>
class JointFeaturesDetachTest :
  public JointFeaturesTest<T>{};
using JointFeaturesDetachTestTypes =
  ::testing::Types<JointFeatureDetachList>;
TYPED_TEST_SUITE(JointFeaturesDetachTest,
                 JointFeaturesDetachTestTypes);

// Test detaching joints.
TYPED_TEST(JointFeaturesDetachTest, JointDetach)
{
  for (const std::string &name : this->pluginNames)
  {
    std::cout << "Testing plugin: " << name << std::endl;
    gz::plugin::PluginPtr plugin = this->loader.Instantiate(name);

    auto engine = gz::physics::RequestEngine3d<JointFeatureDetachList>::From(plugin);
    ASSERT_NE(nullptr, engine);

    sdf::Root root;
    const sdf::Errors errors = root.Load(gz::common::joinPaths(TEST_WORLD_DIR, "test.world"));
    ASSERT_TRUE(errors.empty()) << errors.front();

    const std::string modelName{"double_pendulum_with_base"};
    const std::string upperJointName{"upper_joint"};
    const std::string lowerJointName{"lower_joint"};
    const std::string upperLinkName{"upper_link"};
    const std::string lowerLinkName{"lower_link"};

    auto world = engine->ConstructWorld(*root.WorldByIndex(0));

    auto model = world->GetModel(modelName);
    auto upperLink = model->GetLink(upperLinkName);
    auto lowerLink = model->GetLink(lowerLinkName);
    auto upperJoint = model->GetJoint(upperJointName);
    auto lowerJoint = model->GetJoint(lowerJointName);

    // test Cast*Joint functions
    EXPECT_NE(nullptr, upperJoint->CastToRevoluteJoint());
    EXPECT_NE(nullptr, lowerJoint->CastToRevoluteJoint());
    EXPECT_EQ(nullptr, upperJoint->CastToFreeJoint());
    EXPECT_EQ(nullptr, lowerJoint->CastToFreeJoint());
    //
    // dart::simulation::WorldPtr dartWorld = world->GetDartsimWorld();
    // ASSERT_NE(nullptr, dartWorld);
    //
    // const dart::dynamics::SkeletonPtr skeleton =
    //     dartWorld->getSkeleton(modelName);
    // ASSERT_NE(nullptr, skeleton);
    //
    // const auto *dartUpperLink = skeleton->getBodyNode(upperLinkName);
    // const auto *dartLowerLink = skeleton->getBodyNode(lowerLinkName);
    // EXPECT_EQ("RevoluteJoint", dartUpperLink->getParentJoint()->getType());
    // EXPECT_EQ("RevoluteJoint", dartLowerLink->getParentJoint()->getType());

    const gz::math::Pose3d initialUpperLinkPose(1, 0, 2.1, -GZ_PI/2, 0, 0);
    const gz::math::Pose3d initialLowerLinkPose(1.25, 1, 2.1, -2, 0, 0);

    auto frameDataUpperLink = upperLink->FrameDataRelativeToWorld();
    auto frameDataLowerLink = lowerLink->FrameDataRelativeToWorld();

    EXPECT_EQ(initialUpperLinkPose,
              gz::math::eigen3::convert(frameDataUpperLink.pose));
    EXPECT_EQ(initialLowerLinkPose,
              gz::math::eigen3::convert(frameDataLowerLink.pose));

    // detach lower joint
    lowerJoint->Detach();
    // EXPECT_EQ("FreeJoint", dartLowerLink->getParentJoint()->getType());
    EXPECT_NE(nullptr, lowerJoint->CastToFreeJoint());
    EXPECT_EQ(nullptr, lowerJoint->CastToRevoluteJoint());

    // Detach() can be called again though it has no effect
    lowerJoint->Detach();
    // EXPECT_EQ("FreeJoint", dartLowerLink->getParentJoint()->getType());
    EXPECT_NE(nullptr, lowerJoint->CastToFreeJoint());
    EXPECT_EQ(nullptr, lowerJoint->CastToRevoluteJoint());

    frameDataUpperLink = upperLink->FrameDataRelativeToWorld();
    frameDataLowerLink = lowerLink->FrameDataRelativeToWorld();

    EXPECT_EQ(initialUpperLinkPose,
              gz::math::eigen3::convert(frameDataUpperLink.pose));
    EXPECT_EQ(initialLowerLinkPose,
              gz::math::eigen3::convert(frameDataLowerLink.pose));

    gz::physics::ForwardStep::Output output;
    gz::physics::ForwardStep::State state;
    gz::physics::ForwardStep::Input input;

    const std::size_t numSteps = 10;
    for (std::size_t i = 0; i < numSteps; ++i)
    {
      // step forward and expect lower link to fall
      world->Step(output, state, input);

      // expect upper link to rotate
      EXPECT_LT(upperJoint->GetVelocity(0), 0.0);

      frameDataLowerLink = lowerLink->FrameDataRelativeToWorld();

      // expect lower link to fall down without rotating
      gz::math::Vector3d lowerLinkLinearVelocity =
        gz::math::eigen3::convert(frameDataLowerLink.linearVelocity);
      EXPECT_NEAR(0.0, lowerLinkLinearVelocity.X(), 1e-10);
      EXPECT_NEAR(0.0, lowerLinkLinearVelocity.Y(), 1e-10);
      EXPECT_GT(0.0, lowerLinkLinearVelocity.Z());
      gz::math::Vector3d lowerLinkAngularVelocity =
        gz::math::eigen3::convert(frameDataLowerLink.angularVelocity);
      EXPECT_EQ(gz::math::Vector3d::Zero, lowerLinkAngularVelocity);
    }

    frameDataUpperLink = upperLink->FrameDataRelativeToWorld();

    // now detach the upper joint too, and ensure that velocity is preserved
    gz::math::Pose3d upperLinkPose =
        gz::math::eigen3::convert(frameDataUpperLink.pose);
    gz::math::Vector3d upperLinkLinearVelocity =
        gz::math::eigen3::convert(frameDataUpperLink.linearVelocity);
    gz::math::Vector3d upperLinkAngularVelocity =
        gz::math::eigen3::convert(frameDataUpperLink.angularVelocity);
    // sanity check on velocity values
    EXPECT_LT(1e-5, upperLinkLinearVelocity.Z());
    EXPECT_GT(-0.03, upperLinkAngularVelocity.X());
    EXPECT_NEAR(0.0, upperLinkLinearVelocity.X(), 1e-6);
    EXPECT_NEAR(0.0, upperLinkLinearVelocity.Y(), 1e-6);
    EXPECT_NEAR(0.0, upperLinkAngularVelocity.Y(), 1e-6);
    EXPECT_NEAR(0.0, upperLinkAngularVelocity.Z(), 1e-6);

    upperJoint->Detach();
    // EXPECT_EQ("FreeJoint", dartUpperLink->getParentJoint()->getType());
    EXPECT_NE(nullptr, upperJoint->CastToFreeJoint());
    EXPECT_EQ(nullptr, upperJoint->CastToRevoluteJoint());

    EXPECT_EQ(upperLinkPose,
        gz::math::eigen3::convert(frameDataUpperLink.pose));
    EXPECT_EQ(upperLinkLinearVelocity,
        gz::math::eigen3::convert(frameDataUpperLink.linearVelocity));
    EXPECT_EQ(upperLinkAngularVelocity,
       gz::math::eigen3::convert(frameDataUpperLink.angularVelocity));
  }
}

struct JointFeatureAttachDetachList : gz::physics::FeatureList<
    gz::physics::AttachFixedJointFeature,
    gz::physics::DetachJointFeature,
    gz::physics::ForwardStep,
    gz::physics::GetBasicJointProperties,
    gz::physics::GetBasicJointState,
    gz::physics::GetEngineInfo,
    gz::physics::GetJointFromModel,
    gz::physics::GetLinkFromModel,
    gz::physics::GetModelFromWorld,
    gz::physics::LinkFrameSemantics,
    gz::physics::SetBasicJointState,
    gz::physics::SetJointTransformFromParentFeature,
    gz::physics::SetJointVelocityCommandFeature,
    gz::physics::sdf::ConstructSdfModel,
    gz::physics::sdf::ConstructSdfWorld
> { };

template <class T>
class JointFeaturesAttachDetachTest :
  public JointFeaturesTest<T>{};
using JointFeaturesAttachDetachTestTypes =
  ::testing::Types<JointFeatureAttachDetachList>;
TYPED_TEST_SUITE(JointFeaturesAttachDetachTest,
                 JointFeaturesAttachDetachTestTypes);
/////////////////////////////////////////////////
// Attach a fixed joint between links that belong to different models
TYPED_TEST(JointFeaturesAttachDetachTest, JointAttachDetach)
{
  for (const std::string &name : this->pluginNames)
  {
    std::cout << "Testing plugin: " << name << std::endl;
    gz::plugin::PluginPtr plugin = this->loader.Instantiate(name);

    auto engine = gz::physics::RequestEngine3d<JointFeatureAttachDetachList>::From(plugin);
    ASSERT_NE(nullptr, engine);

    sdf::Root root;
    const sdf::Errors errors = root.Load(gz::common::joinPaths(TEST_WORLD_DIR, "joint_across_models.sdf"));
    ASSERT_TRUE(errors.empty()) << errors.front();

    auto world = engine->ConstructWorld(*root.WorldByIndex(0));

    const std::string modelName1{"M1"};
    const std::string modelName2{"M2"};
    const std::string bodyName{"body"};

    auto model1 = world->GetModel(modelName1);
    auto model2 = world->GetModel(modelName2);
    auto model1Body = model1->GetLink(bodyName);
    auto model2Body = model2->GetLink(bodyName);

    auto frameDataModel1Body = model1Body->FrameDataRelativeToWorld();
    auto frameDataModel2Body = model2Body->FrameDataRelativeToWorld();

    const gz::math::Pose3d initialModel1Pose(0, 0, 0.25, 0, 0, 0);
    const gz::math::Pose3d initialModel2Pose(0, 0, 3.0, 0, 0, 0);

    EXPECT_EQ(initialModel1Pose,
              gz::math::eigen3::convert(frameDataModel1Body.pose));
    EXPECT_EQ(initialModel2Pose,
              gz::math::eigen3::convert(frameDataModel2Body.pose));

    gz::physics::ForwardStep::Output output;
    gz::physics::ForwardStep::State state;
    gz::physics::ForwardStep::Input input;

    const std::size_t numSteps = 100;
    for (std::size_t i = 0; i < numSteps; ++i)
    {
      world->Step(output, state, input);

      frameDataModel1Body = model1Body->FrameDataRelativeToWorld();
      frameDataModel2Body = model2Body->FrameDataRelativeToWorld();
      // Expect the model1 to stay at rest (since it's on the ground) and model2
      // to start falling
      gz::math::Vector3d body1LinearVelocity =
          gz::math::eigen3::convert(frameDataModel1Body.linearVelocity);
      gz::math::Vector3d body2LinearVelocity =
        gz::math::eigen3::convert(frameDataModel2Body.linearVelocity);
      EXPECT_NEAR(0.0, body1LinearVelocity.Z(), 1e-2);
      // Negative z velocity
      EXPECT_GT(0.0, body2LinearVelocity.Z());
    }

    const auto poseParent = frameDataModel1Body.pose;
    const auto poseChild = frameDataModel2Body.pose;
    auto poseParentChild = poseParent.inverse() * poseChild;

    auto fixedJoint = model2Body->AttachFixedJoint(model1Body);

    // AttachFixedJoint snaps the child body to the origin of the parent, so we
    // set a transform on the joint to keep the transform between the two bodies
    // the same as it was before they were attached
    fixedJoint->SetTransformFromParent(poseParentChild);

    // The name of the link obtained using the gz-physics API should remain the
    // same even though AttachFixedJoint renames the associated BodyNode.
    EXPECT_EQ(bodyName, model2Body->GetName());

    for (std::size_t i = 0; i < numSteps; ++i)
    {
      world->Step(output, state, input);
    }

    for (std::size_t i = 0; i < numSteps; ++i)
    {
      world->Step(output, state, input);

      frameDataModel1Body = model1Body->FrameDataRelativeToWorld();
      frameDataModel2Body = model2Body->FrameDataRelativeToWorld();
      // Expect the model1 to remain at rest and model2
      // to stop moving
      gz::math::Vector3d body1LinearVelocity =
        gz::math::eigen3::convert(frameDataModel1Body.linearVelocity);
      gz::math::Vector3d body2LinearVelocity =
        gz::math::eigen3::convert(frameDataModel2Body.linearVelocity);
      EXPECT_NEAR(0.0, body1LinearVelocity.Z(), 1e-3);
      EXPECT_NEAR(0.0, body2LinearVelocity.Z(), 1e-3);
    }

    // now detach joint and expect model2 to start moving again
    fixedJoint->Detach();

    // The name of the link obtained using the gz-physics API should remain the
    // same even though Detach renames the associated BodyNode.
    EXPECT_EQ(bodyName, model2Body->GetName());

    for (std::size_t i = 0; i < numSteps; ++i)
    {
      world->Step(output, state, input);

      frameDataModel1Body = model1Body->FrameDataRelativeToWorld();
      frameDataModel2Body = model2Body->FrameDataRelativeToWorld();

      // Expect the model1 to remain at rest and model2
      // to start moving again
      gz::math::Vector3d body1LinearVelocity =
        gz::math::eigen3::convert(frameDataModel1Body.linearVelocity);
      gz::math::Vector3d body2LinearVelocity =
        gz::math::eigen3::convert(frameDataModel2Body.linearVelocity);
      EXPECT_NEAR(0.0, body1LinearVelocity.Z(), 1e-2);
      // Negative z velocity
      EXPECT_GT(0.0, body2LinearVelocity.Z());
    }

    // After a while, body2 should reach the ground and come to a stop
    for (std::size_t i = 0; i < 1000; ++i)
    {
      world->Step(output, state, input);
    }
    frameDataModel2Body = model2Body->FrameDataRelativeToWorld();

    EXPECT_NEAR(0.0, frameDataModel2Body.linearVelocity.z(), 1e-3);
  }
}

////////////////////////////////////////////////
// Essentially what happens is there are two floating boxes and a box in the
// middle that's resting. We start the system out by creating the two
// fixed joints between the boxes resting on the big box. The middle box will
// now have two parents. However there should be no movement as the middle box
// will be holding the other two boxes that are floating in mid air. We run
// this for 100 steps to make sure that there is no movement. This is because
// the middle box is holding on to the two side boxes. Then we release the
// joints the two boxes should fall away.
TYPED_TEST(JointFeaturesAttachDetachTest, JointAttachMultiple)
{
  for (const std::string &name : this->pluginNames)
  {
    std::cout << "Testing plugin: " << name << std::endl;
    gz::plugin::PluginPtr plugin = this->loader.Instantiate(name);

    auto engine = gz::physics::RequestEngine3d<JointFeatureAttachDetachList>::From(plugin);
    ASSERT_NE(nullptr, engine);

    sdf::Root root;
    const sdf::Errors errors = root.Load(gz::common::joinPaths(TEST_WORLD_DIR, "joint_constraint.sdf"));
    ASSERT_TRUE(errors.empty()) << errors.front();

    auto world = engine->ConstructWorld(*root.WorldByIndex(0));

    // M1 and M3 are floating boxes
    const std::string modelName1{"M1"};
    const std::string modelName2{"M2"};
    const std::string modelName3{"M3"};
    const std::string bodyName{"link"};

    auto model1 = world->GetModel(modelName1);
    auto model2 = world->GetModel(modelName2);
    auto model3 = world->GetModel(modelName3);

    auto model1Body = model1->GetLink(bodyName);
    auto model2Body = model2->GetLink(bodyName);
    auto model3Body = model3->GetLink(bodyName);

    const gz::math::Pose3d initialModel1Pose(0, -0.2, 0.45, 0, 0, 0);
    const gz::math::Pose3d initialModel2Pose(0, 0.2, 0.45, 0, 0, 0);
    const gz::math::Pose3d initialModel3Pose(0, 0.6, 0.45, 0, 0, 0);

    auto frameDataModel1Body = model1Body->FrameDataRelativeToWorld();
    auto frameDataModel2Body = model2Body->FrameDataRelativeToWorld();
    auto frameDataModel3Body = model3Body->FrameDataRelativeToWorld();

    EXPECT_EQ(initialModel1Pose,
              gz::math::eigen3::convert(frameDataModel1Body.pose));
    EXPECT_EQ(initialModel2Pose,
              gz::math::eigen3::convert(frameDataModel2Body.pose));
    EXPECT_EQ(initialModel3Pose,
              gz::math::eigen3::convert(frameDataModel3Body.pose));

    gz::physics::ForwardStep::Output output;
    gz::physics::ForwardStep::State state;
    gz::physics::ForwardStep::Input input;
    // 1 ms time step
    const double dt = 0.001;
    auto dur = std::chrono::duration<double>(dt);
    input.Get<std::chrono::steady_clock::duration>() =
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(dur);

    // Create the first joint. This should be a normal fixed joint.
    const auto poseParent1 = frameDataModel1Body.pose;
    const auto poseChild1 = frameDataModel2Body.pose;
    auto poseParentChild1 = poseParent1.inverse() * poseChild1;
    auto fixedJoint1 = model2Body->AttachFixedJoint(model1Body);
    fixedJoint1->SetTransformFromParent(poseParentChild1);

    frameDataModel1Body = model1Body->FrameDataRelativeToWorld();
    frameDataModel2Body = model2Body->FrameDataRelativeToWorld();
    frameDataModel3Body = model3Body->FrameDataRelativeToWorld();

    EXPECT_EQ(initialModel1Pose,
              gz::math::eigen3::convert(frameDataModel1Body.pose));
    EXPECT_EQ(initialModel2Pose,
              gz::math::eigen3::convert(frameDataModel2Body.pose));
    EXPECT_EQ(initialModel3Pose,
              gz::math::eigen3::convert(frameDataModel3Body.pose));

    // Create the second joint. This should be a WeldJoint constraint
    const auto poseParent2 = frameDataModel3Body.pose;
    const auto poseChild2 = frameDataModel2Body.pose;
    auto poseParentChild2 = poseParent2.inverse() * poseChild2;
    auto fixedJoint2 = model2Body->AttachFixedJoint(model3Body);
    fixedJoint2->SetTransformFromParent(poseParentChild2);

    frameDataModel1Body = model1Body->FrameDataRelativeToWorld();
    frameDataModel2Body = model2Body->FrameDataRelativeToWorld();
    frameDataModel3Body = model3Body->FrameDataRelativeToWorld();

    EXPECT_EQ(initialModel1Pose,
              gz::math::eigen3::convert(frameDataModel1Body.pose));
    EXPECT_EQ(initialModel2Pose,
              gz::math::eigen3::convert(frameDataModel2Body.pose));
    EXPECT_EQ(initialModel3Pose,
              gz::math::eigen3::convert(frameDataModel3Body.pose));

    const std::size_t numSteps = 100;
    for (std::size_t i = 0; i < numSteps; ++i)
    {
      world->Step(output, state, input);

      frameDataModel1Body = model1Body->FrameDataRelativeToWorld();
      frameDataModel2Body = model2Body->FrameDataRelativeToWorld();
      frameDataModel3Body = model3Body->FrameDataRelativeToWorld();

      // Expect all the bodies to be at rest.
      // (since they're held in place by the joints)
      gz::math::Vector3d body1LinearVelocity =
          gz::math::eigen3::convert(frameDataModel1Body.linearVelocity);
      gz::math::Vector3d body2LinearVelocity =
          gz::math::eigen3::convert(frameDataModel2Body.linearVelocity);
      gz::math::Vector3d body3LinearVelocity =
          gz::math::eigen3::convert(frameDataModel3Body.linearVelocity);
      EXPECT_NEAR(0.0, body1LinearVelocity.Z(), 1e-1);
      EXPECT_NEAR(0.0, body2LinearVelocity.Z(), 1e-1);
      EXPECT_NEAR(0.0, body3LinearVelocity.Z(), 1e-1);
    }

    // Detach the joints. M1 and M3 should fall as there is now nothing stopping
    // them from falling.
    fixedJoint1->Detach();
    fixedJoint2->Detach();

    for (std::size_t i = 0; i < numSteps; ++i)
    {
      world->Step(output, state, input);

      frameDataModel1Body = model1Body->FrameDataRelativeToWorld();
      frameDataModel2Body = model2Body->FrameDataRelativeToWorld();
      frameDataModel3Body = model3Body->FrameDataRelativeToWorld();

      // Expect the middle box to be still as it is already at rest.
      // Expect the two side boxes to fall away.
      gz::math::Vector3d body1LinearVelocity =
          gz::math::eigen3::convert(frameDataModel1Body.linearVelocity);
      gz::math::Vector3d body2LinearVelocity =
          gz::math::eigen3::convert(frameDataModel2Body.linearVelocity);
      gz::math::Vector3d body3LinearVelocity =
          gz::math::eigen3::convert(frameDataModel3Body.linearVelocity);
      EXPECT_NEAR(dt * (i + 1) * -9.81, body1LinearVelocity.Z(), 1e-2);
      EXPECT_NEAR(0.0, body2LinearVelocity.Z(), 1e-2);
      EXPECT_NEAR(dt * (i + 1) * -9.81, body3LinearVelocity.Z(), 1e-2);
    }
  }
}

/////////////////////////////////////////////////
// Expectations on number of links before/after attach/detach
TYPED_TEST(JointFeaturesAttachDetachTest, LinkCountsInJointAttachDetach)
{
  for (const std::string &name : this->pluginNames)
  {
    std::cout << "Testing plugin: " << name << std::endl;
    gz::plugin::PluginPtr plugin = this->loader.Instantiate(name);

    auto engine = gz::physics::RequestEngine3d<JointFeatureAttachDetachList>::From(plugin);
    ASSERT_NE(nullptr, engine);

    sdf::Root root;
    const sdf::Errors errors = root.Load(gz::common::joinPaths(TEST_WORLD_DIR, "joint_across_models.sdf"));
    ASSERT_TRUE(errors.empty()) << errors.front();

    auto world = engine->ConstructWorld(*root.WorldByIndex(0));

    const std::string modelName1{"M1"};
    const std::string modelName2{"M2"};
    const std::string bodyName{"body"};

    auto model1 = world->GetModel(modelName1);
    auto model2 = world->GetModel(modelName2);
    auto model1Body = model1->GetLink(bodyName);
    auto model2Body = model2->GetLink(bodyName);

    // Before attaching we expect each model to have 1 link
    EXPECT_EQ(1u, model1->GetLinkCount());
    EXPECT_EQ(1u, model2->GetLinkCount());

    auto fixedJoint = model2Body->AttachFixedJoint(model1Body);

    // After attaching we expect each model to have 1 link
    EXPECT_EQ(1u, model1->GetLinkCount());
    EXPECT_EQ(1u, model2->GetLinkCount());

    // now detach joint and expect model2 to start moving again
    fixedJoint->Detach();
    // After detaching we expect each model to have 1 link
    EXPECT_EQ(1u, model1->GetLinkCount());
    EXPECT_EQ(1u, model2->GetLinkCount());

    // Test that a model with the same name as a link doesn't cause problems
    const std::string modelName3{"body"};
    auto model3 = world->GetModel(modelName3);
    EXPECT_EQ(1u, model3->GetLinkCount());

    auto model3Body = model3->GetLink(bodyName);
    auto fixedJoint2 = model3Body->AttachFixedJoint(model2Body);
    EXPECT_EQ(1u, model2->GetLinkCount());
    fixedJoint2->Detach();
    // After detaching we expect each model to have 1 link
    EXPECT_EQ(1u, model2->GetLinkCount());
    EXPECT_EQ(1u, model3->GetLinkCount());
  }
}

/////////////////////////////////////////////////
// Attach a fixed joint between links that belong to different models where one
// of the models is created after a step is called
TYPED_TEST(JointFeaturesAttachDetachTest, JointAttachDetachSpawnedModel)
{
  std::string model1Str = R"(
  <sdf version="1.6">
    <model name="M1">
      <pose>0 0 0.1 0 0 0</pose>
      <link name="body">
        <collision name="coll_box">
          <geometry>
            <box>
              <size>0.2 0.2 0.2</size>
            </box>
          </geometry>
        </collision>
      </link>
    </model>
  </sdf>)";

  std::string model2Str = R"(
  <sdf version="1.6">
    <model name="M2">
      <pose>1 0 0.1 0 0 0</pose>
      <link name="chassis">
        <collision name="coll_sphere">
          <geometry>
            <sphere>
              <radius>0.1</radius>
            </sphere>
          </geometry>
        </collision>
      </link>
    </model>
  </sdf>)";

  gz::physics::ForwardStep::Output output;
  gz::physics::ForwardStep::State state;
  gz::physics::ForwardStep::Input input;

  gz::physics::World3dPtr<JointFeatureAttachDetachList> world;
  {
    for (const std::string &name : this->pluginNames)
    {
      std::cout << "Testing plugin: " << name << std::endl;
      gz::plugin::PluginPtr plugin = this->loader.Instantiate(name);

      auto engine = gz::physics::RequestEngine3d<JointFeatureAttachDetachList>::From(plugin);
      ASSERT_NE(nullptr, engine);

      sdf::Root root;
      const sdf::Errors errors = root.Load(gz::common::joinPaths(TEST_WORLD_DIR, "ground.sdf"));
      ASSERT_TRUE(errors.empty()) << errors.front();

      world = engine->ConstructWorld(*root.WorldByIndex(0));
      ASSERT_NE(nullptr, world);
    }
  }

  {
    sdf::Root root;
    sdf::Errors errors = root.LoadSdfString(model1Str);
    ASSERT_TRUE(errors.empty()) << errors.front();
    ASSERT_NE(nullptr, root.Model());
    world->ConstructModel(*root.Model());
  }

  world->Step(output, state, input);

  {
    sdf::Root root;
    sdf::Errors errors = root.LoadSdfString(model2Str);
    ASSERT_TRUE(errors.empty()) << errors.front();
    ASSERT_NE(nullptr, root.Model());
    world->ConstructModel(*root.Model());
  }

  const std::string modelName1{"M1"};
  const std::string modelName2{"M2"};
  const std::string bodyName1{"body"};
  const std::string bodyName2{"chassis"};

  auto model1 = world->GetModel(modelName1);
  auto model2 = world->GetModel(modelName2);
  auto model1Body = model1->GetLink(bodyName1);
  auto model2Body = model2->GetLink(bodyName2);

  auto frameDataModel1Body = model1Body->FrameDataRelativeToWorld();
  auto frameDataModel2Body = model2Body->FrameDataRelativeToWorld();

  const auto poseParent = frameDataModel1Body.pose;
  const auto poseChild = frameDataModel2Body.pose;

  // Before gz-physics PR #31, uncommenting the following `step` call makes
  // this test pass, but commenting it out makes it fail.
  // world->Step(output, state, input);
  auto fixedJoint = model2Body->AttachFixedJoint(model1Body);

  // Pose of child relative to parent
  auto poseParentChild = poseParent.inverse() * poseChild;

  // We let the joint be at the origin of the child link.
  fixedJoint->SetTransformFromParent(poseParentChild);

  const std::size_t numSteps = 100;

  for (std::size_t i = 0; i < numSteps; ++i)
  {
    world->Step(output, state, input);
  }

  frameDataModel1Body = model1Body->FrameDataRelativeToWorld();
  frameDataModel2Body = model2Body->FrameDataRelativeToWorld();

  // Expect both bodies to hit the ground and stop
  EXPECT_NEAR(0.0, frameDataModel1Body.linearVelocity.z(), 1e-3);
  EXPECT_NEAR(0.0, frameDataModel2Body.linearVelocity.z(), 1e-3);

  fixedJoint->Detach();

  for (std::size_t i = 0; i < numSteps; ++i)
  {
    world->Step(output, state, input);
  }

  frameDataModel1Body = model1Body->FrameDataRelativeToWorld();
  frameDataModel2Body = model2Body->FrameDataRelativeToWorld();

  // Expect both bodies to remain in contact with the ground with zero velocity.
  EXPECT_NEAR(0.0, frameDataModel1Body.linearVelocity.z(), 1e-3);
  EXPECT_NEAR(0.0, frameDataModel2Body.linearVelocity.z(), 1e-3);
}

template <class T>
class JointTransmittedWrenchFixture : public JointFeaturesTest<T>
{
  public: using WorldPtr = gz::physics::World3dPtr<T>;
  public: using ModelPtr = gz::physics::Model3dPtr<T>;
  public: using JointPtr = gz::physics::Joint3dPtr<T>;
  public: using LinkPtr = gz::physics::Link3dPtr<T>;

  protected: void SetUp() override
  {
    JointFeaturesTest<T>::SetUp();
    for (const std::string &name : this->pluginNames)
    {
      std::cout << "Testing plugin: " << name << std::endl;
      gz::plugin::PluginPtr plugin = this->loader.Instantiate(name);

      auto engine = gz::physics::RequestEngine3d<T>::From(plugin);
      ASSERT_NE(nullptr, engine);

      sdf::Root root;
      const sdf::Errors errors = root.Load(gz::common::joinPaths(TEST_WORLD_DIR, "pendulum_joint_wrench.sdf"));
      ASSERT_TRUE(errors.empty()) << errors.front();

      this->world = engine->ConstructWorld(*root.WorldByIndex(0));
      ASSERT_NE(nullptr, this->world);

      this->model = this->world->GetModel("pendulum");
      ASSERT_NE(nullptr, this->model);
      this->motorJoint = this->model->GetJoint("motor_joint");
      ASSERT_NE(nullptr, this->motorJoint);
      this->sensorJoint = this->model->GetJoint("sensor_joint");
      ASSERT_NE(nullptr, this->sensorJoint);
      this->armLink = this->model->GetLink("arm");
      ASSERT_NE(nullptr, this->armLink);
    }
  }

  public: void Step(int _iters)
  {
    for (int i = 0; i < _iters; ++i)
    {
      this->world->Step(this->output, this->state, this->input);
    }
  }

  public: gz::physics::ForwardStep::Output output;
  public: gz::physics::ForwardStep::State state;
  public: gz::physics::ForwardStep::Input input;
  public: WorldPtr world;
  public: ModelPtr model;
  public: JointPtr motorJoint;
  public: JointPtr sensorJoint;
  public: LinkPtr armLink;

  // From SDFormat file
  static constexpr double kGravity = 9.8;
  static constexpr double kArmLinkMass = 6.0;
  static constexpr double kSensorLinkMass = 0.4;
  // MOI in the z-axis
  static constexpr double kSensorLinkMOI = 0.02;
  static constexpr double kArmLength = 1.0;
};

struct JointTransmittedWrenchFeatureList : gz::physics::FeatureList<
    gz::physics::ForwardStep,
    gz::physics::FreeGroupFrameSemantics,
    gz::physics::GetBasicJointState,
    gz::physics::GetEngineInfo,
    gz::physics::GetJointFromModel,
    gz::physics::GetJointTransmittedWrench,
    gz::physics::GetLinkFromModel,
    gz::physics::GetModelFromWorld,
    gz::physics::LinkFrameSemantics,
    gz::physics::SetBasicJointState,
    gz::physics::SetFreeGroupWorldPose,
    gz::physics::sdf::ConstructSdfWorld
> { };

using JointTransmittedWrenchFeaturesTestTypes =
  ::testing::Types<JointTransmittedWrenchFeatureList>;
TYPED_TEST_SUITE(JointTransmittedWrenchFixture,
                 JointTransmittedWrenchFeaturesTestTypes);

TYPED_TEST(JointTransmittedWrenchFixture, PendulumAtZeroAngle)
{
  // Run a few steps for the constraint forces to stabilize
  this->Step(10);

  // Test wrench expressed in different frames
  {
    auto wrenchAtMotorJoint = this->motorJoint->GetTransmittedWrench();
    gz::physics::Wrench3d expectedWrenchAtMotorJoint{
        gz::physics::Vector3d::Zero(),
        {-this->kGravity * (this->kArmLinkMass + this->kSensorLinkMass), 0, 0}};

    EXPECT_TRUE(
       gz::physics::test::Equal(expectedWrenchAtMotorJoint, wrenchAtMotorJoint, 1e-4));
  }
  {
    auto wrenchAtMotorJointInWorld = this->motorJoint->GetTransmittedWrench(
        this->motorJoint->GetFrameID(), gz::physics::FrameID::World());
    gz::physics::Wrench3d expectedWrenchAtMotorJointInWorld{
        gz::physics::Vector3d::Zero(),
        {0, 0, this->kGravity * (this->kArmLinkMass + this->kSensorLinkMass)}};

    EXPECT_TRUE(gz::physics::test::Equal(expectedWrenchAtMotorJointInWorld,
                                         wrenchAtMotorJointInWorld, 1e-4));
  }
  {
    auto wrenchAtMotorJointInArm = this->motorJoint->GetTransmittedWrench(
        this->armLink->GetFrameID(), this->armLink->GetFrameID());
    // The arm frame is rotated by 90° in the Y-axis of the joint frame.
    gz::physics::Wrench3d expectedWrenchAtMotorJointInArm{
        gz::physics::Vector3d::Zero(),
        {0, 0, this->kGravity * (this->kArmLinkMass + this->kSensorLinkMass)}};

    EXPECT_TRUE(gz::physics::test::Equal(expectedWrenchAtMotorJointInArm,
                                         wrenchAtMotorJointInArm, 1e-4));
  }
}

TYPED_TEST(JointTransmittedWrenchFixture, PendulumInMotion)
{
  // Start pendulum at 90° (parallel to the ground) and stop at about 40°
  // so that we have non-trivial test expectations.
  this->motorJoint->SetPosition(0, GZ_DTOR(90.0));
  this->Step(350);

  // Given the position (θ), velocity (ω), and acceleration (α) of the joint
  // and distance from the joint to the COM (r), the reaction forces in
  // the tangent direction (Ft) and normal direction (Fn) are given by:
  //
  // Ft =  m * α * r + (m * g * sin(θ)) = m * (α * r + g * sin(θ))
  // Fn = -m * ω² * r - (m * g * cos(θ)) = -m * (ω² * r +  g * cos(θ))
  {
    const double theta = this->motorJoint->GetPosition(0);
    const double omega = this->motorJoint->GetVelocity(0);
    // In order to get the math to work out, we need to use the joint
    // acceleration and transmitted wrench from the current time step with the
    // joint position and velocity from the previous time step. That is, we need
    // the position and velocity before they are integrated.
    this->Step(1);
    const double alpha = this->motorJoint->GetAcceleration(0);

    auto wrenchAtMotorJointInJoint = this->motorJoint->GetTransmittedWrench();

    const double armTangentForce =
        this->kArmLinkMass * ((alpha * this->kArmLength / 2.0) + (this->kGravity * sin(theta)));

    const double motorLinkTangentForce =
        this->kSensorLinkMass * this->kGravity * sin(theta);

    const double armNormalForce =
        -this->kArmLinkMass *
        ((std::pow(omega, 2) * this->kArmLength / 2.0) + (this->kGravity * cos(theta)));

    const double motorLinkNormalForce =
        -this->kSensorLinkMass * this->kGravity * cos(theta);

    const double tangentForce = armTangentForce + motorLinkTangentForce;
    const double normalForce = armNormalForce + motorLinkNormalForce;

    // The orientation of the joint frame is such that the normal force is
    // parallel to the x-axis and the tangent force is parallel to the y-axis.
    gz::physics::Wrench3d expectedWrenchAtMotorJointInJoint{
        gz::physics::Vector3d::Zero(), {normalForce, tangentForce, 0}};

    EXPECT_TRUE(gz::physics::test::Equal(expectedWrenchAtMotorJointInJoint,
                                         wrenchAtMotorJointInJoint, 1e-4));
  }

  // Test Wrench expressed in different frames
  {
    auto wrenchAtMotorJointInJoint = this->motorJoint->GetTransmittedWrench();
    // This is just a rotation of the wrench to be expressed in the world's
    // coordinate frame
    auto wrenchAtMotorJointInWorld = this->motorJoint->GetTransmittedWrench(
        this->motorJoint->GetFrameID(), gz::physics::FrameID::World());
    // The joint frame is rotated by 90° along the world's y-axis
    Eigen::Quaterniond R_WJ =
        Eigen::AngleAxisd(GZ_PI_2, Eigen::Vector3d(0, 1, 0)) *
        Eigen::AngleAxisd(this->motorJoint->GetPosition(0),
                          Eigen::Vector3d(0, 0, 1));

    gz::physics::Wrench3d expectedWrenchAtMotorJointInWorld{
        gz::physics::Vector3d::Zero(), R_WJ * wrenchAtMotorJointInJoint.force};
    EXPECT_TRUE(gz::physics::test::Equal(expectedWrenchAtMotorJointInWorld,
                            wrenchAtMotorJointInWorld, 1e-4));

    // This moves the point of application and changes the coordinate frame
    gz::physics::Wrench3d wrenchAtArmInArm = this->motorJoint->GetTransmittedWrench(
        this->armLink->GetFrameID(), this->armLink->GetFrameID());

    // Notation: arm link (A), joint (J)
    Eigen::Isometry3d X_AJ;
    // Pose of joint (J) in arm link (A) as specified in the SDFormat file.
    X_AJ = Eigen::AngleAxisd(GZ_PI_2, Eigen::Vector3d(0, 1, 0));
    X_AJ.translation() = gz::physics::Vector3d(0, 0, this->kArmLength / 2.0);
    gz::physics::Wrench3d expectedWrenchAtArmInArm;

    expectedWrenchAtArmInArm.force =
        X_AJ.linear() * wrenchAtMotorJointInJoint.force;

    expectedWrenchAtArmInArm.torque =
        X_AJ.linear() * wrenchAtMotorJointInJoint.torque +
        X_AJ.translation().cross(expectedWrenchAtArmInArm.force);

    EXPECT_TRUE(gz::physics::test::Equal(expectedWrenchAtArmInArm, wrenchAtArmInArm, 1e-4));
  }
}

TYPED_TEST(JointTransmittedWrenchFixture, ValidateWrenchWithSecondaryJoint)
{
  // Start pendulum at 90° (parallel to the ground) and stop at about 40°
  // so that we have non-trivial test expectations.
  this->motorJoint->SetPosition(0, GZ_DTOR(90.0));
  this->Step(350);
  const double theta = this->motorJoint->GetPosition(0);
  // In order to get the math to work out, we need to use the joint
  // acceleration and transmitted wrench from the current time step with the
  // joint position and velocity from the previous time step. That is, we need
  // the position and velocity before they are integrated.
  this->Step(1);
  const double alpha = this->motorJoint->GetAcceleration(0);

  auto wrenchAtMotorJointInJoint = this->motorJoint->GetTransmittedWrench();
  auto wrenchAtSensorInSensor = this->sensorJoint->GetTransmittedWrench();

  // Since sensor_link has moment of inertia, the fixed joint will transmit a
  // torque necessary to rotate the sensor. This is not detected by the motor
  // joint because no force is transmitted along the revolute axis. On the
  // other hand, the mass of sensor_link will contribute to the constraint
  // forces on the motor joint, but these won't be detected by the sensor
  // joint.
  gz::physics::Vector3d expectedTorqueDiff{0, 0, this->kSensorLinkMOI * alpha};
  gz::physics::Vector3d expectedForceDiff{-this->kSensorLinkMass * this->kGravity * cos(theta),
                             this->kSensorLinkMass * this->kGravity * sin(theta), 0};

  gz::physics::Vector3d torqueDiff =
      wrenchAtMotorJointInJoint.torque - wrenchAtSensorInSensor.torque;
  gz::physics::Vector3d forceDiff =
      wrenchAtMotorJointInJoint.force - wrenchAtSensorInSensor.force;
  EXPECT_TRUE(gz::physics::test::Equal(expectedTorqueDiff, torqueDiff, 1e-4));
  EXPECT_TRUE(gz::physics::test::Equal(expectedForceDiff, forceDiff, 1e-4));
}

TYPED_TEST(JointTransmittedWrenchFixture, JointLosses)
{
  // // Get DART joint pointer to set joint friction, damping, etc.
  // auto dartWorld = this->world->GetDartsimWorld();
  // ASSERT_NE(nullptr, dartWorld);
  // auto dartModel = dartWorld->getSkeleton(this->model->GetIndex());
  // ASSERT_NE(nullptr, dartModel);
  // auto dartJoint = dartModel->getJoint(this->motorJoint->GetIndex());
  // ASSERT_NE(nullptr, dartJoint);
  //
  // // Joint friction
  // {
  //   this->motorJoint->SetPosition(0, GZ_DTOR(90.0));
  //   this->motorJoint->SetVelocity(0, 0);
  //   const double kFrictionCoef = 0.5;
  //   dartJoint->setCoulombFriction(0, kFrictionCoef);
  //   this->Step(10);
  //   auto wrenchAtMotorJointInJoint = this->motorJoint->GetTransmittedWrench();
  //   EXPECT_NEAR(kFrictionCoef, wrenchAtMotorJointInJoint.torque.z(), 1e-4);
  //   dartJoint->setCoulombFriction(0, 0.0);
  // }
  //
  // // Joint damping
  // {
  //   this->motorJoint->SetPosition(0, GZ_DTOR(90.0));
  //   this->motorJoint->SetVelocity(0, 0);
  //   const double kDampingCoef = 0.2;
  //   dartJoint->setDampingCoefficient(0, kDampingCoef);
  //   this->Step(100);
  //   const double omega = this->motorJoint->GetVelocity(0);
  //   this->Step(1);
  //   auto wrenchAtMotorJointInJoint = this->motorJoint->GetTransmittedWrench();
  //   EXPECT_NEAR(-omega * kDampingCoef, wrenchAtMotorJointInJoint.torque.z(),
  //               1e-3);
  //   dartJoint->setDampingCoefficient(0, 0.0);
  // }
  //
  // // Joint stiffness
  // {
  //   // Note: By default, the spring reference position is 0.
  //   this->motorJoint->SetPosition(0, GZ_DTOR(30.0));
  //   this->motorJoint->SetVelocity(0, 0);
  //   const double kSpringStiffness = 0.7;
  //   dartJoint->setSpringStiffness(0, kSpringStiffness);
  //   this->Step(1);
  //   const double theta = this->motorJoint->GetPosition(0);
  //   this->Step(1);
  //   auto wrenchAtMotorJointInJoint = this->motorJoint->GetTransmittedWrench();
  //   EXPECT_NEAR(-theta * kSpringStiffness, wrenchAtMotorJointInJoint.torque.z(),
  //               1e-3);
  //   dartJoint->setSpringStiffness(0, 0.0);
  // }
}

// Check that the transmitted wrench is affected by contact forces
TYPED_TEST(JointTransmittedWrenchFixture, ContactForces)
{
  auto box = this->world->GetModel("box");
  ASSERT_NE(nullptr, box);
  auto boxFreeGroup = box->FindFreeGroup();
  ASSERT_NE(nullptr, boxFreeGroup);
  gz::physics::Pose3d X_WB(Eigen::Translation3d(0, 1, 1));
  boxFreeGroup->SetWorldPose(X_WB);

  this->motorJoint->SetPosition(0, GZ_DTOR(90.0));
  // After this many steps, the pendulum is in contact with the box
  this->Step(1000);
  const double theta = this->motorJoint->GetPosition(0);
  // Sanity check that the pendulum is at rest
  EXPECT_NEAR(0.0, this->motorJoint->GetVelocity(0), 1e-3);

  auto wrenchAtMotorJointInJoint = this->motorJoint->GetTransmittedWrench();

  // To compute the reaction forces, we consider the pivot on the contact point
  // between the pendulum and the box and the fact that the sum of moments about
  // the pivot is zero. We also note that all forces, including the reaction
  // forces, are in the vertical (world's z-axis) direction.
  //
  // Notation:
  // Fp_z: Reaction force at pendulum joint (pin) in the world's z-axis
  // M_b: Moment about the contact point between box and pendulum
  //
  // Fp_z = √(Fn² + Ft²) // Since all of the reaction force is in the world's
  // z-axis
  //
  // ∑M_b = 0 = -Fp_z * sin(θ) * (2*r) + m₁*g*sin(θ)*r + m₂*g*sin(θ)*(2*r)
  //
  // Fp_z = 0.5 * g * (m₁ + 2*m₂)
  //
  // We can then compute the tangential (Ft) and normal (Fn) components as
  //
  // Ft =  Fp_z * sin(θ)
  // Fn = -Fp_z * cos(θ)

  const double reactionForceAtP =
      0.5 * this->kGravity * (this->kArmLinkMass + 2 * this->kSensorLinkMass);

  gz::physics::Wrench3d expectedWrenchAtMotorJointInJoint{
      gz::physics::Vector3d::Zero(),
      {-reactionForceAtP * cos(theta), reactionForceAtP * sin(theta), 0}};

  EXPECT_TRUE(gz::physics::test::Equal(expectedWrenchAtMotorJointInJoint,
                                       wrenchAtMotorJointInJoint, 1e-4));
}

int main(int argc, char *argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  if (!JointFeaturesTest<JointFeatureList>::init(
       argc, argv))
    return -1;
  return RUN_ALL_TESTS();
}
