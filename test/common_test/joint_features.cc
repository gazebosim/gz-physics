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

#include <cstddef>
#include <iostream>
#include <string>
#include <gtest/gtest.h>

#include <chrono>

#include <gz/common/Console.hh>
#include <gz/plugin/Loader.hh>

#include <gz/math/Helpers.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/eigen3/Conversions.hh>

#include "test/TestLibLoader.hh"
#include "test/Utils.hh"
#include "Worlds.hh"

#include "gz/physics/FrameSemantics.hh"
#include <gz/physics/FindFeatures.hh>
#include <gz/physics/FixedJoint.hh>
#include <gz/physics/ForwardStep.hh>
#include <gz/physics/FreeGroup.hh>
#include <gz/physics/FreeJoint.hh>
#include <gz/physics/GetContacts.hh>
#include <gz/physics/GetEntities.hh>
#include <gz/physics/Joint.hh>
#include <gz/physics/RemoveEntities.hh>
#include <gz/physics/RequestEngine.hh>
#include <gz/physics/RevoluteJoint.hh>
#include <gz/physics/Shape.hh>
#include <gz/physics/World.hh>
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
    gz::physics::FindFreeGroupFeature,
    gz::physics::ForwardStep,
    gz::physics::GetBasicJointProperties,
    gz::physics::GetBasicJointState,
    gz::physics::GetContactsFromLastStepFeature,
    gz::physics::GetEngineInfo,
    gz::physics::GetJointFromModel,
    gz::physics::GetLinkFromModel,
    gz::physics::GetModelFromWorld,
    gz::physics::LinkFrameSemantics,
    gz::physics::SetBasicJointState,
    gz::physics::SetFreeGroupWorldPose,
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
    const sdf::Errors errors = root.Load(common_test::worlds::kTestWorld);
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

    // Check that invalid force commands don't cause collisions to fail
    for (std::size_t i = 0; i < 1000; ++i)
    {
      // Silence console spam
      gz::common::Console::SetVerbosity(0);
      joint->SetForce(0, std::numeric_limits<double>::quiet_NaN());
      gz::common::Console::SetVerbosity(4);
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

    for (std::size_t i = 0; i < numSteps; ++i)
    {
      // expect joint to freeze in subsequent steps without SetVelocityCommand
      world->Step(output, state, input);
      EXPECT_NEAR(0.0, joint->GetVelocity(0), 1e-1);
    }

    // Set joint force to 0 and expect that the velocity command is no
    // longer enforced, i.e. joint should not freeze in subsequent steps
    joint->SetForce(0, 0.0);
    for (std::size_t i = 0; i < numSteps; ++i)
    {
      world->Step(output, state, input);
      EXPECT_LT(0.0, std::fabs(joint->GetVelocity(0)));
    }

    // Check that invalid velocity commands don't cause collisions to fail
    for (std::size_t i = 0; i < 1000; ++i)
    {
      // Silence console spam
      gz::common::Console::SetVerbosity(0);
      joint->SetVelocityCommand(0, std::numeric_limits<double>::quiet_NaN());
      gz::common::Console::SetVerbosity(4);
      // expect the position of the pendulum to stay above ground
      world->Step(output, state, input);
      auto frameData = base_link->FrameDataRelativeToWorld();
      EXPECT_NEAR(0.0, frameData.pose.translation().z(), 1e-3);
    }
  }
}

TYPED_TEST(JointFeaturesTest, JointSetPositionWithContact)
{
  for (const std::string &name : this->pluginNames)
  {
    std::cout << "Testing plugin: " << name << std::endl;
    gz::plugin::PluginPtr plugin = this->loader.Instantiate(name);

    auto engine = gz::physics::RequestEngine3d<JointFeatureList>::From(plugin);
    ASSERT_NE(nullptr, engine);

    sdf::Root root;
    const sdf::Errors errors = root.Load(
        common_test::worlds::kPendulumJointWrenchSdf);
    ASSERT_TRUE(errors.empty()) << errors.front();

    auto world = engine->ConstructWorld(*root.WorldByIndex(0));
    ASSERT_NE(nullptr, world);

    auto model = world->GetModel("pendulum");
    ASSERT_NE(nullptr, model);
    auto motorJoint = model->GetJoint("motor_joint");
    ASSERT_NE(nullptr, motorJoint);

    gz::physics::ForwardStep::Output output;
    gz::physics::ForwardStep::State state;
    gz::physics::ForwardStep::Input input;

    world->Step(output, state, input);
    auto contacts = world->GetContactsFromLastStep();
    const std::size_t numInitialContacts = contacts.size();

    // Place box such that it is in collision with the pendulum arm at joint
    // position 0.
    auto box = world->GetModel("box");
    ASSERT_NE(nullptr, box);
    auto boxFreeGroup = box->FindFreeGroup();
    ASSERT_NE(nullptr, boxFreeGroup);
    gz::physics::Pose3d X_WB(Eigen::Translation3d(0.5, 0, 0.65));
    boxFreeGroup->SetWorldPose(X_WB);

    world->Step(output, state, input);
    contacts = world->GetContactsFromLastStep();
    EXPECT_LT(numInitialContacts, contacts.size());

    // Move pendulum away from box.
    motorJoint->SetPosition(0, GZ_DTOR(90.0));

    world->Step(output, state, input);
    contacts = world->GetContactsFromLastStep();
    EXPECT_EQ(numInitialContacts, contacts.size());

    // Step until pendulum falls and rests again on the box.
    for (int i = 0; i < 1000; ++i)
    {
      world->Step(output, state, input);
    }

    // Sanity check that the pendulum is at rest. A small non-zero threshold is
    // set to accommodate small joint velocity due to error reduction.
    EXPECT_NEAR(0.0, motorJoint->GetVelocity(0), 2e-3);

    contacts = world->GetContactsFromLastStep();
    EXPECT_LT(numInitialContacts, contacts.size());
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
    const sdf::Errors errors = root.Load(common_test::worlds::kTestWorld);
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
    gz::physics::GetJointTransmittedWrench,
    gz::physics::GetModelFromWorld,
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
    const sdf::Errors errors = root.Load(common_test::worlds::kTestWorld);
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
    const sdf::Errors errors = root.Load(common_test::worlds::kTestWorld);
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
    const sdf::Errors errors = root.Load(common_test::worlds::kTestWorld);
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
TYPED_TEST(JointFeaturesPositionLimitsForceControlTest, JointSetPositionLimitsWithVelocityControl)
{
  for (const std::string &name : this->pluginNames)
  {
    if (this->PhysicsEngineName(name) == "dartsim")
    {
      GTEST_SKIP();
    }
    else if (this->PhysicsEngineName(name) == "bullet-featherstone")
    {
#ifdef BT_BULLET_VERSION_LE_307
      // joint position limits does not work well with velocity control in
      // bullet versions <= 3.07
      GTEST_SKIP();
#endif
    }

    std::cout << "Testing plugin: " << name << std::endl;
    gz::plugin::PluginPtr plugin = this->loader.Instantiate(name);

    auto engine =
      gz::physics::RequestEngine3d<JointFeaturePositionLimitsForceControlList>::From(plugin);
    ASSERT_NE(nullptr, engine);

    sdf::Root root;
    const sdf::Errors errors = root.Load(common_test::worlds::kTestWorld);
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
        EXPECT_NEAR(0, joint->GetVelocity(0), 1e-5);
      }
    }
  }
}

TYPED_TEST(JointFeaturesPositionLimitsForceControlTest, JointSetVelocityLimitsWithVelocityControl)
{
  for (const std::string &name : this->pluginNames)
  {
    std::cout << "Testing plugin: " << name << std::endl;
    gz::plugin::PluginPtr plugin = this->loader.Instantiate(name);

    auto engine =
      gz::physics::RequestEngine3d<JointFeaturePositionLimitsForceControlList>::From(plugin);
    ASSERT_NE(nullptr, engine);

    sdf::Root root;
    const sdf::Errors errors = root.Load(common_test::worlds::kTestWorld);
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
    std::cout << "Testing plugin: " << name << std::endl;
    gz::plugin::PluginPtr plugin = this->loader.Instantiate(name);

    auto engine =
      gz::physics::RequestEngine3d<JointFeaturePositionLimitsForceControlList>::From(plugin);
    ASSERT_NE(nullptr, engine);

    sdf::Root root;
    const sdf::Errors errors = root.Load(common_test::worlds::kTestWorld);
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
    std::cout << "Testing plugin: " << name << std::endl;
    gz::plugin::PluginPtr plugin = this->loader.Instantiate(name);

    auto engine =
      gz::physics::RequestEngine3d<JointFeaturePositionLimitsForceControlList>::From(plugin);
    ASSERT_NE(nullptr, engine);

    sdf::Root root;
    const sdf::Errors errors = root.Load(common_test::worlds::kTestWorld);
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

TYPED_TEST(JointFeaturesPositionLimitsForceControlTest,
           JointTransmittedWrenchWithVelocityControl)
{
  for (const std::string &name : this->pluginNames)
  {
    // This test requires https://github.com/bulletphysics/bullet3/pull/4462
#ifdef BT_BULLET_VERSION_LE_325
    if (this->PhysicsEngineName(name) == "bullet-featherstone")
      GTEST_SKIP();
#endif

    std::cout << "Testing plugin: " << name << std::endl;
    gz::plugin::PluginPtr plugin = this->loader.Instantiate(name);

    auto engine =
      gz::physics::RequestEngine3d<JointFeaturePositionLimitsForceControlList>::
      From(plugin);
    ASSERT_NE(nullptr, engine);

    sdf::Root root;
    const sdf::Errors errors =
        root.Load(common_test::worlds::kJointOffsetEmptyLinksSdf);
    ASSERT_TRUE(errors.empty()) << errors.front();

    auto world = engine->ConstructWorld(*root.WorldByIndex(0));
    auto model = world->GetModel("model");
    auto joint = model->GetJoint("J0");

    // default step size: 1ms
    double dt = 1e-3;
    // velocity limit set in SDF
    double velocityLimit = 4;
    const double positionGoal = 0.1;
    // Calculate number of time steps expected to reach the position goal at
    // the maximum joint velocity.
    const int expectedSteps =
        static_cast<int>(positionGoal / velocityLimit / dt);
    // Take the expected number of steps.
    gzdbg << "Taking " << expectedSteps << " steps "
          << "to reach the goal." << std::endl;

    gz::physics::ForwardStep::Output output;
    gz::physics::ForwardStep::State state;
    gz::physics::ForwardStep::Input input;

    for (int i = 0; i < expectedSteps; ++i)
    {
      joint->SetVelocityCommand(0, velocityLimit);
      world->Step(output, state, input);
    }

    // Read joint wrench and expect it to be consistent with
    // the dynamic state of the model.
    //
    // Summary of dynamic state at this time in the test:
    // - The base link is fixed to the world.
    // - The child link is attached to the base link via the revolute joint J0.
    // - The child link has a mass of 1 kg and its center of mass is located
    //   0.5 m from joint J0.
    //     m = 1 kg
    //     L = 0.5 m
    // - The joint velocity command moves the joint at its maximum velocity of
    //   4 rad/s about the X-axis of the joint to reach its goal position
    //   of 0.1 rad. With a constant angular velocity, the angular acceleration
    //   is zero.
    //     pos_J0 = 0.1 rad (approximately)
    //     vel_J0 = 4 rad/s
    //     acc_J0 = 0 rad/s^2
    // - The child link is rotating about the joint like a pendulum with a
    //   linear acceleration consisting of centripetal acceleration since its
    //   angular acceleration is zero. When expressed in coordinates of the
    //   joint frame, the linear acceleration is in the y direction.
    //     a_child = {0, m * L * vel_J0^2, 0}
    // - Gravity is acting in the negative Z direction of the world frame with a
    //   magnitude of 9.8 m/s^2.
    //     g = 9.8 m/s^2
    // - The force of gravity is expressed in coordinates of the joint frame as
    //     F_gravity = {0, - m * g * sin(pos_J0), - m * g * cos(pos_J0)}.
    // - The joint transmitted wrench is the wrench applied from the base link
    //   to the child link at joint J0 and expressed in coordinates of
    //   the frame, which happens to coincide with the joint frame.
    // - First apply conservation of linear momentum:
    //   - The sum of forces acting on the child link is equal to the product of
    //     mass and the linear acceleration of its center of mass.
    //       F_child = m * a_child
    //   - The forces acting on the child link include the force of gravity and
    //     the reaction force from the joint:
    //       F_child = F_gravity + F_joint
    //   - The joint reaction force is thus equal to:
    //       F_joint = m * a_child - F_gravity
    //   - which can be expressed in coordinates of the joint frame as:
    //       F_joint = m * {0, L * vel_J0^2 + g * sin(pos_J0), g * cos(pos_J0)}
    // - Now apply conservation of angular momentum with respect to the origin
    //   of the joint frame J0:
    //   - Since the origin of the joint frame J0 is fixed with respect to an
    //     inertial frame, conservation of angular momentum about that point
    //     implies that the sum of torques acting on the child link at the joint
    //     origin (T_child_J0) is equal to the product of its moment of inertia
    //     with respect to the joint origin (I_J0) and its angular acceleration
    //     (which is zero).
    //     Thus the sum of torques acting on the child link is zero.
    //       T_child_J0 = I_J0 * alpha_child = 0
    //   - The torques acting on the child link with respect to the origin of
    //     the joint frame include the torque due to gravity and the reaction
    //     torque at the joint:
    //       T_child_J0 = T_gravity_J0 + T_joint_J0
    //       T_gravity_J0 = {m * g * L * cos(pos_J0), 0, 0}
    //       T_joint_J0 = {-m * g * L cos(pos_J0), 0, 0}
    //
    // After substitution of known constants, the expected wrench is therefore:
    const double expectedForceX = 0;
    // expectedForceY = m * L * vel_J0^2 + g * sin(pos_J0)
    // expectedForceY = 1 * 0.5 * 4^2 + 9.8 * sin(0.1)
    const double expectedForceY = 8 + 9.8 * sin(positionGoal);
    // expectedForceZ = m * g * cos(pos_J0)
    // expectedForceZ = 1 * 9.8 * cos(0.1)
    const double expectedForceZ = 9.8 * cos(positionGoal);
    // expectedTorqueX = -m * g * L cos(pos_J0)
    // expectedTorqueX = -1 * 9.8 * 0.5 cos(0.1)
    const double expectedTorqueX = -4.9 * cos(positionGoal);
    const double expectedTorqueY = 0;
    const double expectedTorqueZ = 0;
    gzdbg << "Checking that wrench values match the dynamic state."
          << std::endl;
    auto wrench = joint->GetTransmittedWrench();
    EXPECT_NEAR(expectedForceX, wrench.force.x(), 1e-6);
    // Looser tolerances are needed for the nonzero terms
    EXPECT_NEAR(expectedForceY, wrench.force.y(), 1e-1);
    EXPECT_NEAR(expectedForceZ, wrench.force.z(), 1e-2);
    EXPECT_NEAR(expectedTorqueX, wrench.torque.x(), 1e-2);
    EXPECT_NEAR(expectedTorqueY, wrench.torque.y(), 1e-6);
    EXPECT_NEAR(expectedTorqueZ, wrench.torque.z(), 1e-6);
  }
}

struct JointFeatureFrictionList : gz::physics::FeatureList<
    gz::physics::ForwardStep,
    gz::physics::GetBasicJointProperties,
    gz::physics::GetBasicJointState,
    gz::physics::GetEngineInfo,
    gz::physics::GetJointFromModel,
    gz::physics::GetModelFromWorld,
    gz::physics::SetBasicJointState,
    gz::physics::SetJointFrictionFeature,
    gz::physics::sdf::ConstructSdfWorld
> { };

template <class T>
class JointFeaturesFrictionTest :
  public JointFeaturesTest<T>{};
using JointFeaturesFrictionTestTypes =
  ::testing::Types<JointFeatureFrictionList>;
TYPED_TEST_SUITE(JointFeaturesFrictionTest,
                 JointFeaturesFrictionTestTypes);

TYPED_TEST(JointFeaturesFrictionTest, JointSetSpringStiffness)
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
      gz::physics::RequestEngine3d<JointFeatureFrictionList>::From(plugin);
    ASSERT_NE(nullptr, engine);

    sdf::Root root;
    const sdf::Errors errors = root.Load(common_test::worlds::kTestWorld);
    ASSERT_TRUE(errors.empty()) << errors.front();

    auto world = engine->ConstructWorld(*root.WorldByIndex(0));
    auto model = world->GetModel("pendulum_with_base");
    auto joint = model->GetJoint("upper_joint");

    gz::physics::ForwardStep::Output output;
    gz::physics::ForwardStep::State state;
    gz::physics::ForwardStep::Input input;

    world->Step(output, state, input);

    joint->SetPosition(0, -GZ_PI/2);

    // default friction value is zero
    // so oscillations are expected
    for (std::size_t i = 0; i < 100; ++i)
    {
      world->Step(output, state, input);
    }
    EXPECT_LT(joint->GetPosition(0), -1.54);
    EXPECT_LT(joint->GetVelocity(0), 1e-2);

    // setting very high friction value
    // pendulum shouldn't move much (expected)
    joint->SetPosition(0, -GZ_PI/2);
    ASSERT_EQ(joint->GetPosition(0), -GZ_PI/2);

    joint->SetVelocity(0, 1);
    ASSERT_EQ(joint->GetVelocity(0), 1);
    joint->SetFriction(0, 100);

    // running simulation for longer to make sure
    // joint position doesn't change (expected)
    for (std::size_t i = 0; i < 1000; ++i)
    {
      world->Step(output, state, input);
    }

    auto joint_pos1 = joint->GetPosition(0);

    EXPECT_NEAR(0, joint->GetVelocity(0), 1e-10);

    // setting some moderate value of joint friction
    joint->SetPosition(0, -GZ_PI/2);
    ASSERT_EQ(joint->GetPosition(0), -GZ_PI/2);

    joint->SetVelocity(0, 1);
    ASSERT_EQ(joint->GetVelocity(0), 1);

    joint->SetFriction(0, 5);

    for (std::size_t i = 0; i < 500; ++i)
    {
      world->Step(output, state, input);
    }

    auto joint_pos2 = joint->GetPosition(0);

    // with good enough simulation time for moderate
    // value of friction, joint position should converge
    // to zero (expected)
    EXPECT_LT(joint->GetVelocity(0), 1e-5);
    EXPECT_LT(joint_pos1, joint_pos2 + 0.1);
  }
}

struct JointFeatureSpringStiffnessList : gz::physics::FeatureList<
    gz::physics::ForwardStep,
    gz::physics::GetBasicJointProperties,
    gz::physics::GetBasicJointState,
    gz::physics::GetEngineInfo,
    gz::physics::GetJointFromModel,
    gz::physics::Gravity,
    gz::physics::GetModelFromWorld,
    gz::physics::SetBasicJointState,
    gz::physics::SetJointSpringStiffnessFeature,
    gz::physics::SetJointSpringReferenceFeature,
    gz::physics::SetJointDampingCoefficientFeature,
    gz::physics::sdf::ConstructSdfWorld
> { };

template <class T>
class JointFeaturesSpringStiffnessTest :
  public JointFeaturesTest<T>{};
using JointFeaturesSpringStiffnessTestTypes =
  ::testing::Types<JointFeatureSpringStiffnessList>;
TYPED_TEST_SUITE(JointFeaturesSpringStiffnessTest,
                 JointFeaturesSpringStiffnessTestTypes);

TYPED_TEST(JointFeaturesSpringStiffnessTest, JointSetSpringStiffness)
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
      gz::physics::RequestEngine3d<JointFeatureSpringStiffnessList>::From(plugin);
    ASSERT_NE(nullptr, engine);

    sdf::Root root;
    const sdf::Errors errors = root.Load(common_test::worlds::kTestWorld);
    ASSERT_TRUE(errors.empty()) << errors.front();

    auto world = engine->ConstructWorld(*root.WorldByIndex(0));
    auto model = world->GetModel("pendulum_with_base");
    auto joint = model->GetJoint("upper_joint");

    gz::physics::ForwardStep::Output output;
    gz::physics::ForwardStep::State state;
    gz::physics::ForwardStep::Input input;

    // turning off gravity so that the system behaves
    // like mass-damper
    world->SetGravity(Eigen::Vector3d::Zero());

    world->Step(output, state, input);
    // setting joint position to start from the bottom
    // pendulum position
    joint->SetPosition(0, GZ_PI/2);
    ASSERT_EQ(joint->GetPosition(0), GZ_PI/2);

    // setting joint velocity to zero
    joint->SetVelocity(0, 0);
    ASSERT_EQ(joint->GetVelocity(0), 0);
    // setting joint velocity to zero
    joint->SetVelocity(0, 0);
    ASSERT_EQ(joint->GetVelocity(0), 0);
    // without reference joint position joint should stay
    // at GZ_PI/2
    for (std::size_t i = 0; i < 2500; ++i)
    {
      world->Step(output, state, input);
    }

    // checking if the link has moved
    ASSERT_NEAR(joint->GetPosition(0), GZ_PI/2, 1e-5);
    ASSERT_NEAR(joint->GetVelocity(0), 0, 1e-5);

    // resetting joint position and velocity
    joint->SetPosition(0, GZ_PI/2);
    ASSERT_EQ(joint->GetPosition(0), GZ_PI/2);

    joint->SetVelocity(0, 0);
    ASSERT_EQ(joint->GetVelocity(0), 0);

    // setting joint rest position to pendulum upright position
    joint->SetSpringReference(0, -GZ_PI/2);

    // setting joint stiffness
    joint->SetSpringStiffness(0, 60);

    // setting joint damping to stabilize the joint's
    // rest position
    joint->SetDampingCoefficient(0, 17);

    // running simulation for longer to make sure
    // joint reaches equilibrium
    for (std::size_t i = 0; i < 2500; ++i)
    {
      world->Step(output, state, input);
    }

    // checking if the joint position is the same
    // as rest position
    ASSERT_NEAR(joint->GetPosition(0), -GZ_PI/2, 1e-4);

    // checking if the link has reached equilibrium
    ASSERT_NEAR(joint->GetVelocity(0), 0, 1e-5);
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
    const sdf::Errors errors = root.Load(common_test::worlds::kTestWorld);
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
#ifdef DART_ODE_CCD_WITH_BOX_CYLINDER_COLLISION
    // Disable some expectations for dartsim plugin with ode version >= 0.16.5,
    // see https://github.com/gazebosim/gz-physics/issues/620.
    if (this->PhysicsEngineName(name) != "dartsim")
#endif
    {
      EXPECT_NEAR(0.0, upperLinkLinearVelocity.X(), 1e-6);
    }
    EXPECT_NEAR(0.0, upperLinkLinearVelocity.Y(), 1e-6);
#ifdef DART_ODE_CCD_WITH_BOX_CYLINDER_COLLISION
    // Disable some expectations for dartsim plugin with ode version >= 0.16.5,
    // see https://github.com/gazebosim/gz-physics/issues/620.
    if (this->PhysicsEngineName(name) != "dartsim")
#endif
    {
      EXPECT_NEAR(0.0, upperLinkAngularVelocity.Y(), 1e-6);
    }
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
    const sdf::Errors errors = root.Load(common_test::worlds::kJointAcrossModelsSdf);
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

    const gz::math::Pose3d initialModel1Pose(0, 0, 0.25, 0, 0, 0.1);
    const gz::math::Pose3d initialModel2Pose(0, 0, 3.0, 0, 0, 0.2);

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

/////////////////////////////////////////////////
// Attach a fixed joint between links from non-static models to a link
// from a model fixed to the world. Verify the models are not moving when
// the fixed joint is attached.
TYPED_TEST(JointFeaturesAttachDetachTest, JointAttachDetachFixedToWorld)
{
  for (const std::string &name : this->pluginNames)
  {
    std::cout << "Testing plugin: " << name << std::endl;
    gz::plugin::PluginPtr plugin = this->loader.Instantiate(name);

    auto engine = gz::physics::RequestEngine3d<JointFeatureAttachDetachList>::From(plugin);
    ASSERT_NE(nullptr, engine);

    sdf::Root root;
    const sdf::Errors errors = root.Load(common_test::worlds::kJointAcrossModelsFixedSdf);
    ASSERT_TRUE(errors.empty()) << errors.front();

    auto world = engine->ConstructWorld(*root.WorldByIndex(0));

    // M1 is fixed to the world
    // M2 is non-static and at some distance away from M1
    // M3 is non-static and overlaps with M1
    const std::string modelName1{"M1"};
    const std::string modelName2{"M2"};
    const std::string modelName3{"M3"};
    const std::string bodyName{"body"};

    auto model1 = world->GetModel(modelName1);
    auto model2 = world->GetModel(modelName2);
    auto model3 = world->GetModel(modelName3);
    auto model1Body = model1->GetLink(bodyName);
    auto model2Body = model2->GetLink(bodyName);
    auto model3Body = model3->GetLink(bodyName);

    auto frameDataModel1Body = model1Body->FrameDataRelativeToWorld();
    auto frameDataModel2Body = model2Body->FrameDataRelativeToWorld();
    auto frameDataModel3Body = model3Body->FrameDataRelativeToWorld();

    const gz::math::Pose3d initialModel1Pose(0, 2, 3.0, 0, 0, 0.0);
    const gz::math::Pose3d initialModel2Pose(0, 0, 3.0, 0, 0, 0.0);
    const gz::math::Pose3d initialModel3Pose(0.3, 2, 3.0, 0, 0, 0.0);

    EXPECT_EQ(initialModel1Pose,
              gz::math::eigen3::convert(frameDataModel1Body.pose));
    EXPECT_EQ(initialModel2Pose,
              gz::math::eigen3::convert(frameDataModel2Body.pose));
    EXPECT_EQ(initialModel3Pose,
              gz::math::eigen3::convert(frameDataModel3Body.pose));

    // attach the fixed joint - model1 body is the parent
    auto poseParent = frameDataModel1Body.pose;
    auto poseChild = frameDataModel2Body.pose;
    auto poseParentChild = poseParent.inverse() * poseChild;
    auto fixedJoint12 = model2Body->AttachFixedJoint(model1Body);
    fixedJoint12->SetTransformFromParent(poseParentChild);

    poseChild = frameDataModel3Body.pose;
    poseParentChild = poseParent.inverse() * poseChild;
    auto fixedJoint13 = model3Body->AttachFixedJoint(model1Body);
    fixedJoint13->SetTransformFromParent(poseParentChild);

    gz::physics::ForwardStep::Output output;
    gz::physics::ForwardStep::State state;
    gz::physics::ForwardStep::Input input;
    const std::size_t numSteps = 100;
    for (std::size_t i = 0; i < numSteps; ++i)
    {
      world->Step(output, state, input);

      frameDataModel1Body = model1Body->FrameDataRelativeToWorld();
      frameDataModel2Body = model2Body->FrameDataRelativeToWorld();
      frameDataModel3Body = model3Body->FrameDataRelativeToWorld();

      // Expect all models to remain at initial pose
      EXPECT_NEAR(initialModel1Pose.Pos().Z(),
                  frameDataModel1Body.pose.translation().z(), 1e-3);
      EXPECT_NEAR(initialModel2Pose.Pos().Z(),
                  frameDataModel2Body.pose.translation().z(), 1e-3);
      // For bullet versions <= 3.06, static collision flags are not set.
      // So it tries to resolve overlapping bodies held together by
      // a fixed joint. Increase tolerance for position.
      double tol = 1e-3;
#ifdef BT_BULLET_VERSION_LE_306
      if (this->PhysicsEngineName(name) == "bullet-featherstone")
        tol = 1e-2;
#endif
      EXPECT_NEAR(initialModel3Pose.Pos().Z(),
                  frameDataModel3Body.pose.translation().z(), tol);

      // Expect all models to have zero velocities
      gz::math::Vector3d body1LinearVelocity =
        gz::math::eigen3::convert(frameDataModel1Body.linearVelocity);
      gz::math::Vector3d body2LinearVelocity =
        gz::math::eigen3::convert(frameDataModel2Body.linearVelocity);
      gz::math::Vector3d body3LinearVelocity =
        gz::math::eigen3::convert(frameDataModel3Body.linearVelocity);
      EXPECT_NEAR(0.0, body1LinearVelocity.Z(), 1e-3);
      EXPECT_NEAR(0.0, body2LinearVelocity.Z(), 1e-3);
      // For bullet versions <= 3.06, static collision flags are not set.
      // So overlapping bodies generate non-zero velocities.
#ifdef BT_BULLET_VERSION_LE_306
      if (this->PhysicsEngineName(name) != "bullet-featherstone")
#endif
      {
        EXPECT_NEAR(0.0, body3LinearVelocity.Z(), 1e-3);
      }
    }

    // now detach joint and expect model2 and model3 to start moving
    fixedJoint12->Detach();
    fixedJoint13->Detach();
    for (std::size_t i = 0; i < numSteps; ++i)
    {
      world->Step(output, state, input);

      frameDataModel1Body = model1Body->FrameDataRelativeToWorld();
      frameDataModel2Body = model2Body->FrameDataRelativeToWorld();
      frameDataModel3Body = model3Body->FrameDataRelativeToWorld();

      // Expect the model1 to be fixed to the world and model2 and model3
      // to start moving
      gz::math::Vector3d body1LinearVelocity =
        gz::math::eigen3::convert(frameDataModel1Body.linearVelocity);
      gz::math::Vector3d body2LinearVelocity =
        gz::math::eigen3::convert(frameDataModel2Body.linearVelocity);
      gz::math::Vector3d body3LinearVelocity =
        gz::math::eigen3::convert(frameDataModel3Body.linearVelocity);
      EXPECT_NEAR(0.0, body1LinearVelocity.Z(), 1e-2);
      EXPECT_GT(0.0, body2LinearVelocity.Z());
      // bullet-featherstone and dartsim has different behavior
      // when detaching a joint between overlapping bodies
      // bullet-featherstone: pushes bodies apart
      // So here we just check for non-zero velocity
      // \todo(iche033) Investigate behavior differences in dartsim.
      // Locally, model3 falls after joint is detached.
      // On CI, model3 has zero velocity which could mean model3 and model1 are
      // stuck together since they overlap with each other
      if (this->PhysicsEngineName(name) != "dartsim")
      {
        EXPECT_NE(gz::math::Vector3d::Zero, body3LinearVelocity);
      }
    }

    // Test attaching fixed joint with reverse the parent and child
    // relationship - model2 body is now the parent and model1 is child and
    // fixed to world
    poseParent = frameDataModel2Body.pose;
    poseChild = frameDataModel1Body.pose;
    poseParentChild = poseParent.inverse() * poseChild;
    auto fixedJoint12b = model1Body->AttachFixedJoint(model2Body);
    fixedJoint12b->SetTransformFromParent(poseParentChild);

    for (std::size_t i = 0; i < numSteps; ++i)
    {
      world->Step(output, state, input);
    }

    for (std::size_t i = 0; i < numSteps; ++i)
    {
      world->Step(output, state, input);

      frameDataModel1Body = model1Body->FrameDataRelativeToWorld();
      frameDataModel2Body = model2Body->FrameDataRelativeToWorld();
      // Expect both models to have zero velocities
      gz::math::Vector3d body1LinearVelocity =
        gz::math::eigen3::convert(frameDataModel1Body.linearVelocity);
      gz::math::Vector3d body2LinearVelocity =
        gz::math::eigen3::convert(frameDataModel2Body.linearVelocity);
      EXPECT_NEAR(0.0, body1LinearVelocity.Z(), 1e-3);
      EXPECT_NEAR(0.0, body2LinearVelocity.Z(), 1e-3);
    }

    // detach joint and expect model2 to start falling again
    fixedJoint12b->Detach();
    for (std::size_t i = 0; i < numSteps; ++i)
    {
      world->Step(output, state, input);

      frameDataModel1Body = model1Body->FrameDataRelativeToWorld();
      frameDataModel2Body = model2Body->FrameDataRelativeToWorld();

      // Expect the model1 to still be fixed to the world and model2
      // to start falling
      gz::math::Vector3d body1LinearVelocity =
        gz::math::eigen3::convert(frameDataModel1Body.linearVelocity);
      gz::math::Vector3d body2LinearVelocity =
        gz::math::eigen3::convert(frameDataModel2Body.linearVelocity);
      EXPECT_NEAR(0.0, body1LinearVelocity.Z(), 1e-2);
      // Negative z velocity
      EXPECT_GT(0.0, body2LinearVelocity.Z());
    }
  }
}

/////////////////////////////////////////////////
// Create a chain of models by attaching them with fixed joints:
// M1 (static) -> M2 (dynamic) -> M3 (dynamic)
// Verify that M2 and M3 become static once attached to M1
TYPED_TEST(JointFeaturesAttachDetachTest, JointAttachDetachChain)
{
  for (const std::string &name : this->pluginNames)
  {
    std::cout << "Testing plugin: " << name << std::endl;
    gz::plugin::PluginPtr plugin = this->loader.Instantiate(name);

    auto engine = gz::physics::RequestEngine3d<JointFeatureAttachDetachList>::From(plugin);
    ASSERT_NE(nullptr, engine);

    sdf::Root root;
    const sdf::Errors errors = root.Load(common_test::worlds::kJointAcrossModelsFixedSdf);
    ASSERT_TRUE(errors.empty()) << errors.front();

    auto world = engine->ConstructWorld(*root.WorldByIndex(0));

    // M1 is fixed to the world
    // M2 is non-static and at some distance away from M1
    // M3 is non-static and overlaps with M1
    const std::string modelName1{"M1"};
    const std::string modelName2{"M2"};
    const std::string modelName3{"M3"};
    const std::string bodyName{"body"};

    auto model1 = world->GetModel(modelName1);
    auto model2 = world->GetModel(modelName2);
    auto model3 = world->GetModel(modelName3);
    auto model1Body = model1->GetLink(bodyName);
    auto model2Body = model2->GetLink(bodyName);
    auto model3Body = model3->GetLink(bodyName);

    auto frameDataModel1Body = model1Body->FrameDataRelativeToWorld();
    auto frameDataModel2Body = model2Body->FrameDataRelativeToWorld();
    auto frameDataModel3Body = model3Body->FrameDataRelativeToWorld();

    const gz::math::Pose3d initialModel1Pose(0, 2, 3.0, 0, 0, 0.0);
    const gz::math::Pose3d initialModel2Pose(0, 0, 3.0, 0, 0, 0.0);
    const gz::math::Pose3d initialModel3Pose(0.3, 2, 3.0, 0, 0, 0.0);

    EXPECT_EQ(initialModel1Pose,
              gz::math::eigen3::convert(frameDataModel1Body.pose));
    EXPECT_EQ(initialModel2Pose,
              gz::math::eigen3::convert(frameDataModel2Body.pose));
    EXPECT_EQ(initialModel3Pose,
              gz::math::eigen3::convert(frameDataModel3Body.pose));

    // attach the fixed joint between M2 (dynamic) and M3 (dynamic)
    auto poseParent = frameDataModel2Body.pose;
    auto poseChild = frameDataModel3Body.pose;
    auto poseParentChild = poseParent.inverse() * poseChild;
    auto fixedJoint23 = model3Body->AttachFixedJoint(model2Body);
    fixedJoint23->SetTransformFromParent(poseParentChild);

    // attach the fixed joint between M1 (static) and M2 (dynamic)
    // this should recusively make M2 and M3 static
    poseParent = frameDataModel2Body.pose;
    poseChild = frameDataModel3Body.pose;
    poseParentChild = poseParent.inverse() * poseChild;
    auto fixedJoint12 = model2Body->AttachFixedJoint(model1Body);
    fixedJoint12->SetTransformFromParent(poseParentChild);

    gz::physics::ForwardStep::Output output;
    gz::physics::ForwardStep::State state;
    gz::physics::ForwardStep::Input input;
    const std::size_t numSteps = 100;
    for (std::size_t i = 0; i < numSteps; ++i)
    {
      world->Step(output, state, input);

      frameDataModel1Body = model1Body->FrameDataRelativeToWorld();
      frameDataModel2Body = model2Body->FrameDataRelativeToWorld();
      frameDataModel3Body = model3Body->FrameDataRelativeToWorld();

      // Expect all models to remain at initial pose
      EXPECT_NEAR(initialModel1Pose.Pos().Z(),
                  frameDataModel1Body.pose.translation().z(), 1e-3);
      // For bullet versions <= 3.06, static collision flags are not set.
      // Increase tolerance for position.
      double tol = 1e-3;
#ifdef BT_BULLET_VERSION_LE_306
      if (this->PhysicsEngineName(name) == "bullet-featherstone")
        tol = 0.1;
#endif
      EXPECT_NEAR(initialModel2Pose.Pos().Z(),
                  frameDataModel2Body.pose.translation().z(), tol);
      EXPECT_NEAR(initialModel3Pose.Pos().Z(),
                  frameDataModel3Body.pose.translation().z(), tol);

      // Expect all models to have zero velocities
      gz::math::Vector3d body1LinearVelocity =
        gz::math::eigen3::convert(frameDataModel1Body.linearVelocity);
      gz::math::Vector3d body2LinearVelocity =
        gz::math::eigen3::convert(frameDataModel2Body.linearVelocity);
      gz::math::Vector3d body3LinearVelocity =
        gz::math::eigen3::convert(frameDataModel3Body.linearVelocity);
      EXPECT_NEAR(0.0, body1LinearVelocity.Z(), 1e-3);
      // For bullet versions <= 3.06, static collision flags are not set.
      // So bodies generate non-zero velocities.
#ifdef BT_BULLET_VERSION_LE_306
      if (this->PhysicsEngineName(name) != "bullet-featherstone")
#endif
      {
        EXPECT_NEAR(0.0, body2LinearVelocity.Z(), 1e-3);
        EXPECT_NEAR(0.0, body3LinearVelocity.Z(), 1e-3);
      }
    }

    // Now detach joint between M2 (dynamic) and M3 (dynamic)
    // Expect M2 to be static as it is still attached to M1 (static)
    // Expect M3 to start moving
    fixedJoint23->Detach();
    for (std::size_t i = 0; i < numSteps; ++i)
    {
      world->Step(output, state, input);

      frameDataModel1Body = model1Body->FrameDataRelativeToWorld();
      frameDataModel2Body = model2Body->FrameDataRelativeToWorld();
      frameDataModel3Body = model3Body->FrameDataRelativeToWorld();

      // Expect the model1 to be fixed to the world and model2 and model3
      // to start moving
      gz::math::Vector3d body1LinearVelocity =
        gz::math::eigen3::convert(frameDataModel1Body.linearVelocity);
      gz::math::Vector3d body2LinearVelocity =
        gz::math::eigen3::convert(frameDataModel2Body.linearVelocity);
      gz::math::Vector3d body3LinearVelocity =
        gz::math::eigen3::convert(frameDataModel3Body.linearVelocity);
      EXPECT_NEAR(0.0, body1LinearVelocity.Z(), 1e-2);
      EXPECT_NEAR(0.0, body2LinearVelocity.Z(), 1e-2);
      // bullet-featherstone and dartsim has different behavior
      // when detaching a joint between overlapping bodies
      // dartsim: body falls after joint is detached
      // bullet-featherstone: pushes bodies apart
      // So here we just check for non-zero velocity
#ifdef __APPLE__
      // Disable check for dartsim plugin on homebrew.
      // model3 has zero velocity in dartsim on macOS. It could be a
      // change in behavior between dartsim versions. model3 overlaps
      // with model1 so could be stuck together
      if (this->PhysicsEngineName(name) != "dartsim")
#endif
      EXPECT_NE(gz::math::Vector3d::Zero, body3LinearVelocity);
    }

    // Now detach joint between M1 (static) and M2 (dynamic)
    // Expect M2 to start falling
    // Expect M3 to continue moving
    fixedJoint12->Detach();
    // fixedJoint13->Detach();
    for (std::size_t i = 0; i < numSteps; ++i)
    {
      world->Step(output, state, input);

      frameDataModel1Body = model1Body->FrameDataRelativeToWorld();
      frameDataModel2Body = model2Body->FrameDataRelativeToWorld();
      frameDataModel3Body = model3Body->FrameDataRelativeToWorld();

      // Expect the model1 to be fixed to the world and model2 and model3
      // to start moving
      gz::math::Vector3d body1LinearVelocity =
        gz::math::eigen3::convert(frameDataModel1Body.linearVelocity);
      gz::math::Vector3d body2LinearVelocity =
        gz::math::eigen3::convert(frameDataModel2Body.linearVelocity);
      gz::math::Vector3d body3LinearVelocity =
        gz::math::eigen3::convert(frameDataModel3Body.linearVelocity);
      EXPECT_NEAR(0.0, body1LinearVelocity.Z(), 1e-2);
      EXPECT_GT(0.0, body2LinearVelocity.Z());
      // bullet-featherstone and dartsim has different behavior
      // when detaching a joint between overlapping bodies
      // dartsim: body falls after joint is detached
      // bullet-featherstone: pushes bodies apart
      // So here we just check for non-zero velocity
#ifdef __APPLE__
      // Disable check for dartsim plugin on homebrew.
      // model3 has zero velocity in dartsim on macOS. It could be a
      // change in behavior between dartsim versions. model3 overlaps
      // with model1 so could be stuck together
      if (this->PhysicsEngineName(name) != "dartsim")
#endif
      EXPECT_NE(gz::math::Vector3d::Zero, body3LinearVelocity);
    }
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
#ifdef _WIN32
    // On windows, there's a tolerance issue with bullet-featherstone on the
    // last body2LinearVelocity expectation. Disabling until it's resolved.
    CHECK_UNSUPPORTED_ENGINE(name, "bullet-featherstone")
#endif
    std::cout << "Testing plugin: " << name << std::endl;
    gz::plugin::PluginPtr plugin = this->loader.Instantiate(name);

    auto engine = gz::physics::RequestEngine3d<JointFeatureAttachDetachList>::From(plugin);
    ASSERT_NE(nullptr, engine);

    sdf::Root root;
    const sdf::Errors errors = root.Load(common_test::worlds::kJointConstraintSdf);
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
    auto fixedJoint_m2m1 = model2Body->AttachFixedJoint(model1Body);
    fixedJoint_m2m1->SetTransformFromParent(poseParentChild1);

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
    auto fixedJoint_m2m3 = model2Body->AttachFixedJoint(model3Body);
    fixedJoint_m2m3->SetTransformFromParent(poseParentChild2);

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
    /// Step through initial transients
    for (std::size_t i = 0; i < numSteps; ++i)
    {
      world->Step(output, state, input);
    }

    {
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
      EXPECT_NEAR(0.0, body1LinearVelocity.Z(), 1e-3);
      EXPECT_NEAR(0.0, body2LinearVelocity.Z(), 1e-3);
      EXPECT_NEAR(0.0, body3LinearVelocity.Z(), 1e-3);
    }

    // Detach the joints. M1 and M3 should fall as there is now nothing stopping
    // them from falling.
    fixedJoint_m2m1->Detach();
    fixedJoint_m2m3->Detach();

    /// Step through initial transients
    for (std::size_t i = 0; i < numSteps; ++i)
    {
      world->Step(output, state, input);
    }

    {
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

      EXPECT_NEAR(dt * (numSteps) * -10, body1LinearVelocity.Z(), 1e-3);
      EXPECT_NEAR(0.0, body2LinearVelocity.Z(), 1e-3);
      EXPECT_NEAR(dt * (numSteps) * -10, body3LinearVelocity.Z(), 1e-3);
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
    const sdf::Errors errors = root.Load(common_test::worlds::kJointAcrossModelsSdf);
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
      const sdf::Errors errors = root.Load(common_test::worlds::kGroundSdf);
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

struct WorldJointFeatureList
    : gz::physics::FeatureList<JointFeatureList, gz::physics::WorldModelFeature>
{
};

class WorldModelTest : public JointFeaturesTest<WorldJointFeatureList>
{
  public: gz::physics::World3dPtr<WorldJointFeatureList> LoadWorld(
      const std::string &_pluginName)
  {
    gz::plugin::PluginPtr plugin = this->loader.Instantiate(_pluginName);

    auto engine =
        gz::physics::RequestEngine3d<WorldJointFeatureList>::From(plugin);

    sdf::Root root;
    const sdf::Errors errors = root.Load(common_test::worlds::kWorldJointTestSdf);
    EXPECT_TRUE(errors.empty()) << errors;
    if (errors.empty())
    {
      auto world = engine->ConstructWorld(*root.WorldByIndex(0));
      return world;
    }
    return nullptr;
  }
};

TEST_F(WorldModelTest, JointSetCommand)
{
  for (const std::string &name : this->pluginNames)
  {
    // bullet-feathersone does not support joints between models yet
    CHECK_UNSUPPORTED_ENGINE(name, "bullet-featherstone")

    auto world = this->LoadWorld(name);
    ASSERT_NE(nullptr, world);

    auto worldModel = world->GetWorldModel();
    ASSERT_NE(nullptr, worldModel);

    EXPECT_EQ(2u, worldModel->GetJointCount());

    {
      auto jointByName = worldModel->GetJoint("j1");
      ASSERT_NE(nullptr, jointByName);
      auto jointByIndex = worldModel->GetJoint(0);
      ASSERT_NE(nullptr, jointByIndex);
      EXPECT_EQ(jointByName, jointByIndex);

      EXPECT_EQ(0u, jointByName->GetIndex());
      EXPECT_EQ(0u, jointByIndex->GetIndex());

      ASSERT_TRUE(jointByIndex.Valid());
      auto modelFromJoint = jointByIndex->GetModel();
      ASSERT_NE(nullptr, modelFromJoint);
      EXPECT_EQ(modelFromJoint, worldModel);

      EXPECT_EQ(0u, jointByIndex->GetDegreesOfFreedom());
    }

    {
      auto jointByName = worldModel->GetJoint("j2");
      ASSERT_NE(nullptr, jointByName);
      auto jointByIndex = worldModel->GetJoint(1);
      ASSERT_NE(nullptr, jointByIndex);
      EXPECT_EQ(jointByName, jointByIndex);


      EXPECT_EQ(1u, jointByIndex->GetIndex());
      EXPECT_EQ(1u, jointByName->GetIndex());

      auto joint = jointByIndex;
      EXPECT_EQ(1u, jointByIndex->GetDegreesOfFreedom());

      gz::physics::ForwardStep::Output output;
      gz::physics::ForwardStep::State state;
      gz::physics::ForwardStep::Input input;

      EXPECT_NEAR(joint->GetVelocity(0), 0, 1e-3);
      // Joint limit set in SDF
      bool jointLimitReached{false};
      const double kJointPosLimit = 0.5;
      double lastJointPos = 0.0;
      for (std::size_t i = 0; i < 1000; ++i)
      {
        joint->SetForce(0, 1.0);
        world->Step(output, state, input);

        const double curJointPos = joint->GetPosition(0);
        if (curJointPos < kJointPosLimit)
        {
          // Check that joint position is always increasing while it's under the
          // position limit.
          EXPECT_GE(curJointPos, lastJointPos);
          // Velocity should be positive until position limit is reached.
          EXPECT_GE(jointByIndex->GetVelocity(0), 0);
        }
        else
        {
          jointLimitReached = true;
          break;
        }
        lastJointPos = curJointPos;
      }

      // One more step without a joint force to ensure the velocity is computed
      // correctly.
      world->Step(output, state, input);
      EXPECT_TRUE(jointLimitReached);
      EXPECT_NEAR(joint->GetPosition(0), kJointPosLimit, 1e-3);
      EXPECT_NEAR(joint->GetVelocity(0), 0, 1e-3);
    }
  }
}

using FixedJointFreeGroupFeatureList = gz::physics::FeatureList<
    gz::physics::FindFreeGroupFeature,
    gz::physics::SetFreeGroupWorldPose,
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
>;

using FixedJointFreeGroupFeatureTestTypes =
  JointFeaturesTest<FixedJointFreeGroupFeatureList>;

TEST_F(FixedJointFreeGroupFeatureTestTypes, FixedJointFreeGroupMove)
{
  // Attach joint between links from 2 models with fixed joint. Move
  // parent model using free group and verify child body moves along with the
  // parent link.
  for (const std::string &name : this->pluginNames)
  {
    std::cout << "Testing plugin: " << name << std::endl;
    gz::plugin::PluginPtr plugin = this->loader.Instantiate(name);

    auto engine =
        gz::physics::RequestEngine3d<FixedJointFreeGroupFeatureList>::From(
        plugin);
    ASSERT_NE(nullptr, engine);

    sdf::Root root;
    const sdf::Errors errors = root.Load(common_test::worlds::kJointAcrossModelsSdf);
    ASSERT_TRUE(errors.empty()) << errors.front();

    auto world = engine->ConstructWorld(*root.WorldByIndex(0));

    const std::string modelName1{"M1"};
    const std::string modelName2{"M2"};
    const std::string bodyName{"body"};

    auto model1 = world->GetModel(modelName1);
    auto model2 = world->GetModel(modelName2);
    auto model1Body = model1->GetLink(bodyName);
    auto model2Body = model2->GetLink(bodyName);

    const gz::math::Pose3d model1Pose(0, 0, 0.25, 0, 0, 0.0);
    const gz::math::Pose3d model2Pose(0, 1, 0.25, 0, 0, 0.0);
    auto freeGroupM1 = model1->FindFreeGroup();
    auto freeGroupM2 = model2->FindFreeGroup();
    freeGroupM1->SetWorldPose(gz::math::eigen3::convert(model1Pose));
    freeGroupM2->SetWorldPose(gz::math::eigen3::convert(model2Pose));

    auto frameDataModel1Body = model1Body->FrameDataRelativeToWorld();
    auto frameDataModel2Body = model2Body->FrameDataRelativeToWorld();

    EXPECT_EQ(model1Pose,
              gz::math::eigen3::convert(frameDataModel1Body.pose));
    EXPECT_EQ(model2Pose,
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
      // Expect the model1 and model2 to stay at rest
      // (since they are on the ground)
      gz::math::Vector3d body1LinearVelocity =
          gz::math::eigen3::convert(frameDataModel1Body.linearVelocity);
      gz::math::Vector3d body2LinearVelocity =
        gz::math::eigen3::convert(frameDataModel2Body.linearVelocity);
      EXPECT_NEAR(0.0, body1LinearVelocity.Z(), 1e-2);
      EXPECT_NEAR(0.0, body2LinearVelocity.Z(), 1e-2);
    }

    // Attach fixed joint
    const auto poseParent = frameDataModel1Body.pose;
    const auto poseChild = frameDataModel2Body.pose;
    auto poseParentChild = poseParent.inverse() * poseChild;
    auto fixedJoint = model2Body->AttachFixedJoint(model1Body);
    fixedJoint->SetTransformFromParent(poseParentChild);

    gz::math::Pose3d poseToMoveModel(1, 0, 0, 0, 0, 0.0);
    gz::math::Pose3d newModel1Pose = poseToMoveModel * model1Pose;
    gz::math::Pose3d newModel2Pose = poseToMoveModel * model2Pose;

    // Move parent model using free group
    freeGroupM1->SetWorldPose(gz::math::eigen3::convert(newModel1Pose));

    for (std::size_t i = 0; i < numSteps; ++i)
    {
      world->Step(output, state, input);
    }
    frameDataModel1Body = model1Body->FrameDataRelativeToWorld();
    frameDataModel2Body = model2Body->FrameDataRelativeToWorld();

    // Parent should be at the new pose and the child should follow
    EXPECT_EQ(newModel1Pose,
              gz::math::eigen3::convert(frameDataModel1Body.pose));
    EXPECT_EQ(newModel2Pose,
              gz::math::eigen3::convert(frameDataModel2Body.pose));
  }
}

using FixedJointWeldFeatureList = gz::physics::FeatureList<
    gz::physics::FindFreeGroupFeature,
    gz::physics::SetFreeGroupWorldPose,
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
    gz::physics::SetFixedJointWeldChildToParentFeature,
    gz::physics::SetJointTransformFromParentFeature,
    gz::physics::SetJointVelocityCommandFeature,
    gz::physics::sdf::ConstructSdfModel,
    gz::physics::sdf::ConstructSdfWorld
>;

using FixedJointWeldFeatureTestTypes =
  JointFeaturesTest<FixedJointWeldFeatureList>;

TEST_F(FixedJointWeldFeatureTestTypes, FixedJointWeldFall)
{
  // Attach joint between links from 2 models (M1 and M2) with a fixed joint.
  // Let them fall and the child model (M2) should collide with a third model
  // (body).
  // Run the test once without welding child to parent then run again
  // with welding enabled. Verify that the fixed joint enforces the relative
  // pose between the parent and child links but there should still be some
  // position error. Verify that the welded joint has a smaller error.
  for (const std::string &name : this->pluginNames)
  {
    std::cout << "Testing plugin: " << name << std::endl;
    gz::plugin::PluginPtr plugin = this->loader.Instantiate(name);

    auto engine =
        gz::physics::RequestEngine3d<FixedJointWeldFeatureList>::From(
        plugin);
    ASSERT_NE(nullptr, engine);

    sdf::Root root;
    const sdf::Errors errors = root.Load(common_test::worlds::kJointAcrossModelsSdf);
    ASSERT_TRUE(errors.empty()) << errors.front();

    auto world = engine->ConstructWorld(*root.WorldByIndex(0));

    const std::string modelName1{"M1"};
    const std::string modelName2{"M2"};
    const std::string bodyName{"body"};

    auto model1 = world->GetModel(modelName1);
    auto model2 = world->GetModel(modelName2);
    auto body = world->GetModel(bodyName);
    auto model1Body = model1->GetLink(bodyName);
    auto model2Body = model2->GetLink(bodyName);

    // \todo(iche033) Extend test to use pose with rotations and
    // verify that the relative pose between parent and child entities
    // are enforced.
    const gz::math::Pose3d model1Pose(0, -1, 1.0, 0, 0, 0.0);
    const gz::math::Pose3d model2Pose(0, 1, 1.0, 0, 0, 0.0);
    const gz::math::Pose3d bodyPose(0, 1, 0.25, 0, 0, 0.0);
    auto freeGroupM1 = model1->FindFreeGroup();
    auto freeGroupM2 = model2->FindFreeGroup();
    auto freeGroupBody = body->FindFreeGroup();
    freeGroupM1->SetWorldPose(gz::math::eigen3::convert(model1Pose));
    freeGroupM2->SetWorldPose(gz::math::eigen3::convert(model2Pose));
    freeGroupBody->SetWorldPose(gz::math::eigen3::convert(bodyPose));

    auto frameDataModel1Body = model1Body->FrameDataRelativeToWorld();
    auto frameDataModel2Body = model2Body->FrameDataRelativeToWorld();

    EXPECT_EQ(model1Pose,
              gz::math::eigen3::convert(frameDataModel1Body.pose));
    EXPECT_EQ(model2Pose,
              gz::math::eigen3::convert(frameDataModel2Body.pose));

    gz::physics::ForwardStep::Output output;
    gz::physics::ForwardStep::State state;
    gz::physics::ForwardStep::Input input;

    const std::size_t numSteps = 500;

    // Attach fixed joint
    auto poseParent = frameDataModel1Body.pose;
    auto poseChild = frameDataModel2Body.pose;
    auto poseParentChild = poseParent.inverse() * poseChild;
    auto fixedJoint = model2Body->AttachFixedJoint(model1Body);
    fixedJoint->SetTransformFromParent(poseParentChild);

    // Run once without welding child to parent
    fixedJoint->SetWeldChildToParent(false);

    gz::math::Pose3d gzPoseParentChild =
        gz::math::eigen3::convert(poseParentChild);
    double errNoWeld = 0.0;
    for (std::size_t i = 0; i < numSteps; ++i)
    {
      world->Step(output, state, input);

      frameDataModel1Body = model1Body->FrameDataRelativeToWorld();
      frameDataModel2Body = model2Body->FrameDataRelativeToWorld();

      auto newPoseParent = frameDataModel1Body.pose;
      auto newPoseChild = frameDataModel2Body.pose;
      auto newPoseParentChild = newPoseParent.inverse() * newPoseChild;
      gz::math::Pose3d gzNewPoseParentChild =
          gz::math::eigen3::convert(newPoseParentChild);
      // The relative parent-child pose should still be enforced even without
      // welding the child to parent but record the pos error
      EXPECT_EQ(gzPoseParentChild, gzNewPoseParentChild);
      errNoWeld +=
          (gzNewPoseParentChild.Pos() - gzPoseParentChild.Pos()).Length();
    }

    // Reset model poses
    freeGroupM1->SetWorldPose(gz::math::eigen3::convert(model1Pose));
    freeGroupM2->SetWorldPose(gz::math::eigen3::convert(model2Pose));
    freeGroupBody->SetWorldPose(gz::math::eigen3::convert(bodyPose));


    // Run the test with child welded to parent
    fixedJoint->SetWeldChildToParent(true);
    double errWeld = 0.0;
    for (std::size_t i = 0; i < numSteps; ++i)
    {
      world->Step(output, state, input);

      frameDataModel1Body = model1Body->FrameDataRelativeToWorld();
      frameDataModel2Body = model2Body->FrameDataRelativeToWorld();

      auto newPoseParent = frameDataModel1Body.pose;
      auto newPoseChild = frameDataModel2Body.pose;
      auto newPoseParentChild = newPoseParent.inverse() * newPoseChild;
      gz::math::Pose3d gzNewPoseParentChild =
          gz::math::eigen3::convert(newPoseParentChild);
      // The relative parent-child pose should be enforced but record the pos
      // error
      EXPECT_EQ(gzPoseParentChild, gzNewPoseParentChild);
      errWeld +=
          (gzNewPoseParentChild.Pos() - gzPoseParentChild.Pos()).Length();
    }
    // Verify that the error is smaller when child is welded to parent
    EXPECT_LT(errWeld, errNoWeld);
  }
}

int main(int argc, char *argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  if (!JointFeaturesTest<JointFeatureList>::init(
       argc, argv))
    return -1;
  return RUN_ALL_TESTS();
}
