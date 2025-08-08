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
#include <gz/math/eigen3/Conversions.hh>
#include <gz/plugin/Loader.hh>

#include "test/TestLibLoader.hh"
#include "test/Utils.hh"
#include "Worlds.hh"

#include <gz/physics/ConstructEmpty.hh>
#include <gz/physics/Joint.hh>
#include <gz/physics/FrameSemantics.hh>
#include <gz/physics/FindFeatures.hh>
#include <gz/physics/ForwardStep.hh>
#include <gz/physics/GetEntities.hh>
#include <gz/physics/Link.hh>
#include <gz/physics/RequestEngine.hh>
#include <gz/physics/sdf/ConstructWorld.hh>

#include <gz/physics/sdf/ConstructModel.hh>
#include <sdf/Root.hh>

template <class T>
class KinematicFeaturesTest:
 public testing::Test, public gz::physics::TestLibLoader
{
 // Documentation inherited
 public: void SetUp() override
 {
   gz::common::Console::SetVerbosity(4);

   loader.LoadLib(KinematicFeaturesTest::GetLibToTest());

   // TODO(ahcorde): We should also run the 3f, 2d, and 2f variants of
   // FindFeatures
   pluginNames = gz::physics::FindFeatures3d<T>::From(loader);
   if (pluginNames.empty())
   {
     std::cerr << "No plugins with required features found in "
               << GetLibToTest() << std::endl;
     GTEST_SKIP();
   }
   // TODO(ahcorde): SKIP bullet, review this test again.
   for (const std::string &name : this->pluginNames)
   {
     if(this->PhysicsEngineName(name) == "bullet")
     {
       GTEST_SKIP();
     }
   }
 }

 public: std::set<std::string> pluginNames;
 public: gz::plugin::Loader loader;
};

struct KinematicFeaturesList : gz::physics::FeatureList<
    gz::physics::GetEngineInfo,
    gz::physics::ForwardStep,
    gz::physics::sdf::ConstructSdfWorld,
    gz::physics::GetShapeFromLink,
    gz::physics::GetModelFromWorld,
    gz::physics::GetLinkFromModel,
    gz::physics::GetJointFromModel,
    gz::physics::JointFrameSemantics,
    gz::physics::LinkFrameSemantics,
    gz::physics::ShapeFrameSemantics
> { };

using KinematicFeaturesTestTypes =
  ::testing::Types<KinematicFeaturesList>;
TYPED_TEST_SUITE(KinematicFeaturesTest,
                 KinematicFeaturesTestTypes);

TYPED_TEST(KinematicFeaturesTest, JointFrameSemantics)
{
  for (const std::string &name : this->pluginNames)
  {
    std::cout << "Testing plugin: " << name << std::endl;
    gz::plugin::PluginPtr plugin = this->loader.Instantiate(name);

    auto engine = gz::physics::RequestEngine3d<KinematicFeaturesList>::From(plugin);
    ASSERT_NE(nullptr, engine);

    sdf::Root root;
    const sdf::Errors errors = root.Load(
       common_test::worlds::kStringPendulumSdf);
    ASSERT_TRUE(errors.empty()) << errors.front();

    auto world = engine->ConstructWorld(*root.WorldByIndex(0));
    ASSERT_NE(nullptr, world);

    auto model = world->GetModel("pendulum");
    ASSERT_NE(nullptr, model);
    auto pivotJoint = model->GetJoint("pivot");
    ASSERT_NE(nullptr, pivotJoint);
    auto childLink = model->GetLink("bob");
    ASSERT_NE(nullptr, childLink);

    gz::physics::ForwardStep::Output output;
    gz::physics::ForwardStep::State state;
    gz::physics::ForwardStep::Input input;

    for (std::size_t i = 0; i < 100; ++i)
    {
      world->Step(output, state, input);
    }
    // Pose of Child link (C) in Joint frame (J)
    gz::physics::Pose3d X_JC = gz::physics::Pose3d::Identity();
    X_JC.translate(gz::physics::Vector3d(0, 0, -1));

    // Notation: Using F_WJ for the frame data of frame J (joint) relative to
    // frame W (world).
    auto F_WJ = pivotJoint->FrameDataRelativeToWorld();
    gz::physics::FrameData3d F_WCexpected = F_WJ;

    gz::physics::Vector3d pendulumArmInWorld =
        F_WJ.pose.rotation() * X_JC.translation();

    F_WCexpected.pose = F_WJ.pose * X_JC;
    // angular acceleration of the child link is the same as the joint, so we
    // don't need to assign a new value.

    // Note that the joint's linear velocity and linear acceleration are zero, so
    // they are omitted here.
    F_WCexpected.linearAcceleration =
        F_WJ.angularAcceleration.cross(pendulumArmInWorld) +
        F_WJ.angularVelocity.cross(
            F_WJ.angularVelocity.cross(pendulumArmInWorld));

    F_WCexpected.linearVelocity = F_WJ.angularVelocity.cross(pendulumArmInWorld);

    auto childLinkFrameData = childLink->FrameDataRelativeToWorld();
    EXPECT_EQ(
          F_WCexpected.pose.rotation(),
          childLinkFrameData.pose.rotation());
    // TODO(ahcorde): Review this in bullet-featherstone
    if (this->PhysicsEngineName(name) != "bullet-featherstone")
    {
      EXPECT_NEAR(
            F_WCexpected.pose.translation().x(),
            childLinkFrameData.pose.translation().x(), 1e-6);
      EXPECT_NEAR(
            F_WCexpected.pose.translation().y(),
            childLinkFrameData.pose.translation().y(), 1e-6);
      EXPECT_NEAR(
            F_WCexpected.pose.translation().z(),
            childLinkFrameData.pose.translation().z(), 1e-6);
    }
    EXPECT_TRUE(
      gz::physics::test::Equal(
          F_WCexpected.linearVelocity,
          childLinkFrameData.linearVelocity,
          1e-6));
    EXPECT_TRUE(
      gz::physics::test::Equal(
          F_WCexpected.linearAcceleration,
          childLinkFrameData.linearAcceleration,
          1e-6));
  }
}

TYPED_TEST(KinematicFeaturesTest, LinkFrameSemanticsPose)
{
  for (const std::string &name : this->pluginNames)
  {
    std::cout << "Testing plugin: " << name << std::endl;
    gz::plugin::PluginPtr plugin = this->loader.Instantiate(name);

    auto engine =
        gz::physics::RequestEngine3d<KinematicFeaturesList>::From(plugin);
    ASSERT_NE(nullptr, engine);

    sdf::Root root;
    const sdf::Errors errors = root.Load(
       common_test::worlds::kPoseOffsetSdf);
    ASSERT_TRUE(errors.empty()) << errors.front();

    auto world = engine->ConstructWorld(*root.WorldByIndex(0));
    ASSERT_NE(nullptr, world);

    auto model = world->GetModel("model");
    ASSERT_NE(nullptr, model);
    auto baseLink = model->GetLink("base");
    ASSERT_NE(nullptr, baseLink);
    auto nonBaseLink = model->GetLink("link");
    ASSERT_NE(nullptr, nonBaseLink);
    auto baseCol = baseLink->GetShape("base_collision");
    ASSERT_NE(nullptr, baseCol);
    auto linkCol = nonBaseLink->GetShape("link_collision");
    ASSERT_NE(nullptr, linkCol);

    gz::math::Pose3d actualModelPose(1, 0, 0, 0, 0, 0);
    auto baseLinkFrameData = baseLink->FrameDataRelativeToWorld();
    auto baseLinkPose = gz::math::eigen3::convert(baseLinkFrameData.pose);
    gz::math::Pose3d actualLinkLocalPose(0, 1, 0, 0, 0, 0);
    gz::math::Pose3d expectedLinkWorldPose =
        actualModelPose * actualLinkLocalPose;
    EXPECT_EQ(expectedLinkWorldPose, baseLinkPose);

    auto baseColFrameData = baseCol->FrameDataRelativeToWorld();
    auto baseColPose = gz::math::eigen3::convert(baseColFrameData.pose);
    gz::math::Pose3d actualColLocalPose(0, 0, 0.01, 0, 0, 0);
    gz::math::Pose3d expectedColWorldPose =
        actualModelPose * actualLinkLocalPose * actualColLocalPose;
    EXPECT_EQ(expectedColWorldPose.Pos(), baseColPose.Pos());
    EXPECT_EQ(expectedColWorldPose.Rot().Euler(),
        baseColPose.Rot().Euler());

    auto nonBaseLinkFrameData = nonBaseLink->FrameDataRelativeToWorld();
    auto nonBaseLinkPose = gz::math::eigen3::convert(nonBaseLinkFrameData.pose);
    actualLinkLocalPose = gz::math::Pose3d (0, 0, 2.1, -1.5708, 0, 0);
    expectedLinkWorldPose = actualModelPose * actualLinkLocalPose;
    EXPECT_EQ(expectedLinkWorldPose, nonBaseLinkPose);

    auto linkColFrameData = linkCol->FrameDataRelativeToWorld();
    auto linkColPose = gz::math::eigen3::convert(linkColFrameData.pose);
    actualColLocalPose = gz::math::Pose3d(-0.05, 0, 0, 0, 1.5708, 0);
    expectedColWorldPose =
        actualModelPose * actualLinkLocalPose * actualColLocalPose;
    EXPECT_EQ(expectedColWorldPose.Pos(), linkColPose.Pos());
    EXPECT_EQ(expectedColWorldPose.Rot().Euler(),
        linkColPose.Rot().Euler());
  }
}

// Kinematic Tag Test
using SetKinematicFeaturesList = gz::physics::FeatureList<
  gz::physics::sdf::ConstructSdfModel,
  gz::physics::sdf::ConstructSdfWorld,
  gz::physics::ForwardStep,
  gz::physics::GetLinkFromModel,
  gz::physics::GetJointFromModel,
  gz::physics::SetBasicJointState,
  gz::physics::SetJointVelocityCommandFeature,
  gz::physics::GetModelFromWorld,
  gz::physics::LinkFrameSemantics
>;

using SetKinematicTestFeaturesList =
  KinematicFeaturesTest<SetKinematicFeaturesList>;

TEST_F(SetKinematicTestFeaturesList, SetFalseKinematic)
{
  for (const std::string &name : this->pluginNames)
  {
    std::cout << "Testing plugin: " << name << std::endl;
    // currently only dartsim is supported
    if (this->PhysicsEngineName(name) != "dartsim")
      continue;

    gz::plugin::PluginPtr plugin = this->loader.Instantiate(name);

    auto engine = gz::physics::RequestEngine3d<SetKinematicFeaturesList>::
        From(plugin);
    ASSERT_NE(nullptr, engine);

    sdf::Root root;
    sdf::Errors errors = root.Load(common_test::worlds::kEmptySdf);
    ASSERT_TRUE(errors.empty()) << errors.front();

    auto world = engine->ConstructWorld(*root.WorldByIndex(0));
    EXPECT_NE(nullptr, world);

    const std::string modelStr = R"(
    <sdf version="1.6">
      <model name="box">
        <pose>0 0 5.0 0 0 0</pose>
        <link name="parent">
          <kinematic>false</kinematic>
        </link>
        <link name="child">
          <collision name="collision2">
            <geometry>
              <box>
                <size>0.5 0.5 0.5 </size>
              </box>
            </geometry>
          </collision>
        </link>
      </model>
    </sdf>)";

    errors = root.LoadSdfString(modelStr);
    ASSERT_TRUE(errors.empty()) << errors.front();
    ASSERT_NE(nullptr, root.Model());
    world->ConstructModel(*root.Model());

    auto model = world->GetModel("box");
    ASSERT_NE(nullptr, model);
    auto link = model->GetLink("parent");
    ASSERT_NE(nullptr, link);

    // verify box initial state
    gz::math::Pose3d initialPose(0, 0, 5, 0, 0, 0);
    auto frameData = link->FrameDataRelativeToWorld();
    EXPECT_EQ(initialPose, gz::math::eigen3::convert(frameData.pose));
    EXPECT_EQ(gz::math::Vector3d::Zero,
              gz::math::eigen3::convert(frameData.linearVelocity));
    EXPECT_EQ(gz::math::Vector3d::Zero,
              gz::math::eigen3::convert(frameData.angularVelocity));

    double time = 2.0;
    double stepSize = 0.001;
    std::size_t steps = static_cast<std::size_t>(time / stepSize);

    gz::physics::ForwardStep::Input input;
    gz::physics::ForwardStep::State state;
    gz::physics::ForwardStep::Output output;
    for (std::size_t i = 0; i < steps; ++i)
    {
      world->Step(output, state, input);
    }
    // BOX falls
    frameData = link->FrameDataRelativeToWorld();
    // Verify the sphere did not move
    EXPECT_NEAR(0.0, frameData.pose.translation().x(), 1e-3);
    EXPECT_NEAR(0.0, frameData.pose.translation().y(), 1e-3);
    
    // Falls and should be below the floor
    // z0 - 1/2 * g * t^2
    double expected_z = 5 - (0.5 * 9.8 * time * time);
    double expected_vz = - 9.8 * time;
    gz::math::Vector3d expected_v(0.0, 0.0, expected_vz);
 
    EXPECT_NEAR(expected_z,
                frameData.pose.translation().z(), 1e-2);
    EXPECT_NEAR(expectedVelZ, frameData.linearVelocity.z(), 1e-2);
    EXPECT_EQ(gz::math::Vector3d::Zero,
              gz::math::eigen3::convert(frameData.angularVelocity));
  }
}

TEST_F(SetKinematicTestFeaturesList, SetTrueKinematic)
{
  for (const std::string &name : this->pluginNames)
  {
    std::cout << "Testing plugin: " << name << std::endl;
    // currently only dartsim is supported
    if (this->PhysicsEngineName(name) != "dartsim")
      continue;

    gz::plugin::PluginPtr plugin = this->loader.Instantiate(name);

    auto engine = gz::physics::RequestEngine3d<SetKinematicFeaturesList>::
        From(plugin);
    ASSERT_NE(nullptr, engine);

    sdf::Root root;
    sdf::Errors errors = root.Load(common_test::worlds::kEmptySdf);
    ASSERT_TRUE(errors.empty()) << errors.front();

    auto world = engine->ConstructWorld(*root.WorldByIndex(0));
    EXPECT_NE(nullptr, world);

    const std::string modelStr = R"(
    <sdf version="1.6">
      <model name="box">
        <pose>0 0 5.0 0 0 0</pose>
        <link name="parent">
          <kinematic>true</kinematic>
        </link>
        <link name="child">
          <collision name="collision2">
            <geometry>
              <box>
                <size>0.5 0.5 0.5 </size>
              </box>
            </geometry>
          </collision>
        </link>
      </model>
    </sdf>)";

    errors = root.LoadSdfString(modelStr);
    ASSERT_TRUE(errors.empty()) << errors.front();
    ASSERT_NE(nullptr, root.Model());
    world->ConstructModel(*root.Model());

    auto model = world->GetModel("box");
    ASSERT_NE(nullptr, model);
    auto link = model->GetLink("parent");
    ASSERT_NE(nullptr, link);

    // verify box initial state
    gz::math::Pose3d initialPose(0, 0, 5, 0, 0, 0);
    auto frameData = link->FrameDataRelativeToWorld();
    EXPECT_EQ(initialPose, gz::math::eigen3::convert(frameData.pose));
    EXPECT_EQ(gz::math::Vector3d::Zero,
              gz::math::eigen3::convert(frameData.linearVelocity));
    EXPECT_EQ(gz::math::Vector3d::Zero,
              gz::math::eigen3::convert(frameData.angularVelocity));

    double time = 1.0;
    double stepSize = 0.001;
    std::size_t steps = static_cast<std::size_t>(time / stepSize);
    gz::physics::ForwardStep::Input input;
    gz::physics::ForwardStep::State state;
    gz::physics::ForwardStep::Output output;
    for (std::size_t i = 0; i < steps; ++i)
    {
      world->Step(output, state, input);
    }
    // BOX does not falls
    frameData = link->FrameDataRelativeToWorld();
    EXPECT_EQ(initialPose, gz::math::eigen3::convert(frameData.pose));
    EXPECT_EQ(gz::math::Vector3d::Zero,
              gz::math::eigen3::convert(frameData.linearVelocity));
    EXPECT_EQ(gz::math::Vector3d::Zero,
              gz::math::eigen3::convert(frameData.angularVelocity));

  }
}

int main(int argc, char *argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  if (!KinematicFeaturesTest<KinematicFeaturesList>::init(
       argc, argv))
    return -1;
  return RUN_ALL_TESTS();
}
