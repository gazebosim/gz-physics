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

#include <cmath>
#include <cstddef>
#include <string>

#include <gz/common/Console.hh>
#include <gz/math/eigen3/Conversions.hh>
#include <gz/plugin/Loader.hh>

#include "test/TestLibLoader.hh"
#include "test/Utils.hh"
#include "Worlds.hh"

#include <gz/physics/ConstructEmpty.hh>
#include <gz/physics/FixedJoint.hh>
#include <gz/physics/FreeGroup.hh>
#include <gz/physics/Joint.hh>
#include <gz/physics/KinematicLink.hh>
#include <gz/physics/FrameSemantics.hh>
#include <gz/physics/FindFeatures.hh>
#include <gz/physics/ForwardStep.hh>
#include <gz/physics/GetEntities.hh>
#include <gz/physics/Link.hh>
#include <gz/physics/RequestEngine.hh>
#include <gz/physics/sdf/ConstructLink.hh>
#include <gz/physics/sdf/ConstructModel.hh>
#include <gz/physics/sdf/ConstructWorld.hh>

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
    gz::physics::GetNestedModelFromModel,
    gz::physics::GetLinkFromModel,
    gz::physics::GetJointFromModel,
    gz::physics::JointFrameSemantics,
    gz::physics::LinkFrameSemantics,
    gz::physics::ShapeFrameSemantics,
    gz::physics::ModelFrameSemantics
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
          gz::math::eigen3::convert(F_WCexpected.pose),
          gz::math::eigen3::convert(childLinkFrameData.pose));

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

    auto nestedModel = model->GetNestedModel("nested_model");
    ASSERT_NE(nullptr, nestedModel);
    auto nestedLink = nestedModel->GetLink("nested_link");
    ASSERT_NE(nullptr, nestedLink);
    auto nestedLinkCol = nestedLink->GetShape("nested_link_collision");
    ASSERT_NE(nullptr, nestedLinkCol);

    gz::math::Pose3d actualModelPose(1, 0, 0, 0, 0, 0);
    auto modelFrameData = model->FrameDataRelativeToWorld();
    EXPECT_EQ(actualModelPose,
              gz::math::eigen3::convert(modelFrameData.pose));
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

    gz::math::Pose3d actualNestedModelLocalPose(0, 0, 1, 0, 0, 0.5);
    auto nestedModelFrameData = nestedModel->FrameDataRelativeToWorld();
    gz::math::Pose3d expectedNestedModelWorldPose =
        actualModelPose * actualNestedModelLocalPose;
    EXPECT_EQ(expectedNestedModelWorldPose,
        gz::math::eigen3::convert(nestedModelFrameData.pose));

    auto nestedLinkFrameData = nestedLink->FrameDataRelativeToWorld();
    auto nestedLinkPose = gz::math::eigen3::convert(nestedLinkFrameData.pose);
    gz::math::Pose3d actualNestedLinkLocalPose(0, 2, 0, 0, 0, 0);
    gz::math::Pose3d expectedNestedLinkWorldPose =
        actualModelPose * actualNestedModelLocalPose *
        actualNestedLinkLocalPose;
    EXPECT_EQ(expectedNestedLinkWorldPose, nestedLinkPose);

    auto nestedLinkColFrameData = nestedLinkCol->FrameDataRelativeToWorld();
    auto nestedLinkColPose =
        gz::math::eigen3::convert(nestedLinkColFrameData.pose);
    auto actualNestedColLocalPose = gz::math::Pose3d(-0.5, 0, 0, 0, 1.5708, 0);
    auto expectedNestedColWorldPose =
        actualModelPose * actualNestedModelLocalPose *
        actualNestedLinkLocalPose * actualNestedColLocalPose;
    EXPECT_EQ(expectedNestedColWorldPose, nestedLinkColPose);
  }
}

using SetKinematicFeaturesList = gz::physics::FeatureList<
  gz::physics::sdf::ConstructSdfModel,
  gz::physics::sdf::ConstructSdfWorld,
  gz::physics::FindFreeGroupFeature,
  gz::physics::ForwardStep,
  gz::physics::GetLinkFromModel,
  gz::physics::GetModelFromWorld,
  gz::physics::KinematicLink,
  gz::physics::LinkFrameSemantics,
  gz::physics::SetFreeGroupWorldVelocity
>;

using SetKinematicTestFeaturesList =
  KinematicFeaturesTest<SetKinematicFeaturesList>;

TEST_F(SetKinematicTestFeaturesList, SetKinematic)
{
  // Test toggling a link between kinematic and dynamic type.
  // When dynamic, the link should fall due to gravity.
  // When made kinematic again, the link should retain its previous velocity
  // but it should no longer be accelerating.
  for (const std::string &name : this->pluginNames)
  {
    std::cout << "Testing plugin: " << name << std::endl;
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
      <model name="M1">
        <pose>0 0 10.0 0 0 0</pose>
        <link name="link">
          <kinematic>true</kinematic>
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

    errors = root.LoadSdfString(modelStr);
    ASSERT_TRUE(errors.empty()) << errors.front();
    ASSERT_NE(nullptr, root.Model());
    world->ConstructModel(*root.Model());

    auto model = world->GetModel("M1");
    ASSERT_NE(nullptr, model);
    auto link = model->GetLink("link");
    ASSERT_NE(nullptr, link);

    // verify sphere initial state
    gz::math::Pose3d initialPose(0, 0, 10, 0, 0, 0);
    auto frameData = link->FrameDataRelativeToWorld();
    EXPECT_EQ(initialPose, gz::math::eigen3::convert(frameData.pose));
    EXPECT_EQ(gz::math::Vector3d::Zero,
              gz::math::eigen3::convert(frameData.linearVelocity));
    EXPECT_EQ(gz::math::Vector3d::Zero,
              gz::math::eigen3::convert(frameData.angularVelocity));

    // Step physics and verify sphere is at the same location because
    // it is kinematic
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
    frameData = link->FrameDataRelativeToWorld();
    EXPECT_EQ(initialPose, gz::math::eigen3::convert(frameData.pose));
    EXPECT_EQ(gz::math::Vector3d::Zero,
              gz::math::eigen3::convert(frameData.linearVelocity));
    EXPECT_EQ(gz::math::Vector3d::Zero,
              gz::math::eigen3::convert(frameData.angularVelocity));

    // Make link dynamic and step
    link->SetKinematic(false);
    for (std::size_t i = 0; i < steps; ++i)
    {
      world->Step(output, state, input);
    }
    frameData = link->FrameDataRelativeToWorld();

    // Verify that sphere is falling by checking its pos and vel
    double gravity = -9.8;
    double distZ = 0.5 * gravity * time * time;
    double expectedPosZ =  initialPose.Pos().Z() + distZ;
    double expectedVelZ = gravity * time;
    EXPECT_NEAR(0.0, frameData.pose.translation().x(), 1e-3);
    EXPECT_NEAR(0.0, frameData.pose.translation().y(), 1e-3);
    EXPECT_NEAR(expectedPosZ,
                frameData.pose.translation().z(), 1e-2);
    EXPECT_NEAR(0.0, frameData.linearVelocity.x(), 1e-3);
    EXPECT_NEAR(0.0, frameData.linearVelocity.y(), 1e-3);
    EXPECT_NEAR(expectedVelZ, frameData.linearVelocity.z(), 1e-2);
    EXPECT_EQ(gz::math::Vector3d::Zero,
              gz::math::eigen3::convert(frameData.angularVelocity));

    // Make link kinematic again and step
    link->SetKinematic(true);

    for (std::size_t i = 0; i < steps; ++i)
    {
      world->Step(output, state, input);
    }
    frameData = link->FrameDataRelativeToWorld();
    expectedPosZ += expectedVelZ * time;

    EXPECT_NEAR(0.0, frameData.pose.translation().x(), 1e-3);
    EXPECT_NEAR(0.0, frameData.pose.translation().y(), 1e-3);
    EXPECT_NEAR(expectedPosZ,
                frameData.pose.translation().z(), 1e-2);

    EXPECT_NEAR(0.0, frameData.linearVelocity.x(), 1e-3);
    EXPECT_NEAR(0.0, frameData.linearVelocity.y(), 1e-3);
    EXPECT_NEAR(expectedVelZ, frameData.linearVelocity.z(), 1e-2);
    EXPECT_EQ(gz::math::Vector3d::Zero,
              gz::math::eigen3::convert(frameData.angularVelocity));

    // Command linear and angular velocity on the kinematic link
    auto freeGroup = link->FindFreeGroup();
    ASSERT_NE(nullptr, freeGroup);
    const gz::math::Vector3d cmdLinVel(1.5, -2.0, 0.5);
    const gz::math::Vector3d cmdAngVel(0.0, 0.0, 1.2);
    freeGroup->SetWorldLinearVelocity(gz::math::eigen3::convert(cmdLinVel));
    freeGroup->SetWorldAngularVelocity(gz::math::eigen3::convert(cmdAngVel));

    for (std::size_t i = 0; i < steps; ++i)
    {
      world->Step(output, state, input);
    }
    frameData = link->FrameDataRelativeToWorld();

    EXPECT_NEAR(cmdLinVel.X() * time, frameData.pose.translation().x(), 1e-2);
    EXPECT_NEAR(cmdLinVel.Y() * time, frameData.pose.translation().y(), 1e-2);
    EXPECT_NEAR(expectedPosZ + cmdLinVel.Z() * time,
                frameData.pose.translation().z(), 1e-2);
    const gz::math::Quaterniond actualRot =
        gz::math::eigen3::convert(Eigen::Quaterniond(frameData.pose.linear()));
    EXPECT_NEAR(0.0, actualRot.Roll(), 1e-2);
    EXPECT_NEAR(0.0, actualRot.Pitch(), 1e-2);
    EXPECT_NEAR(cmdAngVel.Z() * time, actualRot.Yaw(), 1e-2);
    EXPECT_EQ(cmdLinVel, gz::math::eigen3::convert(frameData.linearVelocity));
    EXPECT_EQ(cmdAngVel, gz::math::eigen3::convert(frameData.angularVelocity));
  }
}

TEST_F(SetKinematicTestFeaturesList, SetKinematicLinksWithJoint)
{
  // Load 2 kinematic links connected by a revolute joint.
  // Make one of the links dynamic and verify its motion
  for (const std::string &name : this->pluginNames)
  {
    std::cout << "Testing plugin: " << name << std::endl;
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
      <model name="M1">
        <pose>0 0 1.0 0 0 0</pose>
        <link name="link1">
          <kinematic>true</kinematic>
          <pose>0 0.25 0.0 1.57 0 0</pose>
          <collision name="collision">
            <geometry>
              <cylinder>
                <radius>0.1</radius>
                <length>0.5</length>
              </cylinder>
            </geometry>
          </collision>
        </link>
        <link name="link2">
          <kinematic>true</kinematic>
          <pose>0 -0.25 0.0 1.57 0 0</pose>
          <collision name="collision">
            <geometry>
              <cylinder>
                <radius>0.1</radius>
                <length>0.5</length>
              </cylinder>
            </geometry>
          </collision>
        </link>
        <joint name="joint" type="revolute">
          <pose>0 0 -0.25 0 0 0</pose>
          <parent>link1</parent>
          <child>link2</child>
          <axis>
            <xyz>1.0 0 0</xyz>
          </axis>
        </joint>
        <!--
        <joint name="world_joint" type="fixed">
          <parent>world</parent>
          <child>link1</child>
        </joint>
        -->
      </model>
    </sdf>)";

    errors = root.LoadSdfString(modelStr);
    ASSERT_TRUE(errors.empty()) << errors.front();
    ASSERT_NE(nullptr, root.Model());
    world->ConstructModel(*root.Model());

    auto model = world->GetModel("M1");
    ASSERT_NE(nullptr, model);
    auto link1 = model->GetLink("link1");
    ASSERT_NE(nullptr, link1);
    auto link2 = model->GetLink("link2");
    ASSERT_NE(nullptr, link2);

    // Verify links initial state
    gz::math::Pose3d initialLink1Pose(0, 0.25, 1, 1.57, 0, 0);
    auto frameData1 = link1->FrameDataRelativeToWorld();
    EXPECT_EQ(initialLink1Pose, gz::math::eigen3::convert(frameData1.pose));
    EXPECT_EQ(gz::math::Vector3d::Zero,
              gz::math::eigen3::convert(frameData1.linearVelocity));
    EXPECT_EQ(gz::math::Vector3d::Zero,
              gz::math::eigen3::convert(frameData1.angularVelocity));
    gz::math::Pose3d initialLink2Pose(0, -0.25, 1, 1.57, 0, 0);
    auto frameData2 = link2->FrameDataRelativeToWorld();
    EXPECT_EQ(initialLink2Pose, gz::math::eigen3::convert(frameData2.pose));
    EXPECT_EQ(gz::math::Vector3d::Zero,
              gz::math::eigen3::convert(frameData2.linearVelocity));
    EXPECT_EQ(gz::math::Vector3d::Zero,
              gz::math::eigen3::convert(frameData2.angularVelocity));

    // Step physics and verify links are at the same location because they
    // are kinematic
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

    frameData1 = link1->FrameDataRelativeToWorld();
    EXPECT_EQ(initialLink1Pose, gz::math::eigen3::convert(frameData1.pose));
    EXPECT_EQ(gz::math::Vector3d::Zero,
              gz::math::eigen3::convert(frameData1.linearVelocity));
    EXPECT_EQ(gz::math::Vector3d::Zero,
              gz::math::eigen3::convert(frameData1.angularVelocity));
    frameData2 = link2->FrameDataRelativeToWorld();
    EXPECT_EQ(initialLink2Pose, gz::math::eigen3::convert(frameData2.pose));
    EXPECT_EQ(gz::math::Vector3d::Zero,
              gz::math::eigen3::convert(frameData2.linearVelocity));
    EXPECT_EQ(gz::math::Vector3d::Zero,
              gz::math::eigen3::convert(frameData2.angularVelocity));

    // Make link2 dynamic and step
    link2->SetKinematic(false);

    for (std::size_t i = 0; i < steps; ++i)
    {
      world->Step(output, state, input);
    }
    // Verify link1 remains still
    frameData1 = link1->FrameDataRelativeToWorld();
    EXPECT_EQ(initialLink1Pose, gz::math::eigen3::convert(frameData1.pose));
    EXPECT_EQ(gz::math::Vector3d::Zero,
              gz::math::eigen3::convert(frameData1.linearVelocity));
    EXPECT_EQ(gz::math::Vector3d::Zero,
              gz::math::eigen3::convert(frameData1.angularVelocity));

    // Link2 should start rotating due to gravity
    frameData2 = link2->FrameDataRelativeToWorld();
    EXPECT_NEAR(0.0, frameData2.pose.translation().x(), 1e-3);
    EXPECT_LT(initialLink2Pose.Y(), frameData2.pose.translation().y());
    EXPECT_GT(initialLink2Pose.Z(), frameData2.pose.translation().z());

    // \todo(iche033) bullet-feathersone implementation does not return
    // correct velocities for non-base links when they are attached to a parent
    // base link that is either fixed to the world or kinematic
    // see https://github.com/gazebosim/gz-physics/issues/617
    if (this->PhysicsEngineName(name) != "bullet-featherstone")
    {
      EXPECT_NEAR(0.0, frameData2.linearVelocity.x(), 1e-3);
      EXPECT_LT(0.0, frameData2.linearVelocity.y());
      EXPECT_GT(0.0, frameData2.linearVelocity.z());
      EXPECT_LT(0.0, frameData2.angularVelocity.x());
      EXPECT_NEAR(0.0, frameData2.angularVelocity.y(), 1e-3);
      EXPECT_NEAR(0.0, frameData2.angularVelocity.z(), 1e-3);
    }
    auto updatedLink2Pose = gz::math::eigen3::convert(frameData2.pose);

    // Make link2 kinematic again and step
    link2->SetKinematic(true);

    for (std::size_t i = 0; i < steps; ++i)
    {
      world->Step(output, state, input);
    }

    // Verify the links did not move
    frameData1 = link1->FrameDataRelativeToWorld();
    EXPECT_EQ(initialLink1Pose, gz::math::eigen3::convert(frameData1.pose));
    EXPECT_EQ(gz::math::Vector3d::Zero,
              gz::math::eigen3::convert(frameData1.linearVelocity));
    EXPECT_EQ(gz::math::Vector3d::Zero,
              gz::math::eigen3::convert(frameData1.angularVelocity));
    frameData2 = link2->FrameDataRelativeToWorld();
    EXPECT_EQ(updatedLink2Pose, gz::math::eigen3::convert(frameData2.pose));
    EXPECT_EQ(gz::math::Vector3d::Zero,
              gz::math::eigen3::convert(frameData2.linearVelocity));
    EXPECT_EQ(gz::math::Vector3d::Zero,
              gz::math::eigen3::convert(frameData2.angularVelocity));
  }
}

/////////////////////////////////////////////////
// Helpers for the kinematic tests below
namespace
{
/// \brief Step the world a number of times.
template <typename WorldPtrT>
void StepKinematicWorld(WorldPtrT &_world, std::size_t _steps)
{
  gz::physics::ForwardStep::Input input;
  gz::physics::ForwardStep::State state;
  gz::physics::ForwardStep::Output output;
  for (std::size_t i = 0; i < _steps; ++i)
    _world->Step(output, state, input);
}

/// \brief Load a model from an SDF string and construct it in the world.
template <typename WorldPtrT>
void ConstructModelFromString(WorldPtrT &_world, const std::string &_modelStr)
{
  sdf::Root root;
  const sdf::Errors errors = root.LoadSdfString(_modelStr);
  ASSERT_TRUE(errors.empty()) << errors.front();
  ASSERT_NE(nullptr, root.Model());
  ASSERT_NE(nullptr, _world->ConstructModel(*root.Model()));
}

/// \brief Return the world pose of a link as a gz::math::Pose3d
template <typename LinkPtrT>
gz::math::Pose3d LinkWorldPose(const LinkPtrT &_link)
{
  return gz::math::eigen3::convert(_link->FrameDataRelativeToWorld().pose);
}

/// \brief Return true if two poses are equal within a tolerance.
::testing::AssertionResult PoseNear(const gz::math::Pose3d &_expected,
    const gz::math::Pose3d &_actual, double _tol)
{
  const double rotErr =
      (_expected.Rot().Inverse() * _actual.Rot()).Euler().Length();
  if (_expected.Pos().Equal(_actual.Pos(), _tol) && rotErr < _tol)
    return ::testing::AssertionSuccess();
  return ::testing::AssertionFailure()
      << "expected pose [" << _expected << "] actual pose [" << _actual << "]";
}

constexpr double kStepSize = 0.001;
constexpr double kGravity = -9.8;

/// \brief Model with a world-fixed anchor, a dynamic slider (prismatic along
/// world Y) and an arm attached to the slider by a revolute joint about
/// world X. The arm sticks out horizontally (+Y) from the pivot so that, if
/// it were dynamic, it would swing down under gravity. The arm is kinematic.
const char kDynamicParentKinematicChildSdf[] = R"(
  <sdf version="1.6">
    <model name="M1">
      <pose>0 0 1 0 0 0</pose>
      <link name="anchor">
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.01</ixx><iyy>0.01</iyy><izz>0.01</izz>
          </inertia>
        </inertial>
      </link>
      <link name="slider">
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.01</ixx><iyy>0.01</iyy><izz>0.01</izz>
          </inertia>
        </inertial>
      </link>
      <link name="arm">
        <kinematic>true</kinematic>
        <pose>0 0.5 0 0 0 0</pose>
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.084</ixx><iyy>0.0017</iyy><izz>0.084</izz>
          </inertia>
        </inertial>
      </link>
      <joint name="world_joint" type="fixed">
        <parent>world</parent>
        <child>anchor</child>
      </joint>
      <joint name="slider_joint" type="prismatic">
        <parent>anchor</parent>
        <child>slider</child>
        <axis>
          <xyz>0 1 0</xyz>
        </axis>
      </joint>
      <joint name="arm_joint" type="revolute">
        <pose>0 -0.5 0 0 0 0</pose>
        <parent>slider</parent>
        <child>arm</child>
        <axis>
          <xyz>1 0 0</xyz>
        </axis>
      </joint>
    </model>
  </sdf>)";
}  // namespace

/////////////////////////////////////////////////
TEST_F(SetKinematicTestFeaturesList, SetKinematicFalseAfterSleepTimeout)
{
  // A stationary kinematic body may be put to sleep by the physics engine.
  // Making it dynamic again must wake it up so that it falls under gravity.
  for (const std::string &name : this->pluginNames)
  {
    std::cout << "Testing plugin: " << name << std::endl;
    gz::plugin::PluginPtr plugin = this->loader.Instantiate(name);

    auto engine = gz::physics::RequestEngine3d<SetKinematicFeaturesList>::
        From(plugin);
    ASSERT_NE(nullptr, engine);

    sdf::Root root;
    sdf::Errors errors = root.Load(common_test::worlds::kEmptySdf);
    ASSERT_TRUE(errors.empty()) << errors.front();
    auto world = engine->ConstructWorld(*root.WorldByIndex(0));
    ASSERT_NE(nullptr, world);

    ConstructModelFromString(world, R"(
    <sdf version="1.6">
      <model name="M1">
        <pose>0 0 10.0 0 0 0</pose>
        <link name="link">
          <kinematic>true</kinematic>
          <collision name="coll_sphere">
            <geometry>
              <sphere>
                <radius>0.1</radius>
              </sphere>
            </geometry>
          </collision>
        </link>
      </model>
    </sdf>)");

    auto link = world->GetModel("M1")->GetLink("link");
    ASSERT_NE(nullptr, link);
    const gz::math::Pose3d initialPose(0, 0, 10, 0, 0, 0);

    // Stay kinematic for longer than bullet's default sleep timeout (2 s)
    StepKinematicWorld(world, 2500);
    EXPECT_TRUE(PoseNear(initialPose, LinkWorldPose(link), 1e-6));

    // Make link dynamic and verify that it falls
    link->SetKinematic(false);
    const double time = 1.0;
    StepKinematicWorld(world, static_cast<std::size_t>(time / kStepSize));

    const auto frameData = link->FrameDataRelativeToWorld();
    EXPECT_NEAR(initialPose.Z() + 0.5 * kGravity * time * time,
                frameData.pose.translation().z(), 1e-2);
    EXPECT_NEAR(kGravity * time, frameData.linearVelocity.z(), 1e-2);
  }
}

/////////////////////////////////////////////////
TEST_F(SetKinematicTestFeaturesList, KinematicParentDynamicChildFixedJoint)
{
  // A dynamic link attached to a kinematic base link via a fixed joint
  // should be held in place by the kinematic link. Once the base becomes
  // dynamic, the whole model should fall.
  for (const std::string &name : this->pluginNames)
  {
    std::cout << "Testing plugin: " << name << std::endl;
    gz::plugin::PluginPtr plugin = this->loader.Instantiate(name);

    auto engine = gz::physics::RequestEngine3d<SetKinematicFeaturesList>::
        From(plugin);
    ASSERT_NE(nullptr, engine);

    sdf::Root root;
    sdf::Errors errors = root.Load(common_test::worlds::kEmptySdf);
    ASSERT_TRUE(errors.empty()) << errors.front();
    auto world = engine->ConstructWorld(*root.WorldByIndex(0));
    ASSERT_NE(nullptr, world);

    ConstructModelFromString(world, R"(
    <sdf version="1.6">
      <model name="M1">
        <pose>0 0 2 0 0 0</pose>
        <link name="base">
          <kinematic>true</kinematic>
          <collision name="collision">
            <geometry>
              <box><size>0.2 0.2 0.2</size></box>
            </geometry>
          </collision>
        </link>
        <link name="child">
          <pose>0.5 0 0 0 0 0</pose>
          <collision name="collision">
            <geometry>
              <box><size>0.2 0.2 0.2</size></box>
            </geometry>
          </collision>
        </link>
        <joint name="fixed_joint" type="fixed">
          <parent>base</parent>
          <child>child</child>
        </joint>
      </model>
    </sdf>)");

    auto model = world->GetModel("M1");
    ASSERT_NE(nullptr, model);
    auto base = model->GetLink("base");
    ASSERT_NE(nullptr, base);
    auto child = model->GetLink("child");
    ASSERT_NE(nullptr, child);
    EXPECT_TRUE(base->GetKinematic());
    EXPECT_FALSE(child->GetKinematic());

    const gz::math::Pose3d initialBasePose(0, 0, 2, 0, 0, 0);
    const gz::math::Pose3d initialChildPose(0.5, 0, 2, 0, 0, 0);

    StepKinematicWorld(world, 1000);
    EXPECT_TRUE(PoseNear(initialBasePose, LinkWorldPose(base), 1e-3));
    EXPECT_TRUE(PoseNear(initialChildPose, LinkWorldPose(child), 1e-3));

    // Make base dynamic. The whole rigid model should free fall.
    base->SetKinematic(false);
    const double time = 0.5;
    StepKinematicWorld(world, static_cast<std::size_t>(time / kStepSize));
    const double expectedZ = 2.0 + 0.5 * kGravity * time * time;
    EXPECT_NEAR(expectedZ, LinkWorldPose(base).Z(), 1e-2);
    EXPECT_NEAR(expectedZ, LinkWorldPose(child).Z(), 1e-2);
  }
}

/////////////////////////////////////////////////
using KinematicJointFeaturesList = gz::physics::FeatureList<
  gz::physics::sdf::ConstructSdfModel,
  gz::physics::sdf::ConstructSdfWorld,
  gz::physics::ForwardStep,
  gz::physics::GetLinkFromModel,
  gz::physics::GetModelFromWorld,
  gz::physics::GetJointFromModel,
  gz::physics::GetBasicJointState,
  gz::physics::SetBasicJointState,
  gz::physics::SetJointVelocityCommandFeature,
  gz::physics::KinematicLink,
  gz::physics::LinkFrameSemantics
>;

using KinematicJointTestFeaturesList =
  KinematicFeaturesTest<KinematicJointFeaturesList>;

/////////////////////////////////////////////////
TEST_F(KinematicJointTestFeaturesList, KinematicLinksJointCommands)
{
  // Two kinematic links connected by a revolute joint. The kinematic child
  // link should follow position, velocity and velocity commands set on the
  // joint, and report the correct world velocity.
  for (const std::string &name : this->pluginNames)
  {
    std::cout << "Testing plugin: " << name << std::endl;
    gz::plugin::PluginPtr plugin = this->loader.Instantiate(name);

    auto engine = gz::physics::RequestEngine3d<KinematicJointFeaturesList>::
        From(plugin);
    ASSERT_NE(nullptr, engine);

    sdf::Root root;
    sdf::Errors errors = root.Load(common_test::worlds::kEmptySdf);
    ASSERT_TRUE(errors.empty()) << errors.front();
    auto world = engine->ConstructWorld(*root.WorldByIndex(0));
    ASSERT_NE(nullptr, world);

    ConstructModelFromString(world, R"(
    <sdf version="1.6">
      <model name="M1">
        <pose>0 0 1.0 0 0 0</pose>
        <link name="link1">
          <kinematic>true</kinematic>
          <pose>0 0.25 0.0 1.57 0 0</pose>
          <collision name="collision">
            <geometry>
              <cylinder>
                <radius>0.1</radius>
                <length>0.5</length>
              </cylinder>
            </geometry>
          </collision>
        </link>
        <link name="link2">
          <kinematic>true</kinematic>
          <pose>0 -0.25 0.0 1.57 0 0</pose>
          <collision name="collision">
            <geometry>
              <cylinder>
                <radius>0.1</radius>
                <length>0.5</length>
              </cylinder>
            </geometry>
          </collision>
        </link>
        <joint name="joint" type="revolute">
          <pose>0 0 -0.25 0 0 0</pose>
          <parent>link1</parent>
          <child>link2</child>
          <axis>
            <xyz>1.0 0 0</xyz>
          </axis>
        </joint>
      </model>
    </sdf>)");

    auto model = world->GetModel("M1");
    ASSERT_NE(nullptr, model);
    auto link1 = model->GetLink("link1");
    ASSERT_NE(nullptr, link1);
    auto link2 = model->GetLink("link2");
    ASSERT_NE(nullptr, link2);
    auto joint = model->GetJoint("joint");
    ASSERT_NE(nullptr, joint);

    const gz::math::Pose3d initialLink1Pose(0, 0.25, 1, 1.57, 0, 0);
    const gz::math::Pose3d initialLink2Pose(0, -0.25, 1, 1.57, 0, 0);
    // The joint axis is world X and passes through the pivot.
    const gz::math::Pose3d pivot =
        initialLink2Pose * gz::math::Pose3d(0, 0, -0.25, 0, 0, 0);
    auto expectedLink2Pose = [&](double _q)
    {
      return pivot * gz::math::Pose3d(0, 0, 0, _q, 0, 0) *
          pivot.Inverse() * initialLink2Pose;
    };

    // 1. Set joint position
    joint->SetPosition(0, 0.5);
    StepKinematicWorld(world, 1);
    EXPECT_NEAR(0.5, joint->GetPosition(0), 1e-3);
    EXPECT_TRUE(PoseNear(initialLink1Pose, LinkWorldPose(link1), 1e-3));
    EXPECT_TRUE(PoseNear(expectedLink2Pose(0.5), LinkWorldPose(link2), 1e-3));

    // 2. Set joint velocity before every step
    double q0 = joint->GetPosition(0);
    for (std::size_t i = 0; i < 500; ++i)
    {
      joint->SetVelocity(0, 1.0);
      StepKinematicWorld(world, 1);
    }
    EXPECT_NEAR(q0 + 0.5, joint->GetPosition(0), 1e-2);
    EXPECT_NEAR(1.0, joint->GetVelocity(0), 1e-2);
    EXPECT_TRUE(PoseNear(initialLink1Pose, LinkWorldPose(link1), 1e-3));
    EXPECT_TRUE(PoseNear(expectedLink2Pose(q0 + 0.5), LinkWorldPose(link2),
        1e-2));
    auto frameData2 = link2->FrameDataRelativeToWorld();
    EXPECT_NEAR(1.0, frameData2.angularVelocity.x(), 1e-2);
    EXPECT_NEAR(0.0, frameData2.angularVelocity.y(), 1e-2);
    EXPECT_NEAR(0.0, frameData2.angularVelocity.z(), 1e-2);
    EXPECT_LT(1e-2, frameData2.linearVelocity.norm());

    // 3. Velocity command before every step
    q0 = joint->GetPosition(0);
    for (std::size_t i = 0; i < 500; ++i)
    {
      joint->SetVelocityCommand(0, -1.0);
      StepKinematicWorld(world, 1);
    }
    EXPECT_NEAR(q0 - 0.5, joint->GetPosition(0), 1e-2);
    EXPECT_NEAR(-1.0, joint->GetVelocity(0), 1e-2);
    EXPECT_TRUE(PoseNear(initialLink1Pose, LinkWorldPose(link1), 1e-3));
    EXPECT_TRUE(PoseNear(expectedLink2Pose(q0 - 0.5), LinkWorldPose(link2),
        1e-2));
  }
}

/////////////////////////////////////////////////
TEST_F(KinematicJointTestFeaturesList, DynamicParentKinematicChildLocked)
{
  // A kinematic child link attached to a dynamic parent link via a revolute
  // joint. The joint should be kinematically locked (the arm moves with the
  // dynamic parent and does not react to gravity) unless a joint command is
  // given.
  for (const std::string &name : this->pluginNames)
  {
    std::cout << "Testing plugin: " << name << std::endl;
    gz::plugin::PluginPtr plugin = this->loader.Instantiate(name);

    auto engine = gz::physics::RequestEngine3d<KinematicJointFeaturesList>::
        From(plugin);
    ASSERT_NE(nullptr, engine);

    sdf::Root root;
    sdf::Errors errors = root.Load(common_test::worlds::kEmptySdf);
    ASSERT_TRUE(errors.empty()) << errors.front();
    auto world = engine->ConstructWorld(*root.WorldByIndex(0));
    ASSERT_NE(nullptr, world);

    ConstructModelFromString(world, kDynamicParentKinematicChildSdf);

    auto model = world->GetModel("M1");
    ASSERT_NE(nullptr, model);
    auto slider = model->GetLink("slider");
    ASSERT_NE(nullptr, slider);
    auto arm = model->GetLink("arm");
    ASSERT_NE(nullptr, arm);
    auto sliderJoint = model->GetJoint("slider_joint");
    ASSERT_NE(nullptr, sliderJoint);
    auto armJoint = model->GetJoint("arm_joint");
    ASSERT_NE(nullptr, armJoint);
    EXPECT_FALSE(slider->GetKinematic());
    EXPECT_TRUE(arm->GetKinematic());

    const gz::math::Pose3d initialArmPose(0, 0.5, 1, 0, 0, 0);
    EXPECT_TRUE(PoseNear(initialArmPose, LinkWorldPose(arm), 1e-6));

    // 1. Arm is kinematic: the arm joint is locked so nothing should move.
    StepKinematicWorld(world, 1000);
    EXPECT_NEAR(0.0, armJoint->GetPosition(0), 1e-3);
    EXPECT_NEAR(0.0, armJoint->GetVelocity(0), 1e-3);
    EXPECT_NEAR(0.0, sliderJoint->GetPosition(0), 1e-3);
    EXPECT_NEAR(0.0, sliderJoint->GetVelocity(0), 1e-3);
    EXPECT_TRUE(PoseNear(initialArmPose, LinkWorldPose(arm), 1e-3));

    // 2. Make arm dynamic. It should start from rest, i.e. no velocity should
    // have accumulated while it was kinematic, and swing down under gravity.
    arm->SetKinematic(false);
    StepKinematicWorld(world, 1);
    EXPECT_GT(0.1, std::abs(armJoint->GetVelocity(0)));
    StepKinematicWorld(world, 499);
    EXPECT_GT(initialArmPose.Z() - 0.05, LinkWorldPose(arm).Z());
    EXPECT_LT(1e-3, std::abs(armJoint->GetPosition(0)));

    // 3. Make arm kinematic again. The arm joint should lock at its current
    // position.
    arm->SetKinematic(true);
    const double lockedPos = armJoint->GetPosition(0);
    StepKinematicWorld(world, 1000);
    EXPECT_NEAR(lockedPos, armJoint->GetPosition(0), 1e-3);
    EXPECT_NEAR(0.0, armJoint->GetVelocity(0), 1e-3);
  }
}

/////////////////////////////////////////////////
TEST_F(KinematicJointTestFeaturesList, DynamicParentKinematicChildCommands)
{
  // A kinematic child link attached to a dynamic parent link via a revolute
  // joint should follow velocity and velocity commands set on the joint.
  for (const std::string &name : this->pluginNames)
  {
    std::cout << "Testing plugin: " << name << std::endl;
    gz::plugin::PluginPtr plugin = this->loader.Instantiate(name);

    auto engine = gz::physics::RequestEngine3d<KinematicJointFeaturesList>::
        From(plugin);
    ASSERT_NE(nullptr, engine);

    sdf::Root root;
    sdf::Errors errors = root.Load(common_test::worlds::kEmptySdf);
    ASSERT_TRUE(errors.empty()) << errors.front();
    auto world = engine->ConstructWorld(*root.WorldByIndex(0));
    ASSERT_NE(nullptr, world);

    ConstructModelFromString(world, kDynamicParentKinematicChildSdf);

    auto model = world->GetModel("M1");
    ASSERT_NE(nullptr, model);
    auto armJoint = model->GetJoint("arm_joint");
    ASSERT_NE(nullptr, armJoint);

    // 1. Set joint velocity before every step
    for (std::size_t i = 0; i < 500; ++i)
    {
      armJoint->SetVelocity(0, 1.0);
      StepKinematicWorld(world, 1);
    }
    EXPECT_NEAR(0.5, armJoint->GetPosition(0), 1e-2);
    EXPECT_NEAR(1.0, armJoint->GetVelocity(0), 1e-2);

    // 2. Velocity command before every step
    const double q0 = armJoint->GetPosition(0);
    for (std::size_t i = 0; i < 500; ++i)
    {
      armJoint->SetVelocityCommand(0, -1.0);
      StepKinematicWorld(world, 1);
    }
    EXPECT_NEAR(q0 - 0.5, armJoint->GetPosition(0), 1e-2);
    EXPECT_NEAR(-1.0, armJoint->GetVelocity(0), 1e-2);
  }
}

/////////////////////////////////////////////////
using KinematicFreeGroupFeaturesList = gz::physics::FeatureList<
  gz::physics::sdf::ConstructSdfModel,
  gz::physics::sdf::ConstructSdfWorld,
  gz::physics::ForwardStep,
  gz::physics::GetLinkFromModel,
  gz::physics::GetModelFromWorld,
  gz::physics::KinematicLink,
  gz::physics::LinkFrameSemantics,
  gz::physics::FindFreeGroupFeature,
  gz::physics::SetFreeGroupWorldPose,
  gz::physics::SetFreeGroupWorldVelocity,
  gz::physics::AttachFixedJointFeature,
  gz::physics::DetachJointFeature,
  gz::physics::SetJointTransformFromParentFeature
>;

using KinematicFreeGroupTestFeaturesList =
  KinematicFeaturesTest<KinematicFreeGroupFeaturesList>;

/////////////////////////////////////////////////
TEST_F(KinematicFreeGroupTestFeaturesList, KinematicBaseTwist)
{
  // Command simultaneous linear and angular velocities on kinematic links.
  // Velocities are expressed for the link frame origin in world frame.
  // * M_twist: link frame coincides with the center of mass.
  // * M_offset: center of mass is offset from the link frame.
  for (const std::string &name : this->pluginNames)
  {
    std::cout << "Testing plugin: " << name << std::endl;
    gz::plugin::PluginPtr plugin = this->loader.Instantiate(name);

    auto engine = gz::physics::RequestEngine3d<
        KinematicFreeGroupFeaturesList>::From(plugin);
    ASSERT_NE(nullptr, engine);

    sdf::Root root;
    sdf::Errors errors = root.Load(common_test::worlds::kEmptySdf);
    ASSERT_TRUE(errors.empty()) << errors.front();
    auto world = engine->ConstructWorld(*root.WorldByIndex(0));
    ASSERT_NE(nullptr, world);

    ConstructModelFromString(world, R"(
    <sdf version="1.6">
      <model name="M_twist">
        <pose>0 0 5 0 0 0</pose>
        <link name="link">
          <kinematic>true</kinematic>
          <collision name="collision">
            <geometry>
              <sphere><radius>0.1</radius></sphere>
            </geometry>
          </collision>
        </link>
      </model>
    </sdf>)");
    ConstructModelFromString(world, R"(
    <sdf version="1.6">
      <model name="M_offset">
        <pose>0 5 5 0 0 0</pose>
        <link name="link">
          <kinematic>true</kinematic>
          <inertial>
            <pose>0.5 0 0 0 0 0</pose>
            <mass>1.0</mass>
          </inertial>
          <collision name="collision">
            <geometry>
              <sphere><radius>0.1</radius></sphere>
            </geometry>
          </collision>
        </link>
      </model>
    </sdf>)");

    auto twistModel = world->GetModel("M_twist");
    ASSERT_NE(nullptr, twistModel);
    auto twistLink = twistModel->GetLink("link");
    ASSERT_NE(nullptr, twistLink);
    auto twistGroup = twistModel->FindFreeGroup();
    ASSERT_NE(nullptr, twistGroup);

    auto offsetModel = world->GetModel("M_offset");
    ASSERT_NE(nullptr, offsetModel);
    auto offsetLink = offsetModel->GetLink("link");
    ASSERT_NE(nullptr, offsetLink);
    auto offsetGroup = offsetModel->FindFreeGroup();
    ASSERT_NE(nullptr, offsetGroup);

    const Eigen::Vector3d linVel(1, 0, 0);
    const Eigen::Vector3d angVel(0, 0, 1);
    const double time = 1.0;
    for (std::size_t i = 0; i < static_cast<std::size_t>(time / kStepSize);
         ++i)
    {
      twistGroup->SetWorldLinearVelocity(linVel);
      twistGroup->SetWorldAngularVelocity(angVel);
      offsetGroup->SetWorldLinearVelocity(Eigen::Vector3d::Zero());
      offsetGroup->SetWorldAngularVelocity(angVel);
      StepKinematicWorld(world, 1);
    }

    // M_twist link origin should move in a straight line along world X
    // while yawing.
    auto frameData = twistLink->FrameDataRelativeToWorld();
    EXPECT_TRUE(PoseNear(gz::math::Pose3d(1, 0, 5, 0, 0, time),
        gz::math::eigen3::convert(frameData.pose), 1e-2));
    EXPECT_TRUE(gz::physics::test::Equal(linVel, frameData.linearVelocity,
        1e-2));
    EXPECT_TRUE(gz::physics::test::Equal(angVel, frameData.angularVelocity,
        1e-2));

    // M_offset link origin should rotate in place.
    frameData = offsetLink->FrameDataRelativeToWorld();
    EXPECT_TRUE(PoseNear(gz::math::Pose3d(0, 5, 5, 0, 0, time),
        gz::math::eigen3::convert(frameData.pose), 1e-2));
    EXPECT_TRUE(gz::physics::test::Equal(Eigen::Vector3d::Zero().eval(),
        frameData.linearVelocity, 1e-2));
    EXPECT_TRUE(gz::physics::test::Equal(angVel, frameData.angularVelocity,
        1e-2));
  }
}

/////////////////////////////////////////////////
TEST_F(KinematicFreeGroupTestFeaturesList,
       KinematicParentDynamicChildBaseVelocity)
{
  // A dynamic pendulum bob hanging from a kinematic base link. Moving the
  // kinematic base at constant velocity should carry the bob along without
  // making it swing (constant velocity motion does not induce swinging).
  for (const std::string &name : this->pluginNames)
  {
    std::cout << "Testing plugin: " << name << std::endl;
    gz::plugin::PluginPtr plugin = this->loader.Instantiate(name);

    auto engine = gz::physics::RequestEngine3d<
        KinematicFreeGroupFeaturesList>::From(plugin);
    ASSERT_NE(nullptr, engine);

    sdf::Root root;
    sdf::Errors errors = root.Load(common_test::worlds::kEmptySdf);
    ASSERT_TRUE(errors.empty()) << errors.front();
    auto world = engine->ConstructWorld(*root.WorldByIndex(0));
    ASSERT_NE(nullptr, world);

    ConstructModelFromString(world, R"(
    <sdf version="1.6">
      <model name="M1">
        <pose>0 0 2 0 0 0</pose>
        <link name="base">
          <kinematic>true</kinematic>
          <collision name="collision">
            <geometry>
              <sphere><radius>0.1</radius></sphere>
            </geometry>
          </collision>
        </link>
        <link name="bob">
          <pose>0 0 -0.5 0 0 0</pose>
          <inertial>
            <mass>1.0</mass>
            <inertia>
              <ixx>0.01</ixx><iyy>0.01</iyy><izz>0.01</izz>
            </inertia>
          </inertial>
          <collision name="collision">
            <geometry>
              <sphere><radius>0.1</radius></sphere>
            </geometry>
          </collision>
        </link>
        <joint name="pivot" type="revolute">
          <pose>0 0 0.5 0 0 0</pose>
          <parent>base</parent>
          <child>bob</child>
          <axis>
            <xyz>1 0 0</xyz>
          </axis>
        </joint>
      </model>
    </sdf>)");

    auto model = world->GetModel("M1");
    ASSERT_NE(nullptr, model);
    auto base = model->GetLink("base");
    ASSERT_NE(nullptr, base);
    auto bob = model->GetLink("bob");
    ASSERT_NE(nullptr, bob);
    auto freeGroup = model->FindFreeGroup();
    ASSERT_NE(nullptr, freeGroup);

    const Eigen::Vector3d linVel(1, 0, 0);
    const double time = 1.0;
    for (std::size_t i = 0; i < static_cast<std::size_t>(time / kStepSize);
         ++i)
    {
      freeGroup->SetWorldLinearVelocity(linVel);
      StepKinematicWorld(world, 1);
    }

    EXPECT_TRUE(PoseNear(gz::math::Pose3d(1, 0, 2, 0, 0, 0),
        LinkWorldPose(base), 1e-2));
    EXPECT_TRUE(PoseNear(gz::math::Pose3d(1, 0, 1.5, 0, 0, 0),
        LinkWorldPose(bob), 1e-2));
    EXPECT_TRUE(gz::physics::test::Equal(linVel,
        base->FrameDataRelativeToWorld().linearVelocity, 1e-2));
    EXPECT_TRUE(gz::physics::test::Equal(linVel,
        bob->FrameDataRelativeToWorld().linearVelocity, 1e-2));
  }
}

/////////////////////////////////////////////////
TEST_F(KinematicFreeGroupTestFeaturesList, KinematicModelAttachedToDynamic)
{
  // A dynamic model attached to a kinematic model with a detachable fixed
  // joint should be held by the kinematic model, follow it when it is moved,
  // and fall once detached. The kinematic model should remain kinematic.
  for (const std::string &name : this->pluginNames)
  {
    std::cout << "Testing plugin: " << name << std::endl;
    gz::plugin::PluginPtr plugin = this->loader.Instantiate(name);

    auto engine = gz::physics::RequestEngine3d<
        KinematicFreeGroupFeaturesList>::From(plugin);
    ASSERT_NE(nullptr, engine);

    sdf::Root root;
    sdf::Errors errors = root.Load(common_test::worlds::kEmptySdf);
    ASSERT_TRUE(errors.empty()) << errors.front();
    auto world = engine->ConstructWorld(*root.WorldByIndex(0));
    ASSERT_NE(nullptr, world);

    ConstructModelFromString(world, R"(
    <sdf version="1.6">
      <model name="M_kin">
        <pose>0 0 2 0 0 0</pose>
        <link name="link">
          <kinematic>true</kinematic>
          <collision name="collision">
            <geometry>
              <box><size>0.2 0.2 0.2</size></box>
            </geometry>
          </collision>
        </link>
      </model>
    </sdf>)");
    ConstructModelFromString(world, R"(
    <sdf version="1.6">
      <model name="M_dyn">
        <pose>0 0 1.5 0 0 0</pose>
        <link name="link">
          <collision name="collision">
            <geometry>
              <box><size>0.2 0.2 0.2</size></box>
            </geometry>
          </collision>
        </link>
      </model>
    </sdf>)");

    auto kinModel = world->GetModel("M_kin");
    ASSERT_NE(nullptr, kinModel);
    auto kinLink = kinModel->GetLink("link");
    ASSERT_NE(nullptr, kinLink);
    auto dynModel = world->GetModel("M_dyn");
    ASSERT_NE(nullptr, dynModel);
    auto dynLink = dynModel->GetLink("link");
    ASSERT_NE(nullptr, dynLink);

    auto fixedJoint = dynLink->AttachFixedJoint(kinLink);
    ASSERT_NE(nullptr, fixedJoint);
    // Preserve the current relative pose between the two links
    fixedJoint->SetTransformFromParent(gz::math::eigen3::convert(
        gz::math::Pose3d(0, 0, -0.5, 0, 0, 0)));

    // Dynamic model should be held up by the kinematic model. Step for
    // longer than bullet's default sleep timeout (2 s).
    StepKinematicWorld(world, 2500);
    EXPECT_TRUE(PoseNear(gz::math::Pose3d(0, 0, 2, 0, 0, 0),
        LinkWorldPose(kinLink), 1e-3));
    EXPECT_TRUE(PoseNear(gz::math::Pose3d(0, 0, 1.5, 0, 0, 0),
        LinkWorldPose(dynLink), 1e-3));
    EXPECT_TRUE(kinLink->GetKinematic());

    // Move the kinematic model. The dynamic model should follow.
    auto kinGroup = kinModel->FindFreeGroup();
    ASSERT_NE(nullptr, kinGroup);
    kinGroup->SetWorldPose(gz::math::eigen3::convert(
        gz::math::Pose3d(1, 0, 2, 0, 0, 0)));
    StepKinematicWorld(world, 10);
    EXPECT_TRUE(PoseNear(gz::math::Pose3d(1, 0, 2, 0, 0, 0),
        LinkWorldPose(kinLink), 1e-3));
    EXPECT_TRUE(PoseNear(gz::math::Pose3d(1, 0, 1.5, 0, 0, 0),
        LinkWorldPose(dynLink), 1e-2));

    // Detach. Dynamic model should fall, kinematic model should stay.
    fixedJoint->Detach();
    const double time = 0.5;
    StepKinematicWorld(world, static_cast<std::size_t>(time / kStepSize));
    EXPECT_TRUE(kinLink->GetKinematic());
    EXPECT_TRUE(PoseNear(gz::math::Pose3d(1, 0, 2, 0, 0, 0),
        LinkWorldPose(kinLink), 1e-3));
    EXPECT_NEAR(1.5 + 0.5 * kGravity * time * time,
        LinkWorldPose(dynLink).Z(), 2e-2);
  }
}

/////////////////////////////////////////////////
TEST_F(KinematicFreeGroupTestFeaturesList, KinematicLinkDetachedFromStatic)
{
  // Attach a kinematic link to a static link with a detachable fixed joint,
  // then detach it. The link should still be kinematic after detaching and
  // so it should not fall.
  for (const std::string &name : this->pluginNames)
  {
    std::cout << "Testing plugin: " << name << std::endl;
    gz::plugin::PluginPtr plugin = this->loader.Instantiate(name);

    auto engine = gz::physics::RequestEngine3d<
        KinematicFreeGroupFeaturesList>::From(plugin);
    ASSERT_NE(nullptr, engine);

    sdf::Root root;
    sdf::Errors errors = root.Load(common_test::worlds::kEmptySdf);
    ASSERT_TRUE(errors.empty()) << errors.front();
    auto world = engine->ConstructWorld(*root.WorldByIndex(0));
    ASSERT_NE(nullptr, world);

    ConstructModelFromString(world, R"(
    <sdf version="1.6">
      <model name="M_static">
        <static>true</static>
        <pose>0 0 2 0 0 0</pose>
        <link name="link">
          <collision name="collision">
            <geometry>
              <box><size>0.2 0.2 0.2</size></box>
            </geometry>
          </collision>
        </link>
      </model>
    </sdf>)");
    ConstructModelFromString(world, R"(
    <sdf version="1.6">
      <model name="M_kin">
        <pose>0 0 1.5 0 0 0</pose>
        <link name="link">
          <kinematic>true</kinematic>
          <collision name="collision">
            <geometry>
              <box><size>0.2 0.2 0.2</size></box>
            </geometry>
          </collision>
        </link>
      </model>
    </sdf>)");

    auto staticLink = world->GetModel("M_static")->GetLink("link");
    ASSERT_NE(nullptr, staticLink);
    auto kinLink = world->GetModel("M_kin")->GetLink("link");
    ASSERT_NE(nullptr, kinLink);
    EXPECT_TRUE(kinLink->GetKinematic());

    auto fixedJoint = kinLink->AttachFixedJoint(staticLink);
    ASSERT_NE(nullptr, fixedJoint);
    fixedJoint->SetTransformFromParent(gz::math::eigen3::convert(
        gz::math::Pose3d(0, 0, -0.5, 0, 0, 0)));
    StepKinematicWorld(world, 100);
    fixedJoint->Detach();

    EXPECT_TRUE(kinLink->GetKinematic());
    StepKinematicWorld(world, 1000);
    EXPECT_TRUE(PoseNear(gz::math::Pose3d(0, 0, 1.5, 0, 0, 0),
        LinkWorldPose(kinLink), 1e-3));
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
