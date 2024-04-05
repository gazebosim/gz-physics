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

#include <cstddef>

#include <gz/common/Console.hh>
#include <gz/math/eigen3/Conversions.hh>
#include <gz/plugin/Loader.hh>

#include "test/TestLibLoader.hh"
#include "test/Utils.hh"
#include "Worlds.hh"

#include <gz/physics/ConstructEmpty.hh>
#include <gz/physics/Joint.hh>
#include <gz/physics/Kinematic.hh>
#include <gz/physics/FrameSemantics.hh>
#include <gz/physics/FindFeatures.hh>
#include <gz/physics/ForwardStep.hh>
#include <gz/physics/GetEntities.hh>
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
    gz::physics::GetModelFromWorld,
    gz::physics::GetLinkFromModel,
    gz::physics::GetJointFromModel,
    gz::physics::LinkFrameSemantics,
    gz::physics::JointFrameSemantics
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

    // TODO(ahcorde): Rewiew this in bullet-featherstone
    if(this->PhysicsEngineName(name) == "bullet_featherstone")
    {
      EXPECT_EQ(
            F_WCexpected.pose.translation(),
            childLinkFrameData.pose.translation());
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


using SetKinematicFeaturesList = gz::physics::FeatureList<
  gz::physics::sdf::ConstructSdfModel,
  gz::physics::sdf::ConstructSdfWorld,
  gz::physics::ForwardStep,
  gz::physics::GetLinkFromModel,
  gz::physics::GetModelFromWorld,
  gz::physics::Kinematic,
  gz::physics::LinkFrameSemantics
>;

using SetKinematicTestFeaturesList =
  KinematicFeaturesTest<SetKinematicFeaturesList>;

TEST_F(SetKinematicTestFeaturesList, SetKinematic)
{
  // Test toggling a link between kinematic and dynamic type
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
    double distZ = 0.5 * gravity;
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

    // Verify the sphere did not move
    EXPECT_NEAR(0.0, frameData.pose.translation().x(), 1e-3);
    EXPECT_NEAR(0.0, frameData.pose.translation().y(), 1e-3);
    EXPECT_NEAR(expectedPosZ,
                frameData.pose.translation().z(), 1e-2);
    EXPECT_EQ(gz::math::Vector3d::Zero,
              gz::math::eigen3::convert(frameData.linearVelocity));
    EXPECT_EQ(gz::math::Vector3d::Zero,
              gz::math::eigen3::convert(frameData.angularVelocity));
  }
}

TEST_F(SetKinematicTestFeaturesList, SetKinematicLinksWithJoint)
{
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

int main(int argc, char *argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  if (!KinematicFeaturesTest<KinematicFeaturesList>::init(
       argc, argv))
    return -1;
  return RUN_ALL_TESTS();
}
