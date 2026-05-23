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

#include "test/TestLibLoader.hh"
#include "test/Utils.hh"
#include "Worlds.hh"

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
class DetachableJointTest:
  public testing::Test, public gz::physics::TestLibLoader
{
  // Documentation inherited
  public: void SetUp() override
  {
    gz::common::Console::SetVerbosity(4);

    std::cerr << "DetachableJointTest::GetLibToTest() " << DetachableJointTest::GetLibToTest() << '\n';

    loader.LoadLib(DetachableJointTest::GetLibToTest());

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

struct DetachableJointFeatureList: gz::physics::FeatureList<
    gz::physics::ForwardStep,
    gz::physics::GetBasicJointProperties,
    gz::physics::GetBasicJointState,
    gz::physics::GetEngineInfo,
    gz::physics::GetJointFromModel,
    gz::physics::GetLinkFromModel,
    gz::physics::GetModelFromWorld,
    gz::physics::LinkFrameSemantics,
    gz::physics::SetBasicJointState,
    gz::physics::AttachFixedJointFeature,
    gz::physics::DetachJointFeature,
    gz::physics::SetJointTransformFromParentFeature,
    gz::physics::sdf::ConstructSdfWorld
> { };

using DetachableJointTestTypes =
  ::testing::Types<DetachableJointFeatureList>;
TYPED_TEST_SUITE(DetachableJointTest,
                 DetachableJointTestTypes);

TYPED_TEST(DetachableJointTest, CorrectAttachmentPoints)
{
  for (const std::string &name : this->pluginNames)
  {
    std::cout << "Testing plugin: " << name << std::endl;
    gz::plugin::PluginPtr plugin = this->loader.Instantiate(name);

    auto engine =
      gz::physics::RequestEngine3d<DetachableJointFeatureList>::From(plugin);
    ASSERT_NE(nullptr, engine);

    sdf::Root root;
    const sdf::Errors errors = root.Load(
      common_test::worlds::kDetachableJointWorld);
    ASSERT_TRUE(errors.empty()) << errors.front();

    auto world = engine->ConstructWorld(*root.WorldByIndex(0));
    ASSERT_NE(nullptr, world);

    auto cylinder1 = world->GetModel("cylinder1");
    ASSERT_NE(nullptr, cylinder1);
    auto cylinder1_base_link = cylinder1->GetLink("base_link");
    ASSERT_NE(nullptr, cylinder1_base_link);
    auto cylinder1_link1 = cylinder1->GetLink("link1");
    ASSERT_NE(nullptr, cylinder1_link1);

    auto cylinder2 = world->GetModel("cylinder2");
    ASSERT_NE(nullptr, cylinder2);
    auto cylinder2_link2 = cylinder2->GetLink("link2");
    ASSERT_NE(nullptr, cylinder2_link2);

    // Create a detachable joint
    const auto poseParent =
      cylinder1_link1->FrameDataRelativeToWorld().pose;
    const auto poseChild =
      cylinder2_link2->FrameDataRelativeToWorld().pose;
    const auto poseParentChild = poseParent.inverse() * poseChild;
    auto jointPtrPhys =
      cylinder2_link2->AttachFixedJoint(cylinder1_link1);
    ASSERT_NE(nullptr, jointPtrPhys);
    jointPtrPhys->SetTransformFromParent(poseParentChild);

    {
      // Check initial conditions
      // Cylinder 1 is fixed 0.5m above cylinder 2, which is 1.5m above the
      // ground.
      auto frameDataC1L1 = cylinder1_link1->FrameDataRelativeToWorld();
      auto frameDataC2L2 = cylinder2_link2->FrameDataRelativeToWorld();
      EXPECT_EQ(gz::math::Pose3d(0, 0, 2, 0, 0, 0),
                gz::math::eigen3::convert(frameDataC1L1.pose));
      EXPECT_EQ(gz::math::Pose3d(0, 0, 1.5, 0, 0, 0),
                gz::math::eigen3::convert(frameDataC2L2.pose));
    }

    gz::physics::ForwardStep::Output output;
    gz::physics::ForwardStep::State state;
    gz::physics::ForwardStep::Input input;

    for (std::size_t i = 0; i < 1000 ; ++i)
    {
      // step forward and expect lower link to fall
      world->Step(output, state, input);
    }

    {
      // Check final conditions with joint attached
      // If the joint was attached correctly, the top cylinder should be
      // fixed 0.5m above the bottom cylinder, which is resting on the ground.
      auto frameDataC1L1 = cylinder1_link1->FrameDataRelativeToWorld();
      auto frameDataC2L2 = cylinder2_link2->FrameDataRelativeToWorld();
      EXPECT_NEAR(0.55, frameDataC1L1.pose.translation().z(), 1e-2);
      EXPECT_NEAR(0.05, frameDataC2L2.pose.translation().z(), 1e-2);
    }

    // Detach joint and step physics
    jointPtrPhys->Detach();
    for (std::size_t i = 0; i < 1000; ++i)
    {
      world->Step(output, state, input);
    }

    {
      // Check final conditions after joint detached
      // If the joint was detached correctly, the top cylinder should be
      // resting directly on the bottom cylinder, which is resting on the
      // ground.
      auto frameDataC1L1 = cylinder1_link1->FrameDataRelativeToWorld();
      auto frameDataC2L2 = cylinder2_link2->FrameDataRelativeToWorld();
      EXPECT_NEAR(0.15, frameDataC1L1.pose.translation().z(), 1e-2);
      EXPECT_NEAR(0.05, frameDataC2L2.pose.translation().z(), 1e-2);
    }
  }
}

TYPED_TEST(DetachableJointTest, DetachableJointHeavyPayloadWeld)
{
  for (const std::string &name : this->pluginNames)
  {
    std::cout << "Testing plugin: " << name << std::endl;
    gz::plugin::PluginPtr plugin = this->loader.Instantiate(name);

    auto engine =
        gz::physics::RequestEngine3d<TypeParam>::From(plugin);
    ASSERT_NE(nullptr, engine);

    std::string worldStr = R"(
    <sdf version="1.9">
      <world name="weld_test_world">
        <model name="M1">
          <static>true</static>
          <link name="parent_link">
            <inertial>
              <mass>1.0</mass>
              <inertia>
                <ixx>0.1</ixx><iyy>0.1</iyy><izz>0.1</izz>
              </inertia>
            </inertial>
          </link>
        </model>
        <model name="M2">
          <pose>1.0 0 1.0 0 0 0</pose>
          <link name="child_link">
            <inertial>
              <mass>1000.0</mass>
              <inertia>
                <ixx>100.0</ixx><iyy>100.0</iyy><izz>100.0</izz>
              </inertia>
            </inertial>
          </link>
        </model>
      </world>
    </sdf>)";

    sdf::Root root;
    const sdf::Errors errors = root.LoadSdfString(worldStr);
    ASSERT_TRUE(errors.empty()) << errors;

    auto world = engine->ConstructWorld(*root.WorldByIndex(0));
    ASSERT_NE(nullptr, world);

    auto model1 = world->GetModel("M1");
    auto model2 = world->GetModel("M2");
    ASSERT_NE(nullptr, model1);
    ASSERT_NE(nullptr, model2);

    auto parentLink = model1->GetLink("parent_link");
    auto childLink = model2->GetLink("child_link");
    ASSERT_NE(nullptr, parentLink);
    ASSERT_NE(nullptr, childLink);

    const auto frameDataParent = parentLink->FrameDataRelativeToWorld();
    const auto frameDataChild = childLink->FrameDataRelativeToWorld();

    const auto poseParent = frameDataParent.pose;
    const auto poseChild = frameDataChild.pose;
    const auto poseParentChild = poseParent.inverse() * poseChild;

    auto fixedJoint = childLink->AttachFixedJoint(parentLink);
    ASSERT_NE(nullptr, fixedJoint);
    fixedJoint->SetTransformFromParent(poseParentChild);

    gz::physics::ForwardStep::Output output;
    gz::physics::ForwardStep::State state;
    gz::physics::ForwardStep::Input input;

    // Step 2000 times (2.0 seconds of simulation time)
    for (std::size_t i = 0; i < 2000; ++i)
    {
      world->Step(output, state, input);
    }

    // Verify the relative pose is preserved with a very strict tolerance
    const auto newFrameDataParent = parentLink->FrameDataRelativeToWorld();
    const auto newFrameDataChild = childLink->FrameDataRelativeToWorld();

    const auto newPoseParentChild =
        newFrameDataParent.pose.inverse() * newFrameDataChild.pose;
    const gz::math::Pose3d gzPoseParentChild =
        gz::math::eigen3::convert(poseParentChild);
    const gz::math::Pose3d gzNewPoseParentChild =
        gz::math::eigen3::convert(newPoseParentChild);

    EXPECT_NEAR(gzPoseParentChild.Pos().X(), gzNewPoseParentChild.Pos().X(),
                1e-4);
    EXPECT_NEAR(gzPoseParentChild.Pos().Y(), gzNewPoseParentChild.Pos().Y(),
                1e-4);
    EXPECT_NEAR(gzPoseParentChild.Pos().Z(), gzNewPoseParentChild.Pos().Z(),
                1e-4);
  }
}


struct DetachableJointResetFeatureList: gz::physics::FeatureList<
    DetachableJointFeatureList,
    gz::physics::FindFreeGroupFeature,
    gz::physics::SetFreeGroupWorldPose,
    gz::physics::SetFreeGroupWorldVelocity
> { };

template <class T>
using DetachableJointResetTest = DetachableJointTest<T>;

using DetachableJointResetTestTypes =
  ::testing::Types<DetachableJointResetFeatureList>;
TYPED_TEST_SUITE(DetachableJointResetTest,
                 DetachableJointResetTestTypes);
// This test verifies both the reset behavior and different relative pose
// re-attachment of detachable joints.
// The workflow is:
// 1. Attach the joint initially at the default relative pose, step, and
//    detach it.
// 2. Step so that the child link falls.
// 3. Reset the simulation: Move the child model back to its original pose and
//    velocities, and re-attach the joint at the default relative pose.
//    Verify stability.
// 4. Detach again, manually move the child model to a different relative
//    pose (offset Z = -0.8), and re-attach the joint at this new relative
//    pose. Verify the constraint is correctly enforced at the new pose.
TYPED_TEST(DetachableJointResetTest, DetachableJointResetAndReattach)
{
  for (const std::string &name : this->pluginNames)
  {
    std::cout << "Testing plugin: " << name << std::endl;
    gz::plugin::PluginPtr plugin = this->loader.Instantiate(name);

    auto engine =
      gz::physics::RequestEngine3d<TypeParam>::From(plugin);
    ASSERT_NE(nullptr, engine);

    sdf::Root root;
    const sdf::Errors errors = root.Load(
      common_test::worlds::kDetachableJointWorld);
    ASSERT_TRUE(errors.empty()) << errors.front();

    auto world = engine->ConstructWorld(*root.WorldByIndex(0));
    ASSERT_NE(nullptr, world);

    auto cylinder1 = world->GetModel("cylinder1");
    ASSERT_NE(nullptr, cylinder1);
    auto cylinder1Link1 = cylinder1->GetLink("link1");

    auto cylinder2 = world->GetModel("cylinder2");
    ASSERT_NE(nullptr, cylinder2);
    auto cylinder2Link2 = cylinder2->GetLink("link2");

    // 1. Initial attachment
    const auto poseParent =
      cylinder1Link1->FrameDataRelativeToWorld().pose;
    const auto poseChild =
      cylinder2Link2->FrameDataRelativeToWorld().pose;
    const auto poseParentChild = poseParent.inverse() * poseChild;
    auto jointPtrPhys =
      cylinder2Link2->AttachFixedJoint(cylinder1Link1);
    ASSERT_NE(nullptr, jointPtrPhys);
    jointPtrPhys->SetTransformFromParent(poseParentChild);

    // Step 100 times
    gz::physics::ForwardStep::Output output;
    gz::physics::ForwardStep::State state;
    gz::physics::ForwardStep::Input input;
    for (std::size_t i = 0; i < 100; ++i)
      world->Step(output, state, input);

    // 2. Detach joint
    jointPtrPhys->Detach();

    // Step 100 times (cylinder 2 falls)
    for (std::size_t i = 0; i < 100; ++i)
      world->Step(output, state, input);

    // Verify cylinder 2 has fallen
    auto frameDataC2L2AfterFall =
        cylinder2Link2->FrameDataRelativeToWorld();
    EXPECT_LT(frameDataC2L2AfterFall.pose.translation().z(), 1.4);

    // 3. Reset Simulation Poses & Velocities
    // Reset cylinder 2 pose back to initial pose (z = 1.5) and velocities to 0
    auto freeGroupC2 = cylinder2->FindFreeGroup();
    ASSERT_NE(nullptr, freeGroupC2);
    freeGroupC2->SetWorldPose(poseChild);
    freeGroupC2->SetWorldLinearVelocity(Eigen::Vector3d::Zero());
    freeGroupC2->SetWorldAngularVelocity(Eigen::Vector3d::Zero());

    // Reset cylinder 1 base prismatic joint position and velocity to 0
    auto baseJoint = cylinder1->GetJoint("base_joint");
    ASSERT_NE(nullptr, baseJoint);
    baseJoint->SetPosition(0, 0.0);
    baseJoint->SetVelocity(0, 0.0);

    // Verify cylinder 2 pose has been updated in the physics engine
    auto frameDataC2L2AfterReset =
        cylinder2Link2->FrameDataRelativeToWorld();
    EXPECT_NEAR(1.5, frameDataC2L2AfterReset.pose.translation().z(), 1e-4);

    // 4. Re-attach joint at initial relative pose
    auto jointPtrPhysNew =
      cylinder2Link2->AttachFixedJoint(cylinder1Link1);
    ASSERT_NE(nullptr, jointPtrPhysNew);
    jointPtrPhysNew->SetTransformFromParent(poseParentChild);

    // Step simulation after reset. We expect reasonable velocities & stability
    for (std::size_t i = 0; i < 10; ++i)
    {
      world->Step(output, state, input);
      auto frameDataC1 = cylinder1Link1->FrameDataRelativeToWorld();
      auto frameDataC2 = cylinder2Link2->FrameDataRelativeToWorld();
      EXPECT_LT(frameDataC1.linearVelocity.norm(), 1.0);
      EXPECT_LT(frameDataC2.linearVelocity.norm(), 1.0);
    }

    // 5. Detach and re-attach at a different pose
    jointPtrPhysNew->Detach();

    // Move cylinder 2 to a new offset (different relative pose)
    // Initial z offset was -0.5. Let's make it -0.8 (z = 1.2)
    Eigen::Isometry3d poseChildNew = poseChild;
    poseChildNew.translation().z() = 1.2;
    const auto poseParentChildNew = poseParent.inverse() * poseChildNew;

    freeGroupC2->SetWorldPose(poseChildNew);
    freeGroupC2->SetWorldLinearVelocity(Eigen::Vector3d::Zero());
    freeGroupC2->SetWorldAngularVelocity(Eigen::Vector3d::Zero());

    // Re-attach joint at NEW relative pose
    auto jointPtrPhysDifferentPose =
      cylinder2Link2->AttachFixedJoint(cylinder1Link1);
    ASSERT_NE(nullptr, jointPtrPhysDifferentPose);
    jointPtrPhysDifferentPose->SetTransformFromParent(poseParentChildNew);

    // Step 10 times to let simulation stabilize
    for (std::size_t i = 0; i < 10; ++i)
      world->Step(output, state, input);

    // Verify cylinder 2 is held at the new relative pose (offset z = -0.8)
    auto frameDataC1Final = cylinder1Link1->FrameDataRelativeToWorld();
    auto frameDataC2Final = cylinder2Link2->FrameDataRelativeToWorld();
    double currentZOffset = frameDataC2Final.pose.translation().z() -
        frameDataC1Final.pose.translation().z();
    EXPECT_NEAR(-0.8, currentZOffset, 1e-3);
  }
}

int main(int argc, char *argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  if (!DetachableJointTest<DetachableJointFeatureList>::init(
       argc, argv))
    return -1;
  return RUN_ALL_TESTS();
}
