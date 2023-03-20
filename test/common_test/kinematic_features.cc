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

#include "TestLibLoader.hh"
#include "../Utils.hh"

#include <gz/physics/ConstructEmpty.hh>
#include <gz/physics/Joint.hh>
#include <gz/physics/FrameSemantics.hh>
#include <gz/physics/FindFeatures.hh>
#include <gz/physics/ForwardStep.hh>
#include <gz/physics/GetEntities.hh>
#include <gz/physics/RequestEngine.hh>
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
    const sdf::Errors errors =
      root.Load(gz::common::joinPaths(TEST_WORLD_DIR, "string_pendulum.sdf"));
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

int main(int argc, char *argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  if (!KinematicFeaturesTest<KinematicFeaturesList>::init(
       argc, argv))
    return -1;
  return RUN_ALL_TESTS();
}
