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

int main(int argc, char *argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  if (!DetachableJointTest<DetachableJointFeatureList>::init(
       argc, argv))
    return -1;
  return RUN_ALL_TESTS();
}
