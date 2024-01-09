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

#include <gz/math/Pose3.hh>
#include <gz/math/MassMatrix3.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/eigen3/Conversions.hh>

#include "worlds/Worlds.hh"
#include "test/TestLibLoader.hh"
#include "test/Utils.hh"

#include <gz/physics/FindFeatures.hh>
#include <gz/physics/GetEntities.hh>
#include <gz/physics/RequestEngine.hh>
#include <gz/physics/ForwardStep.hh>
#include <gz/physics/FrameSemantics.hh>
#include <gz/physics/GetBoundingBox.hh>
#include <gz/physics/Link.hh>
#include <gz/physics/World.hh>
#include <gz/physics/sdf/ConstructWorld.hh>
#include <gz/physics/sdf/ConstructModel.hh>
#include <gz/physics/sdf/ConstructLink.hh>

#include <sdf/Root.hh>
#include <sdf/Model.hh>

template <class T>
class LinkFeaturesTest:
  public testing::Test, public gz::physics::TestLibLoader
{
  // Documentation inherited
  public: void SetUp() override
  {
    gz::common::Console::SetVerbosity(4);

    loader.LoadLib(LinkFeaturesTest::GetLibToTest());

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

using AssertVectorApprox = gz::physics::test::AssertVectorApprox;

struct LinkFeaturesList : gz::physics::FeatureList<
    gz::physics::AddLinkExternalForceTorque,
    gz::physics::ForwardStep,
    gz::physics::Gravity,
    gz::physics::sdf::ConstructSdfWorld,
    gz::physics::sdf::ConstructSdfModel,
    gz::physics::sdf::ConstructSdfLink,
    gz::physics::GetEntities,
    gz::physics::GetLinkBoundingBox,
    gz::physics::GetModelBoundingBox
> { };

using LinkFeaturesTestTypes =
  ::testing::Types<LinkFeaturesList>;
TYPED_TEST_SUITE(LinkFeaturesTest,
                 LinkFeaturesTestTypes);

TYPED_TEST(LinkFeaturesTest, JointSetCommand)
{
  for (const std::string &name : this->pluginNames)
  {
    std::cout << "Testing plugin: " << name << std::endl;
    gz::plugin::PluginPtr plugin = this->loader.Instantiate(name);

    auto engine = gz::physics::RequestEngine3d<LinkFeaturesList>::From(plugin);
    ASSERT_NE(nullptr, engine);

    sdf::Root root;
    const sdf::Errors errors = root.Load(worlds::kEmptySdf);
    ASSERT_TRUE(errors.empty()) << errors.front();

    const std::string modelName{"double_pendulum_with_base"};
    const std::string jointName{"upper_joint"};

    auto world = engine->ConstructWorld(*root.WorldByIndex(0));
    EXPECT_NE(nullptr, world);

    EXPECT_NE(nullptr, world);
    world->SetGravity(Eigen::Vector3d::Zero());

    AssertVectorApprox vectorPredicateGravity(1e-10);
    EXPECT_PRED_FORMAT2(vectorPredicateGravity, Eigen::Vector3d::Zero(),
      world->GetGravity());

    // Add a sphere
    sdf::Model modelSDF;
    modelSDF.SetName("sphere");
    modelSDF.SetRawPose(gz::math::Pose3d(0, 0, 2, 0, 0, GZ_PI));
    auto model = world->ConstructModel(modelSDF);

    const double mass = 1.0;
    gz::math::MassMatrix3d massMatrix{
      mass,
      gz::math::Vector3d{0.4, 0.4, 0.4},
      gz::math::Vector3d::Zero};

    sdf::Link linkSDF;
    linkSDF.SetName("sphere_link");
    linkSDF.SetInertial({massMatrix, gz::math::Pose3d::Zero});
    auto link = model->ConstructLink(linkSDF);

    gz::physics::ForwardStep::Input input;
    gz::physics::ForwardStep::State state;
    gz::physics::ForwardStep::Output output;

    AssertVectorApprox vectorPredicate(1e-4);

    // Check that link is at rest
    {
      const auto frameData = link->FrameDataRelativeToWorld();

      EXPECT_PRED_FORMAT2(vectorPredicate, Eigen::Vector3d::Zero(),
                          frameData.linearVelocity);
      EXPECT_PRED_FORMAT2(vectorPredicate, Eigen::Vector3d::Zero(),
                          frameData.angularVelocity);
      EXPECT_PRED_FORMAT2(vectorPredicate, Eigen::Vector3d::Zero(),
                          frameData.linearAcceleration);
      EXPECT_PRED_FORMAT2(vectorPredicate, Eigen::Vector3d::Zero(),
                          frameData.angularAcceleration);
    }

    // The moment of inertia of the sphere is a multiple of the identity matrix.
    // This means that the moi is invariant to rotation so we can use this matrix
    // without converting it to the world frame.
    Eigen::Matrix3d moi = gz::math::eigen3::convert(massMatrix.Moi());

    // Apply forces in the world frame at zero offset
    // API: AddExternalForce(relForce, relPosition)
    // API: AddExternalTorque(relTorque)

    const Eigen::Vector3d cmdForce{1, -1, 0};
    link->AddExternalForce(
        gz::physics::RelativeForce3d(gz::physics::FrameID::World(), cmdForce),
        gz::physics::RelativePosition3d(*link, Eigen::Vector3d::Zero()));

    const Eigen::Vector3d cmdTorque{0, 0, 0.1 * GZ_PI};
    link->AddExternalTorque(
        gz::physics::RelativeTorque3d(gz::physics::FrameID::World(), cmdTorque));

    world->Step(output, state, input);

    {
      const auto frameData = link->FrameDataRelativeToWorld();
      EXPECT_PRED_FORMAT2(vectorPredicate, cmdForce,
                          mass * (frameData.linearAcceleration));

      // The moment of inertia of the sphere is a multiple of the identity matrix.
      // Hence the gyroscopic coupling terms are zero
      EXPECT_PRED_FORMAT2(vectorPredicate, cmdTorque,
                          moi * frameData.angularAcceleration);
    }

    world->Step(output, state, input);

    // Check that the forces and torques are reset
    {
      const auto frameData = link->FrameDataRelativeToWorld();

      EXPECT_PRED_FORMAT2(vectorPredicate, Eigen::Vector3d::Zero(),
                          frameData.linearAcceleration);

      EXPECT_PRED_FORMAT2(vectorPredicate, Eigen::Vector3d::Zero(),
                          frameData.angularAcceleration);
    }

    // Apply forces in the local frame
    // The sphere is rotated by pi in the +z so the local x and y axes are in
    // the -x and -y of the world frame respectively
    const Eigen::Vector3d cmdLocalForce{1, -1, 0};
    link->AddExternalForce(
        gz::physics::RelativeForce3d(*link, cmdLocalForce),
        gz::physics::RelativePosition3d(*link, Eigen::Vector3d::Zero()));

    const Eigen::Vector3d cmdLocalTorque{0.1 * GZ_PI, 0, 0};
    link->AddExternalTorque(gz::physics::RelativeTorque3d(*link, cmdLocalTorque));

    world->Step(output, state, input);

    {
      const Eigen::Vector3d expectedForce =
          Eigen::AngleAxisd(GZ_PI, Eigen::Vector3d::UnitZ()) * cmdLocalForce;

      const Eigen::Vector3d expectedTorque =
          Eigen::AngleAxisd(GZ_PI, Eigen::Vector3d::UnitZ()) * cmdLocalTorque;

      const auto frameData = link->FrameDataRelativeToWorld();

      EXPECT_PRED_FORMAT2(vectorPredicate, expectedForce,
                          mass * (frameData.linearAcceleration));

      // The moment of inertia of the sphere is a multiple of the identity matrix.
      // Hence the gyroscopic coupling terms are zero
      EXPECT_PRED_FORMAT2(vectorPredicate, expectedTorque,
                          moi * frameData.angularAcceleration);
    }

    // Test the other AddExternalForce and AddExternalTorque APIs
    // API: AddExternalForce(force)
    // API: AddExternalTorque(torque)
    link->AddExternalForce(cmdForce);
    link->AddExternalTorque(cmdTorque);

    world->Step(output, state, input);

    {
      const auto frameData = link->FrameDataRelativeToWorld();
      EXPECT_PRED_FORMAT2(vectorPredicate, cmdForce,
                          mass * (frameData.linearAcceleration));

      // The moment of inertia of the sphere is a multiple of the identity matrix.
      // Hence the gyroscopic coupling terms are zero
      EXPECT_PRED_FORMAT2(vectorPredicate, cmdTorque,
                          moi * frameData.angularAcceleration);
    }

    // Apply the force at an offset
    // API: AddExternalForce(relForce, relPosition)
    Eigen::Vector3d offset{0.1, 0.2, 0.3};
    link->AddExternalForce(gz::physics::RelativeForce3d(*link, cmdLocalForce),
                           gz::physics::RelativePosition3d(*link, offset));

    world->Step(output, state, input);
    {
      const auto frameData = link->FrameDataRelativeToWorld();
      EXPECT_PRED_FORMAT2(vectorPredicate,
                          frameData.pose.linear() * cmdLocalForce,
                          mass * (frameData.linearAcceleration));

      // The moment of inertia of the sphere is a multiple of the identity matrix.
      // Hence the gyroscopic coupling terms are zero
      EXPECT_PRED_FORMAT2(vectorPredicate,
                          frameData.pose.linear() * offset.cross(cmdLocalForce),
                          moi * frameData.angularAcceleration);
    }

    // Apply force at an offset using the more convenient API
    // API: AddExternalForce(force, frame, position)
    link->AddExternalForce(cmdLocalForce, *link, offset);

    world->Step(output, state, input);
    {
      const auto frameData = link->FrameDataRelativeToWorld();
      EXPECT_PRED_FORMAT2(vectorPredicate,
                          frameData.pose.linear() * cmdLocalForce,
                          mass * (frameData.linearAcceleration));

      // The moment of inertia of the sphere is a multiple of the identity matrix.
      // Hence the gyroscopic coupling terms are zero
      EXPECT_PRED_FORMAT2(vectorPredicate,
                          frameData.pose.linear() * offset.cross(cmdLocalForce),
                          moi * frameData.angularAcceleration);
    }
  }
}

TYPED_TEST(LinkFeaturesTest, AxisAlignedBoundingBox)
{
  for (const std::string &name : this->pluginNames)
  {
    std::cout << "Testing plugin: " << name << std::endl;
    gz::plugin::PluginPtr plugin = this->loader.Instantiate(name);

    auto engine = gz::physics::RequestEngine3d<LinkFeaturesList>::From(plugin);
    ASSERT_NE(nullptr, engine);

    sdf::Root root;
    const sdf::Errors errors = root.Load(worlds::kTestWorld);
    ASSERT_TRUE(errors.empty()) << errors.front();

    const std::string modelName{"double_pendulum_with_base"};
    const std::string jointName{"upper_joint"};

    auto world = engine->ConstructWorld(*root.WorldByIndex(0));
    EXPECT_NE(nullptr, world);

    EXPECT_NE(nullptr, world);
    world->SetGravity(Eigen::Vector3d{0, 0, -9.8});

    auto model = world->GetModel("double_pendulum_with_base");
    auto baseLink = model->GetLink("base");
    auto bbox = baseLink->GetAxisAlignedBoundingBox();
    AssertVectorApprox vectorPredicate(1e-4);
    EXPECT_PRED_FORMAT2(
        vectorPredicate, gz::physics::Vector3d(0.2, -0.8, 0), bbox.min());
    EXPECT_PRED_FORMAT2(
        vectorPredicate, gz::physics::Vector3d(1.8, 0.8, 2.2), bbox.max());

    // test with non-world frame
    auto bboxModelFrame = baseLink->GetAxisAlignedBoundingBox(
        model->GetFrameID());
    EXPECT_PRED_FORMAT2(
        vectorPredicate, gz::physics::Vector3d(-0.8, -0.8, 0), bboxModelFrame.min());
    EXPECT_PRED_FORMAT2(
        vectorPredicate, gz::physics::Vector3d(0.8, 0.8, 2.2), bboxModelFrame.max());

    // test with non-world rotated frame
    auto upperLink = model->GetLink("upper_link");
    auto bboxUpperLinkFrame = baseLink->GetAxisAlignedBoundingBox(
        upperLink->GetFrameID());
    EXPECT_PRED_FORMAT2(vectorPredicate,
        gz::physics::Vector3d(-0.8, -0.1, -0.8), bboxUpperLinkFrame.min());
    EXPECT_PRED_FORMAT2(vectorPredicate,
        gz::physics::Vector3d(0.8, 2.1, 0.8), bboxUpperLinkFrame.max());
  }
}

TYPED_TEST(LinkFeaturesTest, ModelAxisAlignedBoundingBox)
{
  for (const std::string &name : this->pluginNames)
  {
    std::cout << "Testing plugin: " << name << std::endl;
    gz::plugin::PluginPtr plugin = this->loader.Instantiate(name);

    auto engine = gz::physics::RequestEngine3d<LinkFeaturesList>::From(plugin);
    ASSERT_NE(nullptr, engine);

    sdf::Root root;
    const sdf::Errors errors = root.Load(worlds::kContactSdf);
    ASSERT_TRUE(errors.empty()) << errors.front();

    const std::string modelName{"double_pendulum_with_base"};
    const std::string jointName{"upper_joint"};

    auto world = engine->ConstructWorld(*root.WorldByIndex(0));
    EXPECT_NE(nullptr, world);

    EXPECT_NE(nullptr, world);
    world->SetGravity(Eigen::Vector3d{0, 0, -9.8});

    auto model = world->GetModel("sphere");
    auto bbox = model->GetAxisAlignedBoundingBox();
    AssertVectorApprox vectorPredicate(1e-4);
    EXPECT_PRED_FORMAT2(
        vectorPredicate, gz::physics::Vector3d(-1, -1, -0.5), bbox.min());
    EXPECT_PRED_FORMAT2(
        vectorPredicate, gz::physics::Vector3d(2, 2, 1.5), bbox.max());

    // test with non-world frame
    auto link = model->GetLink("link0");
    auto bboxLinkFrame = model->GetAxisAlignedBoundingBox(
        link->GetFrameID());
    EXPECT_PRED_FORMAT2(
        vectorPredicate, gz::physics::Vector3d(-1, -1, -1.0), bboxLinkFrame.min());
    EXPECT_PRED_FORMAT2(
        vectorPredicate, gz::physics::Vector3d(2, 2, 1.0), bboxLinkFrame.max());
  }
}

int main(int argc, char *argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  if (!LinkFeaturesTest<LinkFeaturesList>::init(
       argc, argv))
    return -1;
  return RUN_ALL_TESTS();
}
