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

// Features
#include <gz/physics/ConstructEmpty.hh>
#include <gz/physics/FindFeatures.hh>
#include <gz/physics/ForwardStep.hh>
#include <gz/physics/FrameSemantics.hh>
#include <gz/physics/FreeJoint.hh>
#include <gz/physics/FixedJoint.hh>
#include <gz/physics/GetEntities.hh>
#include <gz/physics/Joint.hh>
#include <gz/physics/Link.hh>
#include <gz/physics/Shape.hh>
#include <gz/physics/RevoluteJoint.hh>
#include <gz/physics/sdf/ConstructModel.hh>
#include <gz/physics/sdf/ConstructWorld.hh>
#include <gz/physics/sdf/ConstructLink.hh>

#include <sdf/Root.hh>

#include <gz/common/testing/TestPaths.hh>

template <class T>
class ShapeFeaturesTest:
  public testing::Test, public gz::physics::TestLibLoader
{
  // Documentation inherited
  public: void SetUp() override
  {
    gz::common::Console::SetVerbosity(4);

    loader.LoadLib(ShapeFeaturesTest::GetLibToTest());

    // TODO(ahcorde): We should also run the 3f, 2d, and 2f variants of
    // FindFeatures
    pluginNames = gz::physics::FindFeatures3d<T>::From(loader);
    if (pluginNames.empty())
    {
      std::cerr << "No plugins with required features found in "
                << GetLibToTest() << std::endl;
      GTEST_SKIP();
    }
  }

  public: std::set<std::string> pluginNames;
  public: gz::plugin::Loader loader;
};

using AssertVectorApprox = gz::physics::test::AssertVectorApprox;

struct ShapeFeaturesList : gz::physics::FeatureList<
  gz::physics::AttachFixedJointFeature,
  gz::physics::AddLinkExternalForceTorque,
  gz::physics::LinkFrameSemantics,
  gz::physics::DetachJointFeature,
  gz::physics::SetJointTransformFromParentFeature,
  gz::physics::ForwardStep,
  gz::physics::FreeJointCast,
  gz::physics::GetBasicJointState,
  gz::physics::GetEntities,
  gz::physics::RevoluteJointCast,
  gz::physics::SetJointVelocityCommandFeature,
  gz::physics::GetShapeFrictionPyramidSlipCompliance,
  gz::physics::SetShapeFrictionPyramidSlipCompliance,
  gz::physics::sdf::ConstructSdfModel,
  gz::physics::sdf::ConstructSdfWorld,
  gz::physics::sdf::ConstructSdfLink
> { };

using ShapeFeaturesTestTypes =
  ::testing::Types<ShapeFeaturesList>;
TYPED_TEST_SUITE(ShapeFeaturesTest,
                 ShapeFeaturesTestTypes);

TYPED_TEST(ShapeFeaturesTest, PrimarySlipCompliance)
{
  for (const std::string &name : this->pluginNames)
  {
    std::cout << "Testing plugin: " << name << std::endl;
    gz::plugin::PluginPtr plugin = this->loader.Instantiate(name);

    auto engine = gz::physics::RequestEngine3d<ShapeFeaturesList>::From(plugin);
    ASSERT_NE(nullptr, engine);

    const auto worldPath =
      gz::common::testing::TestFile("common_test", "worlds", "slip_compliance.sdf");
    sdf::Root root;
    const sdf::Errors errors = root.Load(worldPath);
    ASSERT_TRUE(errors.empty()) << errors.front();

    const std::string modelName{"box"};
    const std::string linkName{"box_link"};
    const std::string shapeName{"box_collision"};

    auto world = engine->ConstructWorld(*root.WorldByIndex(0));

    auto model = world->GetModel(modelName);
    auto boxLink = model->GetLink(linkName);
    auto boxShape = boxLink->GetShape(shapeName);

    AssertVectorApprox vectorPredicate(1e-4);

    gz::physics::ForwardStep::Input input;
    gz::physics::ForwardStep::State state;
    gz::physics::ForwardStep::Output output;

    // Check that link is at rest
    {
      const auto frameData = boxLink->FrameDataRelativeToWorld();

      EXPECT_PRED_FORMAT2(vectorPredicate, Eigen::Vector3d::Zero(),
                          frameData.linearVelocity);
      EXPECT_PRED_FORMAT2(vectorPredicate, Eigen::Vector3d::Zero(),
                          frameData.angularVelocity);
      EXPECT_PRED_FORMAT2(vectorPredicate, Eigen::Vector3d::Zero(),
                          frameData.linearAcceleration);
      EXPECT_PRED_FORMAT2(vectorPredicate, Eigen::Vector3d::Zero(),
                          frameData.angularAcceleration);
    }

    const Eigen::Vector3d cmdForce{1, 0, 0};
    const double primarySlip = 0.5;

    // expect 0.0 initial slip
    EXPECT_DOUBLE_EQ(0.0, boxShape->GetPrimarySlipCompliance());

    boxShape->SetPrimarySlipCompliance(primarySlip);
    EXPECT_DOUBLE_EQ(primarySlip, boxShape->GetPrimarySlipCompliance());

    const std::size_t numSteps = 10000;
    for (std::size_t i = 0; i < numSteps; ++i)
    {
      world->Step(output, state, input);
      boxLink->AddExternalForce(
          gz::physics::RelativeForce3d(gz::physics::FrameID::World(), cmdForce),
          gz::physics::RelativePosition3d(*boxLink, Eigen::Vector3d::Zero()));
    }

    {
      // velocity = slip * applied force
      const auto frameData = boxLink->FrameDataRelativeToWorld();
      EXPECT_PRED_FORMAT2(vectorPredicate, primarySlip * cmdForce,
                          (frameData.linearVelocity));
    }
  }
}

/////////////////////////////////////////////////
TYPED_TEST(ShapeFeaturesTest, SecondarySlipCompliance)
{
  for (const std::string &name : this->pluginNames)
  {
    std::cout << "Testing plugin: " << name << std::endl;
    gz::plugin::PluginPtr plugin = this->loader.Instantiate(name);

    auto engine = gz::physics::RequestEngine3d<ShapeFeaturesList>::From(plugin);
    ASSERT_NE(nullptr, engine);

    const auto worldPath =
      gz::common::testing::TestFile("common_test", "worlds", "slip_compliance.sdf");
    sdf::Root root;
    const sdf::Errors errors = root.Load(worldPath);
    ASSERT_TRUE(errors.empty()) << errors.front();

    const std::string modelName{"box"};
    const std::string linkName{"box_link"};
    const std::string shapeName{"box_collision"};

    auto world = engine->ConstructWorld(*root.WorldByIndex(0));

    auto model = world->GetModel(modelName);
    auto boxLink = model->GetLink(linkName);
    auto boxShape = boxLink->GetShape(shapeName);

    AssertVectorApprox vectorPredicate(1e-4);

    gz::physics::ForwardStep::Input input;
    gz::physics::ForwardStep::State state;
    gz::physics::ForwardStep::Output output;

    // Check that link is at rest
    {
      const auto frameData = boxLink->FrameDataRelativeToWorld();

      EXPECT_PRED_FORMAT2(vectorPredicate, Eigen::Vector3d::Zero(),
                          frameData.linearVelocity);
      EXPECT_PRED_FORMAT2(vectorPredicate, Eigen::Vector3d::Zero(),
                          frameData.angularVelocity);
      EXPECT_PRED_FORMAT2(vectorPredicate, Eigen::Vector3d::Zero(),
                          frameData.linearAcceleration);
      EXPECT_PRED_FORMAT2(vectorPredicate, Eigen::Vector3d::Zero(),
                          frameData.angularAcceleration);
    }

    const Eigen::Vector3d cmdForce{0, 1, 0};
    const double secondarySlip = 0.25;

    // expect 0.0 initial slip
    EXPECT_DOUBLE_EQ(0.0, boxShape->GetSecondarySlipCompliance());

    boxShape->SetSecondarySlipCompliance(secondarySlip);
    EXPECT_DOUBLE_EQ(secondarySlip, boxShape->GetSecondarySlipCompliance());

    const std::size_t numSteps = 10000;
    for (std::size_t i = 0; i < numSteps; ++i)
    {
      world->Step(output, state, input);
      boxLink->AddExternalForce(
          gz::physics::RelativeForce3d(gz::physics::FrameID::World(), cmdForce),
          gz::physics::RelativePosition3d(*boxLink, Eigen::Vector3d::Zero()));
    }

    {
      // velocity = slip * applied force
      const auto frameData = boxLink->FrameDataRelativeToWorld();
      EXPECT_PRED_FORMAT2(vectorPredicate, secondarySlip * cmdForce,
                          (frameData.linearVelocity));
    }
  }
}

int main(int argc, char *argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  if (!ShapeFeaturesTest<ShapeFeaturesList>::init(
       argc, argv))
    return -1;
  return RUN_ALL_TESTS();
}
