/*
 * Copyright (C) 2019 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 3.0 (the "License");
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

#include <iostream>

#include <ignition/physics/FindFeatures.hh>
#include <ignition/plugin/Loader.hh>
#include <ignition/physics/RequestEngine.hh>

#include <ignition/math/eigen3/Conversions.hh>

// Features
#include <ignition/physics/ForwardStep.hh>
#include <ignition/physics/FrameSemantics.hh>
#include <ignition/physics/GetBoundingBox.hh>
#include <ignition/physics/Link.hh>
#include <ignition/physics/sdf/ConstructWorld.hh>
#include <ignition/physics/sdf/ConstructModel.hh>
#include <ignition/physics/sdf/ConstructLink.hh>

#include <sdf/Root.hh>
#include <sdf/World.hh>

#include "test/Utils.hh"

struct TestFeatureList : ignition::physics::FeatureList<
    ignition::physics::AddLinkExternalForceTorque,
    ignition::physics::ForwardStep,
    ignition::physics::sdf::ConstructSdfWorld,
    ignition::physics::sdf::ConstructSdfModel,
    ignition::physics::sdf::ConstructSdfLink,
    ignition::physics::GetEntities,
    ignition::physics::GetLinkBoundingBox,
    ignition::physics::GetModelBoundingBox
> { };

using namespace ignition;

using TestEnginePtr = physics::Engine3dPtr<TestFeatureList>;
using TestWorldPtr = physics::World3dPtr<TestFeatureList>;

class LinkFeaturesFixture : public ::testing::Test
{
  protected: void SetUp() override
  {
    ignition::plugin::Loader loader;
    loader.LoadLib(DartsimPluginLib());

    ignition::plugin::PluginPtr dartsim =
        loader.Instantiate("ignition::physics::dartsim::Plugin");

    this->engine =
        ignition::physics::RequestEngine3d<TestFeatureList>::From(dartsim);
    ASSERT_NE(nullptr, this->engine);
  }
  protected: TestEnginePtr engine;
};

TestWorldPtr LoadWorld(
    const TestEnginePtr &_engine,
    const std::string &_sdfFile,
    const Eigen::Vector3d &_gravity = Eigen::Vector3d{0, 0, -9.8})
{
  sdf::Root root;
  const sdf::Errors errors = root.Load(_sdfFile);
  EXPECT_TRUE(errors.empty());
  const sdf::World *sdfWorld = root.WorldByIndex(0);
  // Make a copy of the world so we can set the gravity property
  // TODO(addisu) Add a world property feature to set gravity instead of this
  // hack
  sdf::World worldCopy;
  worldCopy.Load(sdfWorld->Element());

  worldCopy.SetGravity(math::eigen3::convert(_gravity));
  return _engine->ConstructWorld(worldCopy);
}

// A predicate-formatter for asserting that two vectors are approximately equal.
class AssertVectorApprox
{
  public: explicit AssertVectorApprox(double _tol = 1e-6) : tol(_tol)
  {
  }

  public: ::testing::AssertionResult operator()(
              const char *_mExpr, const char *_nExpr, Eigen::Vector3d _m,
              Eigen::Vector3d _n)
  {
    if (ignition::physics::test::Equal(_m, _n, this->tol))
      return ::testing::AssertionSuccess();

    return ::testing::AssertionFailure()
           << _mExpr << " and " << _nExpr << " ([" << _m.transpose()
           << "] and [" << _n.transpose() << "]"
           << ") are not equal";
  }

  private: double tol;
};

// Test setting force and torque.
TEST_F(LinkFeaturesFixture, LinkForceTorque)
{
  auto world = LoadWorld(this->engine, TEST_WORLD_DIR "/empty.sdf",
                         Eigen::Vector3d::Zero());
  // Add a sphere
  sdf::Model modelSDF;
  modelSDF.SetName("sphere");
  modelSDF.SetRawPose(math::Pose3d(0, 0, 2, 0, 0, IGN_PI));
  auto model = world->ConstructModel(modelSDF);

  const double mass = 1.0;
  math::MassMatrix3d massMatrix{mass, math::Vector3d{0.4, 0.4, 0.4},
                                math::Vector3d::Zero};

  sdf::Link linkSDF;
  linkSDF.SetName("sphere_link");
  linkSDF.SetInertial({massMatrix, math::Pose3d::Zero});
  auto link = model->ConstructLink(linkSDF);

  ignition::physics::ForwardStep::Input input;
  ignition::physics::ForwardStep::State state;
  ignition::physics::ForwardStep::Output output;

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
  Eigen::Matrix3d moi = math::eigen3::convert(massMatrix.Moi());

  // Apply forces in the world frame at zero offset
  // API: AddExternalForce(relForce, relPosition)
  // API: AddExternalTorque(relTorque)

  const Eigen::Vector3d cmdForce{1, -1, 0};
  link->AddExternalForce(
      physics::RelativeForce3d(physics::FrameID::World(), cmdForce),
      physics::RelativePosition3d(*link, Eigen::Vector3d::Zero()));

  const Eigen::Vector3d cmdTorque{0, 0, 0.1 * IGN_PI};
  link->AddExternalTorque(
      physics::RelativeTorque3d(physics::FrameID::World(), cmdTorque));

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
      physics::RelativeForce3d(*link, cmdLocalForce),
      physics::RelativePosition3d(*link, Eigen::Vector3d::Zero()));

  const Eigen::Vector3d cmdLocalTorque{0.1 * IGN_PI, 0, 0};
  link->AddExternalTorque(physics::RelativeTorque3d(*link, cmdLocalTorque));

  world->Step(output, state, input);

  {
    const Eigen::Vector3d expectedForce =
        Eigen::AngleAxisd(IGN_PI, Eigen::Vector3d::UnitZ()) * cmdLocalForce;

    const Eigen::Vector3d expectedTorque =
        Eigen::AngleAxisd(IGN_PI, Eigen::Vector3d::UnitZ()) * cmdLocalTorque;

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
  link->AddExternalForce(physics::RelativeForce3d(*link, cmdLocalForce),
                         physics::RelativePosition3d(*link, offset));

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

TEST_F(LinkFeaturesFixture, AxisAlignedBoundingBox)
{
  auto world =
      LoadWorld(this->engine, TEST_WORLD_DIR "test.world");
  auto model = world->GetModel("double_pendulum_with_base");
  auto baseLink = model->GetLink("base");
  auto bbox = baseLink->GetAxisAlignedBoundingBox();
  AssertVectorApprox vectorPredicate(1e-4);
  EXPECT_PRED_FORMAT2(
      vectorPredicate, physics::Vector3d(0.2, -0.8, 0), bbox.min());
  EXPECT_PRED_FORMAT2(
      vectorPredicate, physics::Vector3d(1.8, 0.8, 2.2), bbox.max());

  // test with non-world frame
  auto bboxModelFrame = baseLink->GetAxisAlignedBoundingBox(
      model->GetFrameID());
  EXPECT_PRED_FORMAT2(
      vectorPredicate, physics::Vector3d(-0.8, -0.8, 0), bboxModelFrame.min());
  EXPECT_PRED_FORMAT2(
      vectorPredicate, physics::Vector3d(0.8, 0.8, 2.2), bboxModelFrame.max());

  // test with non-world rotated frame
  auto upperLink = model->GetLink("upper_link");
  auto bboxUpperLinkFrame = baseLink->GetAxisAlignedBoundingBox(
      upperLink->GetFrameID());
  EXPECT_PRED_FORMAT2(vectorPredicate,
      physics::Vector3d(-0.8, -0.1, -0.8), bboxUpperLinkFrame.min());
  EXPECT_PRED_FORMAT2(vectorPredicate,
      physics::Vector3d(0.8, 2.1, 0.8), bboxUpperLinkFrame.max());
}

TEST_F(LinkFeaturesFixture, ModelAxisAlignedBoundingBox)
{
  auto world =
      LoadWorld(this->engine, TEST_WORLD_DIR "contact.sdf");
  auto model = world->GetModel("sphere");
  auto bbox = model->GetAxisAlignedBoundingBox();
  AssertVectorApprox vectorPredicate(1e-4);
  EXPECT_PRED_FORMAT2(
      vectorPredicate, physics::Vector3d(-1, -1, -0.5), bbox.min());
  EXPECT_PRED_FORMAT2(
      vectorPredicate, physics::Vector3d(2, 2, 1.5), bbox.max());

  // test with non-world frame
  auto link = model->GetLink("link0");
  auto bboxLinkFrame = model->GetAxisAlignedBoundingBox(
      link->GetFrameID());
  EXPECT_PRED_FORMAT2(
      vectorPredicate, physics::Vector3d(-1, -1, -1.0), bboxLinkFrame.min());
  EXPECT_PRED_FORMAT2(
      vectorPredicate, physics::Vector3d(2, 2, 1.0), bboxLinkFrame.max());
}

/////////////////////////////////////////////////
int main(int argc, char *argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
