/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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
#include <cstddef>
#include <string>

#include <Eigen/Geometry>

#include <sdf/Root.hh>
#include <sdf/World.hh>

#include <gz/plugin/Loader.hh>
#include <gz/physics/FeatureList.hh>
#include <gz/physics/ForwardStep.hh>
#include <gz/physics/FreeGroup.hh>
#include <gz/physics/GetEntities.hh>
#include <gz/physics/Joint.hh>
#include <gz/physics/Link.hh>
#include <gz/physics/RequestEngine.hh>
#include <gz/physics/sdf/ConstructModel.hh>
#include <gz/physics/sdf/ConstructWorld.hh>

#include <mujoco/mujoco.h>

#include <test/Utils.hh>

#include "Base.hh"

struct TestFeatures : public gz::physics::FeatureList<
    gz::physics::AddLinkExternalForceTorque,
    gz::physics::FindFreeGroupFeature,
    gz::physics::ForwardStep,
    gz::physics::GetBasicJointState,
    gz::physics::GetJointFromModel,
    gz::physics::GetLinkFromModel,
    gz::physics::GetModelFromWorld,
    gz::physics::LinkFrameSemantics,
    gz::physics::SetBasicJointState,
    gz::physics::SetFreeGroupWorldVelocity,
    gz::physics::SetJointVelocityCommandFeature,
    gz::physics::sdf::ConstructSdfWorld>
{
};

using WorldPtr = gz::physics::World3dPtr<TestFeatures>;
using LinkPtr = gz::physics::Link3dPtr<TestFeatures>;
using AssertVectorApprox = gz::physics::test::AssertVectorApprox;

/////////////////////////////////////////////////
WorldPtr LoadWorld(const std::string &_sdf)
{
  gz::plugin::Loader loader;
  loader.LoadLib(mujoco_plugin_LIB);
  auto plugin = loader.Instantiate("gz::physics::mujoco::Plugin");
  EXPECT_TRUE(plugin);
  auto engine = gz::physics::RequestEngine3d<TestFeatures>::From(plugin);
  EXPECT_TRUE(engine);
  if (!engine)
    return nullptr;

  sdf::Root root;
  const sdf::Errors errors = root.LoadSdfString(_sdf);
  EXPECT_TRUE(errors.empty()) << errors.front();
  if (!errors.empty())
    return nullptr;
  return engine->ConstructWorld(*root.WorldByIndex(0));
}

/////////////////////////////////////////////////
/// \brief A world with the given gravity and a 2 kg box whose center of mass
/// is offset from its link origin by _comX along x.
std::string BoxWorld(const std::string &_gravity, double _z, double _comX,
                     bool _ground)
{
  const std::string ground = R"(
    <model name="ground">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal></plane></geometry>
        </collision>
      </link>
    </model>)";
  return R"(<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="default">
    <gravity>)" + _gravity + R"(</gravity>)" + (_ground ? ground : "") + R"(
    <model name="box">
      <pose>0 0 )" + std::to_string(_z) + R"( 0 0 0</pose>
      <link name="link">
        <inertial>
          <pose>)" + std::to_string(_comX) + R"( 0 0 0 0 0</pose>
          <mass>2.0</mass>
          <inertia><ixx>0.01</ixx><iyy>0.01</iyy><izz>0.01</izz></inertia>
        </inertial>
        <collision name="collision">
          <pose>)" + std::to_string(_comX) + R"( 0 0 0 0 0</pose>
          <geometry><box><size>0.2 0.2 0.2</size></box></geometry>
        </collision>
      </link>
    </model>
  </world>
</sdf>)";
}

/////////////////////////////////////////////////
LinkPtr GetBoxLink(const WorldPtr &_world)
{
  auto model = _world->GetModel("box");
  EXPECT_TRUE(model);
  return model ? model->GetLink("link") : nullptr;
}

/////////////////////////////////////////////////
// Before the first step there are no solved accelerations, so the link
// reports none, even under gravity.
TEST(LinkAcceleration, ZeroBeforeFirstStep)
{
  auto world = LoadWorld(BoxWorld("0 0 -9.8", 1.0, 0.0, false));
  ASSERT_TRUE(world);
  auto link = GetBoxLink(world);
  ASSERT_TRUE(link);

  AssertVectorApprox vectorPredicate(1e-12);
  const auto frameData = link->FrameDataRelativeToWorld();
  EXPECT_PRED_FORMAT2(vectorPredicate, Eigen::Vector3d::Zero(),
                      frameData.linearAcceleration);
  EXPECT_PRED_FORMAT2(vectorPredicate, Eigen::Vector3d::Zero(),
                      frameData.angularAcceleration);
}

/////////////////////////////////////////////////
// A falling body accelerates at gravity. This is what gz-sim's IMU system
// subtracts gravity from, so a falling accelerometer reads zero.
TEST(LinkAcceleration, FreeFall)
{
  auto world = LoadWorld(BoxWorld("0 0 -9.8", 10.0, 0.0, false));
  ASSERT_TRUE(world);
  auto link = GetBoxLink(world);
  ASSERT_TRUE(link);

  gz::physics::ForwardStep::Input input;
  gz::physics::ForwardStep::State state;
  gz::physics::ForwardStep::Output output;
  world->Step(output, state, input);

  AssertVectorApprox vectorPredicate(1e-9);
  const auto frameData = link->FrameDataRelativeToWorld();
  EXPECT_PRED_FORMAT2(vectorPredicate, Eigen::Vector3d(0, 0, -9.8),
                      frameData.linearAcceleration);
  EXPECT_PRED_FORMAT2(vectorPredicate, Eigen::Vector3d::Zero(),
                      frameData.angularAcceleration);
}

/////////////////////////////////////////////////
// A body resting on the ground doesn't accelerate: the contact force cancels
// gravity.
TEST(LinkAcceleration, RestingOnGround)
{
  auto world = LoadWorld(BoxWorld("0 0 -9.8", 0.1, 0.0, true));
  ASSERT_TRUE(world);
  auto link = GetBoxLink(world);
  ASSERT_TRUE(link);

  gz::physics::ForwardStep::Input input;
  gz::physics::ForwardStep::State state;
  gz::physics::ForwardStep::Output output;
  for (std::size_t i = 0; i < 2000; ++i)
    world->Step(output, state, input);

  AssertVectorApprox vectorPredicate(1e-3);
  const auto frameData = link->FrameDataRelativeToWorld();
  EXPECT_PRED_FORMAT2(vectorPredicate, Eigen::Vector3d::Zero(),
                      frameData.linearAcceleration);
  EXPECT_PRED_FORMAT2(vectorPredicate, Eigen::Vector3d::Zero(),
                      frameData.angularAcceleration);
}

/////////////////////////////////////////////////
// An external force and torque produce F / m and tau / I for one step, and
// the accelerations return to zero once they are cleared.
TEST(LinkAcceleration, ExternalForceAndTorque)
{
  auto world = LoadWorld(BoxWorld("0 0 0", 1.0, 0.0, false));
  ASSERT_TRUE(world);
  auto link = GetBoxLink(world);
  ASSERT_TRUE(link);

  gz::physics::ForwardStep::Input input;
  gz::physics::ForwardStep::State state;
  gz::physics::ForwardStep::Output output;

  link->AddExternalForce(Eigen::Vector3d(2, -4, 0));
  link->AddExternalTorque(Eigen::Vector3d(0, 0, 0.1));
  world->Step(output, state, input);

  AssertVectorApprox vectorPredicate(1e-6);
  auto frameData = link->FrameDataRelativeToWorld();
  EXPECT_PRED_FORMAT2(vectorPredicate, Eigen::Vector3d(1, -2, 0),
                      frameData.linearAcceleration);
  EXPECT_PRED_FORMAT2(vectorPredicate, Eigen::Vector3d(0, 0, 10),
                      frameData.angularAcceleration);

  world->Step(output, state, input);
  frameData = link->FrameDataRelativeToWorld();
  EXPECT_PRED_FORMAT2(vectorPredicate, Eigen::Vector3d::Zero(),
                      frameData.linearAcceleration);
  EXPECT_PRED_FORMAT2(vectorPredicate, Eigen::Vector3d::Zero(),
                      frameData.angularAcceleration);
}

/////////////////////////////////////////////////
// A body spinning freely about its center of mass, which is 0.5 m from the
// link origin. The center of mass doesn't accelerate, but the link origin
// moves on a circle around it, so its acceleration is centripetal:
// w^2 * (com - origin), 2 m/s^2 toward the center of mass. The reported
// acceleration is the one the step integrated, so it equals the change in
// velocity over the step divided by the step size.
TEST(LinkAcceleration, CentripetalAtOffsetOrigin)
{
  auto world = LoadWorld(BoxWorld("0 0 0", 1.0, 0.5, false));
  ASSERT_TRUE(world);
  auto link = GetBoxLink(world);
  ASSERT_TRUE(link);
  auto freeGroup = world->GetModel("box")->FindFreeGroup();
  ASSERT_TRUE(freeGroup);

  const double w = 2.0;
  freeGroup->SetWorldAngularVelocity(Eigen::Vector3d(0, 0, w));

  const double dt = 0.001;
  gz::physics::ForwardStep::Input input;
  gz::physics::ForwardStep::State state;
  gz::physics::ForwardStep::Output output;
  input.Get<std::chrono::steady_clock::duration>() =
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(dt));

  const auto before = link->FrameDataRelativeToWorld();
  const Eigen::Vector3d expected =
      w * w * (before.pose * Eigen::Vector3d(0.5, 0, 0) -
               before.pose.translation());
  ASSERT_NEAR(2.0, expected.norm(), 1e-9);

  world->Step(output, state, input);
  const auto after = link->FrameDataRelativeToWorld();

  AssertVectorApprox vectorPredicate(1e-6);
  EXPECT_PRED_FORMAT2(vectorPredicate, expected, after.linearAcceleration);
  EXPECT_PRED_FORMAT2(vectorPredicate,
                      (after.linearVelocity - before.linearVelocity) / dt,
                      after.linearAcceleration);
  EXPECT_PRED_FORMAT2(vectorPredicate, Eigen::Vector3d::Zero(),
                      after.angularAcceleration);
}

/////////////////////////////////////////////////
/// \brief Step input with the given step size.
gz::physics::ForwardStep::Input StepInput(double _dt)
{
  gz::physics::ForwardStep::Input input;
  input.Get<std::chrono::steady_clock::duration>() =
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(_dt));
  return input;
}

/////////////////////////////////////////////////
/// \brief A world with a base fixed to the world and an arm attached to it by
/// a revolute joint. The arm's link origin is 1 m above the world origin and
/// the joint is _jointX along the arm's x axis from it, rotating about _axis.
std::string ArmWorld(double _jointX, const std::string &_axis)
{
  return R"(<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="default">
    <gravity>0 0 -9.8</gravity>
    <model name="arm">
      <joint name="fix" type="fixed">
        <parent>world</parent>
        <child>base</child>
      </joint>
      <link name="base">
        <inertial>
          <mass>1.0</mass>
          <inertia><ixx>0.01</ixx><iyy>0.01</iyy><izz>0.01</izz></inertia>
        </inertial>
      </link>
      <link name="link">
        <pose>0 0 1 0 0 0</pose>
        <inertial>
          <mass>2.0</mass>
          <inertia><ixx>0.01</ixx><iyy>0.01</iyy><izz>0.01</izz></inertia>
        </inertial>
      </link>
      <joint name="joint" type="revolute">
        <pose>)" + std::to_string(_jointX) + R"( 0 0 0 0 0</pose>
        <parent>base</parent>
        <child>link</child>
        <axis><xyz>)" + _axis + R"(</xyz></axis>
      </joint>
    </model>
  </world>
</sdf>)";
}

/////////////////////////////////////////////////
// A joint commanded to a velocity reaches most of it in one step. The link
// reports the acceleration the step integrated, the change in angular velocity
// over the step divided by the step size, not the explicit acceleration that
// the implicit integrator replaces.
TEST(LinkAcceleration, VelocityCommandedJoint)
{
  auto world = LoadWorld(ArmWorld(0.0, "0 0 1"));
  ASSERT_TRUE(world);
  auto model = world->GetModel("arm");
  ASSERT_TRUE(model);
  auto link = model->GetLink("link");
  ASSERT_TRUE(link);
  auto joint = model->GetJoint("joint");
  ASSERT_TRUE(joint);

  const double dt = 0.001;
  auto input = StepInput(dt);
  gz::physics::ForwardStep::State state;
  gz::physics::ForwardStep::Output output;

  const auto before = link->FrameDataRelativeToWorld();
  joint->SetVelocityCommand(0, 1.0);
  world->Step(output, state, input);
  const auto after = link->FrameDataRelativeToWorld();

  // The servo gets most of the way to the command in one step.
  EXPECT_GT(after.angularVelocity.z(), 0.5);

  AssertVectorApprox vectorPredicate(1e-6);
  EXPECT_PRED_FORMAT2(vectorPredicate,
                      (after.angularVelocity - before.angularVelocity) / dt,
                      after.angularAcceleration);
  // The link origin is on the joint axis, so it doesn't accelerate.
  EXPECT_PRED_FORMAT2(vectorPredicate, Eigen::Vector3d::Zero(),
                      after.linearAcceleration);
}

/////////////////////////////////////////////////
// A pendulum: a non-root link swinging about a joint 0.5 m from its origin,
// starting horizontal with an angular velocity. Its angular acceleration is
// the gravity torque over the inertia about the pivot, and its origin
// accelerates by alpha x r + w x (w x r).
TEST(LinkAcceleration, Pendulum)
{
  // The joint is 0.5 m along -x from the link origin, rotating about y.
  auto world = LoadWorld(ArmWorld(-0.5, "0 1 0"));
  ASSERT_TRUE(world);
  auto model = world->GetModel("arm");
  ASSERT_TRUE(model);
  auto link = model->GetLink("link");
  ASSERT_TRUE(link);
  auto joint = model->GetJoint("joint");
  ASSERT_TRUE(joint);

  const double w0 = 1.0;
  joint->SetVelocity(0, w0);

  const double dt = 0.001;
  auto input = StepInput(dt);
  gz::physics::ForwardStep::State state;
  gz::physics::ForwardStep::Output output;
  world->Step(output, state, input);

  const auto frameData = link->FrameDataRelativeToWorld();
  const double w = joint->GetVelocity(0);
  const double alpha = (w - w0) / dt;

  // Gravity torque about the pivot over the inertia about the pivot, with the
  // arm horizontal. A positive rotation about y lowers the +x end.
  const double m = 2.0;
  const double r = 0.5;
  const double g = 9.8;
  EXPECT_NEAR(m * g * r / (0.01 + m * r * r), alpha, 0.05);

  const Eigen::Vector3d rVec = frameData.pose.translation() -
      (frameData.pose * Eigen::Vector3d(-0.5, 0, 0));
  const Eigen::Vector3d axis(0, 1, 0);
  const Eigen::Vector3d expected = (alpha * axis).cross(rVec) +
      (w * axis).cross((w * axis).cross(rVec));

  AssertVectorApprox vectorPredicate(1e-6);
  EXPECT_PRED_FORMAT2(vectorPredicate, w * axis, frameData.angularVelocity);
  EXPECT_PRED_FORMAT2(vectorPredicate, alpha * axis,
                      frameData.angularAcceleration);
  EXPECT_PRED_FORMAT2(vectorPredicate, expected,
                      frameData.linearAcceleration);
}

/////////////////////////////////////////////////
// With gravity disabled in MuJoCo, cacc carries no gravity offset, so none is
// added back: a body with nothing acting on it reports no acceleration.
TEST(LinkAcceleration, GravityDisabled)
{
  auto world = LoadWorld(BoxWorld("0 0 -9.8", 10.0, 0.0, false));
  ASSERT_TRUE(world);
  auto link = GetBoxLink(world);
  ASSERT_TRUE(link);

  auto *worldInfo = static_cast<gz::physics::mujoco::WorldInfo *>(
      world->FullIdentity().ref.get());
  worldInfo->mjSpecObj->option.disableflags |= mjDSBL_GRAVITY;
  ASSERT_NE(nullptr, worldInfo->mjModelObj);
  worldInfo->mjModelObj->opt.disableflags |= mjDSBL_GRAVITY;

  gz::physics::ForwardStep::Input input;
  gz::physics::ForwardStep::State state;
  gz::physics::ForwardStep::Output output;
  world->Step(output, state, input);

  AssertVectorApprox vectorPredicate(1e-9);
  const auto frameData = link->FrameDataRelativeToWorld();
  EXPECT_PRED_FORMAT2(vectorPredicate, Eigen::Vector3d::Zero(),
                      frameData.linearVelocity);
  EXPECT_PRED_FORMAT2(vectorPredicate, Eigen::Vector3d::Zero(),
                      frameData.linearAcceleration);
}
