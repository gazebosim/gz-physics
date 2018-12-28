/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

#include <iostream>

#include <ignition/physics/FindFeatures.hh>
#include <ignition/plugin/Loader.hh>
#include <ignition/physics/RequestEngine.hh>

// Features
#include <ignition/physics/ForwardStep.hh>
#include <ignition/physics/FrameSemantics.hh>
#include <ignition/physics/GetEntities.hh>
#include <ignition/physics/Link.hh>
#include <ignition/physics/sdf/ConstructWorld.hh>

#include <sdf/Root.hh>
#include <sdf/World.hh>

#include <test/PhysicsPluginsList.hh>

using TestFeatureList = ignition::physics::FeatureList<
  ignition::physics::SetLinkState,
  ignition::physics::GetLinkForceTorque,
  ignition::physics::LinkFrameSemantics,
  ignition::physics::ForwardStep,
  ignition::physics::GetEntities,
  ignition::physics::sdf::ConstructSdfWorld
>;

using TestWorldPtr = ignition::physics::World3dPtr<TestFeatureList>;

std::unordered_set<TestWorldPtr> LoadWorlds(
    const std::string &_library,
    const std::string &_world)
{
  ignition::plugin::Loader loader;
  loader.LoadLib(_library);

  const std::set<std::string> pluginNames =
      ignition::physics::FindFeatures3d<TestFeatureList>::From(loader);

  std::unordered_set<TestWorldPtr> worlds;
  for (const std::string &name : pluginNames)
  {
    ignition::plugin::PluginPtr plugin = loader.Instantiate(name);

    std::cout << " -- Plugin name: " << name << std::endl;

    auto engine =
        ignition::physics::RequestEngine3d<TestFeatureList>::From(plugin);
    EXPECT_NE(nullptr, engine);

    sdf::Root root;
    const sdf::Errors errors = root.Load(_world);
    EXPECT_TRUE(errors.empty());
    const sdf::World *sdfWorld = root.WorldByIndex(0);
    auto world = engine->ConstructWorld(*sdfWorld);

    worlds.insert(world);
  }

  return worlds;
}

class LinkFeatures_TEST
  : public ::testing::Test,
    public ::testing::WithParamInterface<std::string>
{};

// A predicate-formatter for asserting that two vectors are approximately equal.
class AssertVectorApprox
{
  public: AssertVectorApprox(double _tol = 1e-6) : tol(_tol)
  {
  }

  public: ::testing::AssertionResult operator()(
              const char *_mExpr, const char *_nExpr, Eigen::Vector3d _m,
              Eigen::Vector3d _n)
  {
    if (_m.isApprox(_n, this->tol))
      return ::testing::AssertionSuccess();

    return ::testing::AssertionFailure()
           << _mExpr << " and " << _nExpr << " ([" << _m.transpose()
           << "] and [" << _n.transpose() << "]"
           << ") are not equal";
  }

  private: double tol;
};

INSTANTIATE_TEST_CASE_P(PhysicsPlugins, LinkFeatures_TEST,
    ::testing::ValuesIn(ignition::physics::test::g_PhysicsPluginLibraries),); // NOLINT

// Test that linear and angular velocities of a link can be set.
TEST_P(LinkFeatures_TEST, LinkVelocity)
{
  const std::string library = GetParam();
  if (library.empty())
    return;

  std::cout << "Testing library " << library << std::endl;
  auto worlds = LoadWorlds(library, TEST_WORLD_DIR "/falling.world");

  for (const auto &world : worlds)
  {
    auto link = world->GetModel(0)->GetLink(0);
    Eigen::Vector3d initPos =
        link->FrameDataRelativeToWorld().pose.translation();
    Eigen::Matrix3d initRot = link->FrameDataRelativeToWorld().pose.linear();

    // Use small angular velocity so that the angles don't wrap...makes it easy
    // to compare rotations.
    // \todo(anyone): Setting a nonzero value in either the x or y axes of the
    // angular velocity vector causes the link to move unexpectedly in the x or
    // y direction.
    const Eigen::Vector3d cmdAngVelocity{0, 0, 0.1 * IGN_PI};
    link->SetAngularVelocity(cmdAngVelocity);

    const Eigen::Vector3d cmdLinVelocity{1, -1, 10};
    link->SetLinearVelocity(cmdLinVelocity);

    ignition::physics::ForwardStep::Input input;
    ignition::physics::ForwardStep::State state;
    ignition::physics::ForwardStep::Output output;

    // assumes step size is 1ms.
    const double dt = 0.001;
    const size_t iters = 1000;

    const double totalTime = iters * dt;
    for (size_t i = 0; i < iters; ++i)
    {
      world->Step(output, state, input);
    }

    auto frameData = link->FrameDataRelativeToWorld();
    EXPECT_PRED_FORMAT2(AssertVectorApprox(), cmdAngVelocity,
                        frameData.angularVelocity);

    const Eigen::Vector3d pos = frameData.pose.translation();
    EXPECT_NEAR(cmdLinVelocity.x() * totalTime, pos.x() - initPos.x(), 5e-4);
    EXPECT_NEAR(cmdLinVelocity.y() * totalTime, pos.y() - initPos.y(), 5e-4);

    const double gravity = -9.801;
    const double zExpected = cmdLinVelocity.z() + (0.5 * gravity *
                                                   std::pow(totalTime, 2));
    EXPECT_NEAR(zExpected, pos.z() - initPos.z(), 5e-3);

    const Eigen::Matrix3d rot = frameData.pose.linear();
    Eigen::AngleAxisd rotDiff;
    rotDiff = rot * initRot.transpose();
    EXPECT_TRUE(rotDiff.isApprox(
        Eigen::AngleAxisd(cmdAngVelocity.norm(), cmdAngVelocity.normalized()),
        1e-5));
  }
}

// Test setting force and torque.
TEST_P(LinkFeatures_TEST, LinkForceTorque)
{
  const std::string library = GetParam();
  if (library.empty())
    return;

  std::cout << "Testing library " << library << std::endl;
  auto worlds = LoadWorlds(library, TEST_WORLD_DIR "/falling.world");

  for (const auto &world : worlds)
  {
    auto link = world->GetModel(0)->GetLink(0);

    const Eigen::Vector3d cmdForce{1, -1, 10};
    link->SetForce(cmdForce);
    EXPECT_PRED_FORMAT2(AssertVectorApprox(1e-4), cmdForce, link->GetForce());

    const Eigen::Vector3d cmdTorque{0, 0, 0.1 * IGN_PI};
    link->SetTorque(cmdTorque);
    EXPECT_PRED_FORMAT2(AssertVectorApprox(1e-4), cmdTorque, link->GetTorque());

    ignition::physics::ForwardStep::Input input;
    ignition::physics::ForwardStep::State state;
    ignition::physics::ForwardStep::Output output;

    world->Step(output, state, input);

    const double mass = 1.0;

    const Eigen::Vector3d gravity{0, 0, -9.8};
    auto frameData = link->FrameDataRelativeToWorld();
    EXPECT_PRED_FORMAT2(AssertVectorApprox(1e-4), cmdForce,
                        mass * (frameData.linearAcceleration - gravity));

    Eigen::Matrix3d moi;
    moi.diagonal() = Eigen::Vector3d{0.4, 0.4, 0.4};

    EXPECT_PRED_FORMAT2(AssertVectorApprox(1e-4), cmdTorque,
                        moi * frameData.angularAcceleration);
  }
}
int main(int argc, char *argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
