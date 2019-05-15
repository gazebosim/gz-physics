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
#include <ignition/physics/FreeGroup.hh>
#include <ignition/physics/Link.hh>
#include <ignition/physics/sdf/ConstructWorld.hh>
#include <ignition/physics/sdf/ConstructModel.hh>
#include <ignition/physics/sdf/ConstructLink.hh>

#include <sdf/Root.hh>
#include <sdf/World.hh>

#include "test/Utils.hh"

using namespace ignition;

using TestFeatureList = ignition::physics::FeatureList<
  physics::LinkFrameSemantics,
  physics::FindFreeGroupFeature,
  physics::SetFreeGroupWorldPose,
  physics::SetFreeGroupWorldVelocity,
  physics::ForwardStep,
  physics::sdf::ConstructSdfWorld,
  physics::sdf::ConstructSdfModel,
  physics::sdf::ConstructSdfLink
>;

using TestEnginePtr = physics::Engine3dPtr<TestFeatureList>;
using TestWorldPtr = physics::World3dPtr<TestFeatureList>;

class FreeGroupFeaturesFixture : public ::testing::Test
{
  protected: void SetUp() override
  {
    ignition::plugin::Loader loader;
    loader.LoadLib(dartsim_plugin_LIB);

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
TEST_F(FreeGroupFeaturesFixture, ModelPoseCanoncalLinkOffset)
{
  auto world = LoadWorld(this->engine, TEST_WORLD_DIR "/empty.sdf",
                         Eigen::Vector3d::Zero());
  sdf::Model modelSDF;
  modelSDF.SetName("test_model");
  math::Pose3d modelPose = math::Pose3d::Zero;
  modelSDF.SetPose(modelPose);
  auto model = world->ConstructModel(modelSDF);

  // Set the pose of canonical link to be offset from the model
  math::Pose3d linkPose(1, 2, 3, 0, 0, 0);
  sdf::Link linkSDF;
  linkSDF.SetName("link1");
  linkSDF.SetPose(linkPose);
  auto link = model->ConstructLink(linkSDF);

  ignition::physics::ForwardStep::Input input;
  ignition::physics::ForwardStep::State state;
  ignition::physics::ForwardStep::Output output;

  AssertVectorApprox vectorPredicate(1e-4);

  world->Step(output, state, input);

  EXPECT_PRED_FORMAT2(vectorPredicate, math::eigen3::convert(linkPose.Pos()),
                      link->FrameDataRelativeToWorld().pose.translation());

  auto modelFreeGroup = model->FindFreeGroup();
  EXPECT_NE(nullptr, modelFreeGroup);

  // Set the model pose to its initial pose
  modelFreeGroup->SetWorldPose(math::eigen3::convert(modelPose));
  world->Step(output, state, input);

  // We expect the link to stay in the initial pose
  EXPECT_PRED_FORMAT2(vectorPredicate, math::eigen3::convert(linkPose.Pos()),
                      link->FrameDataRelativeToWorld().pose.translation());
}

/////////////////////////////////////////////////
int main(int argc, char *argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
