/*
 * Copyright (C) 2025 Open Source Robotics Foundation
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
#include <gz/physics/GetEntities.hh>
#include <gz/physics/RemoveEntities.hh>
#include <gz/physics/RequestEngine.hh>
#include <gz/physics/sdf/ConstructModel.hh>
#include <gz/physics/sdf/ConstructWorld.hh>
#include <gz/plugin/Loader.hh>
#include <sdf/Root.hh>
#include <sdf/World.hh>
#include <test/Utils.hh>

#include "test/common_test/Worlds.hh"

#include "Base.hh"
#include <mujoco/mujoco.h>

using namespace gz;

struct TestFeatureList : physics::FeatureList<
    physics::GetEngineInfo,
    physics::GetWorldFromEngine,
    physics::GetModelFromWorld,
    physics::sdf::ConstructSdfModel,
    physics::sdf::ConstructSdfWorld,
    physics::RemoveEntities
> { };

using World = physics::World3d<TestFeatureList>;
using WorldPtr = physics::World3dPtr<TestFeatureList>;
using ModelPtr = physics::Model3dPtr<TestFeatureList>;
using LinkPtr = physics::Link3dPtr<TestFeatureList>;

/////////////////////////////////////////////////
auto LoadEngine()
{
  plugin::Loader loader;
  loader.LoadLib(mujoco_plugin_LIB);

  plugin::PluginPtr mujoco =
      loader.Instantiate("gz::physics::mujoco::Plugin");

  auto engine =
      physics::RequestEngine3d<TestFeatureList>::From(mujoco);
  return engine;
}

enum class LoaderType
{
  Whole,
  Piecemeal
};

/////////////////////////////////////////////////
WorldPtr LoadWorldWhole(const std::string &_world)
{
  auto engine = LoadEngine();
  EXPECT_NE(nullptr, engine);

  sdf::Root root;
  const sdf::Errors &errors = root.Load(_world);
  EXPECT_EQ(0u, errors.size());
  for (const auto & error : errors) {
    std::cout << error << std::endl;
  }

  EXPECT_EQ(1u, root.WorldCount());
  const sdf::World *sdfWorld = root.WorldByIndex(0);
  EXPECT_NE(nullptr, sdfWorld);

  auto world = engine->ConstructWorld(*sdfWorld);
  EXPECT_NE(nullptr, world);

  return world;
}

/////////////////////////////////////////////////
class SDFFeatures_TEST : public ::testing::TestWithParam<LoaderType>
{
  public: WorldPtr LoadWorld(const std::string &_world)
  {
    switch(this->GetParam())
    {
      case LoaderType::Whole:
        return LoadWorldWhole(_world);
      // case LoaderType::Piecemeal:
      //   return LoadWorldPiecemeal(_world);
      default:
        std::cout << "Unknown LoaderType "
                  << std::underlying_type_t<LoaderType>(this->GetParam())
                  << " Using LoadWorldWhole" << std::endl;
        return LoadWorldWhole(_world);
    }
  }
};
/////////////////////////////////////////////////
// Test that the mujoco plugin loaded all the relevant information correctly.
TEST_P(SDFFeatures_TEST, CheckMujocoData)
{
  common::Console::SetVerbosity(4);
  using gz::physics::mujoco::Base;
  WorldPtr world = this->LoadWorld(common_test::worlds::kTestWorld);
  ASSERT_NE(nullptr, world);

  auto *worldInfo = static_cast<physics::mujoco::WorldInfo *>(
      world->FullIdentity().ref.get());
  EXPECT_EQ(10u, worldInfo->models.size());
  auto *m = worldInfo->mjModelObj;
  auto *spec = worldInfo->mjSpecObj;
  ASSERT_NE(nullptr, m);
  auto *worldBody = worldInfo->body;
  ASSERT_NE(nullptr, worldBody);

  auto getNumNodesInTree = [&](const mjsBody *_root)
  {
    auto bodyId = mjs_getId(_root->element);
    auto treeId = m->body_treeid[bodyId];
    return m->tree_bodynum[treeId];
  };

  auto doublePendulum = mjs_findChild(
      worldBody, Base::JoinNames("double_pendulum_with_base", "base").c_str());
  EXPECT_EQ(3, getNumNodesInTree(doublePendulum));

  auto verify = [](const mjsJoint *joint, double damping, double friction,
                   double springRest, double stiffness, double lower,
                   double upper, double maxForce)
  {
    EXPECT_DOUBLE_EQ(damping, joint->damping);
    EXPECT_DOUBLE_EQ(friction, joint->frictionloss);
    EXPECT_DOUBLE_EQ(springRest, joint->springref);
    EXPECT_DOUBLE_EQ(stiffness, joint->stiffness);
    EXPECT_DOUBLE_EQ(lower, joint->range[0]);
    EXPECT_DOUBLE_EQ(upper, joint->range[1]);
    EXPECT_DOUBLE_EQ(-maxForce, joint->actfrcrange[0]);
    EXPECT_DOUBLE_EQ(maxForce, joint->actfrcrange[1]);
  };
  // Test that things were parsed correctly. These values are either stated or
  // implied in the test.world SDF file.
  auto joint1 = mjs_asJoint(mjs_findElement(
      spec, mjtObj::mjOBJ_JOINT,
      Base::JoinNames("double_pendulum_with_base", "upper_joint").c_str()));
  ASSERT_NE(nullptr, joint1);
  verify(joint1, 3.0, 0.0, 0.0, 0.0, -std::numeric_limits<double>::infinity(),
         std::numeric_limits<double>::infinity(),
         std::numeric_limits<double>::infinity());

  auto joint2 = mjs_asJoint(mjs_findElement(
      spec, mjtObj::mjOBJ_JOINT,
      Base::JoinNames("double_pendulum_with_base", "lower_joint").c_str()));
  ASSERT_NE(nullptr, joint2);
  verify(joint2, 3.0, 0.0, 0.0, 0.0, -std::numeric_limits<double>::infinity(),
         std::numeric_limits<double>::infinity(),
         std::numeric_limits<double>::infinity());

  for (const auto *joint : {joint1, joint2})
  {
    EXPECT_DOUBLE_EQ(1.0, joint->axis[0]);
    EXPECT_DOUBLE_EQ(0.0, joint->axis[1]);
    EXPECT_DOUBLE_EQ(0.0, joint->axis[2]);
  }

  {
    auto freeBodyLink =
        mjs_findChild(worldBody, Base::JoinNames("free_body", "link").c_str());

    ASSERT_NE(nullptr, freeBodyLink);
    EXPECT_EQ(1, getNumNodesInTree(freeBodyLink));

    EXPECT_DOUBLE_EQ(0.0, freeBodyLink->pos[0]);
    EXPECT_DOUBLE_EQ(10.0, freeBodyLink->pos[1]);
    EXPECT_DOUBLE_EQ(10.0, freeBodyLink->pos[2]);
  }

  {
    auto ballJointTestLink = mjs_findChild(
        worldBody, Base::JoinNames("ball_joint_test", "link0").c_str());
    EXPECT_EQ(2, getNumNodesInTree(ballJointTestLink));

    auto ballJoint = mjs_asJoint(
        mjs_findElement(spec, mjtObj::mjOBJ_JOINT,
                        Base::JoinNames("ball_joint_test", "j0").c_str()));
    ASSERT_NE(nullptr, ballJoint);
  }

  {
    auto universalJointTestLink = mjs_findChild(
        worldBody, Base::JoinNames("universal_joint_test", "link0").c_str());
    EXPECT_EQ(2, getNumNodesInTree(universalJointTestLink));

    auto universalJoint1 = mjs_asJoint(
        mjs_findElement(spec, mjtObj::mjOBJ_JOINT,
                        Base::JoinNames("universal_joint_test", "j0").c_str()));
    ASSERT_NE(nullptr, universalJoint1);
    verify(universalJoint1, 0.2, 0.0, 0.0, 0.0, -std::numeric_limits<double>::infinity(),
           std::numeric_limits<double>::infinity(),
           std::numeric_limits<double>::infinity());
    EXPECT_DOUBLE_EQ(1.0, universalJoint1->axis[0]);
    EXPECT_DOUBLE_EQ(0.0, universalJoint1->axis[1]);
    EXPECT_DOUBLE_EQ(0.0, universalJoint1->axis[2]);

    auto universalJoint2 = mjs_asJoint(
        mjs_findElement(spec, mjtObj::mjOBJ_JOINT,
                        Base::JoinNames("universal_joint_test", "j0_axis2").c_str()));
    ASSERT_NE(nullptr, universalJoint2);
    verify(universalJoint2, 0.3, 0.0, 0.0, 0.0, -std::numeric_limits<double>::infinity(),
           std::numeric_limits<double>::infinity(),
           std::numeric_limits<double>::infinity());
    EXPECT_DOUBLE_EQ(0.0, universalJoint2->axis[0]);
    EXPECT_DOUBLE_EQ(1.0, universalJoint2->axis[1]);
    EXPECT_DOUBLE_EQ(0.0, universalJoint2->axis[2]);
  }
}

/////////////////////////////////////////////////
// Test that models can be removed from the world.
TEST_P(SDFFeatures_TEST, ModelRemoval)
{
  WorldPtr world = this->LoadWorld(common_test::worlds::kShapesWorld);
  ASSERT_NE(nullptr, world);

  std::size_t modelCount = world->GetModelCount();
  EXPECT_GT(modelCount, 0u);

  // Get a specific model
  const std::string modelName = "box";
  ModelPtr model = world->GetModel(modelName);
  ASSERT_NE(nullptr, model);

  EXPECT_TRUE(world->RemoveModel(model));

  // The model should be removed from the world's model list immediately
  EXPECT_EQ(modelCount - 1, world->GetModelCount());

  // Try to get it again, should return nullptr
  EXPECT_EQ(nullptr, world->GetModel(modelName));

  // Try to remove it again, should return false
  EXPECT_FALSE(world->RemoveModel(modelName));
}

INSTANTIATE_TEST_SUITE_P(LoadWorld, SDFFeatures_TEST,
                        ::testing::Values(LoaderType::Whole));
