/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#include <tuple>

#include <sdf/Root.hh>
#include <sdf/World.hh>

#include <test/Utils.hh>
#include <test/common_test/Worlds.hh>

#include <gz/plugin/Loader.hh>

#include <gz/physics/Joint.hh>
#include <gz/physics/RequestEngine.hh>

#include <gz/physics/GetEntities.hh>
#include <gz/physics/sdf/ConstructJoint.hh>
#include <gz/physics/sdf/ConstructLink.hh>
#include <gz/physics/sdf/ConstructModel.hh>
#include <gz/physics/sdf/ConstructNestedModel.hh>
#include <gz/physics/sdf/ConstructWorld.hh>

#include "lib/src/Entity.hh"
#include "lib/src/Model.hh"
#include "lib/src/World.hh"
#include "World.hh"

using namespace gz;

struct TestFeatureList : physics::FeatureList<
    physics::tpeplugin::RetrieveWorld,
    physics::GetModelFromWorld,
    physics::sdf::ConstructSdfLink,
    physics::sdf::ConstructSdfModel,
    physics::sdf::ConstructSdfNestedModel,
    physics::sdf::ConstructSdfWorld
> { };

using World = physics::World3d<TestFeatureList>;
using WorldPtr = physics::World3dPtr<TestFeatureList>;

auto LoadEngine()
{
  plugin::Loader loader;
  loader.LoadLib(tpe_plugin_LIB);

  plugin::PluginPtr tpe_plugin =
      loader.Instantiate("gz::physics::tpeplugin::Plugin");

  auto engine =
      physics::RequestEngine3d<TestFeatureList>::From(tpe_plugin);
  return engine;
}

World LoadWorld(const std::string &_world)
{
  auto engine = LoadEngine();
  EXPECT_NE(nullptr, engine);

  sdf::Root root;
  const sdf::Errors &errors = root.Load(_world);
  EXPECT_EQ(0u, errors.size());

  EXPECT_EQ(1u, root.WorldCount());
  const sdf::World *sdfWorld = root.WorldByIndex(0);
  EXPECT_NE(nullptr, sdfWorld);

  auto world = engine->ConstructWorld(*sdfWorld);
  EXPECT_NE(nullptr, world);

  return *world;
}

// Test that the tpe plugin loaded all the relevant information correctly.
TEST(SDFFeatures_TEST, CheckTpeData)
{
  auto world = LoadWorld(common_test::worlds::kTestWorld);
  auto tpeWorld = world.GetTpeLibWorld();
  ASSERT_NE(nullptr, tpeWorld);

  ASSERT_EQ(7u, tpeWorld->GetChildCount());

  // check model 01
  {
    physics::tpelib::Entity &model =
        tpeWorld->GetChildByName("ground_plane");
    ASSERT_NE(physics::tpelib::Entity::kNullEntity.GetId(),
        model.GetId());
    EXPECT_EQ("ground_plane", model.GetName());
    EXPECT_EQ(math::Pose3d::Zero, model.GetPose());
    EXPECT_EQ(math::Pose3d::Zero, model.GetWorldPose());
    EXPECT_EQ(1u, model.GetChildCount());

    physics::tpelib::Entity &link = model.GetChildByName("link");
    ASSERT_NE(physics::tpelib::Entity::kNullEntity.GetId(),
        link.GetId());
    EXPECT_EQ("link", link.GetName());
    EXPECT_EQ(math::Pose3d::Zero, link.GetPose());
    EXPECT_EQ(math::Pose3d::Zero, link.GetWorldPose());
    EXPECT_EQ(1u, link.GetChildCount());

    physics::tpelib::Entity &collision =
        link.GetChildByName("collision");
    ASSERT_NE(physics::tpelib::Entity::kNullEntity.GetId(),
        collision.GetId());
    EXPECT_EQ("collision", collision.GetName());
    EXPECT_EQ(math::Pose3d::Zero, collision.GetPose());
    EXPECT_EQ(math::Pose3d::Zero, collision.GetWorldPose());
  }

  // check model 02
  {
    physics::tpelib::Entity &model =
        tpeWorld->GetChildByName("double_pendulum_with_base");
    ASSERT_NE(physics::tpelib::Entity::kNullEntity.GetId(),
        model.GetId());
    EXPECT_EQ("double_pendulum_with_base", model.GetName());
    EXPECT_EQ(math::Pose3d(1, 0, 0, 0, 0, 0), model.GetPose());
    EXPECT_EQ(math::Pose3d(1, 0, 0, 0, 0, 0), model.GetWorldPose());
    EXPECT_EQ(3u, model.GetChildCount());

    physics::tpelib::Entity &link = model.GetChildByName("base");
    ASSERT_NE(physics::tpelib::Entity::kNullEntity.GetId(),
        link.GetId());
    EXPECT_EQ("base", link.GetName());
    EXPECT_EQ(math::Pose3d::Zero, link.GetPose());
    EXPECT_EQ(math::Pose3d(1, 0, 0, 0, 0, 0), model.GetWorldPose());
    EXPECT_EQ(2u, link.GetChildCount());

    physics::tpelib::Entity &collision =
        link.GetChildByName("col_plate_on_ground");
    ASSERT_NE(physics::tpelib::Entity::kNullEntity.GetId(),
        collision.GetId());
    EXPECT_EQ("col_plate_on_ground", collision.GetName());
    EXPECT_EQ(math::Pose3d(0, 0, 0.01, 0, 0, 0), collision.GetPose());
    EXPECT_EQ(math::Pose3d(1, 0, 0.01, 0, 0, 0),
        collision.GetWorldPose());

    physics::tpelib::Entity &collision02 =
        link.GetChildByName("col_pole");
    ASSERT_NE(physics::tpelib::Entity::kNullEntity.GetId(),
        collision.GetId());
    EXPECT_EQ("col_pole", collision02.GetName());
    EXPECT_EQ(math::Pose3d(-0.275, 0, 1.1, 0, 0, 0),
        collision02.GetPose());
    EXPECT_EQ(math::Pose3d(0.725, 0, 1.1, 0, 0, 0),
        collision02.GetWorldPose());

    physics::tpelib::Entity &link02 =
        model.GetChildByName("upper_link");
    ASSERT_NE(physics::tpelib::Entity::kNullEntity.GetId(),
        link02.GetId());
    EXPECT_EQ("upper_link", link02.GetName());
    EXPECT_EQ(math::Pose3d(0, 0, 2.1, -1.5708, 0, 0),
        link02.GetPose());
    EXPECT_EQ(math::Pose3d(1, 0, 2.1, -1.5708, 0, 0),
        link02.GetWorldPose());
    EXPECT_EQ(3u, link02.GetChildCount());

    physics::tpelib::Entity &collision03 =
        link02.GetChildByName("col_upper_joint");
    ASSERT_NE(physics::tpelib::Entity::kNullEntity.GetId(),
        collision03.GetId());
    EXPECT_EQ("col_upper_joint", collision03.GetName());
    EXPECT_EQ(math::Pose3d(-0.05, 0, 0, 0, 1.5708, 0),
        collision03.GetPose());
    EXPECT_EQ(
        math::Pose3d(0.95, 0, 2.1, -1.5708, -4e-06, -1.5708),
        collision03.GetWorldPose());

    physics::tpelib::Entity &collision04 =
        link02.GetChildByName("col_lower_joint");
    ASSERT_NE(physics::tpelib::Entity::kNullEntity.GetId(),
        collision04.GetId());
    EXPECT_EQ("col_lower_joint", collision04.GetName());
    EXPECT_EQ(math::Pose3d(0, 0, 1.0, 0, 1.5708, 0),
        collision04.GetPose());
    EXPECT_EQ(
        math::Pose3d(1, 1, 2.1, -1.5708, -4e-06, -1.5708),
        collision04.GetWorldPose());

    physics::tpelib::Entity &collision05 =
        link02.GetChildByName("col_cylinder");
    ASSERT_NE(physics::tpelib::Entity::kNullEntity.GetId(),
        collision05.GetId());
    EXPECT_EQ("col_cylinder", collision05.GetName());
    EXPECT_EQ(math::Pose3d(0, 0, 0.5, 0, 0, 0),
        collision05.GetPose());
    EXPECT_EQ(math::Pose3d(1, 0.5, 2.1, -1.5708, 0, 0),
        collision05.GetWorldPose());

    physics::tpelib::Entity &link03 =
        model.GetChildByName("lower_link");
    ASSERT_NE(physics::tpelib::Entity::kNullEntity.GetId(),
        link03.GetId());
    EXPECT_EQ("lower_link", link03.GetName());
    EXPECT_EQ(math::Pose3d(0.25, 1.0, 2.1, -2, 0, 0),
        link03.GetPose());
    EXPECT_EQ(math::Pose3d(1.25, 1.0, 2.1, -2, 0, 0),
        link03.GetWorldPose());
    EXPECT_EQ(2u, link03.GetChildCount());

    physics::tpelib::Entity &collision06 =
        link03.GetChildByName("col_lower_joint");
    ASSERT_NE(physics::tpelib::Entity::kNullEntity.GetId(),
        collision06.GetId());
    EXPECT_EQ("col_lower_joint", collision06.GetName());
    EXPECT_EQ(math::Pose3d(0, 0, 0, 0, 1.5708, 0),
        collision06.GetPose());
    EXPECT_EQ(
        math::Pose3d(1.25, 1, 2.1, -1.57079, -0.429204, -1.5708),
        collision06.GetWorldPose());

    physics::tpelib::Entity &collision07 =
        link03.GetChildByName("col_cylinder");
    ASSERT_NE(physics::tpelib::Entity::kNullEntity.GetId(),
        collision07.GetId());
    EXPECT_EQ("col_cylinder", collision07.GetName());
    EXPECT_EQ(math::Pose3d(0, 0, 0.5, 0, 0, 0),
        collision07.GetPose());
    EXPECT_EQ(math::Pose3d(1.25, 1.45465, 1.89193, -2, 0, 0),
        collision07.GetWorldPose());
  }

  // check model 03
  {
    physics::tpelib::Entity &model =
        tpeWorld->GetChildByName("free_body");
    ASSERT_NE(physics::tpelib::Entity::kNullEntity.GetId(),
        model.GetId());
    EXPECT_EQ("free_body", model.GetName());
    EXPECT_EQ(math::Pose3d(0, 10, 10, 0, 0, 0), model.GetPose());
    EXPECT_EQ(1u, model.GetChildCount());

    physics::tpelib::Entity &link = model.GetChildByName("link");
    ASSERT_NE(physics::tpelib::Entity::kNullEntity.GetId(),
        link.GetId());
    EXPECT_EQ("link", link.GetName());
    EXPECT_EQ(math::Pose3d::Zero, link.GetPose());
    EXPECT_EQ(1u, link.GetChildCount());

    auto &collision = link.GetChildByName("collision1");
    ASSERT_NE(physics::tpelib::Entity::kNullEntity.GetId(),
        collision.GetId());
    EXPECT_EQ("collision1", collision.GetName());
    EXPECT_EQ(0u, collision.GetChildCount());
  }

  // check model 04
  {
    physics::tpelib::Entity &model =
        tpeWorld->GetChildByName("joint_limit_test");
    ASSERT_NE(physics::tpelib::Entity::kNullEntity.GetId(),
        model.GetId());
    EXPECT_EQ("joint_limit_test", model.GetName());
    EXPECT_EQ(math::Pose3d(10, 0, 2, 0, 0, 0), model.GetPose());
    EXPECT_EQ(2u, model.GetChildCount());

    physics::tpelib::Entity &link = model.GetChildByName("base");
    ASSERT_NE(physics::tpelib::Entity::kNullEntity.GetId(),
        link.GetId());
    EXPECT_EQ("base", link.GetName());
    EXPECT_EQ(math::Pose3d::Zero, link.GetPose());
    EXPECT_EQ(0u, link.GetChildCount());

    physics::tpelib::Entity &link02 = model.GetChildByName("bar");
    ASSERT_NE(physics::tpelib::Entity::kNullEntity.GetId(),
        link02.GetId());
    EXPECT_EQ("bar", link02.GetName());
    EXPECT_EQ(math::Pose3d(1, 0, 0, 0, 0, 0), link02.GetPose());
    EXPECT_EQ(0u, link02.GetChildCount());
  }

  // check model 05
  {
    physics::tpelib::Entity &model =
        tpeWorld->GetChildByName("screw_joint_test");
    ASSERT_NE(physics::tpelib::Entity::kNullEntity.GetId(),
        model.GetId());
    EXPECT_EQ("screw_joint_test", model.GetName());
    EXPECT_EQ(math::Pose3d::Zero, model.GetPose());
    EXPECT_EQ(2u, model.GetChildCount());

    physics::tpelib::Entity &link = model.GetChildByName("link0");
    ASSERT_NE(physics::tpelib::Entity::kNullEntity.GetId(),
        link.GetId());
    EXPECT_EQ("link0", link.GetName());
    EXPECT_EQ(math::Pose3d::Zero, link.GetPose());
    EXPECT_EQ(0u, link.GetChildCount());

    physics::tpelib::Entity &link02 = model.GetChildByName("link1");
    ASSERT_NE(physics::tpelib::Entity::kNullEntity.GetId(),
        link02.GetId());
    EXPECT_EQ("link1", link02.GetName());
    EXPECT_EQ(math::Pose3d::Zero, link02.GetPose());
    EXPECT_EQ(0u, link02.GetChildCount());
  }

  // check model 06
  {
    physics::tpelib::Entity &model =
        tpeWorld->GetChildByName("unsupported_joint_test");
    ASSERT_NE(physics::tpelib::Entity::kNullEntity.GetId(),
        model.GetId());
    EXPECT_EQ("unsupported_joint_test", model.GetName());
    EXPECT_EQ(math::Pose3d::Zero, model.GetPose());
    EXPECT_EQ(6u, model.GetChildCount());

    physics::tpelib::Entity &link = model.GetChildByName("link0");
    ASSERT_NE(physics::tpelib::Entity::kNullEntity.GetId(),
        link.GetId());
    EXPECT_EQ("link0", link.GetName());
    EXPECT_EQ(math::Pose3d::Zero, link.GetPose());
    EXPECT_EQ(0u, link.GetChildCount());

    physics::tpelib::Entity &link02 = model.GetChildByName("link1");
    ASSERT_NE(physics::tpelib::Entity::kNullEntity.GetId(),
        link02.GetId());
    EXPECT_EQ("link1", link02.GetName());
    EXPECT_EQ(math::Pose3d::Zero, link02.GetPose());
    EXPECT_EQ(0u, link02.GetChildCount());

    physics::tpelib::Entity &link03 = model.GetChildByName("link2");
    ASSERT_NE(physics::tpelib::Entity::kNullEntity.GetId(),
        link03.GetId());
    EXPECT_EQ("link2", link03.GetName());
    EXPECT_EQ(math::Pose3d::Zero, link03.GetPose());
    EXPECT_EQ(0u, link03.GetChildCount());

    physics::tpelib::Entity &link04 = model.GetChildByName("link3");
    ASSERT_NE(physics::tpelib::Entity::kNullEntity.GetId(),
        link04.GetId());
    EXPECT_EQ("link3", link04.GetName());
    EXPECT_EQ(math::Pose3d::Zero, link04.GetPose());
    EXPECT_EQ(0u, link04.GetChildCount());

    physics::tpelib::Entity &link05 = model.GetChildByName("link4");
    ASSERT_NE(physics::tpelib::Entity::kNullEntity.GetId(),
        link05.GetId());
    EXPECT_EQ("link4", link05.GetName());
    EXPECT_EQ(math::Pose3d::Zero, link05.GetPose());
    EXPECT_EQ(0u, link05.GetChildCount());

    physics::tpelib::Entity &link06 = model.GetChildByName("link5");
    ASSERT_NE(physics::tpelib::Entity::kNullEntity.GetId(),
        link06.GetId());
    EXPECT_EQ("link5", link06.GetName());
    EXPECT_EQ(math::Pose3d::Zero, link06.GetPose());
    EXPECT_EQ(0u, link06.GetChildCount());
  }
}

// Test that the tpe plugin loaded nested models correctly.
TEST(SDFFeatures_TEST, NestedModel)
{
  auto world = LoadWorld(common_test::worlds::kWorldWithNestedModelSdf);
  auto tpeWorld = world.GetTpeLibWorld();
  ASSERT_NE(nullptr, tpeWorld);

  ASSERT_EQ(2u, tpeWorld->GetChildCount());
  EXPECT_EQ(2u, world.GetModelCount());

  // check top level model
  physics::tpelib::Entity &model =
      tpeWorld->GetChildByName("parent_model");
  ASSERT_NE(physics::tpelib::Entity::kNullEntity.GetId(),
      model.GetId());
  EXPECT_EQ("parent_model", model.GetName());
  EXPECT_EQ(math::Pose3d::Zero, model.GetPose());
  EXPECT_EQ(4u, model.GetChildCount());

  physics::tpelib::Entity &link = model.GetChildByName("link1");
  ASSERT_NE(physics::tpelib::Entity::kNullEntity.GetId(),
      link.GetId());
  EXPECT_EQ("link1", link.GetName());
  EXPECT_EQ(math::Pose3d::Zero, link.GetPose());
  EXPECT_EQ(1u, link.GetChildCount());

  physics::tpelib::Entity &collision =
      link.GetChildByName("collision");
  ASSERT_NE(physics::tpelib::Entity::kNullEntity.GetId(),
      collision.GetId());
  EXPECT_EQ("collision", collision.GetName());
  EXPECT_EQ(math::Pose3d::Zero, collision.GetPose());

  // check nested model
  physics::tpelib::Entity &nestedModel =
      model.GetChildByName("nested_model");
  ASSERT_NE(physics::tpelib::Entity::kNullEntity.GetId(),
      nestedModel.GetId());
  EXPECT_EQ("nested_model", nestedModel.GetName());
  EXPECT_EQ(math::Pose3d(1, 2, 2, 0, 0, 0), nestedModel.GetPose());
  EXPECT_EQ(2u, nestedModel.GetChildCount());

  physics::tpelib::Entity &nestedLink =
      nestedModel.GetChildByName("nested_link1");
  ASSERT_NE(physics::tpelib::Entity::kNullEntity.GetId(),
      nestedLink.GetId());
  EXPECT_EQ("nested_link1", nestedLink.GetName());
  EXPECT_EQ(math::Pose3d(3, 1, 1, 0, 0, 1.5707),
      nestedLink.GetPose());
  EXPECT_EQ(1u, nestedLink.GetChildCount());

  physics::tpelib::Entity &nestedCollision =
      nestedLink.GetChildByName("nested_collision1");
  ASSERT_NE(physics::tpelib::Entity::kNullEntity.GetId(),
      nestedCollision.GetId());
  EXPECT_EQ("nested_collision1", nestedCollision.GetName());
  EXPECT_EQ(math::Pose3d::Zero, nestedCollision.GetPose());

  // canonical link
  auto *m = dynamic_cast<physics::tpelib::Model *>(&model);
  physics::tpelib::Entity canLink = m->GetCanonicalLink();
  EXPECT_EQ(link.GetId(), canLink.GetId());
}

// Test ConstructModel function.
TEST(SDFFeatures_TEST, ConstructModel)
{
  auto engine = LoadEngine();
  ASSERT_NE(nullptr, engine);
  sdf::World sdfWorld;
  sdfWorld.SetName("default");
  auto world = engine->ConstructWorld(sdfWorld);
  EXPECT_NE(nullptr, world);

  sdf::Model sdfModel;
  sdfModel.SetName("model");
  auto model = world->ConstructModel(sdfModel);
  EXPECT_NE(nullptr, model);
  EXPECT_EQ("model", model->GetName());

  sdfModel.SetName("nested_model_by_world");
  auto nestedModelByWorld = world->ConstructNestedModel(sdfModel);
  EXPECT_NE(nullptr, nestedModelByWorld);
  EXPECT_EQ("nested_model_by_world", nestedModelByWorld->GetName());

  sdfModel.SetName("nested_model_by_model");
  auto nestedModelByModel = model->ConstructNestedModel(sdfModel);
  EXPECT_NE(nullptr, nestedModelByModel);
  EXPECT_EQ("nested_model_by_model", nestedModelByModel->GetName());
}
