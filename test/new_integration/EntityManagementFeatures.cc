/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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

#include "test_integration_config.h"  // NOLINT(build/include)

#include <gz/common/Filesystem.hh>
#include <gz/common/Console.hh>
#include <gz/plugin/Loader.hh>

#include <gz/physics/ConstructEmpty.hh>
#include <gz/physics/Shape.hh>
#include <gz/physics/FindFeatures.hh>
#include <gz/physics/GetEntities.hh>
#include <gz/physics/RequestEngine.hh>
#include <gz/physics/RevoluteJoint.hh>
#include "gz/physics/Joint.hh"
#include "gz/physics/BoxShape.hh"
#include <gz/physics/RemoveEntities.hh>

class EntityManagementFeaturesTest:
  public testing::Test,
  public testing::WithParamInterface<const char *>
{
  // Documentation inherited
  public: void SetUp() override
  {
    gz::common::Console::SetVerbosity(4);

    gz::plugin::Loader loader;
    std::string pluginPath = gz::common::joinPaths(GZ_PHYSICS_TEST_PLUGIN_PATH,
      std::string("libignition-physics6-") + std::string(GetParam()) +
      std::string("-plugin.so"));
    loader.LoadLib(pluginPath);

    std::string physicsPluginName = GetParam();
    if (std::string(GetParam()) == "tpe")
    {
      physicsPluginName = GetParam() + std::string("plugin");
    }

    physicsPlugin =
      loader.Instantiate(std::string("gz::physics::") +
                         physicsPluginName +
                         std::string("::Plugin"));
  }

  public: gz::plugin::PluginPtr physicsPlugin;
};

// The features that an engine must have to be loaded by this loader.
using Features = gz::physics::FeatureList<
    gz::physics::GetEngineInfo,
    gz::physics::GetWorldFromEngine,
    gz::physics::ConstructEmptyWorldFeature,
    gz::physics::ConstructEmptyModelFeature,
    gz::physics::GetModelFromWorld,
    gz::physics::GetLinkFromModel,
    gz::physics::ConstructEmptyLinkFeature,
    gz::physics::AttachBoxShapeFeature,
    gz::physics::GetShapeFromLink,
    gz::physics::GetBoxShapeProperties,
    gz::physics::RemoveEntities,
    gz::physics::GetNestedModelFromModel,
    gz::physics::ConstructEmptyNestedModelFeature
    // gz::physics::AttachFixedJointFeature
    // gz::physics::AttachRevoluteJointFeature,
    // gz::physics::GetRevoluteJointProperties,
    // gz::physics::GetBasicJointState
>;

// ** dartsim
// GetEntities,
// RemoveEntities,
// ConstructEmptyWorldFeature,
// ConstructEmptyModelFeature,
// ConstructEmptyNestedModelFeature,
// ConstructEmptyLinkFeature,
// CollisionFilterMaskFeature

// ** tpe
// GetEngineInfo,
// GetWorldFromEngine,
// GetModelFromWorld,
// GetNestedModelFromModel,
// GetLinkFromModel,
// GetShapeFromLink,
// RemoveEntities,
// ConstructEmptyWorldFeature,
// ConstructEmptyModelFeature,
// ConstructEmptyNestedModelFeature,
// ConstructEmptyLinkFeature,
// CollisionFilterMaskFeature

using FeaturesBullet = gz::physics::FeatureList<
  gz::physics::GetEntities,
  gz::physics::RemoveModelFromWorld,
  gz::physics::ConstructEmptyWorldFeature
  // gz::physics::ConstructEmptyModelFeature
>;

// bullet
// GetEntities,
// RemoveModelFromWorld,
// ConstructEmptyWorldFeature

/////////////////////////////////////////////////
TEST_P(EntityManagementFeaturesTest, ConstructEmptyWorld)
{
  auto commonTest = [&](auto engine)
  {
    ASSERT_NE(nullptr, engine);
    EXPECT_TRUE(engine->GetName().find(GetParam()) != std::string::npos);
    // EXPECT_EQ(0u, engine->GetIndex());
    // EXPECT_EQ(0u, engine->GetWorldCount());
  };

  auto specificTest = [&](auto engine)
  {
    auto world = engine->ConstructEmptyWorld("empty world");
    ASSERT_NE(nullptr, world);
    EXPECT_EQ("empty world", world->GetName());
    EXPECT_EQ(1u, engine->GetWorldCount());
    // EXPECT_EQ(0u, world->GetIndex());
    // EXPECT_EQ(0u, world->GetModelCount());

    EXPECT_EQ(engine, world->GetEngine());
    EXPECT_EQ(world, engine->GetWorld(0));
    EXPECT_EQ(world, engine->GetWorld("empty world"));

    auto model = world->ConstructEmptyModel("empty model");
    ASSERT_NE(nullptr, model);
    EXPECT_EQ("empty model", model->GetName());
    EXPECT_EQ(world, model->GetWorld());
    EXPECT_NE(model, world->ConstructEmptyModel("dummy"));

    // auto nestedModel = model->ConstructEmptyNestedModel("empty nested model");
    // ASSERT_NE(nullptr, nestedModel);
    // EXPECT_EQ("empty nested model", nestedModel->GetName());
    // EXPECT_EQ(1u, model->GetNestedModelCount());
    // EXPECT_EQ(world, nestedModel->GetWorld());
    // EXPECT_EQ(0u, model->GetIndex());
    // EXPECT_EQ(nestedModel, model->GetNestedModel(0));
    // EXPECT_EQ(nestedModel, model->GetNestedModel("empty nested model"));
    // EXPECT_NE(nestedModel, nestedModel->ConstructEmptyNestedModel("dummy"));

    auto link = model->ConstructEmptyLink("empty link");
    ASSERT_NE(nullptr, link);
    EXPECT_EQ("empty link", link->GetName());
    EXPECT_EQ(model, link->GetModel());
    EXPECT_NE(link, model->ConstructEmptyLink("dummy"));
    EXPECT_EQ(0u, link->GetIndex());
    EXPECT_EQ(model, link->GetModel());

    // auto joint = link->AttachRevoluteJoint(nullptr);
    // EXPECT_NEAR((Eigen::Vector3d::UnitX() - joint->GetAxis()).norm(), 0.0, 1e-6);
    // EXPECT_DOUBLE_EQ(0.0, joint->GetPosition(0));

    // joint->SetAxis(Eigen::Vector3d::UnitZ());
    // EXPECT_NEAR((Eigen::Vector3d::UnitZ() - joint->GetAxis()).norm(), 0.0, 1e-6);

    auto child = model->ConstructEmptyLink("child link");
    EXPECT_EQ(2u, child->GetIndex());
    EXPECT_EQ(model, child->GetModel());

    const std::string boxName = "box";
    const Eigen::Vector3d boxSize(0.1, 0.2, 0.3);
    auto box = link->AttachBoxShape(boxName, boxSize);
    EXPECT_EQ(boxName, box->GetName());
    EXPECT_NEAR((boxSize - box->GetSize()).norm(), 0.0, 1e-6);

    EXPECT_EQ(1u, link->GetShapeCount());
    auto boxCopy = link->GetShape(0u);
    EXPECT_EQ(box, boxCopy);


    // auto meshLink = model->ConstructEmptyLink("mesh_link");
    // meshLink->AttachFixedJoint(child, "fixed");
    //
    // const std::string meshFilename = gz::common::joinPaths(
    //     GZ_PHYSICS_RESOURCE_DIR, "chassis.dae");
    // auto &meshManager = *gz::common::MeshManager::Instance();
    // auto *mesh = meshManager.Load(meshFilename);
    //
    // auto meshShape = meshLink->AttachMeshShape("chassis", *mesh);
    // const auto originalMeshSize = mesh->Max() - mesh->Min();
    // const auto meshShapeSize = meshShape->GetSize();
  };

  if (std::string(GetParam()) == "bullet")
  {
    auto engineBullet =
        gz::physics::RequestEngine3d<FeaturesBullet>::From(physicsPlugin);
    ASSERT_NE(nullptr, engineBullet);
    commonTest(engineBullet);
  }
  else
  {
    auto engine =
        gz::physics::RequestEngine3d<Features>::From(physicsPlugin);
    ASSERT_NE(nullptr, engine);
    commonTest(engine);
    specificTest(engine);
  }

}

TEST_P(EntityManagementFeaturesTest, RemoveEntities)
{
  auto commonTest = [&](auto engine)
  {
      ASSERT_NE(nullptr, engine);
      if (std::string(GetParam()) == "bullet")
        return;
      auto world = engine->ConstructEmptyWorld("empty world");
      ASSERT_NE(nullptr, world);
  };

  auto specificTest = [&](auto engine)
  {
    auto world = engine->ConstructEmptyWorld("empty world");
    auto model = world->ConstructEmptyModel("empty model");
    ASSERT_NE(nullptr, model);
    auto modelAlias = world->GetModel(0);

    model->Remove();
    EXPECT_TRUE(model->Removed());
    EXPECT_TRUE(modelAlias->Removed());
    EXPECT_EQ(nullptr, world->GetModel(0));
    EXPECT_EQ(nullptr, world->GetModel("empty model"));
    EXPECT_EQ(0ul, world->GetModelCount());

    auto model2 = world->ConstructEmptyModel("model2");
    ASSERT_NE(nullptr, model2);
    EXPECT_EQ(0ul, model2->GetIndex());
    world->RemoveModel(0);
    EXPECT_EQ(0ul, world->GetModelCount());

    auto model3 = world->ConstructEmptyModel("model 3");
    ASSERT_NE(nullptr, model3);
    EXPECT_EQ(1u, world->GetModelCount());
    world->RemoveModel("model 3");
    EXPECT_EQ(0ul, world->GetModelCount());
    EXPECT_EQ(nullptr, world->GetModel("model 3"));
  };

  if (std::string(GetParam()) == "bullet")
  {
    auto engineBullet =
        gz::physics::RequestEngine3d<FeaturesBullet>::From(physicsPlugin);
    commonTest(engineBullet);
  }
  else
  {
    auto engine =
        gz::physics::RequestEngine3d<Features>::From(physicsPlugin);
    commonTest(engine);
    specificTest(engine);
  }

  //
  // auto parentModel = world->ConstructEmptyModel("parent model");
  // ASSERT_NE(nullptr, parentModel);
  // EXPECT_EQ(0u, parentModel->GetNestedModelCount());
  // auto nestedModel1 =
  //     parentModel->ConstructEmptyNestedModel("empty nested model1");
  // ASSERT_NE(nullptr, nestedModel1);
  // EXPECT_EQ(1u, parentModel->GetNestedModelCount());
  //
  // EXPECT_TRUE(parentModel->RemoveNestedModel(0));
  // EXPECT_EQ(0u, parentModel->GetNestedModelCount());
  // EXPECT_TRUE(nestedModel1->Removed());
  //
  // auto nestedModel2 =
  //     parentModel->ConstructEmptyNestedModel("empty nested model2");
  // ASSERT_NE(nullptr, nestedModel2);
  // EXPECT_EQ(nestedModel2, parentModel->GetNestedModel(0));
  // EXPECT_TRUE(parentModel->RemoveNestedModel("empty nested model2"));
  // EXPECT_EQ(0u, parentModel->GetNestedModelCount());
  // EXPECT_TRUE(nestedModel2->Removed());
  //
  // auto nestedModel3 =
  //     parentModel->ConstructEmptyNestedModel("empty nested model3");
  // ASSERT_NE(nullptr, nestedModel3);
  // EXPECT_EQ(nestedModel3, parentModel->GetNestedModel(0));
  // EXPECT_TRUE(nestedModel3->Remove());
  // EXPECT_EQ(0u, parentModel->GetNestedModelCount());
  // EXPECT_TRUE(nestedModel3->Removed());
  //
  // auto nestedModel4 =
  //     parentModel->ConstructEmptyNestedModel("empty nested model4");
  // ASSERT_NE(nullptr, nestedModel4);
  // EXPECT_EQ(nestedModel4, parentModel->GetNestedModel(0));
  // // Remove the parent model and check that the nested model is removed as well
  // EXPECT_TRUE(parentModel->Remove());
  // EXPECT_TRUE(nestedModel4->Removed());
}

INSTANTIATE_TEST_CASE_P(EntityManagementFeatures, EntityManagementFeaturesTest,
    PHYSICS_ENGINE_VALUES,
    gz::physics::PrintToStringParam());
