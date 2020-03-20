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

#include <ignition/plugin/Loader.hh>

#include <ignition/common/MeshManager.hh>

#include <ignition/math/eigen3/Conversions.hh>

#include <ignition/physics/RequestEngine.hh>

#include "EntityManagementFeatures.hh"
#include "ShapeFeatures.hh"

struct TestFeatureList : ignition::physics::FeatureList<
  ignition::physics::tpeplugin::EntityManagementFeatureList,
  ignition::physics::tpeplugin::ShapeFeatureList
> { };

TEST(EntityManagement_TEST, ConstructEmptyWorld)
{
  ignition::plugin::Loader loader;
  loader.LoadLib(tpe_plugin_LIB);

  ignition::plugin::PluginPtr tpe_plugin =
    loader.Instantiate("ignition::physics::tpeplugin::Plugin");

  auto engine =
    ignition::physics::RequestEngine3d<TestFeatureList>::From(tpe_plugin);
  ASSERT_NE(nullptr, engine);

  // GetEntities_TEST
  auto world = engine->ConstructEmptyWorld("empty world");
  ASSERT_NE(nullptr, world);
  EXPECT_EQ("empty_world", world->GetName());
  EXPECT_EQ(engine, world->GetEngine());
  EXPECT_EQ(world, engine->ConstructEmptyWorld("dummy"));

  auto model = world->ConstructEmptyModel("empty model");
  ASSERT_NE(nullptr, model);
  EXPECT_EQ("empty model", model->GetName());
  EXPECT_EQ(world, model->GetWorld());
  EXPECT_EQ(model, world->ConstructEmptyModel("dummy"));

  auto link = model->ConstructEmptyLink("empty link");
  ASSERT_NE(nullptr, link);
  EXPECT_EQ("empty link", link->GetName());
  EXPECT_EQ(model, link->GetModel());
  EXPECT_EQ(link, model->ConstructEmptyLink("dummy"));

  const std::string boxName = "box";
  const Eigen::Vector3d boxSize(0.5, 0.5, 0.5);
  auto box = link->AttachBoxShape(boxName, boxSize);
  EXPECT_EQ(boxName, box->GetName());
  EXPECT_NEAR((boxSize - box->GetSize()).norm(), 0.0, 1e-6);

  EXPECT_EQ(1u, link->GetShapeCount());
  auto boxCopy = link->GetShape(0u);
  EXPECT_EQ(box, boxCopy);

  auto cylinderLink = model->ConstructEmptyLink("cylinder link");
  const std::string cylinderName = "cylinder";
  double cylinderRadius(1.0);
  double cylinderHeight(1.0);
  auto cylinder = cylinderLink->AttachCylinderShape(
    cylinderName, cylinderRadius, cylinderHeight);

  EXPECT_EQ(cylinderName, cylinder->GetName());
  EXPECT_NEAR(cylinderRadius - cylinder->GetRadius(), 0.0, 1e-6);
  EXPECT_NEAR(cylinderHeight - cylinder->GetHeight(), 0.0, 1e-6);

  auto sphereLink = model->ConstructEmptyLink("sphere link");
  const std::string sphereName = "sphere";
  double sphereRadius(1.0);
  auto sphere = sphereLink->AttachSphereShape(
    sphereName, sphereRadius);

  EXPECT_EQ(sphereName, sphere->GetName());
  EXPECT_NEAR(sphereRadius - sphere->GetRadius(), 0.0, 1e-6);

  auto meshLink = model->ConstructEmptyLink("mesh link");
  const std::string meshFilename = IGNITION_PHYSICS_RESOURCE_DIR "/chassis.dae";
  auto &meshManager = *ignition::common::MeshManager::Instance();
  auto *mesh = meshManager.Load(meshFilename);

  auto meshShape = meshLink->AttachMeshShape("chasis", *mesh);
  const auto originalMeshSize = mesh->Max() - mesh->Min();
  const auto meshShapeSize = meshShape->GetSize();

  // copied from dartsim test
  for (std::size_t i = 0; i < 3; ++i)
    EXPECT_NEAR(originalMeshSize[i], meshShapeSize[i], 1e-6);

  EXPECT_NEAR(meshShapeSize[0], 0.5106, 1e-4);
  EXPECT_NEAR(meshShapeSize[1], 0.3831, 1e-4);
  EXPECT_NEAR(meshShapeSize[2], 0.1956, 1e-4);

  const ignition::math::Pose3d pose(0, 0, 0.2, 0, 0, 0);
  const ignition::math::Vector3d scale(0.5, 1.0, 0.25);
  auto meshShapeScaled = meshLink->AttachMeshShape("small_chassis", *mesh,
                          ignition::math::eigen3::convert(pose),
                          ignition::math::eigen3::convert(scale));
  const auto meshShapeScaledSize = meshShapeScaled->GetSize();

  for (std::size_t i = 0; i < 3; ++i)
    EXPECT_NEAR(originalMeshSize[i] * scale[i], meshShapeScaledSize[i], 1e-6);

  EXPECT_NEAR(meshShapeScaledSize[0], 0.2553, 1e-4);
  EXPECT_NEAR(meshShapeScaledSize[1], 0.3831, 1e-4);
  EXPECT_NEAR(meshShapeScaledSize[2], 0.0489, 1e-4);
}

TEST(EntityManagement_TEST, RemoveEntities)
{
  ignition::plugin::Loader loader;
  loader.LoadLib(tpe_plugin_LIB);

  ignition::plugin::PluginPtr tpe_plugin =
    loader.Instantiate("ignition::physics::tpeplugin::Plugin");

  auto engine =
    ignition::physics::RequestEngine3d<TestFeatureList>::From(tpe_plugin);
  ASSERT_NE(nullptr, engine);

  auto world = engine->ConstructEmptyWorld("empty world");
  ASSERT_NE(nullptr, world);
  auto model = world->ConstructEmptyModel("empty model");
  ASSERT_NE(nullptr, model);

  auto modelAlias = world->GetModel(0);

  model->Remove();
  EXPECT_TRUE(model->Removed());
  EXPECT_TRUE(modelAlias->Removed());
  EXPECT_EQ(nullptr, world->GetModel(0));
  EXPECT_EQ(nullptr, world->GetModel("empty model"));
  EXPECT_EQ(0ul, world->GetModelCount());

  EXPECT_EQ("empty_model", model->GetName());

  auto model2 = world->ConstructEmptyModel("model2");
  ASSERT_NE(nullptr, model2);
  EXPECT_EQ(0ul, model2->GetIndex());
  world->RemoveModel(0);
  EXPECT_EQ(0ul, world->GetModelCount());
}

int main(int argc, char *argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
