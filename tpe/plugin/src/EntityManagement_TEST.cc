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

#include <ignition/physics/RequestEngine.hh>

#include "EntityManagementFeatures.hh"

#include "test/Utils.hh"

struct TestFeatureList : ignition::physics::FeatureList<
  ignition::physics::tpeplugin::EntityManagementFeatureList
> { };

TEST(EntityManagement_TEST, ConstructEmptyWorld)
{
  ignition::plugin::Loader loader;
  loader.LoadLib(TpePluginLib());

  ignition::plugin::PluginPtr tpe_plugin =
    loader.Instantiate("ignition::physics::tpeplugin::Plugin");

  // GetEntities_TEST
  auto engine =
    ignition::physics::RequestEngine3d<TestFeatureList>::From(tpe_plugin);
  ASSERT_NE(nullptr, engine);
  EXPECT_EQ("tpe", engine->GetName());
  EXPECT_EQ(0u, engine->GetIndex());
  EXPECT_EQ(0u, engine->GetWorldCount());

  auto world = engine->ConstructEmptyWorld("empty world");
  EXPECT_EQ(1u, engine->GetWorldCount());
  ASSERT_NE(nullptr, world);
  EXPECT_EQ("empty world", world->GetName());
  EXPECT_EQ(0u, world->GetIndex());
  EXPECT_EQ(0u, world->GetModelCount());

  EXPECT_EQ(engine, world->GetEngine());
  EXPECT_EQ(world, engine->GetWorld(0));
  EXPECT_EQ(world, engine->GetWorld("empty world"));

  auto model = world->ConstructEmptyModel("empty model");
  EXPECT_EQ(1u, world->GetModelCount());
  ASSERT_NE(nullptr, model);
  EXPECT_EQ("empty model", model->GetName());
  ASSERT_NE(model, world->ConstructEmptyModel("dummy"));
  EXPECT_EQ(2u, world->GetModelCount());
  EXPECT_EQ(0u, model->GetIndex());
  EXPECT_EQ(0u, model->GetLinkCount());

  EXPECT_EQ(world, model->GetWorld());
  EXPECT_EQ(model, world->GetModel(0));
  EXPECT_EQ(model, world->GetModel("empty model"));

  auto link = model->ConstructEmptyLink("empty link");
  EXPECT_EQ(1u, model->GetLinkCount());
  ASSERT_NE(nullptr, link);
  EXPECT_EQ("empty link", link->GetName());
  ASSERT_NE(link, model->ConstructEmptyLink("dummy"));
  EXPECT_EQ(2u, model->GetLinkCount());
  EXPECT_EQ(0u, link->GetIndex());
  EXPECT_EQ(0u, link->GetShapeCount());

  EXPECT_EQ(model, link->GetModel());
  EXPECT_EQ(link, model->GetLink(0));
  EXPECT_EQ(link, model->GetLink("empty link"));
}

TEST(EntityManagement_TEST, RemoveEntities)
{
  ignition::plugin::Loader loader;
  auto list = loader.LoadLib(TpePluginLib());

  std::cout << TpePluginLib() << " " << list.size() << std::endl;

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
}

int main(int argc, char *argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
