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

struct TestFeatureList : ignition::physics::FeatureList<
  ignition::physics::tpe::plugin::EntityManagementFeatureList
> { };

TEST(EntityManagement_TEST, ConstructEmptyWorld)
{
  ignition::plugin::Loader loader;
  loader.LoadLib(tpe_plugin_LIB);

  ignition::plugin::PluginPtr tpe_plugin =
    loader.Instantiate("ignition::physics::tpe::plugin::Plugin");

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
}

TEST(EntityManagement_TEST, RemoveEntities)
{
  ignition::plugin::Loader loader;
  loader.LoadLib(tpe_plugin_LIB);

  ignition::plugin::PluginPtr tpe_plugin =
    loader.Instantiate("ignition::physics::tpe::plugin::Plugin");

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
