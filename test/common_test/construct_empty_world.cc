/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#include "../helpers/TestLibLoader.hh"

#include <gz/physics/ConstructEmpty.hh>
#include <gz/physics/FindFeatures.hh>
#include <gz/physics/GetEntities.hh>
#include <gz/physics/RequestEngine.hh>
#include <gz/physics/RemoveEntities.hh>

#include "gz/physics/BoxShape.hh"

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
>;

class ConstructEmptyWorldTest:
  public testing::Test, public gz::physics::TestLibLoader
{
  // Documentation inherited
  public: void SetUp() override
  {
    gz::common::Console::SetVerbosity(4);

    loader.LoadLib(ConstructEmptyWorldTest::GetLibToTest());

    // TODO(ahcorde): We should also run the 3f, 2d, and 2f variants of
    // FindFeatures
    pluginNames = gz::physics::FindFeatures3d<Features>::From(loader);
    if (pluginNames.empty())
    {
      std::cerr << "No plugins with required features found in "
                << GetLibToTest() << std::endl;
      // TODO(ahcorde): If we update gtest we can use here GTEST_SKIP()
    }
  }
};

/////////////////////////////////////////////////
TEST_F(ConstructEmptyWorldTest, ConstructEmptyWorld)
{
  for (const std::string &name : pluginNames)
  {
    std::cout << "Testing plugin: " << name << std::endl;
    gz::plugin::PluginPtr plugin = loader.Instantiate(name);

    auto engine = gz::physics::RequestEngine3d<Features>::From(plugin);
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

    auto parentModel = world->ConstructEmptyModel("parent model");
    ASSERT_NE(nullptr, parentModel);
    EXPECT_EQ(0u, parentModel->GetNestedModelCount());
    auto nestedModel1 =
        parentModel->ConstructEmptyNestedModel("empty nested model1");
    ASSERT_NE(nullptr, nestedModel1);
    EXPECT_EQ(1u, parentModel->GetNestedModelCount());

    EXPECT_TRUE(parentModel->RemoveNestedModel(0));
    EXPECT_EQ(0u, parentModel->GetNestedModelCount());
    EXPECT_TRUE(nestedModel1->Removed());

    auto nestedModel2 =
        parentModel->ConstructEmptyNestedModel("empty nested model2");
    ASSERT_NE(nullptr, nestedModel2);
    EXPECT_EQ(nestedModel2, parentModel->GetNestedModel(0));
    EXPECT_TRUE(parentModel->RemoveNestedModel("empty nested model2"));
    EXPECT_EQ(0u, parentModel->GetNestedModelCount());
    EXPECT_TRUE(nestedModel2->Removed());

    auto nestedModel3 =
        parentModel->ConstructEmptyNestedModel("empty nested model3");
    ASSERT_NE(nullptr, nestedModel3);
    EXPECT_EQ(nestedModel3, parentModel->GetNestedModel(0));
    EXPECT_TRUE(nestedModel3->Remove());
    EXPECT_EQ(0u, parentModel->GetNestedModelCount());
    EXPECT_TRUE(nestedModel3->Removed());

    auto nestedModel4 =
        parentModel->ConstructEmptyNestedModel("empty nested model4");
    ASSERT_NE(nullptr, nestedModel4);
    EXPECT_EQ(nestedModel4, parentModel->GetNestedModel(0));
    // Remove the parent model and check that the nested model is removed as well
    EXPECT_TRUE(parentModel->Remove());
    EXPECT_TRUE(nestedModel4->Removed());

    model = world->ConstructEmptyModel("empty model");
    ASSERT_NE(nullptr, model);
    auto link = model->ConstructEmptyLink("empty link");
    ASSERT_NE(nullptr, link);
    EXPECT_EQ("empty link", link->GetName());
    EXPECT_EQ(model, link->GetModel());
    EXPECT_NE(link, model->ConstructEmptyLink("dummy"));
    EXPECT_EQ(0u, link->GetIndex());
    EXPECT_EQ(model, link->GetModel());

    const std::string boxName = "box";
    const Eigen::Vector3d boxSize(0.1, 0.2, 0.3);
    auto box = link->AttachBoxShape(boxName, boxSize);
    EXPECT_EQ(boxName, box->GetName());
    EXPECT_NEAR((boxSize - box->GetSize()).norm(), 0.0, 1e-6);

    EXPECT_EQ(1u, link->GetShapeCount());
    auto boxCopy = link->GetShape(0u);
    EXPECT_EQ(box, boxCopy);

  }
}

/////////////////////////////////////////////////
TEST_F(ConstructEmptyWorldTest, RemoveEntities)
{
  for (const std::string &name : pluginNames)
  {
    std::cout << "Testing plugin: " << name << std::endl;
    gz::plugin::PluginPtr plugin = loader.Instantiate(name);

    auto engine = gz::physics::RequestEngine3d<Features>::From(plugin);
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

    auto parentModel = world->ConstructEmptyModel("parent model");
    ASSERT_NE(nullptr, parentModel);
    EXPECT_EQ(0u, parentModel->GetNestedModelCount());
    auto nestedModel1 =
        parentModel->ConstructEmptyNestedModel("empty nested model1");
    ASSERT_NE(nullptr, nestedModel1);
    EXPECT_EQ(1u, parentModel->GetNestedModelCount());

    EXPECT_TRUE(parentModel->RemoveNestedModel(0));
    EXPECT_EQ(0u, parentModel->GetNestedModelCount());
    EXPECT_TRUE(nestedModel1->Removed());

    auto nestedModel2 =
        parentModel->ConstructEmptyNestedModel("empty nested model2");
    ASSERT_NE(nullptr, nestedModel2);
    EXPECT_EQ(nestedModel2, parentModel->GetNestedModel(0));
    EXPECT_TRUE(parentModel->RemoveNestedModel("empty nested model2"));
    EXPECT_EQ(0u, parentModel->GetNestedModelCount());
    EXPECT_TRUE(nestedModel2->Removed());

    auto nestedModel3 =
        parentModel->ConstructEmptyNestedModel("empty nested model3");
    ASSERT_NE(nullptr, nestedModel3);
    EXPECT_EQ(nestedModel3, parentModel->GetNestedModel(0));
    EXPECT_TRUE(nestedModel3->Remove());
    EXPECT_EQ(0u, parentModel->GetNestedModelCount());
    EXPECT_TRUE(nestedModel3->Removed());

    auto nestedModel4 =
        parentModel->ConstructEmptyNestedModel("empty nested model4");
    ASSERT_NE(nullptr, nestedModel4);
    EXPECT_EQ(nestedModel4, parentModel->GetNestedModel(0));
    // Remove the parent model and check that the nested model is removed as well
    EXPECT_TRUE(parentModel->Remove());
    EXPECT_TRUE(nestedModel4->Removed());
  }
}

TEST_F(ConstructEmptyWorldTest, ModelByIndexWithNestedModels)
{
  for (const std::string &name : pluginNames)
  {
    std::cout << "Testing plugin: " << name << std::endl;
    gz::plugin::PluginPtr plugin = loader.Instantiate(name);

    auto engine = gz::physics::RequestEngine3d<Features>::From(plugin);
    ASSERT_NE(nullptr, engine);

    auto world = engine->ConstructEmptyWorld("empty world");
    ASSERT_NE(nullptr, world);
    auto model1 = world->ConstructEmptyModel("model1");
    ASSERT_NE(nullptr, model1);
    EXPECT_EQ(0ul, model1->GetIndex());

    auto parentModel = world->ConstructEmptyModel("parent model");
    ASSERT_NE(nullptr, parentModel);
    EXPECT_EQ(1ul, parentModel->GetIndex());

    auto nestedModel1 =
        parentModel->ConstructEmptyNestedModel("empty nested model1");
    ASSERT_NE(nullptr, nestedModel1);
    EXPECT_EQ(0ul, nestedModel1->GetIndex());

    auto model2 = world->ConstructEmptyModel("model2");
    ASSERT_NE(nullptr, model2);
    EXPECT_EQ(2ul, model2->GetIndex());
    EXPECT_TRUE(model2->Remove());

    auto model2Again = world->ConstructEmptyModel("model2_again");
    ASSERT_NE(nullptr, model2Again);
    EXPECT_EQ(2ul, model2Again->GetIndex());
  }
}

int main(int argc, char *argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  if (!ConstructEmptyWorldTest::init(argc, argv))
    return -1;
  return RUN_ALL_TESTS();
}
