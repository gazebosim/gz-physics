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
#include <gz/plugin/Loader.hh>

#include "TestLibLoader.hh"
#include "../Utils.hh"

#include <gz/physics/ConstructEmpty.hh>
#include <gz/physics/FindFeatures.hh>
#include <gz/physics/ForwardStep.hh>
#include <gz/physics/FrameSemantics.hh>
#include <gz/physics/GetEntities.hh>
#include <gz/physics/RemoveEntities.hh>
#include <gz/physics/RequestEngine.hh>

#include <gz/physics/World.hh>
#include <gz/physics/sdf/ConstructLink.hh>
#include <gz/physics/sdf/ConstructJoint.hh>
#include <gz/physics/sdf/ConstructModel.hh>
#include <gz/physics/sdf/ConstructNestedModel.hh>
#include <gz/physics/sdf/ConstructWorld.hh>

#include <sdf/Root.hh>

using AssertVectorApprox = gz::physics::test::AssertVectorApprox;

template <class T>
class WorldFeaturesTest:
  public testing::Test, public gz::physics::TestLibLoader
{
  // Documentation inherited
  public: void SetUp() override
  {
    gz::common::Console::SetVerbosity(4);

    loader.LoadLib(WorldFeaturesTest::GetLibToTest());

    // TODO(ahcorde): We should also run the 3f, 2d, and 2f variants of
    // FindFeatures
    pluginNames = gz::physics::FindFeatures3d<T>::From(loader);
    if (pluginNames.empty())
    {
      std::cerr << "No plugins with required features found in "
                << GetLibToTest() << std::endl;
      GTEST_SKIP();
    }
  }

  public: std::set<std::string> pluginNames;
  public: gz::plugin::Loader loader;
};

struct GravityFeatures : gz::physics::FeatureList<
  gz::physics::GetEngineInfo,
  gz::physics::Gravity,
  gz::physics::sdf::ConstructSdfWorld,
  gz::physics::LinkFrameSemantics,
  gz::physics::GetModelFromWorld,
  gz::physics::GetLinkFromModel,
  gz::physics::ForwardStep
> { };

using WorldFeaturesTestGravity = WorldFeaturesTest<GravityFeatures>;

/////////////////////////////////////////////////
TEST_F(WorldFeaturesTestGravity, GravityFeatures)
{
  for (const std::string &name : this->pluginNames)
  {
    std::cout << "Testing plugin: " << name << std::endl;
    gz::plugin::PluginPtr plugin = this->loader.Instantiate(name);

    auto engine = gz::physics::RequestEngine3d<GravityFeatures>::From(plugin);
    ASSERT_NE(nullptr, engine);
    EXPECT_TRUE(engine->GetName().find(this->PhysicsEngineName(name)) !=
                std::string::npos);

    sdf::Root root;
    const sdf::Errors errors = root.Load(
      gz::common::joinPaths(TEST_WORLD_DIR, "falling.world"));
    EXPECT_TRUE(errors.empty()) << errors;
    const sdf::World *sdfWorld = root.WorldByIndex(0);
    EXPECT_NE(nullptr, sdfWorld);

    auto world = engine->ConstructWorld(*root.WorldByIndex(0));
    EXPECT_NE(nullptr, world);

    auto graphErrors = sdfWorld->ValidateGraphs();
    EXPECT_EQ(0u, graphErrors.size()) << graphErrors;

    Eigen::Vector3d gravity = {0, 0, -9.8};

    AssertVectorApprox vectorPredicate(1e-6);
    EXPECT_PRED_FORMAT2(vectorPredicate, gravity,
                        world->GetGravity());

    world->SetGravity({8, 4, 3});
    EXPECT_PRED_FORMAT2(vectorPredicate, Eigen::Vector3d(8, 4, 3),
                        world->GetGravity());

    world->SetGravity(gravity);

    auto model = world->GetModel("sphere");
    ASSERT_NE(nullptr, model);

    auto link = model->GetLink(0);
    ASSERT_NE(nullptr, link);

    AssertVectorApprox vectorPredicate6(1e-6);

    // initial link pose
    const Eigen::Vector3d initialLinkPosition(0, 0, 2);
    {
      Eigen::Vector3d pos = link->FrameDataRelativeToWorld().pose.translation();
      EXPECT_PRED_FORMAT2(vectorPredicate6,
                          initialLinkPosition,
                          pos);
    }

    auto linkFrameID = link->GetFrameID();

    // Get default gravity in link frame, which is pitched by pi/4
    EXPECT_PRED_FORMAT2(vectorPredicate6,
                        Eigen::Vector3d(6.92964645563, 0, -6.92964645563),
                        world->GetGravity(linkFrameID));

    // set gravity along X axis of linked frame, which is pitched by pi/4
    world->SetGravity(Eigen::Vector3d(1.4142135624, 0, 0), linkFrameID);

    EXPECT_PRED_FORMAT2(vectorPredicate6,
                        Eigen::Vector3d(1, 0, -1),
                        world->GetGravity());

    // test other SetGravity API
    // set gravity along Z axis of linked frame, which is pitched by pi/4
    gz::physics::RelativeForce3d relativeGravity(
        linkFrameID, Eigen::Vector3d(0, 0, 1.4142135624));
    world->SetGravity(relativeGravity);

    EXPECT_PRED_FORMAT2(vectorPredicate6,
                        Eigen::Vector3d(1, 0, 1),
                        world->GetGravity());

    // Confirm that changed gravity direction affects pose of link
    gz::physics::ForwardStep::Input input;
    gz::physics::ForwardStep::State state;
    gz::physics::ForwardStep::Output output;

    const size_t numSteps = 1000;
    for (size_t i = 0; i < numSteps; ++i)
    {
      world->Step(output, state, input);
    }

    AssertVectorApprox vectorPredicate2(1e-2);
    {
      Eigen::Vector3d pos = link->FrameDataRelativeToWorld().pose.translation();
      EXPECT_PRED_FORMAT2(vectorPredicate2,
                          Eigen::Vector3d(0.5, 0, 2.5),
                          pos);
    }
  }
}

struct ConstructModelFeatures : gz::physics::FeatureList<
  gz::physics::GetEngineInfo,
  gz::physics::sdf::ConstructSdfWorld,
  gz::physics::sdf::ConstructSdfModel,
  gz::physics::GetModelFromWorld,
  gz::physics::ForwardStep
> { };

using WorldFeaturesTestConstructModel =
  WorldFeaturesTest<ConstructModelFeatures>;

/////////////////////////////////////////////////
TEST_F(WorldFeaturesTestConstructModel, ConstructModelUnsortedLinks)
{
  for (const std::string &name : this->pluginNames)
  {
    std::cout << "Testing plugin: " << name << std::endl;
    gz::plugin::PluginPtr plugin = this->loader.Instantiate(name);

    auto engine =
      gz::physics::RequestEngine3d<ConstructModelFeatures>::From(plugin);
    ASSERT_NE(nullptr, engine);
    EXPECT_TRUE(engine->GetName().find(this->PhysicsEngineName(name)) !=
                std::string::npos);

    sdf::Root root;
    const sdf::Errors errors = root.Load(
      gz::common::joinPaths(TEST_WORLD_DIR, "world_unsorted_links.sdf"));
    EXPECT_TRUE(errors.empty()) << errors;
    const sdf::World *sdfWorld = root.WorldByIndex(0);
    ASSERT_NE(nullptr, sdfWorld);

    // Per https://github.com/gazebosim/gz-physics/pull/562, there is a
    // seg-fault in the bullet-featherstone implementation of ConstructSdfModel
    // (called by ConstructSdfWorld). So loading the world successfully
    // is enough for this test.
    auto world = engine->ConstructWorld(*sdfWorld);
    EXPECT_NE(nullptr, world);
  }
}

struct WorldModelFeatureList : gz::physics::FeatureList<
  GravityFeatures,
  gz::physics::WorldModelFeature,
  gz::physics::RemoveEntities,
  gz::physics::GetNestedModelFromModel,
  gz::physics::sdf::ConstructSdfLink,
  gz::physics::sdf::ConstructSdfJoint,
  gz::physics::sdf::ConstructSdfModel,
  gz::physics::sdf::ConstructSdfNestedModel,
  gz::physics::ConstructEmptyLinkFeature,
  gz::physics::ConstructEmptyNestedModelFeature
> { };

class WorldModelTest : public WorldFeaturesTest<WorldModelFeatureList>
{
  public: gz::physics::World3dPtr<WorldModelFeatureList> LoadWorld(
      const std::string &_pluginName)
  {
    gz::plugin::PluginPtr plugin = this->loader.Instantiate(_pluginName);

    auto engine =
        gz::physics::RequestEngine3d<WorldModelFeatureList>::From(plugin);

    sdf::Root root;
    const sdf::Errors errors = root.Load(
        gz::common::joinPaths(TEST_WORLD_DIR, "world_joint_test.sdf"));
    EXPECT_TRUE(errors.empty()) << errors;
    if (errors.empty())
    {
      auto world = engine->ConstructWorld(*root.WorldByIndex(0));
      return world;
    }
    return nullptr;
  }
};

TEST_F(WorldModelTest, WorldModelAPI)
{
  for (const std::string &name : this->pluginNames)
  {
    auto world = this->LoadWorld(name);
    ASSERT_NE(nullptr, world);

    auto worldModel = world->GetWorldModel();
    ASSERT_NE(nullptr, worldModel);
    EXPECT_EQ(world, worldModel->GetWorld());
    EXPECT_EQ("default", worldModel->GetName());
    EXPECT_EQ(0, worldModel->GetLinkCount());
    EXPECT_EQ(nullptr, worldModel->GetLink(0));
    EXPECT_EQ(nullptr, worldModel->GetLink("nonexistent_link"));
    EXPECT_EQ(0, worldModel->GetIndex());
    EXPECT_EQ(world->GetModelCount(), worldModel->GetNestedModelCount());
    const auto nestedModel = worldModel->GetNestedModel(0);
    ASSERT_NE(nullptr, nestedModel);
    EXPECT_EQ("m1", nestedModel->GetName());
    EXPECT_NE(nullptr, worldModel->GetNestedModel("m2"));

    // Check that removing a World model proxy is not allowed
    EXPECT_FALSE(worldModel->Remove());
    EXPECT_TRUE(worldModel.Valid());
    EXPECT_FALSE(worldModel->Removed());
    ASSERT_NE(nullptr, worldModel);
    EXPECT_EQ(world, worldModel->GetWorld());

    auto worldModel2 = world->GetWorldModel();
    ASSERT_NE(nullptr, worldModel2);
    EXPECT_EQ(world, worldModel2->GetWorld());

    EXPECT_EQ(2u, world->GetModelCount());
    EXPECT_EQ(2u, worldModel->GetNestedModelCount());

    // Check that we can remove models via RemoveNestedModel
    EXPECT_TRUE(worldModel->RemoveNestedModel(0));
    EXPECT_TRUE(nestedModel->Removed());
    EXPECT_EQ(1u, world->GetModelCount());
    EXPECT_EQ(1u, worldModel->GetNestedModelCount());
    EXPECT_NE(nullptr, worldModel->GetNestedModel(0));
    EXPECT_EQ(nullptr, worldModel->GetNestedModel("m1"));

    const auto m2 = worldModel->GetNestedModel("m2");
    ASSERT_NE(nullptr, m2);
    EXPECT_TRUE(worldModel->RemoveNestedModel("m2"));
    EXPECT_TRUE(m2->Removed());
    EXPECT_EQ(0u, world->GetModelCount());
    EXPECT_EQ(0u, worldModel->GetNestedModelCount());
    EXPECT_EQ(nullptr, worldModel->GetNestedModel("m2"));

    // Check that we can construct nested models and joints, but not links
    auto m3 = worldModel->ConstructEmptyNestedModel("m3");
    ASSERT_TRUE(m3);
    EXPECT_EQ(m3, world->GetModel("m3"));
    EXPECT_EQ(m3, worldModel->GetNestedModel("m3"));
    EXPECT_FALSE(worldModel->ConstructEmptyLink("test_link"));

    sdf::Link sdfLink;
    sdfLink.SetName("link3");
    EXPECT_FALSE(worldModel->ConstructLink(sdfLink));

    // Create a link in m3 for testing joint creation.
    EXPECT_TRUE(m3->ConstructLink(sdfLink));
    sdf::Joint sdfJoint;
    sdfJoint.SetName("test_joint");
    sdfJoint.SetType(sdf::JointType::FIXED);
    sdfJoint.SetParentName("m2::link2");
    sdfJoint.SetChildName("m3::link3");
    EXPECT_TRUE(worldModel->ConstructJoint(sdfJoint));

    sdf::Model sdfModel;
    sdfModel.SetName("m4");
    sdfModel.AddLink(sdfLink);
    EXPECT_TRUE(worldModel->ConstructNestedModel(sdfModel));
    EXPECT_TRUE(world->GetModel("m4"));
    EXPECT_TRUE(worldModel->GetNestedModel("m4"));

    EXPECT_EQ(2u, world->GetModelCount());
    EXPECT_EQ(2u, worldModel->GetNestedModelCount());
    // Check that we can remove created via the World model proxy
    EXPECT_EQ(m3, worldModel->GetNestedModel(0));
    EXPECT_TRUE(worldModel->RemoveNestedModel(0));
    EXPECT_TRUE(m3->Removed());
    EXPECT_EQ(1u, world->GetModelCount());
    EXPECT_EQ(1u, worldModel->GetNestedModelCount());
    EXPECT_EQ(nullptr, worldModel->GetNestedModel("m3"));
    EXPECT_EQ(nullptr, world->GetModel("m3"));

    const auto m4 = worldModel->GetNestedModel("m4");
    ASSERT_NE(nullptr, m4);
    EXPECT_TRUE(worldModel->RemoveNestedModel("m4"));
    EXPECT_EQ(0u, world->GetModelCount());
    EXPECT_EQ(0u, worldModel->GetNestedModelCount());
    EXPECT_TRUE(m4->Removed());
    EXPECT_EQ(nullptr, worldModel->GetNestedModel("m4"));
    EXPECT_EQ(nullptr, world->GetModel("m4"));
  }
}

struct WorldNestedModelFeatureList : gz::physics::FeatureList<
  GravityFeatures,
  gz::physics::WorldModelFeature,
  gz::physics::RemoveEntities,
  gz::physics::GetNestedModelFromModel,
  // gz::physics::sdf::ConstructSdfLink,
  gz::physics::sdf::ConstructSdfJoint,
  gz::physics::sdf::ConstructSdfModel,
  gz::physics::sdf::ConstructSdfNestedModel
  // gz::physics::ConstructEmptyLinkFeature,
  // gz::physics::ConstructEmptyNestedModelFeature
> { };


class WorldNestedModelTest : public WorldFeaturesTest<WorldNestedModelFeatureList>
{
  public: gz::physics::World3dPtr<WorldNestedModelFeatureList> LoadWorld(
      const std::string &_pluginName)
  {
    gz::plugin::PluginPtr plugin = this->loader.Instantiate(_pluginName);

    auto engine =
        gz::physics::RequestEngine3d<WorldNestedModelFeatureList>::From(plugin);

    sdf::Root root;
    const sdf::Errors errors = root.Load(
        gz::common::joinPaths(TEST_WORLD_DIR, "world_nested_model_test.sdf"));
    EXPECT_TRUE(errors.empty()) << errors;
    if (errors.empty())
    {
      auto world = engine->ConstructWorld(*root.WorldByIndex(0));
      return world;
    }
    return nullptr;
  }
};

TEST_F(WorldModelTest, WorldConstructNestedModel)
{
  for (const std::string &name : this->pluginNames)
  {
    auto world = this->LoadWorld(name);
    ASSERT_NE(nullptr, world);

    auto worldModel = world->GetWorldModel();
    ASSERT_NE(nullptr, worldModel);
    EXPECT_EQ(world, worldModel->GetWorld());
    EXPECT_EQ("default", worldModel->GetName());
    EXPECT_EQ(0, worldModel->GetLinkCount());
    EXPECT_EQ(nullptr, worldModel->GetLink(0));
    EXPECT_EQ(nullptr, worldModel->GetLink("nonexistent_link"));
    EXPECT_EQ(0, worldModel->GetIndex());
    EXPECT_EQ(world->GetModelCount(), worldModel->GetNestedModelCount());
    const auto nestedModel = worldModel->GetNestedModel(0);
    ASSERT_NE(nullptr, nestedModel);
    EXPECT_EQ("m1", nestedModel->GetName());
    EXPECT_NE(nullptr, worldModel->GetNestedModel("m2"));

    // Check that removing a World model proxy is not allowed
    EXPECT_FALSE(worldModel->Remove());
    EXPECT_TRUE(worldModel.Valid());
    EXPECT_FALSE(worldModel->Removed());
    ASSERT_NE(nullptr, worldModel);
    EXPECT_EQ(world, worldModel->GetWorld());

    auto worldModel2 = world->GetWorldModel();
    ASSERT_NE(nullptr, worldModel2);
    EXPECT_EQ(world, worldModel2->GetWorld());

    EXPECT_EQ(2u, world->GetModelCount());
    EXPECT_EQ(2u, worldModel->GetNestedModelCount());

    // Check that we can remove models via RemoveNestedModel
    EXPECT_TRUE(worldModel->RemoveNestedModel(0));
    EXPECT_TRUE(nestedModel->Removed());
    EXPECT_EQ(1u, world->GetModelCount());
    EXPECT_EQ(1u, worldModel->GetNestedModelCount());
    EXPECT_NE(nullptr, worldModel->GetNestedModel(0));
    EXPECT_EQ(nullptr, worldModel->GetNestedModel("m1"));

    const auto m2 = worldModel->GetNestedModel("m2");
    ASSERT_NE(nullptr, m2);
    EXPECT_TRUE(worldModel->RemoveNestedModel("m2"));
    EXPECT_TRUE(m2->Removed());
    EXPECT_EQ(0u, world->GetModelCount());
    EXPECT_EQ(0u, worldModel->GetNestedModelCount());
    EXPECT_EQ(nullptr, worldModel->GetNestedModel("m2"));

    // Check that we can construct nested models and joints, but not links
    auto m3 = worldModel->ConstructEmptyNestedModel("m3");
    ASSERT_TRUE(m3);
    EXPECT_EQ(m3, world->GetModel("m3"));
    EXPECT_EQ(m3, worldModel->GetNestedModel("m3"));
    EXPECT_FALSE(worldModel->ConstructEmptyLink("test_link"));

    sdf::Link sdfLink;
    sdfLink.SetName("link3");
    EXPECT_FALSE(worldModel->ConstructLink(sdfLink));

    // Create a link in m3 for testing joint creation.
    EXPECT_TRUE(m3->ConstructLink(sdfLink));
    sdf::Joint sdfJoint;
    sdfJoint.SetName("test_joint");
    sdfJoint.SetType(sdf::JointType::FIXED);
    sdfJoint.SetParentName("m2::link2");
    sdfJoint.SetChildName("m3::link3");
    EXPECT_TRUE(worldModel->ConstructJoint(sdfJoint));

    sdf::Model sdfModel;
    sdfModel.SetName("m4");
    sdfModel.AddLink(sdfLink);
    EXPECT_TRUE(worldModel->ConstructNestedModel(sdfModel));
    EXPECT_TRUE(world->GetModel("m4"));
    EXPECT_TRUE(worldModel->GetNestedModel("m4"));

    EXPECT_EQ(2u, world->GetModelCount());
    EXPECT_EQ(2u, worldModel->GetNestedModelCount());
    // Check that we can remove created via the World model proxy
    EXPECT_EQ(m3, worldModel->GetNestedModel(0));
    EXPECT_TRUE(worldModel->RemoveNestedModel(0));
    EXPECT_TRUE(m3->Removed());
    EXPECT_EQ(1u, world->GetModelCount());
    EXPECT_EQ(1u, worldModel->GetNestedModelCount());
    EXPECT_EQ(nullptr, worldModel->GetNestedModel("m3"));
    EXPECT_EQ(nullptr, world->GetModel("m3"));

    const auto m4 = worldModel->GetNestedModel("m4");
    ASSERT_NE(nullptr, m4);
    EXPECT_TRUE(worldModel->RemoveNestedModel("m4"));
    EXPECT_EQ(0u, world->GetModelCount());
    EXPECT_EQ(0u, worldModel->GetNestedModelCount());
    EXPECT_TRUE(m4->Removed());
    EXPECT_EQ(nullptr, worldModel->GetNestedModel("m4"));
    EXPECT_EQ(nullptr, world->GetModel("m4"));
  }
}


int main(int argc, char *argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  if(!WorldFeaturesTest<GravityFeatures>::init(argc, argv))
    return -1;
  return RUN_ALL_TESTS();
}
