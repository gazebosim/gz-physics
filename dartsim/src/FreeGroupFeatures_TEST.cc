/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include <gz/common/Util.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/eigen3/Conversions.hh>
#include <gz/plugin/Loader.hh>
#include <sdf/Root.hh>
#include <sdf/World.hh>

#include "gz/physics/FrameSemantics.hh"
#include "gz/physics/FreeGroup.hh"
#include "gz/physics/GetEntities.hh"
#include "gz/physics/RequestEngine.hh"
#include "gz/physics/sdf/ConstructModel.hh"
#include "gz/physics/sdf/ConstructNestedModel.hh"
#include "gz/physics/sdf/ConstructWorld.hh"
#include "test/Utils.hh"

struct TestFeatureList : gz::physics::FeatureList<
    gz::physics::sdf::ConstructSdfWorld,
    gz::physics::sdf::ConstructSdfModel,
    gz::physics::sdf::ConstructSdfNestedModel,
    gz::physics::GetWorldFromEngine,
    gz::physics::GetModelFromWorld,
    gz::physics::GetNestedModelFromModel,
    gz::physics::GetLinkFromModel,
    gz::physics::LinkFrameSemantics,
    gz::physics::FindFreeGroupFeature,
    gz::physics::SetFreeGroupWorldPose > { };

using World = gz::physics::World3d<TestFeatureList>;
using WorldPtr = gz::physics::World3dPtr<TestFeatureList>;
using ModelPtr = gz::physics::Model3dPtr<TestFeatureList>;
using LinkPtr = gz::physics::Link3dPtr<TestFeatureList>;

/////////////////////////////////////////////////
auto LoadEngine()
{
  gz::plugin::Loader loader;
  loader.LoadLib(dartsim_plugin_LIB);

  gz::plugin::PluginPtr dartsim =
      loader.Instantiate("gz::physics::dartsim::Plugin");

  auto engine =
      gz::physics::RequestEngine3d<TestFeatureList>::From(dartsim);
  return engine;
}

/////////////////////////////////////////////////
WorldPtr LoadWorld(const std::string &_world)
{
  auto engine = LoadEngine();
  EXPECT_NE(nullptr, engine);
  if (nullptr == engine)
  {
    return nullptr;
  }

  sdf::Root root;
  const sdf::Errors &errors = root.Load(_world);
  EXPECT_EQ(0u, errors.size()) << errors;

  EXPECT_EQ(1u, root.WorldCount());
  const sdf::World *sdfWorld = root.WorldByIndex(0);
  EXPECT_NE(nullptr, sdfWorld);

  auto world = engine->ConstructWorld(*sdfWorld);
  EXPECT_NE(nullptr, world);

  return world;
}

ModelPtr GetModelFromAbsoluteName(const WorldPtr &_world,
                                  const std::string &_absoluteName)
{
  std::vector<std::string> names =
      gz::common::split(_absoluteName, sdf::kSdfScopeDelimiter);
  if (names.empty())
  {
    return nullptr;
  }

  auto currentModel = _world->GetModel(names.front());
  for (std::size_t i = 1; i < names.size(); ++i)
  {
    if (nullptr == currentModel)
      return nullptr;
    currentModel = currentModel->GetNestedModel(names[i]);
  }
  return currentModel;
}

TEST(FreeGroupFeatures, NestedFreeGroup)
{
  WorldPtr world = LoadWorld(TEST_WORLD_DIR "world_with_nested_model.sdf");
  ASSERT_NE(nullptr, world);

  auto checkFreeGroupForModel = [&](const std::string &_modelName)
  {
    auto model = GetModelFromAbsoluteName(world, _modelName);
    if (nullptr == model)
      return testing::AssertionFailure() << "Model could not be found";
    auto freeGroup = model->FindFreeGroup();
    if (nullptr == freeGroup)
      return testing::AssertionFailure() << "Freegroup could not be found";
    if (nullptr == freeGroup->RootLink())
      return testing::AssertionFailure() << "RootLink could not be found";
    return testing::AssertionSuccess();
  };

  EXPECT_TRUE(checkFreeGroupForModel("parent_model"));
  // Expect false because the link in nested_model is referenced by a joint.
  EXPECT_FALSE(checkFreeGroupForModel("parent_model::nested_model"));
  EXPECT_TRUE(checkFreeGroupForModel("parent_model::nested_model2"));
  EXPECT_TRUE(checkFreeGroupForModel("parent_model::nested_model3"));
}

TEST(FreeGroupFeatures, NestedFreeGroupSetWorldPose)
{
  WorldPtr world = LoadWorld(TEST_WORLD_DIR "world_with_nested_model.sdf");
  ASSERT_NE(nullptr, world);

  gz::math::Pose3d parentModelNewPose(0, 0, 2, 0, 0, 0);
  {
    auto parentModel = world->GetModel("parent_model");
    ASSERT_NE(nullptr, parentModel);
    auto freeGroup = parentModel->FindFreeGroup();
    ASSERT_NE(nullptr, freeGroup);

    freeGroup->SetWorldPose(
        gz::math::eigen3::convert(parentModelNewPose));

    auto link1 = parentModel->GetLink("link1");
    ASSERT_NE(nullptr, link1);
    auto frameData = link1->FrameDataRelativeToWorld();
    EXPECT_EQ(parentModelNewPose,
              gz::math::eigen3::convert(frameData.pose));
  }
  {
    auto nestedModel =
        GetModelFromAbsoluteName(world, "parent_model::nested_model");
    ASSERT_NE(nullptr, nestedModel);
    auto nestedLink1 = nestedModel->GetLink("nested_link1");
    ASSERT_NE(nullptr, nestedLink1);
    auto frameData = nestedLink1->FrameDataRelativeToWorld();
    // Poses from SDF
    gz::math::Pose3d nestedModelPose(1, 2, 2, 0, 0, 0);
    gz::math::Pose3d nestedLinkPose(3, 1, 1, 0, 0, IGN_PI_2);

    gz::math::Pose3d nestedLinkExpectedPose =
        parentModelNewPose * nestedModelPose * nestedLinkPose;

    EXPECT_EQ(nestedLinkExpectedPose,
        gz::math::eigen3::convert(frameData.pose));
  }
  {
    auto parentModel = world->GetModel("parent_model2");
    ASSERT_NE(nullptr, parentModel);
    auto freeGroup = parentModel->FindFreeGroup();
    ASSERT_NE(nullptr, freeGroup);

    freeGroup->SetWorldPose(
        gz::math::eigen3::convert(parentModelNewPose));

    auto grandChild = GetModelFromAbsoluteName(
        world, "parent_model2::child_model::grand_child_model");
    ASSERT_NE(nullptr, grandChild);
    auto link1 = grandChild->GetLink("link1");
    ASSERT_NE(nullptr, link1);
    auto frameData = link1->FrameDataRelativeToWorld();
    EXPECT_EQ(parentModelNewPose,
              gz::math::eigen3::convert(frameData.pose));
  }
}
