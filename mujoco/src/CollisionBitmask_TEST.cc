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

#include <algorithm>
#include <cstdint>
#include <memory>
#include <string>

#include <gz/physics/ForwardStep.hh>
#include <gz/physics/GetEntities.hh>
#include <gz/physics/RemoveEntities.hh>
#include <gz/physics/RequestEngine.hh>
#include <gz/physics/RequestFeatures.hh>
#include <gz/plugin/Loader.hh>
#include <sdf/Root.hh>
#include <sdf/World.hh>

#include "Base.hh"
#include "EntityManagementFeatures.hh"
#include "SDFFeatures.hh"
#include "SimulationFeatures.hh"
#include "test/common_test/Worlds.hh"

using namespace gz;

struct LoadFeatureList : physics::FeatureList<
    physics::GetEngineInfo,
    physics::GetWorldFromEngine,
    physics::GetModelFromWorld,
    physics::sdf::ConstructSdfModel,
    physics::sdf::ConstructSdfWorld,
    physics::RemoveEntities
> { };

struct TestFeatureList : physics::FeatureList<
    physics::mujoco::EntityManagementFeatureList,
    physics::mujoco::SDFFeatureList,
    physics::mujoco::SimulationFeatureList
> { };

using World = physics::World3d<TestFeatureList>;
using WorldPtr = physics::World3dPtr<TestFeatureList>;
using LoadWorldPtr = physics::World3dPtr<LoadFeatureList>;
using ModelPtr = physics::Model3dPtr<TestFeatureList>;
using ShapePtr = physics::Shape3dPtr<TestFeatureList>;

namespace
{

/////////////////////////////////////////////////
plugin::Loader &TestLoader()
{
  static plugin::Loader loader;
  static const auto plugins = loader.LoadLib(mujoco_plugin_LIB);
  static_cast<void>(plugins);
  return loader;
}

/////////////////////////////////////////////////
auto LoadEngine()
{
  plugin::PluginPtr mujoco =
      TestLoader().Instantiate("gz::physics::mujoco::Plugin");
  EXPECT_TRUE(static_cast<bool>(mujoco));

  auto engine = physics::RequestEngine3d<LoadFeatureList>::From(mujoco);
  return engine;
}

/////////////////////////////////////////////////
LoadWorldPtr LoadWorldWhole(const std::string &_world)
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

  return world;
}

/////////////////////////////////////////////////
void CompileWorld(const WorldPtr &_world)
{
  physics::ForwardStep::Input input;
  physics::ForwardStep::State state;
  physics::ForwardStep::Output output;
  _world->Step(output, state, input);
}

}  // namespace

class CollisionBitmask_TEST : public ::testing::Test
{
  protected: WorldPtr LoadWorld(const std::string &_world)
  {
    return physics::RequestFeatures<TestFeatureList>::From(
        LoadWorldWhole(_world));
  }

  protected: ShapePtr ShapeByModelName(
      const WorldPtr &_world, const std::string &_modelName)
  {
    ModelPtr model = _world->GetModel(_modelName);
    EXPECT_NE(nullptr, model);
    if (nullptr == model)
      return nullptr;

    auto link = model->GetLink(0);
    EXPECT_NE(nullptr, link);
    if (nullptr == link)
      return nullptr;

    ShapePtr shape = link->GetShape(0);
    EXPECT_NE(nullptr, shape);
    return shape;
  }

  protected: std::shared_ptr<physics::mujoco::ShapeInfo> ShapeInfoFromShape(
      const ShapePtr &_shape) const
  {
    return std::static_pointer_cast<physics::mujoco::ShapeInfo>(
        _shape->EntityReference());
  }

  protected: std::shared_ptr<physics::mujoco::WorldInfo> WorldInfoFromWorld(
      const WorldPtr &_world) const
  {
    return std::static_pointer_cast<physics::mujoco::WorldInfo>(
        _world->EntityReference());
  }

  protected: int GeomId(const WorldPtr &_world, const ShapePtr &_shape) const
  {
    auto worldInfo = this->WorldInfoFromWorld(_world);
    auto shapeInfo = this->ShapeInfoFromShape(_shape);
    const auto it = std::find(
        worldInfo->geomIdToShapeInfo.begin(),
        worldInfo->geomIdToShapeInfo.end(),
        shapeInfo);
    if (it == worldInfo->geomIdToShapeInfo.end())
      return -1;

    return static_cast<int>(
        std::distance(worldInfo->geomIdToShapeInfo.begin(), it));
  }
};

class CollisionBitmaskBoundary_TEST :
    public CollisionBitmask_TEST,
    public ::testing::WithParamInterface<uint16_t>
{
};

/////////////////////////////////////////////////
TEST_P(CollisionBitmaskBoundary_TEST, CollisionMaskGetAfterSetRoundtrip)
{
  const uint16_t mask = GetParam();
  auto world = this->LoadWorld(common_test::worlds::kShapesBitmaskWorld);
  ASSERT_NE(nullptr, world);

  auto shape = this->ShapeByModelName(world, "box_colliding");
  ASSERT_NE(nullptr, shape);

  auto shapeInfo = this->ShapeInfoFromShape(shape);
  ASSERT_NE(nullptr, shapeInfo);
  ASSERT_NE(nullptr, shapeInfo->geom);
  EXPECT_FALSE(shapeInfo->categoryMask.has_value());

  shape->SetCollisionFilterMask(mask);

  EXPECT_EQ(mask, shape->GetCollisionFilterMask());
  EXPECT_EQ(mask, shape->GetCategoryFilterMask());
  EXPECT_EQ(static_cast<int>(mask), shapeInfo->geom->conaffinity);
  EXPECT_EQ(static_cast<int>(mask), shapeInfo->geom->contype);
  EXPECT_FALSE(shapeInfo->categoryMask.has_value());
}

/////////////////////////////////////////////////
TEST_P(CollisionBitmaskBoundary_TEST, CategoryMaskGetAfterSetRoundtrip)
{
  const uint16_t mask = GetParam();
  auto world = this->LoadWorld(common_test::worlds::kShapesBitmaskWorld);
  ASSERT_NE(nullptr, world);

  auto shape = this->ShapeByModelName(world, "box_colliding");
  ASSERT_NE(nullptr, shape);

  auto shapeInfo = this->ShapeInfoFromShape(shape);
  ASSERT_NE(nullptr, shapeInfo);
  ASSERT_NE(nullptr, shapeInfo->geom);
  const auto initialCollisionMask = shape->GetCollisionFilterMask();

  shape->SetCategoryFilterMask(mask);

  EXPECT_EQ(mask, shape->GetCategoryFilterMask());
  EXPECT_EQ(initialCollisionMask, shape->GetCollisionFilterMask());
  ASSERT_TRUE(shapeInfo->categoryMask.has_value());
  EXPECT_EQ(mask, shapeInfo->categoryMask.value());
  EXPECT_EQ(static_cast<int>(mask), shapeInfo->geom->contype);
  EXPECT_EQ(static_cast<int>(initialCollisionMask), shapeInfo->geom->conaffinity);
}

/////////////////////////////////////////////////
TEST_F(CollisionBitmask_TEST, CollisionMaskOnlyMirrorsCategoryWhenUnset)
{
  auto world = this->LoadWorld(common_test::worlds::kShapesBitmaskWorld);
  ASSERT_NE(nullptr, world);

  auto shape = this->ShapeByModelName(world, "box_colliding");
  ASSERT_NE(nullptr, shape);

  auto shapeInfo = this->ShapeInfoFromShape(shape);
  ASSERT_NE(nullptr, shapeInfo);
  ASSERT_NE(nullptr, shapeInfo->geom);
  EXPECT_FALSE(shapeInfo->categoryMask.has_value());

  shape->SetCollisionFilterMask(0xFF00);
  EXPECT_EQ(0xFF00, shape->GetCollisionFilterMask());
  EXPECT_EQ(0xFF00, shape->GetCategoryFilterMask());
  EXPECT_FALSE(shapeInfo->categoryMask.has_value());

  shape->SetCategoryFilterMask(0x0001);
  ASSERT_TRUE(shapeInfo->categoryMask.has_value());
  EXPECT_EQ(0x0001, shapeInfo->categoryMask.value());
  EXPECT_EQ(0x0001, shape->GetCategoryFilterMask());
  EXPECT_EQ(0xFF00, shape->GetCollisionFilterMask());

  shape->SetCollisionFilterMask(0x8000);
  EXPECT_EQ(0x8000, shape->GetCollisionFilterMask());
  EXPECT_EQ(0x0001, shape->GetCategoryFilterMask());
  EXPECT_EQ(0x0001, shapeInfo->categoryMask.value());
  EXPECT_EQ(0x8000, static_cast<uint16_t>(shapeInfo->geom->conaffinity));
  EXPECT_EQ(0x0001, static_cast<uint16_t>(shapeInfo->geom->contype));
}

/////////////////////////////////////////////////
TEST_F(CollisionBitmask_TEST, RemoveCategoryMaskRestoresDefaultTracking)
{
  auto world = this->LoadWorld(common_test::worlds::kShapesBitmaskWorld);
  ASSERT_NE(nullptr, world);

  auto shape = this->ShapeByModelName(world, "box_colliding");
  ASSERT_NE(nullptr, shape);

  auto shapeInfo = this->ShapeInfoFromShape(shape);
  ASSERT_NE(nullptr, shapeInfo);

  shape->SetCollisionFilterMask(0x00FF);
  shape->SetCategoryFilterMask(0x1234);
  ASSERT_TRUE(shapeInfo->categoryMask.has_value());

  shape->RemoveCategoryFilterMask();

  EXPECT_FALSE(shapeInfo->categoryMask.has_value());
  EXPECT_EQ(0x00FF, shape->GetCollisionFilterMask());
  EXPECT_EQ(0x00FF, shape->GetCategoryFilterMask());

  shape->SetCollisionFilterMask(0xF000);
  EXPECT_FALSE(shapeInfo->categoryMask.has_value());
  EXPECT_EQ(0xF000, shape->GetCollisionFilterMask());
  EXPECT_EQ(0xF000, shape->GetCategoryFilterMask());
}

/////////////////////////////////////////////////
TEST_F(CollisionBitmask_TEST, SdfCollideBitmaskRoundtripThroughGetters)
{
  auto world = this->LoadWorld(common_test::worlds::kShapesBitmaskWorld);
  ASSERT_NE(nullptr, world);

  const auto baseShape = this->ShapeByModelName(world, "box_base");
  const auto filteredShape = this->ShapeByModelName(world, "box_filtered");
  const auto collidingShape = this->ShapeByModelName(world, "box_colliding");

  ASSERT_NE(nullptr, baseShape);
  ASSERT_NE(nullptr, filteredShape);
  ASSERT_NE(nullptr, collidingShape);

  EXPECT_EQ(0x01, baseShape->GetCollisionFilterMask());
  EXPECT_EQ(0x01, baseShape->GetCategoryFilterMask());
  EXPECT_EQ(0x02, filteredShape->GetCollisionFilterMask());
  EXPECT_EQ(0x02, filteredShape->GetCategoryFilterMask());
  EXPECT_EQ(0x03, collidingShape->GetCollisionFilterMask());
  EXPECT_EQ(0x03, collidingShape->GetCategoryFilterMask());
}

/////////////////////////////////////////////////
TEST_F(CollisionBitmask_TEST, SdfCategoryBitmaskRoundtripThroughGetters)
{
  auto world = this->LoadWorld(common_test::worlds::kShapesCategoryBitmaskWorld);
  ASSERT_NE(nullptr, world);

  const auto categoryABox0Shape = this->ShapeByModelName(world, "category_a_box_0");
  const auto categoryBBox0Shape = this->ShapeByModelName(world, "category_b_box_0");
  const auto categoryCBox0Shape = this->ShapeByModelName(world, "category_c_box_0");

  ASSERT_NE(nullptr, categoryABox0Shape);
  ASSERT_NE(nullptr, categoryBBox0Shape);
  ASSERT_NE(nullptr, categoryCBox0Shape);

  EXPECT_EQ(1, categoryABox0Shape->GetCategoryFilterMask());
  EXPECT_EQ(6, categoryABox0Shape->GetCollisionFilterMask());
  EXPECT_EQ(2, categoryBBox0Shape->GetCategoryFilterMask());
  EXPECT_EQ(5, categoryBBox0Shape->GetCollisionFilterMask());
  EXPECT_EQ(4, categoryCBox0Shape->GetCategoryFilterMask());
  EXPECT_EQ(3, categoryCBox0Shape->GetCollisionFilterMask());
}

/////////////////////////////////////////////////
TEST_F(CollisionBitmask_TEST, SettersUpdateLiveModelImmediately)
{
  auto world = this->LoadWorld(common_test::worlds::kShapesBitmaskWorld);
  ASSERT_NE(nullptr, world);
  CompileWorld(world);

  auto worldInfo = this->WorldInfoFromWorld(world);
  ASSERT_NE(nullptr, worldInfo);
  ASSERT_NE(nullptr, worldInfo->mjModelObj);
  EXPECT_FALSE(worldInfo->specDirty);

  auto shape = this->ShapeByModelName(world, "box_colliding");
  ASSERT_NE(nullptr, shape);

  auto shapeInfo = this->ShapeInfoFromShape(shape);
  ASSERT_NE(nullptr, shapeInfo);
  ASSERT_NE(nullptr, shapeInfo->geom);

  const int geomId = this->GeomId(world, shape);
  ASSERT_GE(geomId, 0);
  ASSERT_LT(geomId, worldInfo->mjModelObj->ngeom);

  shape->SetCollisionFilterMask(0x00F0);

  EXPECT_TRUE(worldInfo->specDirty);
  EXPECT_EQ(0x00F0,
      static_cast<uint16_t>(worldInfo->mjModelObj->geom_conaffinity[geomId]));
  EXPECT_EQ(0x00F0,
      static_cast<uint16_t>(worldInfo->mjModelObj->geom_contype[geomId]));
  EXPECT_EQ(0x00F0, static_cast<uint16_t>(shapeInfo->geom->conaffinity));
  EXPECT_EQ(0x00F0, static_cast<uint16_t>(shapeInfo->geom->contype));

  shape->SetCategoryFilterMask(0x0003);

  EXPECT_EQ(0x00F0,
      static_cast<uint16_t>(worldInfo->mjModelObj->geom_conaffinity[geomId]));
  EXPECT_EQ(0x0003,
      static_cast<uint16_t>(worldInfo->mjModelObj->geom_contype[geomId]));
  EXPECT_EQ(0x00F0, static_cast<uint16_t>(shapeInfo->geom->conaffinity));
  EXPECT_EQ(0x0003, static_cast<uint16_t>(shapeInfo->geom->contype));
  ASSERT_TRUE(shapeInfo->categoryMask.has_value());
  EXPECT_EQ(0x0003, shapeInfo->categoryMask.value());
}

INSTANTIATE_TEST_SUITE_P(
    BoundaryMasks,
    CollisionBitmaskBoundary_TEST,
    ::testing::Values(
        static_cast<uint16_t>(0x0000),
        static_cast<uint16_t>(0x8000),
        static_cast<uint16_t>(0xFF00),
        static_cast<uint16_t>(0xFFFF)));
