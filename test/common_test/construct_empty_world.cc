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

#include <vector>

#include <gz/common/Console.hh>
#include <gz/common/geospatial/Dem.hh>
#include <gz/common/geospatial/ImageHeightmap.hh>
#include <gz/common/MeshManager.hh>
#include <gz/math/eigen3/Conversions.hh>
#include <gz/plugin/Loader.hh>

#include "test/Resources.hh"
#include "test/TestLibLoader.hh"

#include <gz/physics/FixedJoint.hh>
#include <gz/physics/ConstructEmpty.hh>
#include <gz/physics/FindFeatures.hh>
#include <gz/physics/FrameSemantics.hh>
#include <gz/physics/GetEntities.hh>
#include <gz/physics/Joint.hh>
#include <gz/physics/PrismaticJoint.hh>
#include <gz/physics/RequestEngine.hh>
#include <gz/physics/RemoveEntities.hh>
#include <gz/physics/RevoluteJoint.hh>
#include <gz/physics/SphereShape.hh>
#include <gz/physics/heightmap/HeightmapShape.hh>
#include <gz/physics/mesh/MeshShape.hh>

#include "gz/physics/BoxShape.hh"

template <class T>
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
    pluginNames = gz::physics::FindFeatures3d<T>::From(loader);
    if (pluginNames.empty())
    {
      std::cerr << "No plugins with required features found in "
                << GetLibToTest() << std::endl;
      GTEST_SKIP();
    }
  }

  using FeaturePolicy3d = gz::physics::FeaturePolicy3d;

  public: void MakeEmptyWorld(
              const std::string &_pluginName,
              gz::physics::EnginePtr<FeaturePolicy3d, T> &_engine,
              gz::physics::WorldPtr<FeaturePolicy3d, T> &_world)
  {
    std::cout << "Testing plugin: " << _pluginName << std::endl;
    gz::plugin::PluginPtr plugin = this->loader.Instantiate(_pluginName);

    _engine = gz::physics::RequestEngine3d<T>::From(plugin);
    ASSERT_NE(nullptr, _engine);

    _world = _engine->ConstructEmptyWorld("empty world");
    ASSERT_NE(nullptr, _world);
  }

  public: std::set<std::string> pluginNames;
  public: gz::plugin::Loader loader;
};

using gz::physics::FeaturePolicy3d;

using FeaturesUpToEmptyWorld = gz::physics::FeatureList<
  gz::physics::GetEngineInfo,
  gz::physics::ConstructEmptyWorldFeature
>;

using ConstructEmptyWorldTestUpToEmptyWorldTypes =
  ::testing::Types<FeaturesUpToEmptyWorld>;
TYPED_TEST_SUITE(ConstructEmptyWorldTest, ConstructEmptyWorldTestUpToEmptyWorldTypes);

/////////////////////////////////////////////////
TYPED_TEST(ConstructEmptyWorldTest, ConstructUpToEmptyWorld)
{
  for (const std::string &name : this->pluginNames)
  {
    gz::physics::EnginePtr<FeaturePolicy3d, TypeParam> engine;
    gz::physics::WorldPtr<FeaturePolicy3d, TypeParam> world;

    this->MakeEmptyWorld(name, engine, world);
  }
}

using FeaturesUpToGetWorldFromEngine = gz::physics::FeatureList<
  FeaturesUpToEmptyWorld,
  gz::physics::GetWorldFromEngine
>;

template <class T>
class ConstructEmptyWorldTestUpToGetWorldFromEngine :
  public ConstructEmptyWorldTest<T>{};

using ConstructEmptyWorldTestUpToGetWorldFromEngineTypes =
  ::testing::Types<FeaturesUpToGetWorldFromEngine>;
TYPED_TEST_SUITE(ConstructEmptyWorldTestUpToGetWorldFromEngine,
                 ConstructEmptyWorldTestUpToGetWorldFromEngineTypes);

/////////////////////////////////////////////////
TYPED_TEST(ConstructEmptyWorldTestUpToGetWorldFromEngine,
           ConstructUpToEmptyWorld)
{
  for (const std::string &name : this->pluginNames)
  {
    gz::physics::EnginePtr<FeaturePolicy3d, TypeParam> engine;
    gz::physics::WorldPtr<FeaturePolicy3d, TypeParam> world;
    this->MakeEmptyWorld(name, engine, world);

    EXPECT_EQ("empty world", world->GetName());
    EXPECT_EQ(engine, world->GetEngine());
  }
}

using FeaturesUpToEmptyModelFeature = gz::physics::FeatureList<
  FeaturesUpToGetWorldFromEngine,
  gz::physics::ConstructEmptyModelFeature
>;

template <class T>
class ConstructEmptyWorldTestUpToEmptyModelFeature :
  public ConstructEmptyWorldTest<T>{};
using ConstructEmptyWorldTestUpToEmptyModelFeatureTypes =
  ::testing::Types<FeaturesUpToEmptyModelFeature>;
TYPED_TEST_SUITE(ConstructEmptyWorldTestUpToEmptyModelFeature,
                 ConstructEmptyWorldTestUpToEmptyModelFeatureTypes);

/////////////////////////////////////////////////
TYPED_TEST(ConstructEmptyWorldTestUpToEmptyModelFeature,
           ConstructUpToEmptyWorld)
{
  for (const std::string &name : this->pluginNames)
  {
    gz::physics::EnginePtr<FeaturePolicy3d, TypeParam> engine;
    gz::physics::WorldPtr<FeaturePolicy3d, TypeParam> world;
    this->MakeEmptyWorld(name, engine, world);

    EXPECT_EQ("empty world", world->GetName());
    EXPECT_EQ(engine, world->GetEngine());

    auto model = world->ConstructEmptyModel("empty model");
    ASSERT_NE(nullptr, model);
    EXPECT_NE(model, world->ConstructEmptyModel("dummy"));
  }
}

using FeaturesUpToGetModelFromWorld = gz::physics::FeatureList<
  FeaturesUpToEmptyModelFeature,
  gz::physics::GetModelFromWorld
>;

template <class T>
class ConstructEmptyWorldTestUpToGetModelFromWorld :
  public ConstructEmptyWorldTest<T>{};
using ConstructEmptyWorldTestUpToGetModelFromWorldTypes =
  ::testing::Types<FeaturesUpToGetModelFromWorld>;
TYPED_TEST_SUITE(ConstructEmptyWorldTestUpToGetModelFromWorld,
                 ConstructEmptyWorldTestUpToGetModelFromWorldTypes);

/////////////////////////////////////////////////
TYPED_TEST(ConstructEmptyWorldTestUpToGetModelFromWorld,
           ConstructUpToEmptyWorld)
{
  for (const std::string &name : this->pluginNames)
  {
    gz::physics::EnginePtr<FeaturePolicy3d, TypeParam> engine;
    gz::physics::WorldPtr<FeaturePolicy3d, TypeParam> world;
    this->MakeEmptyWorld(name, engine, world);

    EXPECT_EQ("empty world", world->GetName());
    EXPECT_EQ(engine, world->GetEngine());

    auto model = world->ConstructEmptyModel("empty model");
    ASSERT_NE(nullptr, model);
    EXPECT_EQ("empty model", model->GetName());
    EXPECT_EQ(world, model->GetWorld());
    EXPECT_NE(model, world->ConstructEmptyModel("dummy"));
  }
}

using FeaturesUpToEmptyNestedModelFeature = gz::physics::FeatureList<
  FeaturesUpToGetModelFromWorld,
  gz::physics::ConstructEmptyNestedModelFeature,
  gz::physics::GetNestedModelFromModel
>;

template <class T>
class ConstructEmptyWorldTestUpToEmptyNestedModelFeature :
  public ConstructEmptyWorldTest<T>{};
using ConstructEmptyWorldTestUpToEmptyNestedModelFeatureTypes =
  ::testing::Types<FeaturesUpToEmptyNestedModelFeature>;
TYPED_TEST_SUITE(ConstructEmptyWorldTestUpToEmptyNestedModelFeature,
                 ConstructEmptyWorldTestUpToEmptyNestedModelFeatureTypes);

/////////////////////////////////////////////////
TYPED_TEST(ConstructEmptyWorldTestUpToEmptyNestedModelFeature,
           ConstructUpToEmptyWorld)
{
  for (const std::string &name : this->pluginNames)
  {
    gz::physics::EnginePtr<FeaturePolicy3d, TypeParam> engine;
    gz::physics::WorldPtr<FeaturePolicy3d, TypeParam> world;
    this->MakeEmptyWorld(name, engine, world);

    auto model = world->ConstructEmptyModel("empty model");

    auto nestedModel = model->ConstructEmptyNestedModel("empty nested model");
    ASSERT_NE(nullptr, nestedModel);
    EXPECT_EQ("empty nested model", nestedModel->GetName());
    EXPECT_EQ(1u, model->GetNestedModelCount());
    EXPECT_EQ(0u, model->GetIndex());
    EXPECT_EQ(nestedModel, model->GetNestedModel(0));
    EXPECT_EQ(nestedModel, model->GetNestedModel("empty nested model"));
    EXPECT_NE(nestedModel, nestedModel->ConstructEmptyNestedModel("dummy"));
    // This should remain 1 since we're adding a nested model in `nestedModel` not
    // in `model`.
    EXPECT_EQ(1u, model->GetNestedModelCount());
    EXPECT_EQ(1u, nestedModel->GetNestedModelCount());
  }
}

using FeaturesUpToEmptyLink = gz::physics::FeatureList<
  FeaturesUpToEmptyModelFeature,
  gz::physics::GetLinkFromModel,
  gz::physics::ConstructEmptyLinkFeature
>;

template <class T>
class ConstructEmptyWorldTestUpToEmptyLink :
  public ConstructEmptyWorldTest<T>{};
using ConstructEmptyWorldTestUpToEmptyLinkTypes =
  ::testing::Types<FeaturesUpToEmptyLink>;
TYPED_TEST_SUITE(ConstructEmptyWorldTestUpToEmptyLink,
                 ConstructEmptyWorldTestUpToEmptyLinkTypes);

/////////////////////////////////////////////////
TYPED_TEST(ConstructEmptyWorldTestUpToEmptyLink, ConstructUpToEmptyWorld)
{
  for (const std::string &name : this->pluginNames)
  {
    gz::physics::EnginePtr<FeaturePolicy3d, TypeParam> engine;
    gz::physics::WorldPtr<FeaturePolicy3d, TypeParam> world;
    this->MakeEmptyWorld(name, engine, world);

    EXPECT_EQ("empty world", world->GetName());
    EXPECT_EQ(engine, world->GetEngine());

    auto model = world->ConstructEmptyModel("empty model");

    auto link = model->ConstructEmptyLink("empty link");
    ASSERT_NE(nullptr, link);
    EXPECT_EQ("empty link", link->GetName());
    EXPECT_EQ(model, link->GetModel());
    EXPECT_NE(link, model->ConstructEmptyLink("dummy"));
    EXPECT_EQ(model, link->GetModel());

    auto child = model->ConstructEmptyLink("child link");
    EXPECT_EQ(model, child->GetModel());
  }
}

using FeaturesUpToRemove = gz::physics::FeatureList<
  gz::physics::GetEngineInfo,
  gz::physics::ConstructEmptyWorldFeature,
  gz::physics::GetWorldFromEngine,
  gz::physics::ConstructEmptyModelFeature,
  gz::physics::GetModelFromWorld,
  gz::physics::GetLinkFromModel,
  gz::physics::ConstructEmptyLinkFeature,
  gz::physics::ConstructEmptyNestedModelFeature,
  gz::physics::GetNestedModelFromModel,
  gz::physics::RemoveEntities
>;

template <class T>
class ConstructEmptyWorldTestUpToRemove : public ConstructEmptyWorldTest<T>{};
using ConstructEmptyWorldTestUpToRemoveTypes =
  ::testing::Types<FeaturesUpToRemove>;
TYPED_TEST_SUITE(ConstructEmptyWorldTestUpToRemove,
                 ConstructEmptyWorldTestUpToRemoveTypes);

/////////////////////////////////////////////////
TYPED_TEST(ConstructEmptyWorldTestUpToRemove, ConstructUpToEmptyWorld)
{
  for (const std::string &name : this->pluginNames)
  {
    gz::physics::EnginePtr<FeaturePolicy3d, TypeParam> engine;
    gz::physics::WorldPtr<FeaturePolicy3d, TypeParam> world;
    this->MakeEmptyWorld(name, engine, world);

    auto model = world->ConstructEmptyModel("empty model");
    ASSERT_NE(nullptr, model);
    auto modelAlias = world->GetModel(0);

    model->Remove();
    EXPECT_TRUE(model->Removed());
    EXPECT_TRUE(modelAlias->Removed());
    EXPECT_EQ(nullptr, world->GetModel(0));
    EXPECT_EQ(nullptr, world->GetModel("empty model"));
    EXPECT_EQ(0ul, world->GetModelCount());
    EXPECT_EQ("empty model", model->GetName());

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

using FeaturesUpToEmptyNestedModelFeature2 = gz::physics::FeatureList<
  gz::physics::GetEngineInfo,
  gz::physics::ConstructEmptyWorldFeature,
  gz::physics::GetWorldFromEngine,
  gz::physics::ConstructEmptyModelFeature,
  gz::physics::GetModelFromWorld,
  gz::physics::ConstructEmptyNestedModelFeature,
  gz::physics::GetNestedModelFromModel,
  gz::physics::RemoveEntities
>;

template <class T>
class ConstructEmptyWorldTestUpToEmptyNestedModelFeature2 :
  public ConstructEmptyWorldTest<T>{};
using ConstructEmptyWorldTestUpToEmptyNestedModelFeature2Types =
  ::testing::Types<FeaturesUpToEmptyNestedModelFeature2>;
TYPED_TEST_SUITE(ConstructEmptyWorldTestUpToEmptyNestedModelFeature2,
                 FeaturesUpToEmptyNestedModelFeature2);

/////////////////////////////////////////////////
TYPED_TEST(ConstructEmptyWorldTestUpToEmptyNestedModelFeature2,
           ModelByIndexWithNestedModels)
{
  for (const std::string &name : this->pluginNames)
  {
    gz::physics::EnginePtr<FeaturePolicy3d, TypeParam> engine;
    gz::physics::WorldPtr<FeaturePolicy3d, TypeParam> world;
    this->MakeEmptyWorld(name, engine, world);

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

using FeaturesUpToConstructJointsAndShapes = gz::physics::FeatureList<
  gz::physics::GetEngineInfo,
  gz::physics::ConstructEmptyWorldFeature,
  gz::physics::ConstructEmptyModelFeature,
  gz::physics::ConstructEmptyLinkFeature,
  gz::physics::GetEntities,
  gz::physics::AttachRevoluteJointFeature,
  gz::physics::GetRevoluteJointProperties,
  gz::physics::SetRevoluteJointProperties,
  gz::physics::AttachPrismaticJointFeature,
  gz::physics::GetPrismaticJointProperties,
  gz::physics::GetBasicJointState,
  gz::physics::SetBasicJointState,
  gz::physics::LinkFrameSemantics,
  gz::physics::ShapeFrameSemantics,
  gz::physics::GetBoxShapeProperties,
  gz::physics::AttachBoxShapeFeature,
  gz::physics::GetSphereShapeProperties,
  gz::physics::AttachSphereShapeFeature
>;

template <class T>
class ConstructEmptyWorldTestUpToConstructJointsAndShapes :
  public ConstructEmptyWorldTest<T>{};
using ConstructEmptyWorldTestUpToConstructJointsAndShapesTypes =
  ::testing::Types<FeaturesUpToConstructJointsAndShapes>;
TYPED_TEST_SUITE(ConstructEmptyWorldTestUpToConstructJointsAndShapes,
                 ConstructEmptyWorldTestUpToConstructJointsAndShapesTypes);

/////////////////////////////////////////////////
TYPED_TEST(ConstructEmptyWorldTestUpToConstructJointsAndShapes,
           ConstructJointsAndShapes)
{
  for (const std::string &name : this->pluginNames)
  {
    gz::physics::EnginePtr<FeaturePolicy3d, TypeParam> engine;
    gz::physics::WorldPtr<FeaturePolicy3d, TypeParam> world;
    this->MakeEmptyWorld(name, engine, world);

    auto model = world->ConstructEmptyModel("empty model");
    ASSERT_NE(nullptr, model);

    auto link = model->ConstructEmptyLink("empty link");
    ASSERT_NE(nullptr, link);
    EXPECT_EQ(0u, link->GetIndex());

    auto revolute = link->AttachRevoluteJoint(nullptr);
    ASSERT_NE(nullptr, revolute);
    EXPECT_NEAR(
      (Eigen::Vector3d::UnitX() - revolute->GetAxis()).norm(), 0.0, 1e-6);
    EXPECT_DOUBLE_EQ(0.0, revolute->GetPosition(0));

    revolute->SetAxis(Eigen::Vector3d::UnitZ());
    EXPECT_NEAR(
      (Eigen::Vector3d::UnitZ() - revolute->GetAxis()).norm(), 0.0, 1e-6);

    auto child = model->ConstructEmptyLink("child link");
    ASSERT_NE(nullptr, child);

    const std::string boxName = "box";
    const Eigen::Vector3d boxSize(0.1, 0.2, 0.3);
    auto box = link->AttachBoxShape(boxName, boxSize);
    ASSERT_NE(nullptr, box);
    EXPECT_EQ(boxName, box->GetName());
    EXPECT_NEAR((boxSize - box->GetSize()).norm(), 0.0, 1e-6);

    EXPECT_EQ(1u, link->GetShapeCount());
    EXPECT_EQ(box, link->GetShape(0u));

    auto prismatic = child->AttachPrismaticJoint(
      link, "prismatic", Eigen::Vector3d::UnitZ());
    ASSERT_NE(nullptr, prismatic);

    const double zPos = 2.5;
    const double zVel = 9.1;
    const double zAcc = 10.2;
    prismatic->SetPosition(0, zPos);
    prismatic->SetVelocity(0, zVel);
    prismatic->SetAcceleration(0, zAcc);

    const auto childData = child->FrameDataRelativeToWorld();
    EXPECT_DOUBLE_EQ(0.0, childData.pose.translation().x());
    EXPECT_DOUBLE_EQ(0.0, childData.pose.translation().y());
    EXPECT_DOUBLE_EQ(zPos, childData.pose.translation().z());
    EXPECT_DOUBLE_EQ(0.0, childData.linearVelocity.x());
    EXPECT_DOUBLE_EQ(0.0, childData.linearVelocity.y());
    EXPECT_DOUBLE_EQ(zVel, childData.linearVelocity.z());
    EXPECT_DOUBLE_EQ(0.0, childData.linearAcceleration.x());
    EXPECT_DOUBLE_EQ(0.0, childData.linearAcceleration.y());
    EXPECT_DOUBLE_EQ(zAcc, childData.linearAcceleration.z());

    const double yPos = 11.5;
    Eigen::Isometry3d childSpherePose = Eigen::Isometry3d::Identity();
    childSpherePose.translate(Eigen::Vector3d(0.0, yPos, 0.0));
    auto sphere = child->AttachSphereShape("child sphere", 1.0, childSpherePose);
    ASSERT_NE(nullptr, sphere);

    const auto sphereData = sphere->FrameDataRelativeToWorld();
    EXPECT_DOUBLE_EQ(0.0, sphereData.pose.translation().x());
    EXPECT_DOUBLE_EQ(yPos, sphereData.pose.translation().y());
    EXPECT_DOUBLE_EQ(zPos, sphereData.pose.translation().z());
    EXPECT_DOUBLE_EQ(0.0, sphereData.linearVelocity.x());
    EXPECT_DOUBLE_EQ(0.0, sphereData.linearVelocity.y());
    EXPECT_DOUBLE_EQ(zVel, sphereData.linearVelocity.z());
    EXPECT_DOUBLE_EQ(0.0, sphereData.linearAcceleration.x());
    EXPECT_DOUBLE_EQ(0.0, sphereData.linearAcceleration.y());
    EXPECT_DOUBLE_EQ(zAcc, sphereData.linearAcceleration.z());

    const auto relativeSphereData = sphere->FrameDataRelativeTo(*child);
    EXPECT_DOUBLE_EQ(0.0, relativeSphereData.pose.translation().x());
    EXPECT_DOUBLE_EQ(yPos, relativeSphereData.pose.translation().y());
    EXPECT_DOUBLE_EQ(0.0, relativeSphereData.pose.translation().z());
  }
}

using FeaturesUpToMeshAndHeightmaps = gz::physics::FeatureList<
  gz::physics::GetEngineInfo,
  gz::physics::ConstructEmptyWorldFeature,
  gz::physics::ConstructEmptyModelFeature,
  gz::physics::ConstructEmptyLinkFeature,
  gz::physics::GetEntities,
  gz::physics::AttachFixedJointFeature,
  gz::physics::mesh::GetMeshShapeProperties,
  gz::physics::mesh::AttachMeshShapeFeature,
  gz::physics::heightmap::GetHeightmapShapeProperties,
  gz::physics::heightmap::AttachHeightmapShapeFeature
>;

template <class T>
class ConstructEmptyWorldTestUpToMeshAndHeightmaps :
  public ConstructEmptyWorldTest<T>{};
using ConstructEmptyWorldTestUpToMeshAndHeightmapsTypes =
  ::testing::Types<FeaturesUpToMeshAndHeightmaps>;
TYPED_TEST_SUITE(ConstructEmptyWorldTestUpToMeshAndHeightmaps,
                 ConstructEmptyWorldTestUpToMeshAndHeightmapsTypes);

/////////////////////////////////////////////////
TYPED_TEST(ConstructEmptyWorldTestUpToMeshAndHeightmaps,
           ConstructMeshAndHeightmaps)
{
  for (const std::string &name : this->pluginNames)
  {
    gz::physics::EnginePtr<FeaturePolicy3d, TypeParam> engine;
    gz::physics::WorldPtr<FeaturePolicy3d, TypeParam> world;
    this->MakeEmptyWorld(name, engine, world);

    auto model = world->ConstructEmptyModel("empty model");
    ASSERT_NE(nullptr, model);

    auto child = model->ConstructEmptyLink("child link");
    ASSERT_NE(nullptr, child);

    auto meshLink = model->ConstructEmptyLink("mesh_link");
    ASSERT_NE(nullptr, meshLink);
    ASSERT_NE(nullptr, meshLink->AttachFixedJoint(child, "fixed"));

    const auto meshFilename = gz::physics::test::resources::kChassisDae;
    auto &meshManager = *gz::common::MeshManager::Instance();
    auto *mesh = meshManager.Load(meshFilename);
    ASSERT_NE(nullptr, mesh);

    auto meshShape = meshLink->AttachMeshShape("chassis", *mesh);
    ASSERT_NE(nullptr, meshShape);
    const auto originalMeshSize = mesh->Max() - mesh->Min();
    const auto meshShapeSize = meshShape->GetSize();

    for (std::size_t i = 0; i < 3; ++i)
    {
      EXPECT_NEAR(originalMeshSize[i], meshShapeSize[i], 1e-6);
    }

    const gz::math::Pose3d pose(0, 0, 0.2, 0, 0, 0);
    const gz::math::Vector3d scale(0.5, 1.0, 0.25);
    auto meshShapeScaled = meshLink->AttachMeshShape("small_chassis", *mesh,
      gz::math::eigen3::convert(pose),
      gz::math::eigen3::convert(scale));
    ASSERT_NE(nullptr, meshShapeScaled);
    const auto meshShapeScaledSize = meshShapeScaled->GetSize();

    for (std::size_t i = 0; i < 3; ++i)
    {
      EXPECT_NEAR(originalMeshSize[i] * scale[i], meshShapeScaledSize[i], 1e-6);
    }

    auto heightmapLink = model->ConstructEmptyLink("heightmap_link");
    ASSERT_NE(nullptr, heightmapLink);
    ASSERT_NE(nullptr, heightmapLink->AttachFixedJoint(child, "heightmap_joint"));

    const auto heightmapFilename =
      gz::physics::test::resources::kHeightmapBowlPng;
    gz::common::ImageHeightmap data;
    ASSERT_EQ(0, data.Load(heightmapFilename));

    const gz::math::Vector3d size(129, 129, 10);
    auto heightmapShape = heightmapLink->AttachHeightmapShape("heightmap", data,
      gz::math::eigen3::convert(pose),
      gz::math::eigen3::convert(size));
    ASSERT_NE(nullptr, heightmapShape);

    EXPECT_NEAR(size.X(), heightmapShape->GetSize()[0], 1e-6);
    EXPECT_NEAR(size.Y(), heightmapShape->GetSize()[1], 1e-6);
    EXPECT_NEAR(size.Z(), heightmapShape->GetSize()[2], 1e-6);

    auto heightmapShapeGeneric = heightmapLink->GetShape("heightmap");
    ASSERT_NE(nullptr, heightmapShapeGeneric);
    auto heightmapShapeRecast = heightmapShapeGeneric->CastToHeightmapShape();
    ASSERT_NE(nullptr, heightmapShapeRecast);
    EXPECT_NEAR(size.X(), heightmapShapeRecast->GetSize()[0], 1e-6);
    EXPECT_NEAR(size.Y(), heightmapShapeRecast->GetSize()[1], 1e-6);
    EXPECT_NEAR(size.Z(), heightmapShapeRecast->GetSize()[2], 1e-6);

    auto demLink = model->ConstructEmptyLink("dem_link");
    ASSERT_NE(nullptr, demLink);
    ASSERT_NE(nullptr, demLink->AttachFixedJoint(child, "dem_joint"));

    const auto demFilename = gz::physics::test::resources::kVolcanoTif;
    gz::common::Dem dem;
    ASSERT_EQ(0, dem.Load(demFilename));

    const gz::math::Vector3d sizeDem(
      dem.WorldWidth(), dem.WorldHeight(),
      dem.MaxElevation() - dem.MinElevation());
    auto demShape = demLink->AttachHeightmapShape("dem", dem,
      gz::math::eigen3::convert(pose),
      gz::math::eigen3::convert(sizeDem));
    ASSERT_NE(nullptr, demShape);

    EXPECT_NEAR(sizeDem.X(), demShape->GetSize()[0], 1e-3);
    EXPECT_NEAR(sizeDem.Y(), demShape->GetSize()[1], 1e-3);
    EXPECT_NEAR(sizeDem.Z(), demShape->GetSize()[2], 1e-6);

    auto demShapeGeneric = demLink->GetShape("dem");
    ASSERT_NE(nullptr, demShapeGeneric);
    auto demShapeRecast = demShapeGeneric->CastToHeightmapShape();
    ASSERT_NE(nullptr, demShapeRecast);
    EXPECT_NEAR(sizeDem.X(), demShapeRecast->GetSize()[0], 1e-3);
    EXPECT_NEAR(sizeDem.Y(), demShapeRecast->GetSize()[1], 1e-3);
    EXPECT_NEAR(sizeDem.Z(), demShapeRecast->GetSize()[2], 1e-6);
  }
}

using FeaturesUpToRemoveNestedModelCrash = gz::physics::FeatureList<
  gz::physics::GetEngineInfo,
  gz::physics::ConstructEmptyWorldFeature,
  gz::physics::ConstructEmptyModelFeature,
  gz::physics::ConstructEmptyNestedModelFeature,
  gz::physics::RemoveEntities
>;

template <class T>
class ConstructEmptyWorldTestUpToRemoveNestedModelCrash :
  public ConstructEmptyWorldTest<T>{};
using ConstructEmptyWorldTestUpToRemoveNestedModelCrashTypes =
  ::testing::Types<FeaturesUpToRemoveNestedModelCrash>;
TYPED_TEST_SUITE(ConstructEmptyWorldTestUpToRemoveNestedModelCrash,
                 ConstructEmptyWorldTestUpToRemoveNestedModelCrashTypes);

/////////////////////////////////////////////////
TYPED_TEST(ConstructEmptyWorldTestUpToRemoveNestedModelCrash,
           RemoveNestedModelCrash)
{
  for (const std::string &name : this->pluginNames)
  {
    gz::physics::EnginePtr<FeaturePolicy3d, TypeParam> engine;
    gz::physics::WorldPtr<FeaturePolicy3d, TypeParam> world;
    this->MakeEmptyWorld(name, engine, world);

    auto parentModel = world->ConstructEmptyModel("parent model");
    ASSERT_NE(nullptr, parentModel);

    using NestedModelPtr = decltype(parentModel->ConstructEmptyNestedModel(""));
    std::vector<NestedModelPtr> nestedModels;
    for (int i = 0; i < 6; ++i)
    {
      auto nestedModel =
        parentModel->ConstructEmptyNestedModel("nested_" + std::to_string(i));
      ASSERT_NE(nullptr, nestedModel);
      nestedModels.push_back(nestedModel);
    }

    EXPECT_NO_THROW({
      parentModel->Remove();
    });

    EXPECT_NO_THROW({
      for (auto &nestedModel : nestedModels)
      {
        nestedModel->Remove();
      }
    });
  }
}

int main(int argc, char *argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  if (!ConstructEmptyWorldTest<FeaturesUpToEmptyWorld>::init(
       argc, argv))
    return -1;
  return RUN_ALL_TESTS();
}
