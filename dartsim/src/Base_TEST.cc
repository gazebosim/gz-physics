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

#include <gz/physics/Implements.hh>

#include <gz/physics/sdf/ConstructCollision.hh>
#include <gz/physics/sdf/ConstructJoint.hh>
#include <gz/physics/sdf/ConstructLink.hh>
#include <gz/physics/sdf/ConstructModel.hh>
#include <gz/physics/sdf/ConstructVisual.hh>
#include <gz/physics/sdf/ConstructWorld.hh>

#include <sdf/Root.hh>
#include <sdf/Model.hh>

#include "dart/dynamics/BoxShape.hpp"
#include "dart/dynamics/FreeJoint.hpp"
#include "dart/dynamics/Skeleton.hpp"
#include "dart/simulation/World.hpp"

#include "Base.hh"
#include "EntityManagementFeatures.hh"
#include "SDFFeatures.hh"

using namespace gz::physics;



TEST(BaseClass, RemoveModel)
{
  dartsim::Base base;
  base.InitiateEngine(0);

  dart::simulation::WorldPtr world = dart::simulation::World::create("default");

  auto worldID = base.AddWorld(world, world->getName());
  EXPECT_TRUE(base.worlds.HasEntity(worldID));
  EXPECT_EQ(worldID, base.worlds.IdentityOf(world->getName()));

  std::map<std::string, std::size_t> modelIDs;
  auto addDummyModel = [&](int ind)
  {
    std::string name = std::string("skel") + std::to_string(ind);
    auto skel = dart::dynamics::Skeleton::create(name);
    auto frame = dart::dynamics::SimpleFrame::createShared(
        dart::dynamics::Frame::World(), name + "_frame");
    auto boxShape = std::make_shared<dart::dynamics::BoxShape>(
        Eigen::Vector3d::Constant(1.0));

    auto pair = skel->createJointAndBodyNodePair<dart::dynamics::FreeJoint>();
    auto sn = pair.second->createShapeNodeWith<dart::dynamics::CollisionAspect>(
      boxShape);

    auto res = base.AddModel({skel, name, frame, ""}, worldID);
    ASSERT_TRUE(base.models.HasEntity(std::get<0>(res)));
    const auto &modelInfo = base.models.at(std::get<0>(res));
    EXPECT_EQ(skel, modelInfo->model);

    const std::string fullName = ::sdf::JoinName(
        world->getName(),
        ::sdf::JoinName(skel->getName(), pair.second->getName()));
    auto linkID = base.AddLink(pair.second, fullName, std::get<0>(res));
    ASSERT_TRUE(base.links.HasEntity(linkID));
    const auto &linkInfo = base.links.at(linkID);
    EXPECT_EQ(pair.second->getName(), linkInfo->name);
    EXPECT_EQ(pair.second, linkInfo->link);

    auto modelID = base.models.IdentityOf(modelInfo->model);
    const std::string fullJointName = ::sdf::JoinName(
        world->getName(),
        ::sdf::JoinName(modelInfo->model->getName(), pair.first->getName()));

    base.AddJoint(pair.first, fullJointName, modelID);
    base.AddShape({sn, name + "_shape"});

    modelIDs[name] = std::get<0>(res);
  };

  for (int i = 0; i < 5; ++i)
  {
    addDummyModel(i);
  }
  // Result is skel0, skel1, skel2, skel3, skel4

  EXPECT_EQ(5u, base.models.size());
  EXPECT_EQ(5u, base.links.size());
  EXPECT_EQ(5u, base.linksByName.size());
  EXPECT_EQ(5u, base.joints.size());
  EXPECT_EQ(5u, base.shapes.size());

  std::size_t testModelID = modelIDs["skel2"];
  EXPECT_EQ(2u, base.models.idToIndexInContainer[testModelID]);

  // Remove skel2
  base.RemoveModelImpl(worldID, testModelID);
  modelIDs.erase("skel2");

  // Check that other resouces (links, shapes, etc) are also removed
  EXPECT_EQ(4u, base.models.size());
  EXPECT_EQ(4u, base.links.size());
  EXPECT_EQ(4u, base.linksByName.size());
  EXPECT_EQ(4u, base.joints.size());
  EXPECT_EQ(4u, base.shapes.size());

  // Check that the index of each model matches up with its index in the world
  auto checkModelIndices = [&]
  {
    for (const auto &[name, modelID] : modelIDs)
    {
      auto modelIndex = base.models.idToIndexInContainer[modelID];
      EXPECT_EQ(name, world->getSkeleton(modelIndex)->getName());
    }
  };

  // Check model indices after removing skel2
  checkModelIndices();

  // Collect the model names in a vector so we can remove them from the modelIDs
  // map in a for loop.
  std::vector<std::string> modelNames;
  modelNames.reserve(modelIDs.size());
  for (const auto& item : modelIDs)
  {
    modelNames.push_back(item.first);
  }

  // Remove all of the models while checking sizes and indices;
  std::size_t curSize = 4;
  for (const auto& name : modelNames) {
    auto modelID = modelIDs[name];
    base.RemoveModelImpl(worldID, modelID);
    modelIDs.erase(name);
    --curSize;
    EXPECT_EQ(curSize, base.models.size());
    EXPECT_EQ(curSize, base.links.size());
    EXPECT_EQ(curSize, base.linksByName.size());
    EXPECT_EQ(curSize, base.joints.size());
    EXPECT_EQ(curSize, base.shapes.size());
    checkModelIndices();
  }
  EXPECT_EQ(0u, curSize);
}


TEST(BaseClass, SdfConstructionBookkeeping)
{
  dartsim::SDFFeatures sdfFeatures;
  auto engineId = sdfFeatures.InitiateEngine(0);
  auto worldID = sdfFeatures.ConstructEmptyWorld(engineId, "default");

  ::sdf::Root root;

  auto errors = root.Load(GZ_PHYSICS_RESOURCE_DIR "/rrbot.xml");
  ASSERT_TRUE(errors.empty());
  const ::sdf::Model *sdfModel = root.Model();
  ASSERT_NE(nullptr, sdfModel);

  auto modelID = sdfFeatures.ConstructSdfModel(worldID, *sdfModel);
  EXPECT_EQ(1u, sdfFeatures.models.size());
  EXPECT_EQ(sdfModel->LinkCount(), sdfFeatures.links.size());
  EXPECT_EQ(sdfModel->LinkCount(), sdfFeatures.linksByName.size());
  EXPECT_EQ(sdfModel->JointCount(), sdfFeatures.joints.size());
  EXPECT_EQ(2u, sdfFeatures.shapes.size());

  EXPECT_EQ(sdfModel->LinkCount(), sdfFeatures.GetLinkCount(modelID));
  EXPECT_EQ(sdfModel->JointCount(), sdfFeatures.GetJointCount(modelID));

  for (uint64_t i = 0; i < sdfModel->LinkCount(); ++i)
  {
    EXPECT_EQ(sdfModel->LinkByIndex(i)->CollisionCount(),
              sdfFeatures.GetShapeCount(sdfFeatures.GetLink(modelID, i)));
  }

  for (uint64_t i =0; i < sdfModel->JointCount(); ++i)
  {
    auto jointID = sdfFeatures.GetJoint(modelID, i);
    ASSERT_TRUE(jointID);
    ASSERT_EQ(1u, sdfFeatures.joints.idToIndexInContainer.count(jointID));
    EXPECT_EQ(sdfFeatures.joints.idToIndexInContainer[jointID], i);
    EXPECT_EQ(sdfFeatures.joints.idToContainerID[jointID], modelID.id);

    ASSERT_GE(sdfFeatures.joints.indexInContainerToID[modelID].size(), i);
    EXPECT_EQ(sdfFeatures.joints.indexInContainerToID[modelID][i], jointID.id);
  }
}

TEST(BaseClass, AddNestedModel)
{
  dartsim::Base base;
  base.InitiateEngine(0);

  dart::simulation::WorldPtr world = dart::simulation::World::create("default");

  auto worldID = base.AddWorld(world, world->getName());
  EXPECT_TRUE(base.worlds.HasEntity(worldID));
  EXPECT_EQ(worldID, base.worlds.IdentityOf(world->getName()));
  auto createSkel = [](const std::string &_skelName)
  {
    auto skel = dart::dynamics::Skeleton::create(_skelName);
    auto frame = dart::dynamics::SimpleFrame::createShared(
        dart::dynamics::Frame::World(), _skelName + "_frame");
    return dartsim::ModelInfo{skel, _skelName, frame, ""};
  };

  const auto &[parentModelID, parentModelInfo] =
      base.AddModel(createSkel("parent_model"), worldID);
  EXPECT_EQ(0u, parentModelInfo.nestedModels.size());

  const auto &[nestedModel1ID, nestedModel1Info] = base.AddNestedModel(
      createSkel("parent_model::nested_model1"), parentModelID, worldID);
  ASSERT_TRUE(base.models.HasEntity(nestedModel1ID));
  EXPECT_EQ(nestedModel1Info.model, base.models.at(nestedModel1ID)->model);
  ASSERT_EQ(1u, parentModelInfo.nestedModels.size());
  EXPECT_EQ(nestedModel1ID, parentModelInfo.nestedModels[0]);

  const auto &[nestedModel2ID, nestedModel2Info] = base.AddNestedModel(
      createSkel("parent_model::nested_model2"), parentModelID, worldID);
  ASSERT_TRUE(base.models.HasEntity(nestedModel2ID));
  EXPECT_EQ(nestedModel2Info.model, base.models.at(nestedModel2ID)->model);
  ASSERT_EQ(2u, parentModelInfo.nestedModels.size());
  EXPECT_EQ(nestedModel2ID, parentModelInfo.nestedModels[1]);
}
