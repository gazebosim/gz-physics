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

#include <ignition/physics/Implements.hh>

#include <ignition/physics/sdf/ConstructCollision.hh>
#include <ignition/physics/sdf/ConstructJoint.hh>
#include <ignition/physics/sdf/ConstructLink.hh>
#include <ignition/physics/sdf/ConstructModel.hh>
#include <ignition/physics/sdf/ConstructVisual.hh>
#include <ignition/physics/sdf/ConstructWorld.hh>

#include "dart/dynamics/BoxShape.hpp"
#include "dart/dynamics/FreeJoint.hpp"
#include "dart/dynamics/Skeleton.hpp"
#include "dart/simulation/World.hpp"

#include "Base.hh"
#include "EntityManagementFeatures.hh"
#include "SDFFeatures.hh"

using namespace ignition::physics;



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

    base.AddJoint(pair.first);
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
  for (auto item : modelIDs)
  {
    modelNames.push_back(item.first);
  }

  // Remove all of the models while checking sizes and indices;
  std::size_t curSize = 4;
  for (auto name : modelNames) {
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

int main(int argc, char *argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
