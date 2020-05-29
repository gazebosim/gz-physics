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

#include <memory>

#include <ignition/physics/Implements.hh>

#include <ignition/math/Vector3.hh>

#include <ignition/physics/sdf/ConstructCollision.hh>
#include <ignition/physics/sdf/ConstructLink.hh>
#include <ignition/physics/sdf/ConstructModel.hh>
#include <ignition/physics/sdf/ConstructWorld.hh>

#include "Base.hh"
#include "EntityManagementFeatures.hh"


using namespace ignition::physics;

TEST(BaseClass, AddEntities)
{
  tpeplugin::Base base;
  base.InitiateEngine(0);

  // add world
  auto world = std::make_shared<tpelib::World>();
  world->SetName("default");
  std::size_t worldId = world->GetId();

  EXPECT_EQ(0u, base.worlds.size());
  auto worldIdentity = base.AddWorld(world);
  EXPECT_EQ(1u, base.worlds.size());

  EXPECT_TRUE(base.worlds.find(worldId) != base.worlds.end());
  EXPECT_EQ(worldId, base.worlds.find(worldId)->second->world->GetId());
  EXPECT_EQ(
    world->GetName(), base.worlds.find(worldId)->second->world->GetName());

  // add models to world
  auto &modelEnt1 = world->AddModel();
  modelEnt1.SetName("box");
  auto *model1 = static_cast<tpelib::Model *>(&modelEnt1);
  std::size_t modelId1 = model1->GetId();

  EXPECT_EQ(0u, base.models.size());
  auto modelIdentity1 = base.AddModel(worldId, *model1);
  EXPECT_EQ(1u, base.models.size());

  EXPECT_TRUE(base.models.find(modelId1) != base.models.end());
  EXPECT_EQ(modelId1, base.models.find(modelId1)->second->model->GetId());
  EXPECT_EQ(
    model1->GetName(), base.models.find(modelId1)->second->model->GetName());

  auto &modelEnt2 = world->AddModel();
  modelEnt2.SetName("cylinder");
  auto *model2 = static_cast<tpelib::Model *>(&modelEnt2);
  std::size_t modelId2 = model2->GetId();

  EXPECT_EQ(1u, base.models.size());
  auto modelIdentity2 = base.AddModel(worldId, *model2);
  EXPECT_EQ(2u, base.models.size());

  EXPECT_TRUE(base.models.find(modelId2) != base.models.end());
  EXPECT_EQ(modelId2, base.models.find(modelId2)->second->model->GetId());
  EXPECT_EQ(
    model2->GetName(), base.models.find(modelId2)->second->model->GetName());

  // add first link to model1
  auto &linkEnt1 = model1->AddLink();
  linkEnt1.SetName("box_link");
  auto *link1 = static_cast<tpelib::Link *>(&linkEnt1);
  std::size_t linkId1 = link1->GetId();

  EXPECT_EQ(0u, base.links.size());
  auto linkIdentity1 = base.AddLink(modelId1, *link1);
  EXPECT_EQ(1u, base.links.size());
  EXPECT_EQ(1u, model1->GetChildCount());

  EXPECT_TRUE(base.links.find(linkId1) != base.links.end());
  EXPECT_EQ(linkId1, base.links.find(linkId1)->second->link->GetId());
  EXPECT_EQ(
    link1->GetName(), base.links.find(linkId1)->second->link->GetName());
  EXPECT_EQ(modelId1, base.childIdToParentId.find(linkId1)->second);

  // add second link to model2
  auto &linkEnt2 = model2->AddLink();
  linkEnt2.SetName("cylinder_link");
  auto *link2 = static_cast<tpelib::Link *>(&linkEnt2);
  std::size_t linkId2 = link2->GetId();

  EXPECT_EQ(1u, base.links.size());
  auto linkIdentity2 = base.AddLink(modelId2, *link2);
  EXPECT_EQ(2u, base.links.size());
  EXPECT_EQ(1u, model2->GetChildCount());

  EXPECT_TRUE(base.links.find(linkId2) != base.links.end());
  EXPECT_EQ(linkId2, base.links.find(linkId2)->second->link->GetId());
  EXPECT_EQ(
    link2->GetName(), base.links.find(linkId2)->second->link->GetName());
  EXPECT_EQ(modelId2, base.childIdToParentId.find(linkId2)->second);

  // add collision shape box to link1
  auto &boxEnt = link1->AddCollision();
  boxEnt.SetName("box_collision");
  auto *box = static_cast<tpelib::Collision *>(&boxEnt);
  std::size_t boxId = box->GetId();

  EXPECT_EQ(0u, base.collisions.size());
  auto boxIdentity = base.AddCollision(linkId1, *box);
  EXPECT_EQ(1u, base.collisions.size());
  EXPECT_EQ(1u, link1->GetChildCount());

  EXPECT_TRUE(base.collisions.find(boxId) != base.collisions.end());
  EXPECT_EQ(
    boxId, base.collisions.find(boxId)->second->collision->GetId());
  EXPECT_EQ(
    box->GetName(),
    base.collisions.find(boxId)->second->collision->GetName());
  EXPECT_EQ(linkId1, base.childIdToParentId.find(boxId)->second);

  // add collision shape cylinder to link2
  auto &cylinderEnt = link2->AddCollision();
  cylinderEnt.SetName("cylinder_collision");
  auto *cylinder = static_cast<tpelib::Collision *>(&cylinderEnt);
  std::size_t cylinderId = cylinder->GetId();

  EXPECT_EQ(1u, base.collisions.size());
  auto cylinderIdentity = base.AddCollision(linkId2, *cylinder);
  EXPECT_EQ(2u, base.collisions.size());
  EXPECT_EQ(1u, link2->GetChildCount());

  EXPECT_TRUE(base.collisions.find(cylinderId) != base.collisions.end());
  EXPECT_EQ(
    cylinderId,
    base.collisions.find(cylinderId)->second->collision->GetId());
  EXPECT_EQ(
    cylinder->GetName(),
    base.collisions.find(cylinderId)->second->collision->GetName());
  EXPECT_EQ(linkId2, base.childIdToParentId.find(cylinderId)->second);

  // check indices
  std::size_t modelInd1 = base.idToIndexInContainer(modelId1);
  EXPECT_EQ(0u, modelInd1);
  EXPECT_EQ(modelId1, base.indexInContainerToId(worldId, 0u));
  std::size_t modelInd2 = base.idToIndexInContainer(modelId2);
  EXPECT_EQ(1u, modelInd2);
  EXPECT_EQ(modelId2, base.indexInContainerToId(worldId, 1u));

  std::size_t linkInd1 = base.idToIndexInContainer(linkId1);
  EXPECT_EQ(0u, linkInd1);
  EXPECT_EQ(linkId1, base.indexInContainerToId(modelId1, 0u));
  std::size_t linkInd2 = base.idToIndexInContainer(linkId2);
  EXPECT_EQ(0u, linkInd2);
  EXPECT_EQ(linkId2, base.indexInContainerToId(modelId2, 0u));

  std::size_t boxInd = base.idToIndexInContainer(boxId);
  EXPECT_EQ(0u, boxInd);
  EXPECT_EQ(boxId, base.indexInContainerToId(linkId1, 0u));
  std::size_t cylinderInd = base.idToIndexInContainer(cylinderId);
  EXPECT_EQ(0u, cylinderInd);
  EXPECT_EQ(cylinderId, base.indexInContainerToId(linkId2, 0u));
}

int main(int argc, char *argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
