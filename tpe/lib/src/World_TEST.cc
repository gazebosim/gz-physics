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

#include "ignition/physics/tpe/World.hh"
#include "ignition/physics/tpe/Model.hh"

using namespace ignition;
using namespace physics;
using namespace tpe;

/////////////////////////////////////////////////
TEST(World, BasicAPI)
{
  World world;
  world.SetId(1234u);
  EXPECT_EQ(1234u, world.GetId());

  world.SetName("world_1");
  EXPECT_EQ("world_1", world.GetName());

  world.SetTime(1.0);
  EXPECT_EQ(1.0, world.GetTime());

  world.SetTimeStep(0.1);
  EXPECT_EQ(0.1, world.GetTimeStep());

  world.Step();
  EXPECT_EQ(1.1, world.GetTime());

  World world2;
  EXPECT_NE(world.GetId(), world2.GetId());
}

/////////////////////////////////////////////////
TEST(World, Model)
{
  World world;
  EXPECT_EQ(0u, world.GetChildCount());

  // add a child
  Entity &modelEnt = world.AddModel();
  modelEnt.SetName("model_1");
  modelEnt.SetPose(math::Pose3d(2, 3, 4, 0, 0, 1));
  EXPECT_EQ(1u, world.GetChildCount());

  uint64_t modelId = modelEnt.GetId();
  Entity ent = world.GetChildById(modelId);
  EXPECT_EQ(modelId, ent.GetId());
  EXPECT_EQ("model_1", ent.GetName());
  EXPECT_EQ(math::Pose3d(2, 3, 4, 0, 0, 1), ent.GetPose());

  // test casting to model
  Model *model = static_cast<Model *>(&modelEnt);
  EXPECT_NE(nullptr, model);
  EXPECT_EQ(modelEnt.GetId(), model->GetId());

  // add another child
  Entity &modelEnt2 = world.AddModel();
  EXPECT_EQ(2u, world.GetChildCount());

  Model *model2 = static_cast<Model *>(&modelEnt2);
  EXPECT_NE(nullptr, model2);
  EXPECT_EQ(modelEnt2.GetId(), model2->GetId());

  // test remove child by id
  world.RemoveChildById(modelId);
  EXPECT_EQ(1u, world.GetChildCount());

  Entity nullEnt = world.GetChildById(modelId);
  EXPECT_EQ(Entity::kNullEntity.GetId(), nullEnt.GetId());
}
