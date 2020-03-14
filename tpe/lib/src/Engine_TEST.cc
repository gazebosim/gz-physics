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
#include "ignition/physics/tpe/Engine.hh"

using namespace ignition;
using namespace physics;
using namespace tpe;

/////////////////////////////////////////////////
TEST(Engine, World)
{
  Engine engine;
  EXPECT_EQ(0u, engine.GetWorldCount());

  // add a world
  Entity &world = engine.AddWorld();
  EXPECT_EQ(1u, engine.GetWorldCount());

  uint64_t worldId = world.GetId();
  Entity ent = engine.GetWorldById(worldId);
  EXPECT_EQ(worldId, ent.GetId());

  world.SetName("world");
  EXPECT_EQ("world", world.GetName());

  // test casting to link
  World *worldPtr = static_cast<World *>(&world);
  EXPECT_NE(nullptr, worldPtr);
  EXPECT_EQ(world.GetId(), worldPtr->GetId());

  // add another child
  Entity &world2 = engine.AddWorld();
  EXPECT_EQ(2u, engine.GetWorldCount());

  World *world2Ptr = static_cast<World *>(&world2);
  EXPECT_NE(nullptr, world2Ptr);
  EXPECT_EQ(world2.GetId(), world2Ptr->GetId());

  // test remove child by id
  engine.RemoveWorldById(worldId);
  EXPECT_EQ(1u, engine.GetWorldCount());

  Entity nullWorld = engine.GetWorldById(worldId);
  EXPECT_EQ(Entity::kNullEntity.GetId(), nullWorld.GetId());
}
