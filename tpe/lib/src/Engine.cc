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

#include <string>

#include "World.hh"
#include "Engine.hh"

using namespace ignition;
using namespace physics;
using namespace tpe;
using namespace lib;

/////////////////////////////////////////////////
Engine::Engine()
{
}

/////////////////////////////////////////////////
Entity &Engine::AddWorld()
{
  World world;
  uint64_t worldId = world.GetId();
  const auto [it, success] = this->worlds.insert({worldId, world});
  return it->second;
}

/////////////////////////////////////////////////
uint64_t Engine::GetWorldCount() const
{
  return this->worlds.size();
}

/////////////////////////////////////////////////
std::map<uint64_t, Entity> Engine::GetWorlds()
{
  return this->worlds;
}

/////////////////////////////////////////////////
Entity &Engine::GetWorldById(uint64_t _worldId)
{
  auto it = this->worlds.find(_worldId);
  if (it != this->worlds.end())
  {
    return it->second;
  }
  return Entity::kNullEntity;
}

/////////////////////////////////////////////////
bool Engine::RemoveWorldById(uint64_t _worldId)
{
  auto it = this->worlds.find(_worldId);
  if (it != this->worlds.end())
  {
    this->worlds.erase(it);
    return true;
  }
  return false;
}
