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
#include <map>

#include "World.hh"
#include "Engine.hh"

using namespace gz;
using namespace physics;
using namespace tpelib;

/////////////////////////////////////////////////
Engine::Engine()
{
}

/////////////////////////////////////////////////
Entity &Engine::AddWorld()
{
  auto world = std::make_shared<World>();
  const auto[it, success] =
    this->worlds.insert({world->GetId(), world});
  return *it->second;
}

/////////////////////////////////////////////////
std::size_t Engine::GetWorldCount() const
{
  return this->worlds.size();
}

/////////////////////////////////////////////////
std::map<std::size_t, std::shared_ptr<Entity>> Engine::GetWorlds() const
{
  return this->worlds;
}

/////////////////////////////////////////////////
Entity &Engine::GetWorldById(std::size_t _worldId)
{
  auto it = this->worlds.find(_worldId);
  if (it != this->worlds.end())
  {
    return *it->second;
  }
  return Entity::kNullEntity;
}

/////////////////////////////////////////////////
bool Engine::RemoveWorldById(std::size_t _worldId)
{
  auto it = this->worlds.find(_worldId);
  if (it != this->worlds.end())
  {
    this->worlds.erase(it);
    return true;
  }
  return false;
}
