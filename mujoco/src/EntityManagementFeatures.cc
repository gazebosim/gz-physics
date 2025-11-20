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

#include "EntityManagementFeatures.hh"

#include <gz/physics/Entity.hh>
#include <mujoco/mujoco.h>

#include <string>


namespace gz {
namespace physics {
namespace mujoco {

/////////////////////////////////////////////////
std::size_t EntityManagementFeatures::GetWorldCount(
    const Identity &/*_engineID*/) const
{
  return this->worlds.size();
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetWorld(
    const Identity &, std::size_t _worldIndex) const
{
  const auto &worldInfo = this->worlds[_worldIndex];
  auto worldID = static_cast<std::size_t>(mjs_getId(worldInfo->body->element));
  return this->GenerateIdentity(worldID, worldInfo);
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetWorld(
    const Identity &, const std::string &_worldName) const
{
  // TODO(azeey) Get world by name
  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
const std::string &EntityManagementFeatures::GetWorldName(
    const Identity &_worldID) const
{
  return this->ReferenceInterface<WorldInfo>(_worldID)->name;
}

/////////////////////////////////////////////////
std::size_t EntityManagementFeatures::GetWorldIndex(
    const Identity &_worldID) const
{
  // TODO(azeey) Implement GetWorldIndex
  return 0;
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetEngineOfWorld(
    const Identity &/*_worldID*/) const
{
  return this->GenerateIdentity(0);
}
Identity EntityManagementFeatures::ConstructEmptyWorld(
    const Identity &/*_engineID*/, const std::string &_name)
{
  auto worldInfo = std::make_shared<WorldInfo>();
  this->worlds.push_back(worldInfo);

  mjSpec *spec = mj_makeSpec();
  worldInfo->mjSpecObj = spec;
  worldInfo->mjModelObj = mj_compile(spec, nullptr);
  worldInfo->mjDataObj = mj_makeData(worldInfo->mjModelObj);
  worldInfo->body = mjs_findBody(spec, "world");
  // We record the name of the world, but we don't change the name in the
  // worldbody so that it is easy to find it with mjs_findBody(s, "world") elsewhere.
  worldInfo->name = _name;
  auto worldID = static_cast<std::size_t>(mjs_getId(worldInfo->body->element));
  return this->GenerateIdentity(worldID, worldInfo);
}
}
}
}
