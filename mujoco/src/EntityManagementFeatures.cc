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

#include <mujoco/mjspec.h>

#include <string>


namespace gz {
namespace physics {
namespace mujoco {

Identity EntityManagementFeatures::ConstructEmptyWorld(
    const Identity &/*_engineID*/, const std::string &_name)
{
  auto &worldInfo = this->worlds.emplace_back();
  auto spec = mj_makeSpec();
  mjs_setName(spec->element, _name.c_str());

  worldInfo->mjSpecObj = spec;
  worldInfo->body = mjs_findBody(spec, "world");
  auto worldID = static_cast<std::size_t>(mjs_getId(worldInfo->body->element));
  return this->GenerateIdentity(worldID, worldInfo);
}
}
}
}
