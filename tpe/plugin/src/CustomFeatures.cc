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

#include <gz/common/Console.hh>

#include "CustomFeatures.hh"

using namespace gz;
using namespace physics;
using namespace tpeplugin;

/////////////////////////////////////////////////
std::shared_ptr<tpelib::World> CustomFeatures::GetTpeLibWorld(
  const Identity &_worldID)
{
  auto it = this->worlds.find(_worldID);
  if (it == this->worlds.end())
  {
    gzerr << "Unable to retrieve world ["
      << _worldID.id
      << "]"
      << std::endl;
    return nullptr;
  }
  return it->second->world;
}
