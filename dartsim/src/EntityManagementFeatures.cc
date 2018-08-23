/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

#include <dart/config.hpp>

#include "EntityManagementFeatures.hh"

namespace ignition {
namespace physics {
namespace dartsim {

/////////////////////////////////////////////////
const std::string &EntityManagementFeatures::GetEngineName(
    const std::size_t /*_engineID*/) const
{
  static const std::string engineName = "dartsim-" DART_VERSION;
  return engineName;
}

/////////////////////////////////////////////////
std::size_t EntityManagementFeatures::GetEngineIndex(
    const std::size_t /*_engineID*/) const
{
  // The dartsim plugin does not make a distinction between different engine
  // indexes.
  return 0;
}

/////////////////////////////////////////////////
std::size_t EntityManagementFeatures::GetWorldCount(
    const std::size_t /*_engineID*/) const
{
  return worlds.size();
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetWorld(
    const std::size_t, const std::string &_worldName) const
{

}


}
}
}
