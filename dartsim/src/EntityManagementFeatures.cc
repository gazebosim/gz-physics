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
    const std::size_t, std::size_t _worldIndex) const
{
  const std::size_t id = this->worlds.indexInParentToID[_worldIndex];
  return this->GenerateIdentity(id, this->worlds.idToObject.at(id));
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetWorld(
    const std::size_t, const std::string &_worldName) const
{
  const std::size_t id = this->worlds.IdentityOf(_worldName);
  return this->GenerateIdentity(id, this->worlds.idToObject.at(id));
}

/////////////////////////////////////////////////
const std::string &EntityManagementFeatures::GetWorldName(
    const std::size_t _worldID) const
{
  return this->worlds.at(_worldID)->getName();
}

/////////////////////////////////////////////////
std::size_t EntityManagementFeatures::GetWorldIndex(
    const std::size_t _worldID) const
{
  return this->worlds.idToIndexInParent.at(_worldID);
}

/////////////////////////////////////////////////
std::size_t EntityManagementFeatures::GetModelCount(
    const std::size_t _worldID) const
{
  return this->worlds.at(_worldID)->getNumSkeletons();
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetModel(
    const std::size_t _worldID, const std::size_t _modelIndex) const
{
  const DartSkeletonPtr &model =
      this->worlds.at(_worldID)->getSkeleton(_modelIndex);

  return this->GenerateIdentity(this->models.IdentityOf(model), model);
}

/////////////////////////////////////////////////
Identity EntityManagementFeatures::GetModel(
    const std::size_t _worldID, const std::string &_modelName) const
{
  const DartSkeletonPtr &model =
      this->worlds.at(_worldID)->getSkeleton(_modelName);

  return this->GenerateIdentity(this->models.IdentityOf(model), model);
}

/////////////////////////////////////////////////
const std::string &EntityManagementFeatures::GetModelName(
    const std::size_t _modelID) const
{
  return this->models.at(_modelID).model->getName();
}

/////////////////////////////////////////////////
std::size_t EntityManagementFeatures::GetModelIndex(
    const std::size_t _modelID) const
{
  // TODO(MXG): I guess we need to store this information somewhere? Or else do
  // a N-complexity search through World.
}

}
}
}
