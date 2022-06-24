/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include <gz/common/Console.hh>

#include "WorldFeatures.hh"

namespace gz {
namespace physics {
namespace bullet {

/////////////////////////////////////////////////
void WorldFeatures::SetWorldGravity(
    const Identity &_id, const LinearVectorType &_gravity)
{
  const WorldInfoPtr &worldInfo = this->worlds.at(_id.id);
  worldInfo->world->setGravity(
    btVector3(_gravity(0), _gravity(1), _gravity(2)));

  // auto world = this->ReferenceInterface<dart::simulation::World>(_id);
  // world->setGravity(_gravity);
}

/////////////////////////////////////////////////
WorldFeatures::LinearVectorType WorldFeatures::GetWorldGravity(
    const Identity &_id) const
{
  std::cerr << "_id " << _id.id << '\n';
  for (const auto& [key, value] : this->worlds)
  {
      std::cerr << "id worlds " << key << '\n';
  }

  for (const auto& [key, value] : this->models)
  {
      std::cerr << "id model " << key << '\n';
  }
  for (const auto& [key, value] : this->links)
  {
      std::cerr << "id links " << key << '\n';
  }

  if (this->links.find(_id.id) != this->links.end())
  {
    std::cerr << "/Link GetWorldGravity " << _id.id << '\n';
  }
  else if (this->models.find(_id.id) != this->models.end())
  {
    std::cerr << "/Model GetWorldGravity */ " << _id.id << '\n';
  }
  else
  {
    const WorldInfoPtr &worldInfo = this->worlds.at(_id);

    // auto world = this->ReferenceInterface<dart::simulation::World>(_id);
    return WorldFeatures::LinearVectorType(
      worldInfo->world->getGravity().x(),
      worldInfo->world->getGravity().y(),
      worldInfo->world->getGravity().z());
  }
}
}
}
}
