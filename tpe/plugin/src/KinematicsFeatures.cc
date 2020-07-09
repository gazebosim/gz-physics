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

#include <ignition/common/Console.hh>
#include <ignition/math/eigen3/Conversions.hh>

#include "KinematicsFeatures.hh"

using namespace ignition;
using namespace physics;
using namespace tpeplugin;

/////////////////////////////////////////////////
FrameData3d KinematicsFeatures::FrameDataRelativeToWorld(
  const FrameID &_id) const
{
  FrameData3d data;

  // The feature system should never send us the world ID.
  if (_id.IsWorld())
  {
    ignerr << "Given a FrameID belonging to the world. This should not be "
           << "possible! Please report this bug!\n";
    assert(false);
    return data;
  }

  auto worldIt = this->worlds.find(it->second);
  auto modelIt = this->models.find(it->second);
  auto linkIt = this->links.find(it->second);
  auto collisionIt = this->collisions.find(it->second);

  if (modelIt != this->models.end())
  {
    auto model = modelIt->second->model;
    data.pose = math::eigen3::convert(model->GetPose());
    data.linearVelocity = math::eigen3::convert(model->GetLinearVelocity());
    data.angularVelocity = math::eigen3::convert(model->GetAngularVelocity());
  }
  else if (worldIt != this->worlds.end())
  {
    auto world = worldIt->second->world;
    data.pose = math::eigen3::convert(world->GetPose());
  }
  else if (linkIt != this->worlds.end())
  {
    auto link = linkIt->second->world;
    data.pose = math::eigen::convert(link->GetPose());
  }
  else if (collisionIt != this->collisions.end())
  {
    auto collision = collisionIt->second->world;
    data.pose = math::eigen::convert(link->GetPose());
  }
  else
  {
    ignwarn << "Entity with id ["
      << it->second << "] is not found" << std::endl;
  }

  return data;
}
