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

using namespace gz;
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

  // check if it's model
  auto modelIt = this->models.find(_id.ID());
  if (modelIt != this->models.end())
  {
    auto model = modelIt->second->model;
    data.pose = math::eigen3::convert(model->GetWorldPose());
    data.linearVelocity = math::eigen3::convert(model->GetLinearVelocity());
    data.angularVelocity = math::eigen3::convert(model->GetAngularVelocity());
  }
  else
  {
    // check if it's link
    auto linkIt = this->links.find(_id.ID());
    if (linkIt != this->links.end())
    {
      auto link = linkIt->second->link;
      data.pose = math::eigen3::convert(link->GetWorldPose());
    }
    else
    {
      // check if it's collision
      auto colIt = this->collisions.find(_id.ID());
      if (colIt != this->collisions.end())
      {
        auto collision = colIt->second->collision;
        data.pose = math::eigen3::convert(collision->GetWorldPose());
      }
      else
      {
        ignwarn << "Entity with id ["
          << _id.ID() << "] is not found" << std::endl;
      }
    }
  }
  return data;
}
