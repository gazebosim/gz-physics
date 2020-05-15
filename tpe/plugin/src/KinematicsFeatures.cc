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
  // check if id is present and skip if not
  auto it = this->childIdToParentId.find(_id.ID());
  if (it == this->childIdToParentId.end())
  {
    ignwarn << "Link [" << _id.ID() << "]  is not found." << std::endl;
    return data;
  }
  std::size_t modelId = it->second;
  auto modelIt = this->models.find(modelId);
  if (modelIt == this->models.end())
  {
    ignwarn << "Parent model ["
      << modelId
      << "] of link ["
      << _id.ID()
      << "] is not found."
      << std::endl;
    return data;
  }
  tpelib::Model *model = modelIt->second->model;

  data.pose = math::eigen3::convert(model->GetPose());
  data.linearVelocity = math::eigen3::convert(model->GetLinearVelocity());
  data.angularVelocity = math::eigen3::convert(model->GetAngularVelocity());

  return data;
}
