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

#include <dart/dynamics/Frame.hpp>

#include <gz/common/Console.hh>
#include "KinematicsFeatures.hh"

namespace gz {
namespace physics {
namespace dartsim {

/////////////////////////////////////////////////
FrameData3d KinematicsFeatures::FrameDataRelativeToWorld(
    const FrameID &_id) const
{
  FrameData3d data;

  // The feature system should never send us the world ID.
  if (_id.IsWorld())
  {
    gzerr << "Given a FrameID belonging to the world. This should not be "
           << "possible! Please report this bug!\n";
    assert(false);
    return data;
  }

  const dart::dynamics::Frame *frame = SelectFrame(_id);
  // A missing frame ID indicates that frame semantics is not properly
  // implemented for the type of frame represented by the ID.
  if (nullptr == frame)
  {
    gzerr << "The frame ID " << _id.ID()
           << " was not found in the list of known frames. This should not be "
              "possible! Please report this bug!\n";
    assert(false);
    return data;
  }

  data.pose = frame->getWorldTransform();
  data.linearVelocity = frame->getLinearVelocity();
  data.angularVelocity = frame->getAngularVelocity();
  data.linearAcceleration = frame->getLinearAcceleration();
  data.angularAcceleration = frame->getAngularAcceleration();

  return data;
}

/////////////////////////////////////////////////
const dart::dynamics::Frame *KinematicsFeatures::SelectFrame(
    const FrameID &_id) const
{
  const auto model_it = this->models.idToObject.find(_id.ID());
  if (model_it != this->models.idToObject.end())
  {
    // This is a model FreeGroup frame, so we'll use the first root link as the
    // frame
    return model_it->second->model->getRootBodyNode();
  }

  auto framesIt = this->frames.find(_id.ID());
  if (framesIt == this->frames.end())
  {
    return nullptr;
  }

  return framesIt->second;
}

}
}
}
