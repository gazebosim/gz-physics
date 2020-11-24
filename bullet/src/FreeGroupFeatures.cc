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

#include "FreeGroupFeatures.hh"

namespace ignition {
namespace physics {
namespace bullet {

/////////////////////////////////////////////////
Identity FreeGroupFeatures::FindFreeGroupForModel(
    const Identity &_modelID) const
{
  // DUMMY FEATURE
  ignwarn << "Using dummy feature FindFreeGroupForModel.\n";
  (void) _modelID;
  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
Identity FreeGroupFeatures::FindFreeGroupForLink(
    const Identity &_linkID) const
{
  // DUMMY FEATURE
  ignwarn << "Using dummy feature FindFreeGroupForLink.\n";
  (void) _linkID;
  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
Identity FreeGroupFeatures::GetFreeGroupCanonicalLink(
    const Identity &_groupID) const
{
  // DUMMY FEATURE
  ignwarn << "Using dummy feature GetFreeGroupCanonicalLink.\n";
  (void) _groupID;
  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
void FreeGroupFeatures::SetFreeGroupWorldPose(
    const Identity &_groupID,
    const PoseType &_pose)
{
  // DUMMY FEATURE
  ignwarn << "Using dummy feature SetFreeGroupWorldPose.\n";
  (void) _groupID;
  (void) _pose;
}

}
}
}
