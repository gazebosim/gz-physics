/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/FreeJoint.hpp>

#include "LinkFeatures.hh"

namespace ignition {
namespace physics {
namespace dartsim {

/////////////////////////////////////////////////
void LinkFeatures::AddLinkExternalForceInWorld(
    const Identity &_id, const LinearVectorType &_force,
    const LinearVectorType &_position)
{
  auto bn = this->ReferenceInterface<LinkInfo>(_id)->link;
  bn->addExtForce(_force, _position, false, false);
}

/////////////////////////////////////////////////
void LinkFeatures::AddLinkExternalTorqueInWorld(
    const Identity &_id, const AngularVectorType &_torque)
{
  auto bn = this->ReferenceInterface<LinkInfo>(_id)->link;
  bn->addExtTorque(_torque, false);
}

}
}
}
