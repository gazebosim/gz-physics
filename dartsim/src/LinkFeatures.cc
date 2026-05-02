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

namespace gz {
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

/////////////////////////////////////////////////
void LinkFeatures::SetLinkGravityEnabled(const Identity &_id, bool _enabled)
{
  auto linkInfo = this->ReferenceInterface<LinkInfo>(_id);
  if (!linkInfo || !linkInfo->link)
    return;

  // Added-mass links must keep DART gravity disabled: gravity is applied
  // manually as F=ma each step. Let SetLinkAddedMass own that flag.
  if (linkInfo->inertial.has_value() &&
      linkInfo->inertial->FluidAddedMass().has_value())
  {
    return;
  }

  linkInfo->link->setGravityMode(_enabled);
}

/////////////////////////////////////////////////
bool LinkFeatures::GetLinkGravityEnabled(const Identity &_id) const
{
  auto linkInfo = this->ReferenceInterface<LinkInfo>(_id);
  if (!linkInfo || !linkInfo->link)
    return true;

  // Added-mass links have DART gravity forcibly disabled; exclude them so
  // their internal state doesn't pollute the user-visible gravity flag.
  if (linkInfo->inertial.has_value() &&
      linkInfo->inertial->FluidAddedMass().has_value())
  {
    return true;
  }

  return linkInfo->link->getGravityMode();
}

}
}
}
