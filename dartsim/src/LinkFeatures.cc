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

#include <gz/common/Profiler.hh>

#include "LinkFeatures.hh"

namespace gz {
namespace physics {
namespace dartsim {

/////////////////////////////////////////////////
void LinkFeatures::AddLinkExternalForceInWorld(
    const Identity &_id, const LinearVectorType &_force,
    const LinearVectorType &_position)
{
  GZ_PROFILE("LinkFeatures::AddLinkExternalForceInWorld");
  auto bn = this->ReferenceInterface<LinkInfo>(_id)->link;
  bn->addExtForce(_force, _position, false, false);
}

/////////////////////////////////////////////////
void LinkFeatures::AddLinkExternalTorqueInWorld(
    const Identity &_id, const AngularVectorType &_torque)
{
  GZ_PROFILE("LinkFeatures::AddLinkExternalTorqueInWorld");
  auto bn = this->ReferenceInterface<LinkInfo>(_id)->link;
  bn->addExtTorque(_torque, false);
}

/////////////////////////////////////////////////
void LinkFeatures::SetLinkGravityEnabled(const Identity &_id, bool _enabled)
{
  GZ_PROFILE("LinkFeatures::SetLinkGravityEnabled");
  auto linkInfo = this->ReferenceInterface<LinkInfo>(_id);
  this->SetLinkGravityMode(linkInfo, _enabled);
}

/////////////////////////////////////////////////
bool LinkFeatures::GetLinkGravityEnabled(const Identity &_id) const
{
  GZ_PROFILE("LinkFeatures::GetLinkGravityEnabled");
  auto linkInfo = this->ReferenceInterface<LinkInfo>(_id);
  return this->GetLinkGravityMode(linkInfo);
}

}
}
}
