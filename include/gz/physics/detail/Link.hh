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

#ifndef GZ_PHYSICS_DETAIL_LINK_HH_
#define GZ_PHYSICS_DETAIL_LINK_HH_

#include <gz/physics/Link.hh>
#include <gz/physics/RequestEngine.hh>
#include <gz/physics/RelativeQuantity.hh>

namespace ignition
{
namespace physics
{
/////////////////////////////////////////////////
template <typename PolicyT, typename FeaturesT>
void AddLinkExternalForceTorque::Link<PolicyT, FeaturesT>::AddExternalForce(
    const RelativeForceType &_force, const RelativePositionType &_position)
{
  const auto &impl = *this->template Interface<FrameSemantics>();
  const auto forceWorld =
      detail::Resolve(impl, _force, FrameID::World(), FrameID::World());
  const auto positionWorld =
      detail::Resolve(impl, _position, FrameID::World(), FrameID::World());

  this->template Interface<AddLinkExternalForceTorque>()
      ->AddLinkExternalForceInWorld(this->identity, forceWorld, positionWorld);
}
/////////////////////////////////////////////////
template <typename PolicyT, typename FeaturesT>
void AddLinkExternalForceTorque::Link<PolicyT, FeaturesT>::AddExternalForce(
    const LinearVectorType &_force, const FrameID &_forceInCoordinatesOf,
    const LinearVectorType &_position)
{
  const auto &impl = *this->template Interface<FrameSemantics>();
  // Special case for world coordinates
  auto forceWorld = _force;
  if (_forceInCoordinatesOf != FrameID::World())
  {
    RelativeForceType forceInRef(_forceInCoordinatesOf, _force);
    forceWorld =
        detail::Resolve(impl, forceInRef, FrameID::World(), FrameID::World());
  }

  RelativePositionType positionInLink(this->GetFrameID(), _position);
  const auto positionWorld =
      detail::Resolve(impl, positionInLink, FrameID::World(), FrameID::World());

  this->template Interface<AddLinkExternalForceTorque>()
      ->AddLinkExternalForceInWorld(this->identity, forceWorld, positionWorld);
}

/////////////////////////////////////////////////
template <typename PolicyT, typename FeaturesT>
void AddLinkExternalForceTorque::Link<PolicyT, FeaturesT>::AddExternalTorque(
    const RelativeTorqueType &_torque)
{
  const auto &impl = *this->template Interface<FrameSemantics>();
  const auto torqueWorld =
      detail::Resolve(impl, _torque, FrameID::World(), FrameID::World());

  this->template Interface<AddLinkExternalForceTorque>()
      ->AddLinkExternalTorqueInWorld(this->identity, torqueWorld);
}

/////////////////////////////////////////////////
template <typename PolicyT, typename FeaturesT>
void AddLinkExternalForceTorque::Link<PolicyT, FeaturesT>::AddExternalTorque(
    const AngularVectorType &_torque, const FrameID &_inCoordinatesOf)
{
  const auto &impl = *this->template Interface<FrameSemantics>();
  // Special case for world coordinates
  auto torqueWorld = _torque;
  if (_inCoordinatesOf != FrameID::World())
  {
    RelativeTorqueType torqueInRef(_inCoordinatesOf, _torque);
    torqueWorld =
        detail::Resolve(impl, torqueInRef, FrameID::World(), FrameID::World());
  }

  this->template Interface<AddLinkExternalForceTorque>()
      ->AddLinkExternalTorqueInWorld(this->identity, torqueWorld);
}

}  // namespace physics
}  // namespace ignition

#endif
