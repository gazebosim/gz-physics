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

#ifndef IGNITION_PHYSICS_DETAIL_LINK_HH_
#define IGNITION_PHYSICS_DETAIL_LINK_HH_

#include <ignition/physics/Link.hh>
#include <ignition/physics/RequestEngine.hh>
#include <ignition/physics/RelativeQuantity.hh>

namespace ignition
{
  namespace physics
  {
    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto GetLinkForceTorque::Link<PolicyT, FeaturesT>::GetExternalForce(
        const FrameID &_inCoordinatesOf) const -> LinearVectorType
    {
      const auto forceWorld =
          this->template Interface<GetLinkForceTorque>()
              ->GetLinkExternalForceInWorld(this->identity);

      // express the force in the desired coordinates frame
      RelativeForceType relativeForce(FrameID::World(), forceWorld);

      return detail::Resolve(*this->template Interface<FrameSemantics>(),
                             relativeForce, FrameID::World(), _inCoordinatesOf);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto GetLinkForceTorque::Link<PolicyT, FeaturesT>::GetExternalTorque(
        const FrameID &_inCoordinatesOf) const -> AngularVectorType
    {
      const auto torqueWorld =
          this->template Interface<GetLinkForceTorque>()
              ->GetLinkExternalTorqueInWorld(this->identity);

      // express the torque in the desired coordinates frame
      RelativeTorqueType relativeTorque(FrameID::World(), torqueWorld);

      return detail::Resolve(*this->template Interface<FrameSemantics>(),
                             relativeTorque, FrameID::World(),
                             _inCoordinatesOf);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    void SetLinkForceTorque::Link<PolicyT, FeaturesT>::SetExternalForce(
        const LinearVectorType &_force, const FrameID &_inCoordinatesOf)
    {
      // First express the force in the world frame
      const RelativeForceType forceLocal(_inCoordinatesOf, _force);
      const auto forceWorld =
          detail::Resolve(*this->template Interface<FrameSemantics>(),
                          forceLocal, FrameID::World(), FrameID::World());

      return this->template Interface<SetLinkForceTorque>()
          ->SetLinkExternalForceInWorld(this->identity, forceWorld);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    void SetLinkForceTorque::Link<PolicyT, FeaturesT>::SetExternalTorque(
        const AngularVectorType &_torque, const FrameID &_inCoordinatesOf)
    {
      // First express the force in the world frame
      const RelativeTorqueType torqueLocal(_inCoordinatesOf, _torque);
      const auto torqueWorld =
          detail::Resolve(*this->template Interface<FrameSemantics>(),
                          torqueLocal, FrameID::World(), FrameID::World());
      return this->template Interface<SetLinkForceTorque>()
          ->SetLinkExternalTorqueInWorld(this->identity, torqueWorld);
    }
  }
}

#endif
