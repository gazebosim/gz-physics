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

namespace ignition
{
  namespace physics
  {
    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto GetLinkForceTorque::Link<PolicyT, FeaturesT>::GetForce()
        -> LinearVectorType
    {
      auto model = static_cast<Link<PolicyT, FeaturesT>*>(this)->GetModel();

      return this->template Interface<GetLinkForceTorque>()->GetLinkForce(
          this->identity);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto GetLinkForceTorque::Link<PolicyT, FeaturesT>::GetTorque()
        -> AngularVectorType
    {
      return this->template Interface<GetLinkForceTorque>()->GetLinkTorque(
            this->identity);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    void SetLinkState::Link<PolicyT, FeaturesT>::SetLinearVelocity(
        const LinearVectorType &_vel)
    {
      return this->template Interface<SetLinkState>()->SetLinkLinearVelocity(
            this->identity, _vel);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    void SetLinkState::Link<PolicyT, FeaturesT>::SetAngularVelocity(
        const AngularVectorType &_vel)
    {
      return this->template Interface<SetLinkState>()->SetLinkAngularVelocity(
            this->identity, _vel);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    void SetLinkState::Link<PolicyT, FeaturesT>::SetForce(
        const LinearVectorType &_force)
    {
      return this->template Interface<SetLinkState>()->SetLinkForce(
          this->identity, _force);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    void SetLinkState::Link<PolicyT, FeaturesT>::SetTorque(
        const AngularVectorType &_torque)
    {
      return this->template Interface<SetLinkState>()->SetLinkTorque(
            this->identity, _torque);
    }
  }
}

#endif
