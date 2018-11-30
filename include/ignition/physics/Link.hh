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

#ifndef IGNITION_PHYSICS_LINK_HH_
#define IGNITION_PHYSICS_LINK_HH_

#include <ignition/physics/FeatureList.hh>
#include <ignition/physics/Geometry.hh>

namespace ignition
{
  namespace physics
  {
    /////////////////////////////////////////////////
    class IGNITION_PHYSICS_VISIBLE SetLinkState : public virtual Feature
    {
      /// \brief The Link API for getting basic link state
      public: template <typename PolicyT, typename FeaturesT>
      class Link : public virtual Feature::Link<PolicyT, FeaturesT>
      {
        public: using LinearVectorType =
                typename FromPolicy<PolicyT>::template Use<LinearVector>;

        public: using AngularVectorType =
                typename FromPolicy<PolicyT>::template Use<AngularVector>;

        /// \brief Set the linear velocity of the link
        /// \param[in] _vel The desired linear velocity
        public: void SetLinearVelocity(const LinearVectorType &_vel);

        /// \brief Set the angular velocity of the link
        /// \param[in] _vel The desired angular velocity
        public: void SetAngularVelocity(const AngularVectorType &_vel);

        /// \brief Set the force applied on the link
        /// \param[in] _force The desired force
        // public: void SetForce(const LinearVectorType &_force);

        /// \brief Set the torque applied on the link
        /// \param[in] _torque The desired torque
        // public: void SetTorque(const AngularVectorType &_torque);
      };

      /// \private The implementation API for getting basic joint state
      public: template <typename PolicyT>
      class Implementation : public virtual Feature::Implementation<PolicyT>
      {
        public: using LinearVectorType =
                typename FromPolicy<PolicyT>::template Use<LinearVector>;

        public: using AngularVectorType =
                typename FromPolicy<PolicyT>::template Use<AngularVector>;

        // see Link::SetLinearVelocity above
        public: virtual void SetLinkLinearVelocity(
            std::size_t _id, const LinearVectorType &_vel) = 0;

        // see Link::SetAngularVelocity above
        public: virtual void SetLinkAngularVelocity(
            std::size_t _id, const AngularVectorType &_vel) = 0;
      };
    };
  }
}

#include <ignition/physics/detail/Link.hh>

#endif
