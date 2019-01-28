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
#include <ignition/physics/FrameID.hh>
#include <ignition/physics/FrameSemantics.hh>
#include <ignition/physics/Geometry.hh>

namespace ignition
{
  namespace physics
  {
    /////////////////////////////////////////////////
    class IGNITION_PHYSICS_VISIBLE GetLinkForceTorque
      : public virtual FeatureWithRequirements<FrameSemantics>
    {
      /// \brief The Link API for getting force/torque on a link
      public: template <typename PolicyT, typename FeaturesT>
      class Link : public virtual Feature::Link<PolicyT, FeaturesT>
      {
        public: using LinearVectorType =
            typename FromPolicy<PolicyT>::template Use<LinearVector>;

        public: using RelativeForceType =
            typename FromPolicy<PolicyT>::template Use<RelativeTorque>;

        public: using AngularVectorType =
            typename FromPolicy<PolicyT>::template Use<AngularVector>;

        public: using RelativeTorqueType =
            typename FromPolicy<PolicyT>::template Use<RelativeTorque>;

        /// \brief Get the force applied on the link
        /// \param[in] _inCoordinatesOf The frame in which the force is
        /// expressed
        /// \return The applied force on the link
        public: LinearVectorType GetExternalForce(
            const FrameID &_inCoordinatesOf = FrameID::World()) const;

        /// \brief Get the torque applied on the link
        /// \param[in] _inCoordinatesOf The frame in which the torque is
        /// expressed
        /// \return The applied torque on the link
        public: AngularVectorType GetExternalTorque(
            const FrameID &_inCoordinatesOf = FrameID::World()) const;
      };

      /// \private The implementation API for getting force/torque on a link
      public: template <typename PolicyT>
      class Implementation : public virtual Feature::Implementation<PolicyT>
      {
        public: using LinearVectorType =
            typename FromPolicy<PolicyT>::template Use<LinearVector>;

        public: using AngularVectorType =
            typename FromPolicy<PolicyT>::template Use<AngularVector>;

        // see Link::GetExternalForce above
        public: virtual LinearVectorType GetLinkExternalForceInWorld(
            const Identity &_id) const = 0;

        // see Link::GetExternalTorque above
        public: virtual AngularVectorType GetLinkExternalTorqueInWorld(
            const Identity &_id) const = 0;
      };
    };

    /////////////////////////////////////////////////
    class IGNITION_PHYSICS_VISIBLE SetLinkForceTorque
      : public virtual FeatureWithRequirements<FrameSemantics>
    {
      /// \brief The Link API for setting link force/torque
      public: template <typename PolicyT, typename FeaturesT>
      class Link : public virtual Feature::Link<PolicyT, FeaturesT>
      {
        public: using LinearVectorType =
            typename FromPolicy<PolicyT>::template Use<LinearVector>;

        public: using RelativeForceType =
            typename FromPolicy<PolicyT>::template Use<RelativeTorque>;

        public: using AngularVectorType =
            typename FromPolicy<PolicyT>::template Use<AngularVector>;

        public: using RelativeTorqueType =
            typename FromPolicy<PolicyT>::template Use<RelativeTorque>;

        /// \brief Set the force applied on the link
        /// \param[in] _force The desired force
        /// \param[in] _inCoordinatesOf The frame in which the force is
        /// expressed
        public: void SetExternalForce(
            const LinearVectorType &_force,
            const FrameID &_inCoordinatesOf = FrameID::World());

        /// \brief Set the torque applied on the link
        /// \param[in] _torque The desired torque
        /// \param[in] _inCoordinatesOf The frame in which the torque is
        /// expressed
        public: void SetExternalTorque(
            const AngularVectorType &_torque,
            const FrameID &_inCoordinatesOf = FrameID::World());
      };

      /// \private The implementation API for setting force/torque on a link
      public: template <typename PolicyT>
      class Implementation : public virtual Feature::Implementation<PolicyT>
      {
        public: using LinearVectorType =
            typename FromPolicy<PolicyT>::template Use<LinearVector>;

        public: using AngularVectorType =
            typename FromPolicy<PolicyT>::template Use<AngularVector>;

        // see Link::SetExternalForce above
        public: virtual void SetLinkExternalForceInWorld(
            const Identity &_id, const LinearVectorType &_force) = 0;

        // see Link::SetExternalTorque above
        public: virtual void SetLinkExternalTorqueInWorld(
            const Identity &_id, const AngularVectorType &_torque) = 0;
      };
    };
  }
}

#include <ignition/physics/detail/Link.hh>

#endif
