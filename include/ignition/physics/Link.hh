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
    class IGNITION_PHYSICS_VISIBLE AddLinkExternalForceTorque
      : public virtual FeatureWithRequirements<LinkFrameSemantics>
    {
      /// \brief The Link API for adding link force/torque
      public: template <typename PolicyT, typename FeaturesT>
      class Link : public virtual LinkFrameSemantics::Link<PolicyT, FeaturesT>
      {
        public: using LinearVectorType =
            typename FromPolicy<PolicyT>::template Use<LinearVector>;

        public: using RelativePositionType =
            typename FromPolicy<PolicyT>::template Use<RelativePosition>;

        public: using RelativeForceType =
            typename FromPolicy<PolicyT>::template Use<RelativeForce>;

        public: using AngularVectorType =
            typename FromPolicy<PolicyT>::template Use<AngularVector>;

        public: using RelativeTorqueType =
            typename FromPolicy<PolicyT>::template Use<RelativeTorque>;

        /// \brief Add a force on the link applied at a specified position. This
        /// force is applied for one simulation step only.
        /// \param[in] _force The desired force as a Relative Force (a quantity
        /// that contains information about the coordinates in which it is
        /// expressed)
        /// \param[in] _position The point of application of the force. This
        /// parameter is a Relative Position (a quantity that contains
        /// information about the frame in which it is expressed).
        public: void AddExternalForce(
            const RelativeForceType &_force,
            const RelativePositionType &_position);

        /// \brief Add a force on the link applied at a specified position. This
        /// force is applied for one simulation step only. This is a convenience
        /// form of AddExternalForce where the force and position are free
        /// vectors and an additional parameter is used to specify the
        /// coordinates in which the force is expressed.
        /// \param[in] _force The desired force
        /// \param[in] _forceInCoordinatesOf The coordinate in which the force
        /// is expressed
        /// \param[in] _position The point of application of the force expressed
        /// in the link-fixed frame.
        ///
        /// Note that in this version of AddExternalForce the position is
        /// expressed in the link-fixed frame while the coordinate frame for the
        /// force is a parameter that can be set. If you want to specify the
        /// reference frame for the position quanity, please use the
        /// AddExternalForce function that takes Relative quantities.
        public: void AddExternalForce(
            const LinearVectorType &_force,
            const FrameID &_forceInCoordinatesOf = FrameID::World(),
            const LinearVectorType &_position = LinearVectorType::Zero());

        /// \brief Set the torque applied on the link
        /// \param[in] _torque The desired torque as a Relative Torque (a
        /// quantity that contains information about the coordinates in which it
        /// is expressed)
        public: void AddExternalTorque(const RelativeTorqueType &_torque);

        /// \brief Add an external torque on the link. The torque is applied for
        /// one simulation step only.
        /// \param[in] _torque The desired torque
        /// \param[in] _inCoordinatesOf The coordinates in which the torque is
        /// expressed
        public: void AddExternalTorque(
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

        // \brief Implementation API for adding a force to link at a specified
        // position. The force and the position are both expressed in the World
        // frame
        /// \param[in] _id Identity of the link on which the force is applied
        /// \param[in] _force The desired force in world frame
        /// \param[in] _position The point of application of the force in world
        /// frame.
        public: virtual void AddLinkExternalForceInWorld(
            const Identity &_id, const LinearVectorType &_force,
            const LinearVectorType &_position) = 0;

        // \brief Implementation API for adding a torque to link position.
        /// \param[in] _id Identity of the link on which the torque is applied
        /// \param[in] _torque The desired torque expressed in World
        /// coordinates.
        public: virtual void AddLinkExternalTorqueInWorld(
            const Identity &_id, const AngularVectorType &_torque) = 0;
      };
    };
  }
}

#include <ignition/physics/detail/Link.hh>

#endif
