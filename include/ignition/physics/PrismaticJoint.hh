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

#ifndef IGNITION_PHYSICS_PRISMATICJOINT_HH_
#define IGNITION_PHYSICS_PRISMATICJOINT_HH_

#include <ignition/physics/CreateJointType.hh>

namespace ignition
{
  namespace physics
  {
    IGN_PHYSICS_CREATE_JOINT_TYPE(PrismaticJoint)

    class IGNITION_PHYSICS_VISIBLE GetPrismaticJointProperties
        : public virtual Feature
    {
      /// \brief The API for getting basic prismatic joint properties
      public: template <typename PolicyT, typename FeaturesT>
      class PrismaticJoint : public virtual Entity<PolicyT, FeaturesT>
      {
        public: using Axis =
            typename FromPolicy<PolicyT>::template Use<LinearVector>;

        /// \brief Get the axis of this PrismaticJoint.
        ///
        /// For 3D simulations, this will be a 3D vector.
        ///
        /// For 2D simulations, this will be a 2D vector.
        ///
        /// \return this joint's axis.
        public: Axis GetAxis() const;
      };

      /// \private The implementation API for getting basic prismatic joint
      /// properties
      public: template <typename PolicyT>
      class Implementation : public virtual Feature::Implementation<PolicyT>
      {
        public: using Axis =
            typename FromPolicy<PolicyT>::template Use<LinearVector>;

        public: virtual Axis GetPrismaticJointAxis(std::size_t _id) const = 0;
      };

      public: using RequiredFeatures =
          FeatureList<ignition::physics::PrismaticJoint>;
    };

    /// \brief Provide the API for setting a prismatic joint's axis. Not all
    /// physics engines are able to change properties during run-time, so some
    /// might support getting the joint axis but not setting it.
    class IGNITION_PHYSICS_VISIBLE SetPrismaticJointProperties
        : public virtual Feature
    {
      /// \brief The API for setting basic prismatic joint properties
      public: template <typename PolicyT, typename FeaturesT>
      class PrismaticJoint : public virtual Entity<PolicyT, FeaturesT>
      {
        public: using Axis =
            typename FromPolicy<PolicyT>::template Use<LinearVector>;

        /// \brief Set the axis of this PrismaticJoint.
        ///
        /// For 3D simulations, this will be a 3D vector.
        ///
        /// For 2D simulations, this will be a 2D vector.
        public: void SetAxis(const Axis &_axis);
      };

      /// \private The implementation API for setting basic prismatic joint
      /// properties
      public: template <typename PolicyT>
      class Implementation : public virtual Feature::Implementation<PolicyT>
      {
        public: using Axis =
            typename FromPolicy<PolicyT>::template Use<LinearVector>;

        /// \brief Set the axis of the PrismaticJoint.
        public: virtual void SetPrismaticJointAxis(
            std::size_t _id, const Axis &_axis) = 0;
      };

      public: using RequiredFeatures =
          FeatureList<ignition::physics::PrismaticJoint>;
    };
  }
}

#include <ignition/physics/detail/PrismaticJoint.hh>

#endif
