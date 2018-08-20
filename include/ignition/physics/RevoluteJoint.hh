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

#ifndef IGNITION_PHYSICS_REVOLUTEJOINT_HH_
#define IGNITION_PHYSICS_REVOLUTEJOINT_HH_

#include <ignition/physics/DeclareJointType.hh>

namespace ignition
{
  namespace physics
  {
    IGN_PHYSICS_DECLARE_JOINT_TYPE(RevoluteJoint)

    class IGNITION_PHYSICS_VISIBLE GetRevoluteJointProperties
        : public virtual Feature
    {
      /// \brief The API for getting basic revolute joint properties
      public: template <typename PolicyT, typename FeaturesT>
      class RevoluteJoint : public virtual Entity<PolicyT, FeaturesT>
      {
        public: using Axis =
            typename FromPolicy<PolicyT>::template Use<AngularVector>;

        /// \brief Get the axis of this RevoluteJoint.
        ///
        /// For 3D simulations, this will be a 3D vector.
        ///
        /// For 2D simulations, this will be a 1D "vector" (essentially a
        /// scalar, but its value must be accessed from the first element).
        /// This will typically have a value of +1.0 or -1.0.
        ///
        /// \return this joint's axis.
        public: Axis GetAxis() const;
      };

      /// \private The implementation API for getting basic revolute joint
      /// properties
      public: template <typename PolicyT>
      class Implementation : public virtual Feature::Implementation<PolicyT>
      {
        public: using Axis =
            typename FromPolicy<PolicyT>::template Use<AngularVector>;

        /// \brief Get the axis of this RevoluteJoint.
        /// \return the axis of joint _id.
        public: virtual Axis GetRevoluteJointAxis(std::size_t _id) const = 0;
      };

      public: using RequiredFeatures =
          FeatureList<ignition::physics::RevoluteJointCast>;
    };

    /// \brief Provide the API for setting a revolute joint's axis. Not all
    /// physics engines are able to change properties during run-time, so some
    /// might support getting the joint axis but not setting it.
    class IGNITION_PHYSICS_VISIBLE SetRevoluteJointProperties
        : public virtual Feature
    {
      /// \brief The API for setting basic revolute joint properties
      public: template <typename PolicyT, typename FeaturesT>
      class RevoluteJoint : public virtual Entity<PolicyT, FeaturesT>
      {
        public: using Axis =
            typename FromPolicy<PolicyT>::template Use<AngularVector>;

        /// \brief Set the axis of this RevoluteJoint
        ///
        /// For 3D simulations, this expects a 3D vector.
        ///
        /// For 2D simulations, this expects a 1D "vector" (essentially a
        /// scalar, but its value must be accessed from the first element).
        /// It is recommended that you only pass a value of +1.0 or -1.0.
        /// Physics engines might not normalize the value before using it.
        public: void SetAxis(const Axis &_axis);
      };

      /// \private The implementation API for setting basic revolute joint
      /// properties
      public: template <typename PolicyT>
      class Implementation : public virtual Feature::Implementation<PolicyT>
      {
        public: using Axis =
            typename FromPolicy<PolicyT>::template Use<AngularVector>;

        /// \brief Set the axis of the RevoluteJoint.
        public: virtual void SetRevoluteJointAxis(
            std::size_t _id, const Axis &_axis) = 0;
      };

      public: using RequiredFeatures =
          FeatureList<ignition::physics::RevoluteJointCast>;
    };
  }
}

#include <ignition/physics/detail/RevoluteJoint.hh>

#endif
