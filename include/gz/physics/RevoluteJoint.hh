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

#include <string>

#include <ignition/physics/DeclareJointType.hh>
#include <ignition/physics/Geometry.hh>

namespace ignition
{
  namespace physics
  {
    IGN_PHYSICS_DECLARE_JOINT_TYPE(RevoluteJoint)

    class IGNITION_PHYSICS_VISIBLE GetRevoluteJointProperties
        : public virtual FeatureWithRequirements<RevoluteJointCast>
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
        /// \param[in] _id Identity of joint
        /// \return the axis of the joint.
        public: virtual Axis GetRevoluteJointAxis(
            const Identity &_id) const = 0;
      };
    };

    /// \brief Provide the API for setting a revolute joint's axis. Not all
    /// physics engines are able to change properties during run-time, so some
    /// might support getting the joint axis but not setting it.
    class IGNITION_PHYSICS_VISIBLE SetRevoluteJointProperties
        : public virtual FeatureWithRequirements<RevoluteJointCast>
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
            const Identity &_id, const Axis &_axis) = 0;
      };
    };

    /// \brief Provide the API for attaching a Link to another Link (or directly
    /// to the World) with a revolute joint. After calling AttachRevoluteJoint,
    /// the Link's parent joint will be a revolute joint.
    class IGNITION_PHYSICS_VISIBLE AttachRevoluteJointFeature
        : public virtual FeatureWithRequirements<RevoluteJointCast>
    {
      public: template <typename PolicyT, typename FeaturesT>
      class Link : public virtual Feature::Link<PolicyT, FeaturesT>
      {
        public: using Axis =
            typename FromPolicy<PolicyT>::template Use<AngularVector>;

        public: using JointPtrType = RevoluteJointPtr<PolicyT, FeaturesT>;

        /// \brief Attach this link to another link using a revolute joint.
        /// \param[in] _parent
        ///   The parent link for the joint. Pass in a nullptr to attach the
        ///   link to the world.
        /// \param[in] _name
        ///   The name of this joint.
        /// \param[in] _axis
        ///   The joint axis for the new joint. The rest of the joint properties
        ///   will be left to the default values of the physics engine.
        /// \return A reference to the newly constructed RevoluteJoint.
        //
        // TODO(MXG): Instead of _name and _axis, consider passing in a struct
        // containing all base joint properties plus the axis.
        public: JointPtrType AttachRevoluteJoint(
            const BaseLinkPtr<PolicyT> &_parent,
            const std::string &_name = "revolute",
            const Axis &_axis = Axis::UnitX());
      };

      public: template <typename PolicyT>
      class Implementation : public virtual Feature::Implementation<PolicyT>
      {
        public: using Axis =
            typename FromPolicy<PolicyT>::template Use<AngularVector>;

        /// \param[in] _childID
        ///   The ID of the child link.
        /// \param[in] _parent
        ///   A reference to the parent link. If this evaluates to a nullptr,
        ///   then the parent should be the world.
        /// \param[in] _name
        ///   The name of this joint.
        /// \param[in] _axis
        ///   The desired axis of the new revolute joint
        /// \returns the Identity of the newly created RevoluteJoint
        public: virtual Identity AttachRevoluteJoint(
            const Identity &_childID,
            const BaseLinkPtr<PolicyT> &_parent,
            const std::string &_name,
            const Axis &_axis) = 0;
      };
    };
  }
}

#include <ignition/physics/detail/RevoluteJoint.hh>

#endif
