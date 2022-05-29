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

#ifndef GZ_PHYSICS_PRISMATICJOINT_HH_
#define GZ_PHYSICS_PRISMATICJOINT_HH_

#include <string>

#include <gz/physics/DeclareJointType.hh>
#include <gz/physics/Geometry.hh>

namespace gz
{
  namespace physics
  {
    IGN_PHYSICS_DECLARE_JOINT_TYPE(PrismaticJoint)

    class GZ_PHYSICS_VISIBLE GetPrismaticJointProperties
        : public virtual FeatureWithRequirements<PrismaticJointCast>
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

        public: virtual Axis GetPrismaticJointAxis(
            const Identity &_id) const = 0;
      };
    };

    /// \brief Provide the API for setting a prismatic joint's axis. Not all
    /// physics engines are able to change properties during run-time, so some
    /// might support getting the joint axis but not setting it.
    class GZ_PHYSICS_VISIBLE SetPrismaticJointProperties
        : public virtual FeatureWithRequirements<PrismaticJointCast>
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
            const Identity &_id, const Axis &_axis) = 0;
      };
    };

    /// \brief Provide the API for attaching a Link to another Link (or directly
    /// to the World) with a prismatic joint. After calling AttachPrismaticJoint
    /// the Link's parent joint will be a prismatic joint.
    class GZ_PHYSICS_VISIBLE AttachPrismaticJointFeature
        : public virtual FeatureWithRequirements<PrismaticJointCast>
    {
      public: template <typename PolicyT, typename FeaturesT>
      class Link : public virtual Feature::Link<PolicyT, FeaturesT>
      {
        public: using Axis =
            typename FromPolicy<PolicyT>::template Use<LinearVector>;

        public: using JointPtrType = PrismaticJointPtr<PolicyT, FeaturesT>;

        /// \brief Attach this link to another link using a prismatic joint.
        /// \param[in] _parent
        ///   The parent link for the joint. Pass in a nullptr to attach the
        ///   link to the world.
        /// \param[in] _name
        ///   The name for this joint.
        /// \param[in] _axis
        ///   The joint axis for the new joint. The rest of the joint properties
        ///   will be left to the default values of the physics engine.
        ///     TODO(MXG): Instead of _axis, consider passing in a struct
        ///     containing all base joint properties plus the axis.
        /// \return A reference to the newly constructed PrismaticJoint.
        public: JointPtrType AttachPrismaticJoint(
            const BaseLinkPtr<PolicyT> &_parent,
            const std::string &_name = "prismatic",
            const Axis &_axis = Axis::UnitX());
      };

      public: template <typename PolicyT>
      class Implementation : public virtual Feature::Implementation<PolicyT>
      {
        public: using Axis =
            typename FromPolicy<PolicyT>::template Use<LinearVector>;

        /// \param[in] _childID
        ///   The ID of the child link.
        /// \param[in] _parent
        ///   A reference to the parent link. If this evaluates to a nullptr,
        ///   then the parent should be the world.
        /// \param[in] _name
        ///   The name for this joint.
        /// \param[in] _axis
        ///   The desired axis of the new revolute joint
        /// \returns the Identity of the newly created PrismaticJoint
        public: virtual Identity AttachPrismaticJoint(
            const Identity &_childID,
            const BaseLinkPtr<PolicyT> &_parent,
            const std::string &_name,
            const Axis &_axis) = 0;
      };
    };
  }
}

#include <gz/physics/detail/PrismaticJoint.hh>

#endif
