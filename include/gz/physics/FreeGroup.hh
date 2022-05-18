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

#ifndef GZ_PHYSICS_FREEGROUP_HH_
#define GZ_PHYSICS_FREEGROUP_HH_

#include <gz/physics/FeatureList.hh>
#include <gz/physics/FrameSemantics.hh>
#include <gz/physics/Geometry.hh>

namespace gz
{
  namespace physics
  {
    DETAIL_IGN_PHYSICS_DEFINE_ENTITY(FreeGroup)

    /////////////////////////////////////////////////
    /// \brief This feature provides an interface between the Model and Link
    /// classes and the FreeGroup class, which represents a group of links
    /// that are not connected to the world with any kinematic constraints.
    /// A FreeGroup can represent a single connected group of links that
    /// forms a tree with the root of the tree connected to the world with
    /// a FreeJoint, but it can also represent a group of other FreeGroups.
    /// Each FreeGroup has 1 canonical link, whose frame is used for getting
    /// and setting properties like pose and velocity.
    /// If the FreeGroup is a single tree of connected links, the canonical
    /// link should be the root of that tree.
    /// If the FreeGroup contains multiple FreeGroups, the canonical link
    /// should be selected from one of the component FreeGroups.
    class IGNITION_PHYSICS_VISIBLE FindFreeGroupFeature : public virtual Feature
    {
      public: template <typename PolicyT, typename FeaturesT>
      class Model : public virtual Feature::Model<PolicyT, FeaturesT>
      {
        using FreeGroupPtrType = FreeGroupPtr<PolicyT, FeaturesT>;
        using ConstFreeGroupPtrType = ConstFreeGroupPtr<PolicyT, FeaturesT>;

        /// \brief Find a FreeGroup that includes all the links in this model.
        /// \return a FreeGroup that envelops all links in the model if such a
        /// group is available. Otherwise a nullptr is returned.
        public: FreeGroupPtrType FindFreeGroup();

        /// \brief const version of FindFreeGroup()
        public: ConstFreeGroupPtrType FindFreeGroup() const;
      };

      public: template <typename PolicyT, typename FeaturesT>
      class Link : public virtual Feature::Link<PolicyT, FeaturesT>
      {
        using FreeGroupPtrType = FreeGroupPtr<PolicyT, FeaturesT>;
        using ConstFreeGroupPtrType = ConstFreeGroupPtr<PolicyT, FeaturesT>;

        /// \brief Find the smallest FreeGroup that includes this Link.
        /// \return a FreeGroup that includes this link and any connected links.
        /// If this link is constrained to the world in some way, then a nullptr
        /// is returned.
        public: FreeGroupPtrType FindFreeGroup();

        /// \brief const version of FindFreeGroup()
        public: ConstFreeGroupPtrType FindFreeGroup() const;
      };

      public: template <typename PolicyT, typename FeaturesT>
      class FreeGroup : public virtual Entity<PolicyT, FeaturesT>
      {
        /// \brief The root link of this FreeGroup. This link is the root of one
        /// of the kinematic trees represented by this FreeGroup. Getting and
        /// setting properties (like poses and velocities) on the group will be
        /// done in terms of this link.
        public: LinkPtr<PolicyT, FeaturesT> RootLink();

        /// \brief const version of RootLink()
        public: ConstLinkPtr<PolicyT, FeaturesT> RootLink() const;

        /// \brief The root link of this FreeGroup. This link is the root of one
        /// of the kinematic trees represented by this FreeGroup. Getting and
        /// setting properties (like poses and velocities) on the group will be
        /// done in terms of this link.
        /// DEPRECATED. Please use RootLink()
        public: LinkPtr<PolicyT, FeaturesT> IGN_DEPRECATED(4.0) CanonicalLink();

        /// \brief const version of CanonicalLink()
        /// DEPRECATED. Please use RootLink()
        public: ConstLinkPtr<PolicyT, FeaturesT> IGN_DEPRECATED(4.0)
                    CanonicalLink() const;
      };

      public: template <typename PolicyT>
      class Implementation : public virtual Feature::Implementation<PolicyT>
      {
        public: virtual Identity FindFreeGroupForModel(
            const Identity &_modelID) const = 0;

        public: virtual Identity FindFreeGroupForLink(
            const Identity &_linkID) const = 0;

        public: virtual Identity GetFreeGroupRootLink(
            const Identity &_groupID) const = 0;
      };
    };

    /////////////////////////////////////////////////
    class IGNITION_PHYSICS_VISIBLE FreeGroupFrameSemantics
        : public virtual FeatureWithRequirements<
        FindFreeGroupFeature, FrameSemantics>
    {
      public: template <typename PolicyT, typename FeaturesT>
      using FreeGroup = FrameSemantics::Frame<PolicyT, FeaturesT>;
    };

    /////////////////////////////////////////////////
    /// \brief This features sets the FreeGroup pose in world frame. However,
    /// while a physics engine with maximal coordinates can provide
    /// Link::SetWorldPose and similar functions for setting velocity
    /// regardless of the kinematic constraints on that link, this behavior
    /// for FreeGroup is not well defined and difficult to implement
    /// with generalized coordinates. The FreeGroup::SetWorldPose function
    /// should provide an analog to both Link::SetWorldPose and
    /// Model::SetWorldPose.
    class IGNITION_PHYSICS_VISIBLE SetFreeGroupWorldPose
        : public virtual FeatureWithRequirements<FindFreeGroupFeature>
    {
      /// \brief This class defines the FreeGroup concept, which represents a
      /// group of links that are not connected to the world with any kinematic
      /// constraints. This class also provides a rough definition of this
      /// FreeGroup pose in world frame. See FindFreeGroupFeature class
      /// documentation for more detail.
      public: template <typename PolicyT, typename FeaturesT>
      class FreeGroup : public virtual Entity<PolicyT, FeaturesT>
      {
        public: using PoseType =
            typename FromPolicy<PolicyT>::template Use<Pose>;

        /// \brief Set this FreeGroup pose in world frame.
        public: void SetWorldPose(const PoseType &_pose);
      };

      public: template <typename PolicyT>
      class Implementation : public virtual Feature::Implementation<PolicyT>
      {
        public: using PoseType =
            typename FromPolicy<PolicyT>::template Use<Pose>;

        public: virtual void SetFreeGroupWorldPose(
            const Identity &_groupID,
            const PoseType &_pose) = 0;
      };
    };

    /////////////////////////////////////////////////
    /// \brief This features sets the FreeGroup linear and angular velocity in
    /// world frame.
    class IGNITION_PHYSICS_VISIBLE SetFreeGroupWorldVelocity
        : public virtual FeatureWithRequirements<FindFreeGroupFeature>
    {
      /// \brief This class defines the FreeGroup concept, which represents a
      /// group of links that are not connected to the world with any kinematic
      /// constraints. This class also provides a rough definition of this
      /// FreeGroup linear and angular velocity in world frame. See
      /// FindFreeGroupFeature class documentation for more detail.
      public: template <typename PolicyT, typename FeaturesT>
      class FreeGroup : public virtual Entity<PolicyT, FeaturesT>
      {
        public: using LinearVelocity =
            typename FromPolicy<PolicyT>::template Use<LinearVector>;

        public: using AngularVelocity =
            typename FromPolicy<PolicyT>::template Use<AngularVector>;

        /// \brief Set this FreeGroup linear velocity in world frame.
        public: void SetWorldLinearVelocity(
            const LinearVelocity &_linearVelocity);

        /// \brief Set this FreeGroup angular velocity in world frame.
        public: void SetWorldAngularVelocity(
            const AngularVelocity &_angularVelocity);
      };

      public: template <typename PolicyT>
      class Implementation : public virtual Feature::Implementation<PolicyT>
      {
        public: using LinearVelocity =
            typename FromPolicy<PolicyT>::template Use<LinearVector>;

        public: using AngularVelocity =
            typename FromPolicy<PolicyT>::template Use<AngularVector>;

        public: virtual void SetFreeGroupWorldLinearVelocity(
            const Identity &_groupID,
            const LinearVelocity &_linearVelocity) = 0;

        public: virtual void SetFreeGroupWorldAngularVelocity(
            const Identity &_groupID,
            const AngularVelocity &_angularVelocity) = 0;
      };
    };
  }
}

#include <gz/physics/detail/FreeGroup.hh>

#endif
