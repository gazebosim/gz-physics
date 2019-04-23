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

#ifndef IGNITION_PHYSICS_FREEGROUP_HH_
#define IGNITION_PHYSICS_FREEGROUP_HH_

#include <ignition/physics/FeatureList.hh>
#include <ignition/physics/FrameSemantics.hh>
#include <ignition/physics/Geometry.hh>

namespace ignition
{
  namespace physics
  {
    DETAIL_IGN_PHYSICS_DEFINE_ENTITY(FreeGroup)

    /////////////////////////////////////////////////
    class IGNITION_PHYSICS_VISIBLE FindFreeGroupFeature : public virtual Feature
    {
      public: template <typename PolicyT, typename FeaturesT>
      class Model : public virtual Feature::Model<PolicyT, FeaturesT>
      {
        using FreeGroupPtrType = FreeGroupPtr<PolicyT, FeaturesT>;
        using ConstFreeGroupPtrType = ConstFreeGroupPtr<PolicyT, FeaturesT>;

        /// \brief Find a FreeGroup that envelops the entire model.
        /// \returns a FreeGroup that envelops all links in the model if such a
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

        /// \brief Find a FreeGroup that includes this Link.
        /// \returns a FreeGroup that includes this link at a minimum. If this
        /// link is constrained to the world in some way, then a nullptr is
        /// returned.
        public: FreeGroupPtrType FindFreeGroup();

        /// \brief const version of FindFreeGroup()
        public: ConstFreeGroupPtrType FindFreeGroup() const;
      };

      public: template <typename PolicyT, typename FeaturesT>
      class FreeGroup : public virtual Feature::Link<PolicyT, FeaturesT>
      {
        /// \brief The canonical link of this FreeGroup. Getting and setting
        /// properties (like poses and velocities) on the group will be done
        /// in terms of this link.
        public: LinkPtr<PolicyT, FeaturesT> CanonicalLink();

        /// \brief const version of CanonicalLink()
        public: ConstLinkPtr<PolicyT, FeaturesT> CanonicalLink() const;
      };

      public: template <typename PolicyT>
      class Implementation : public virtual Feature::Implementation<PolicyT>
      {
        public: virtual Identity FindFreeGroupForModel(
            const Identity &_modelID) const = 0;

        public: virtual Identity FindFreeGroupForLink(
            const Identity &_linkID) const = 0;

        public: virtual Identity GetFreeGroupCanonicalLink(
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
    class IGNITION_PHYSICS_VISIBLE SetFreeGroupPose
        : public virtual FeatureWithRequirements<FindFreeGroupFeature>
    {
      public: template <typename PolicyT, typename FeaturesT>
      class FreeGroup : public virtual Entity<PolicyT, FeaturesT>
      {
        public: using PoseType =
            typename FromPolicy<PolicyT>::template Use<Pose>;

        public: void SetWorldPose(const PoseType &_pose);
      };

      public: template <typename PolicyT>
      class Implementation : public virtual Feature::Implementation<PolicyT>
      {
        public: using PoseType =
            typename FromPolicy<PolicyT>::template Use<Pose>;

        public: virtual void SetFreeGroupWorldPose(
            const Identity &_groupID,
            const PoseType &_pose);
      };
    };

    /////////////////////////////////////////////////
    class IGNITION_PHYSICS_VISIBLE SetFreeGroupVelocity
        : public virtual FeatureWithRequirements<FindFreeGroupFeature>
    {
      public: template <typename PolicyT, typename FeaturesT>
      class FreeGroup : public virtual Entity<PolicyT, FeaturesT>
      {
        public: using LinearVelocity =
            typename FromPolicy<PolicyT>::template Use<LinearVector>;

        public: using AngularVelocity =
            typename FromPolicy<PolicyT>::template Use<AngularVector>;

        public: void SetWorldLinearVelocity(
            const LinearVelocity &_linearVelocity);

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

#include <ignition/physics/detail/FreeGroup.hh>

#endif
