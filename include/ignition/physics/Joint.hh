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

#ifndef IGNITION_PHYSICS_JOINT_HH_
#define IGNITION_PHYSICS_JOINT_HH_

#include <ignition/physics/FeatureList.hh>
#include <ignition/physics/Geometry.hh>

namespace ignition
{
  namespace physics
  {
    class IGNITION_PHYSICS_VISIBLE GetBasicJointProperties
        : public virtual Feature
    {
      public: template <typename PolicyT, typename FeaturesT>
      class Joint : public virtual Feature::Joint<PolicyT, FeaturesT>
      {
        public: using Pose = typename FromPolicy<PolicyT>::template Use<Pose>;

        public: Pose GetTransformFromParent() const
        {
          return this->template Interface<GetBasicJointProperties>()
              ->GetJointTransformFromParent(this->identity);
        }

        public: Pose GetTransformToChild() const
        {
          return this->template Interface<GetBasicJointProperties>()
              ->GetJointTransformToChild(this->identity);
        }
      };

      public: template <typename PolicyT>
      class Implementation : public virtual Feature::Implementation<PolicyT>
      {
        public: using Pose = typename FromPolicy<PolicyT>::template Use<Pose>;

        public: virtual Pose GetJointTransformFromParent(
            std::size_t _id) const = 0;

        public: virtual Pose GetJointTransformToChild(
            std::size_t _id) const = 0;
      };
    };

    class IGNITION_PHYSICS_VISIBLE SetJointTransformFromParentFeature
        : public virtual Feature
    {
      public: template <typename PolicyT, typename FeaturesT>
      class Joint : public virtual Feature::Joint<PolicyT, FeaturesT>
      {
        public: using Pose = typename FromPolicy<PolicyT>::template Use<Pose>;

        public: void SetTransformFromParent(const Pose &_pose)
        {
          this->template Interface<SetJointTransformFromParent>()
            ->SetJointTransformFromParent(this->identity, _pose);
        }
      };

      public: template <typename PolicyT>
      class Implementation : public virtual Feature::Implementation<PolicyT>
      {
        public: using Pose = typename FromPolicy<PolicyT>::template Use<Pose>;

        public: virtual void SetJointTransformFromParent(
            std::size_t _id, const Pose &_pose) = 0;
      };
    };

    class IGNITION_PHYSICS_VISIBLE SetJointTransformToChildFeature
        : public virtual Feature
    {
      public: template <typename PolicyT, typename FeaturesT>
      class Joint : public virtual Feature::Joint<PolicyT, FeaturesT>
      {
        public: using Pose = typename FromPolicy<PolicyT>::template Use<Pose>;

        public: void SetTransformToChild(const Pose &_pose)
        {
          this->template Interface<SetJointTransformToChild>()
            ->SetJointTransformToChild(this->identity, _pose);
        }
      };

      public: template <typename PolicyT>
      class Implementation : public virtual Feature::Implementation<PolicyT>
      {
        public: using Pose = typename FromPolicy<PolicyT>::template Use<Pose>;

        public: virtual void SetJointTransformToChild(
            std::size_t _id, const Pose &_pose) = 0;
      };
    };
  }
}

#endif
