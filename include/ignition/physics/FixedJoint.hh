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

#ifndef IGNITION_PHYSICS_FIXEDJOINT_HH_
#define IGNITION_PHYSICS_FIXEDJOINT_HH_

#include <ignition/physics/DeclareJointType.hh>

namespace ignition
{
  namespace physics
  {
    IGN_PHYSICS_DECLARE_JOINT_TYPE(FixedJoint)

    class IGNITION_PHYSICS_VISIBLE AttachFixedJointFeature
        : public virtual FeatureWithRequirements<FixedJointCast>
    {
      public: template <typename PolicyT, typename FeaturesT>
      class Link : public virtual Entity<PolicyT, FeaturesT>
      {
        public: using JointPtrType = FixedJointPtr<PolicyT, FeaturesT>;

        /// \brief Attach this link to another link using a FixedJoint.
        /// \param[in] _parent
        ///   The parent link for the joint. Pass in a nullptr to attach the
        ///   link to the world.
        /// \param[in] _name
        ///   The name for this joint.
        /// \return A reference to the newly constructed FixedJoint. You can use
        /// the SetJointTransformFromParentFeature and
        /// SetJointTransformToChildFeature on this JointPtr to set the relative
        /// transform between the parent and child if your physics engine offers
        /// those features.
        public: JointPtrType AttachFixedJoint(
            const BaseLinkPtr<PolicyT> &_parent,
            const std::string &_name = "fixed");
      };

      public: template <typename PolicyT>
      class Implementation : public virtual Feature::Implementation<PolicyT>
      {
        public: virtual Identity AttachFixedJoint(
            std::size_t _childID,
            const BaseLinkPtr<PolicyT> &_parent,
            const std::string &_name) = 0;
      };
    };
  }
}

#include <ignition/physics/detail/FixedJoint.hh>

#endif
