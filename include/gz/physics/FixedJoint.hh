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

#ifndef GZ_PHYSICS_FIXEDJOINT_HH_
#define GZ_PHYSICS_FIXEDJOINT_HH_

#include <string>

#include <gz/physics/DeclareJointType.hh>

namespace gz
{
  namespace physics
  {
    GZ_PHYSICS_DECLARE_JOINT_TYPE(FixedJoint)

    class GZ_PHYSICS_VISIBLE AttachFixedJointFeature
        : public virtual FeatureWithRequirements<FixedJointCast>
    {
      public: template <typename PolicyT, typename FeaturesT>
      class Link : public virtual Feature::Link<PolicyT, FeaturesT>
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
            const Identity &_childID,
            const BaseLinkPtr<PolicyT> &_parent,
            const std::string &_name) = 0;
      };
    };

    /////////////////////////////////////////////////
    /// \brief This feature sets dynamic constraint properties.
    class GZ_PHYSICS_VISIBLE SetDynamicJointConstraintPropertiesFeature
        : public virtual FeatureWithRequirements<FixedJointCast>
    {
      /// \brief Set dynamic joint constraint properties.
      public: template <typename PolicyT, typename FeaturesT>
      class Link : public virtual Feature::Link<PolicyT, FeaturesT>
      {
        public: using Scalar = typename PolicyT::Scalar;

        /// \brief Set the global constraint force mixing parameter.
        /// \param[in] _value
        ///   The desired constraint force mixing parameter.
        public: void SetConstraintForceMixing(const Scalar _value);

        /// \brief Set the global error reduction parameter.
        /// \param[in] _value
        ///   The desired global error reduction parameter.
        public: void SetErrorReductionParameter(const Scalar _value);
      };

      /// \private The implementation API to set dynamic joint constraint
      /// properties.
      public: template <typename PolicyT>
      class Implementation : public virtual Feature::Implementation<PolicyT>
      {
        public: using Scalar = typename PolicyT::Scalar;

        // see Link::SetConstraintForceMixing above
        public: virtual void SetConstraintForceMixing(
            const Identity &_childID, Scalar _value) = 0;

        // See Link::SetErrorReductionParameter above
        public: virtual void SetErrorReductionParameter(
            const Identity &_childID, Scalar _value) = 0;
      };
    };
  }
}

#include <gz/physics/detail/FixedJoint.hh>

#endif
