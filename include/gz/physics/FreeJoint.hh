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

#ifndef GZ_PHYSICS_FREEJOINT_HH_
#define GZ_PHYSICS_FREEJOINT_HH_

#include <ignition/physics/DeclareJointType.hh>
#include <ignition/physics/Geometry.hh>

namespace gz
{
  namespace physics
  {
    IGN_PHYSICS_DECLARE_JOINT_TYPE(FreeJoint)

    class IGNITION_PHYSICS_VISIBLE SetFreeJointRelativeTransformFeature
        : public virtual FeatureWithRequirements<FreeJointCast>
    {
      public: template <typename PolicyT, typename FeaturesT>
      class FreeJoint : public virtual Feature::Link<PolicyT, FeaturesT>
      {
        public: using PoseType =
            typename FromPolicy<PolicyT>::template Use<Pose>;

        /// \brief Set the transform from the joint's parent link to its child
        /// link by changing the generalized positions of this joint.
        /// \param[in] _pose
        ///   The desired transformation matrix from the joint's parent
        public: void SetRelativeTransform(const PoseType &_pose);
      };

      /// \private The implementation API for setting a free joint transform
      public: template <typename PolicyT>
      class Implementation : public virtual Feature::Implementation<PolicyT>
      {
        public: using PoseType =
            typename FromPolicy<PolicyT>::template Use<Pose>;

        public: virtual void SetFreeJointRelativeTransform(
            const Identity &_id, const PoseType &_pose) = 0;
      };
    };
  }
}

#include <gz/physics/detail/FreeJoint.hh>

#endif
