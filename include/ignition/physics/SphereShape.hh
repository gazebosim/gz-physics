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

#ifndef IGNITION_PHYSICS_SPHERESHAPE_HH_
#define IGNITION_PHYSICS_SPHERESHAPE_HH_

#include <ignition/physics/DeclareShapeType.hh>
#include <ignition/physics/Geometry.hh>

namespace ignition
{
  namespace physics
  {
    IGN_PHYSICS_DECLARE_SHAPE_TYPE(SphereShape)

    /////////////////////////////////////////////////
    class IGNITION_PHYSICS_VISIBLE GetSphereShapeProperties
        : public virtual FeatureWithRequirements<SphereShapeCast>
    {
      public: template <typename PolicyT, typename FeaturesT>
      class SphereShape : public virtual Entity<PolicyT, FeaturesT>
      {
        public: using Scalar = typename PolicyT::Scalar;

        /// \brief Get the radius of this SphereShape
        /// \return the radius of this SphereShape
        public: Scalar GetRadius() const;
      };

      public: template <typename PolicyT>
      class Implementation : public virtual Feature::Implementation<PolicyT>
      {
        public: using Scalar = typename PolicyT::Scalar;

        public: virtual Scalar GetSphereShapeRadius(
            std::size_t _sphereID) const = 0;
      };
    };

    /////////////////////////////////////////////////
    class IGNITION_PHYSICS_VISIBLE SetSphereShapeProperties
        : public virtual FeatureWithRequirements<SphereShapeCast>
    {
      public: template <typename PolicyT, typename FeaturesT>
      class SphereShape : public virtual Entity<PolicyT, FeaturesT>
      {
        public: using Scalar = typename PolicyT::Scalar;

        /// \brief Set the radius of this SphereShape
        /// \param[in] _value
        ///   The desired radius of this SphereShape
        public: void SetRadius(Scalar _radius);
      };

      public: template <typename PolicyT>
      class Implementation : public virtual Feature::Implementation<PolicyT>
      {
        public: using Scalar = typename PolicyT::Scalar;

        public: virtual void SetSphereShapeRadius(
            std::size_t _sphereID, Scalar _radius) = 0;
      };
    };

    /////////////////////////////////////////////////
    class IGNITION_PHYSICS_VISIBLE AttachSphereShapeFeature
        : public virtual FeatureWithRequirements<SphereShapeCast>
    {
      public: template <typename PolicyT, typename FeaturesT>
      class Link : public virtual Feature::Link<PolicyT, FeaturesT>
      {
        public: using Scalar = typename PolicyT::Scalar;

        public: using PoseType =
            typename FromPolicy<PolicyT>::template Use<Pose>;

        public: using ShapePtrType = SphereShapePtr<PolicyT, FeaturesT>;

        public: ShapePtrType AttachSphereShape(
            const std::string &_name,
            Scalar _radius = 1.0,
            const PoseType &_pose = PoseType::Identity());
      };

      public: template <typename PolicyT>
      class Implementation : public virtual Feature::Implementation<PolicyT>
      {
        public: using Scalar = typename PolicyT::Scalar;

        public: using PoseType =
            typename FromPolicy<PolicyT>::template Use<Pose>;

        public: virtual Identity AttachSphereShape(
            std::size_t _linkID,
            const std::string &_name,
            Scalar _radius,
            const PoseType &_pose) = 0;
      };
    };
  }
}

#include <ignition/physics/detail/SphereShape.hh>

#endif
