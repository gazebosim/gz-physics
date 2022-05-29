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

#ifndef GZ_PHYSICS_SPHERESHAPE_HH_
#define GZ_PHYSICS_SPHERESHAPE_HH_

#include <string>

#include <gz/physics/DeclareShapeType.hh>
#include <gz/physics/Geometry.hh>

namespace gz
{
  namespace physics
  {
    IGN_PHYSICS_DECLARE_SHAPE_TYPE(SphereShape)

    /////////////////////////////////////////////////
    class GZ_PHYSICS_VISIBLE GetSphereShapeProperties
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
            const Identity &_sphereID) const = 0;
      };
    };

    /////////////////////////////////////////////////
    class GZ_PHYSICS_VISIBLE SetSphereShapeProperties
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
            const Identity &_sphereID, Scalar _radius) = 0;
      };
    };

    /////////////////////////////////////////////////
    /// \brief This feature constructs a new sphere shape and attaches the
    /// desired pose in the link frame. The pose is defined as the
    /// sphere center point in actual implementation.
    class GZ_PHYSICS_VISIBLE AttachSphereShapeFeature
        : public virtual FeatureWithRequirements<SphereShapeCast>
    {
      public: template <typename PolicyT, typename FeaturesT>
      class Link : public virtual Feature::Link<PolicyT, FeaturesT>
      {
        public: using Scalar = typename PolicyT::Scalar;

        public: using PoseType =
            typename FromPolicy<PolicyT>::template Use<Pose>;

        public: using ShapePtrType = SphereShapePtr<PolicyT, FeaturesT>;

        /// \brief Rigidly attach a SphereShape to this link.
        /// \param[in] _radius
        ///   The radius of the sphere.
        /// \param[in] _pose
        ///   The desired pose of the SphereShape relative to the Link frame.
        /// \returns a ShapePtrType to the newly constructed SphereShape
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
            const Identity &_linkID,
            const std::string &_name,
            Scalar _radius,
            const PoseType &_pose) = 0;
      };
    };
  }
}

#include <gz/physics/detail/SphereShape.hh>

#endif
