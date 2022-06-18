/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#ifndef GZ_PHYSICS_CAPSULESHAPE_HH_
#define GZ_PHYSICS_CAPSULESHAPE_HH_

#include <string>

#include <gz/physics/DeclareShapeType.hh>
#include <gz/physics/Geometry.hh>

namespace gz
{
  namespace physics
  {
    GZ_PHYSICS_DECLARE_SHAPE_TYPE(CapsuleShape)

    class GZ_PHYSICS_VISIBLE GetCapsuleShapeProperties
        : public virtual FeatureWithRequirements<CapsuleShapeCast>
    {
      public: template <typename PolicyT, typename FeaturesT>
      class CapsuleShape : public virtual Entity<PolicyT, FeaturesT>
      {
        public: using Scalar = typename PolicyT::Scalar;

        /// \brief Get the radius of this CapsuleShape
        /// \return the radius of this CapsuleShape
        public: Scalar GetRadius() const;

        /// \brief Get the length along the local z-axis of this
        /// CapsuleShape's cylinder.
        /// \return the length of this CapsuleShape
        public: Scalar GetLength() const;
      };

      public: template <typename PolicyT>
      class Implementation : public virtual Feature::Implementation<PolicyT>
      {
        public: using Scalar = typename PolicyT::Scalar;

        public: virtual Scalar GetCapsuleShapeRadius(
            const Identity &_capsuleID) const = 0;

        public: virtual Scalar GetCapsuleShapeLength(
            const Identity &_capsuleID) const = 0;
      };
    };

    /////////////////////////////////////////////////
    /// \brief This feature sets the CapsuleShape properties such as
    /// the capsule radius and length.
    class GZ_PHYSICS_VISIBLE SetCapsuleShapeProperties
        : public virtual FeatureWithRequirements<CapsuleShapeCast>
    {
      public: template <typename PolicyT, typename FeaturesT>
      class CapsuleShape : public virtual Entity<PolicyT, FeaturesT>
      {
        public: using Scalar = typename PolicyT::Scalar;

        /// \brief Set the radius of this CapsuleShape
        /// \param[in] _radius
        ///   The desired radius of this CapsuleShape
        public: void SetRadius(Scalar _radius);

        /// \brief Set the length of this CapsuleShape
        /// \param[in] _length
        ///   The desired length of this CapsuleShape
        public: void SetLength(Scalar length);
      };

      public: template <typename PolicyT>
      class Implementation : public virtual Feature::Implementation<PolicyT>
      {
        public: using Scalar = typename PolicyT::Scalar;

        public: virtual void SetCapsuleShapeRadius(
            const Identity &_capsuleID, Scalar _radius) = 0;

        public: virtual void SetCapsuleShapeLength(
            const Identity &_capsuleID, Scalar _length) = 0;
      };
    };

    /////////////////////////////////////////////////
    /// \brief This feature constructs a new capsule shape and attaches the
    /// desired pose in the link frame. The pose could be defined to be the
    /// capsule center point in actual implementation.
    class GZ_PHYSICS_VISIBLE AttachCapsuleShapeFeature
        : public virtual FeatureWithRequirements<CapsuleShapeCast>
    {
      public: template <typename PolicyT, typename FeaturesT>
      class Link : public virtual Feature::Link<PolicyT, FeaturesT>
      {
        public: using Scalar = typename PolicyT::Scalar;

        public: using PoseType =
            typename FromPolicy<PolicyT>::template Use<Pose>;

        public: using ShapePtrType = CapsuleShapePtr<PolicyT, FeaturesT>;

        /// \brief Rigidly attach a CapsuleShape to this link.
        /// \param[in] _radius
        ///   The radius of the capsule.
        /// \param[in] _length
        ///   The length of the capsule.
        /// \param[in] _pose
        ///   The desired pose of the CapsuleShape relative to the Link frame.
        /// \returns a ShapePtrType to the newly constructed CapsuleShape
        public: ShapePtrType AttachCapsuleShape(
            const std::string &_name = "capsule",
            Scalar _radius = 0.5,
            Scalar _length = 1.0,
            const PoseType &_pose = PoseType::Identity());
      };

      public: template <typename PolicyT>
      class Implementation : public virtual Feature::Implementation<PolicyT>
      {
        public: using Scalar = typename PolicyT::Scalar;

        public: using PoseType =
            typename FromPolicy<PolicyT>::template Use<Pose>;

        public: virtual Identity AttachCapsuleShape(
            const Identity &_linkID,
            const std::string &_name,
            Scalar _radius,
            Scalar _length,
            const PoseType &_pose) = 0;
      };
    };
  }
}

#include <gz/physics/detail/CapsuleShape.hh>

#endif
