/*
 * Copyright 2024 CogniPilot Foundation
 * Copyright 2024 Open Source Robotics Foundation
 * Copyright 2024 Rudis Laboratories
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

#ifndef GZ_PHYSICS_CONESHAPE_HH_
#define GZ_PHYSICS_CONESHAPE_HH_

#include <string>

#include <gz/physics/DeclareShapeType.hh>
#include <gz/physics/Geometry.hh>

namespace gz
{
  namespace physics
  {
    GZ_PHYSICS_DECLARE_SHAPE_TYPE(ConeShape)

    class GZ_PHYSICS_VISIBLE GetConeShapeProperties
        : public virtual FeatureWithRequirements<ConeShapeCast>
    {
      public: template <typename PolicyT, typename FeaturesT>
      class ConeShape : public virtual Entity<PolicyT, FeaturesT>
      {
        public: using Scalar = typename PolicyT::Scalar;

        /// \brief Get the radius of this ConeShape
        /// \return the radius of this ConeShape
        public: Scalar GetRadius() const;

        /// \brief Get the height (length along the local z-axis) of this
        /// ConeShape.
        /// \return the height of this ConeShape
        public: Scalar GetHeight() const;
      };

      public: template <typename PolicyT>
      class Implementation : public virtual Feature::Implementation<PolicyT>
      {
        public: using Scalar = typename PolicyT::Scalar;

        public: virtual Scalar GetConeShapeRadius(
            const Identity &_coneID) const = 0;

        public: virtual Scalar GetConeShapeHeight(
            const Identity &_coneID) const = 0;
      };
    };

    /////////////////////////////////////////////////
    /// \brief This feature sets the ConeShape properties such as
    /// the cone radius and height.
    class GZ_PHYSICS_VISIBLE SetConeShapeProperties
        : public virtual FeatureWithRequirements<ConeShapeCast>
    {
      public: template <typename PolicyT, typename FeaturesT>
      class ConeShape : public virtual Entity<PolicyT, FeaturesT>
      {
        public: using Scalar = typename PolicyT::Scalar;

        /// \brief Set the radius of this ConeShape
        /// \param[in] _radius
        ///   The desired radius of this ConeShape
        public: void SetRadius(Scalar _radius);

        /// \brief Set the height of this ConeShape
        /// \param[in] _height
        ///   The desired height of this ConeShape
        public: void SetHeight(Scalar _height);
      };

      public: template <typename PolicyT>
      class Implementation : public virtual Feature::Implementation<PolicyT>
      {
        public: using Scalar = typename PolicyT::Scalar;

        public: virtual void SetConeShapeRadius(
            const Identity &_coneID, Scalar _radius) = 0;

        public: virtual void SetConeShapeHeight(
            const Identity &_coneID, Scalar _height) = 0;
      };
    };

    /////////////////////////////////////////////////
    /// \brief This feature constructs a new cone shape and attaches the
    /// desired pose in the link frame. The pose could be defined to be the
    /// cone center point in actual implementation.
    class GZ_PHYSICS_VISIBLE AttachConeShapeFeature
        : public virtual FeatureWithRequirements<ConeShapeCast>
    {
      public: template <typename PolicyT, typename FeaturesT>
      class Link : public virtual Feature::Link<PolicyT, FeaturesT>
      {
        public: using Scalar = typename PolicyT::Scalar;

        public: using PoseType =
            typename FromPolicy<PolicyT>::template Use<Pose>;

        public: using ShapePtrType = ConeShapePtr<PolicyT, FeaturesT>;

        /// \brief Rigidly attach a ConeShape to this link.
        /// \param[in] _name
        /// \param[in] _radius
        ///   The radius of the cone.
        /// \param[in] _height
        ///   The height of the cone.
        /// \param[in] _pose
        ///   The desired pose of the ConeShape relative to the Link frame.
        /// \returns a ShapePtrType to the newly constructed ConeShape
        public: ShapePtrType AttachConeShape(
            const std::string &_name = "cone",
            Scalar _radius = 1.0,
            Scalar _height = 1.0,
            const PoseType &_pose = PoseType::Identity());
      };

      public: template <typename PolicyT>
      class Implementation : public virtual Feature::Implementation<PolicyT>
      {
        public: using Scalar = typename PolicyT::Scalar;

        public: using PoseType =
            typename FromPolicy<PolicyT>::template Use<Pose>;

        public: virtual Identity AttachConeShape(
            const Identity &_linkID,
            const std::string &_name,
            Scalar _radius,
            Scalar _height,
            const PoseType &_pose) = 0;
      };
    };
  }
}

#include <gz/physics/detail/ConeShape.hh>

#endif
