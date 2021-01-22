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

#ifndef IGNITION_PHYSICS_ELLIPSOIDSHAPE_HH_
#define IGNITION_PHYSICS_ELLIPSOIDSHAPE_HH_

#include <string>

#include <ignition/physics/DeclareShapeType.hh>
#include <ignition/physics/Geometry.hh>

namespace ignition
{
  namespace physics
  {
    IGN_PHYSICS_DECLARE_SHAPE_TYPE(EllipsoidShape)

    class IGNITION_PHYSICS_VISIBLE GetEllipsoidShapeProperties
        : public virtual FeatureWithRequirements<EllipsoidShapeCast>
    {
      public: template <typename PolicyT, typename FeaturesT>
      class EllipsoidShape : public virtual Entity<PolicyT, FeaturesT>
      {
        public: using Dimensions =
            typename FromPolicy<PolicyT>::template Use<LinearVector>;

        /// \brief Get the radius of this EllipsoidShape
        /// \return the radius of this EllipsoidShape
        public: Dimensions GetRadii() const;
      };

      public: template <typename PolicyT>
      class Implementation : public virtual Feature::Implementation<PolicyT>
      {
        public: using Dimensions =
            typename FromPolicy<PolicyT>::template Use<LinearVector>;

        public: virtual Dimensions GetEllipsoidShapeRadii(
            const Identity &_ellipsoidID) const = 0;
      };
    };

    /////////////////////////////////////////////////
    /// \brief This feature sets the EllipsoidShape properties such as
    /// the ellipsoid radius and height.
    class IGNITION_PHYSICS_VISIBLE SetEllipsoidShapeProperties
        : public virtual FeatureWithRequirements<EllipsoidShapeCast>
    {
      public: template <typename PolicyT, typename FeaturesT>
      class EllipsoidShape : public virtual Entity<PolicyT, FeaturesT>
      {
        public: using Dimensions =
            typename FromPolicy<PolicyT>::template Use<LinearVector>;

        /// \brief Set the radius of this EllipsoidShape
        /// \param[in] _radii
        ///   The desired radius of this EllipsoidShape
        public: void SetRadii(Dimensions _radii);
      };

      public: template <typename PolicyT>
      class Implementation : public virtual Feature::Implementation<PolicyT>
      {
        public: using Dimensions =
            typename FromPolicy<PolicyT>::template Use<LinearVector>;

        public: virtual void SetEllipsoidShapeRadii(
            const Identity &_ellipsoidID, Dimensions _radii) = 0;
      };
    };

    /////////////////////////////////////////////////
    /// \brief This feature constructs a new ellipsoid shape and attaches the
    /// desired pose in the link frame. The pose could be defined to be the
    /// ellipsoid center point in actual implementation.
    class IGNITION_PHYSICS_VISIBLE AttachEllipsoidShapeFeature
        : public virtual FeatureWithRequirements<EllipsoidShapeCast>
    {
      public: template <typename PolicyT, typename FeaturesT>
      class Link : public virtual Feature::Link<PolicyT, FeaturesT>
      {
        public: using Dimensions =
            typename FromPolicy<PolicyT>::template Use<LinearVector>;

        public: using PoseType =
            typename FromPolicy<PolicyT>::template Use<Pose>;

        public: using ShapePtrType = EllipsoidShapePtr<PolicyT, FeaturesT>;

        /// \brief Rigidly attach a EllipsoidShape to this link.
        /// \param[in] _radii
        ///   The radius of the ellipsoid.
        /// \param[in] _pose
        ///   The desired pose of the EllipsoidShape relative to the Link frame.
        /// \returns a ShapePtrType to the newly constructed EllipsoidShape
        public: ShapePtrType AttachEllipsoidShape(
            const std::string &_name = "ellipsoid",
            Dimensions _radii = 1.0,
            const PoseType &_pose = PoseType::Identity());
      };

      public: template <typename PolicyT>
      class Implementation : public virtual Feature::Implementation<PolicyT>
      {
        public: using Dimensions =
            typename FromPolicy<PolicyT>::template Use<LinearVector>;

        public: using PoseType =
            typename FromPolicy<PolicyT>::template Use<Pose>;

        public: virtual Identity AttachEllipsoidShape(
            const Identity &_linkID,
            const std::string &_name,
            Dimensions _radii,
            const PoseType &_pose) = 0;
      };
    };
  }
}

#include <ignition/physics/detail/EllipsoidShape.hh>

#endif
