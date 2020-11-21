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

#ifndef IGNITION_PHYSICS_PLANESHAPE_HH_
#define IGNITION_PHYSICS_PLANESHAPE_HH_

#include <string>

#include <ignition/physics/DeclareShapeType.hh>
#include <ignition/physics/Geometry.hh>

namespace ignition
{
namespace physics
{
  IGN_PHYSICS_DECLARE_SHAPE_TYPE(PlaneShape)

  /////////////////////////////////////////////////
  class GetPlaneShapeProperties
    : public virtual FeatureWithRequirements<PlaneShapeCast>
  {
    public: template <typename PolicyT, typename FeaturesT>
    class PlaneShape : public virtual Entity<PolicyT, FeaturesT>
    {
      public: using Normal =
          typename FromPolicy<PolicyT>::template Use<LinearVector>;

      public: using Point =
          typename FromPolicy<PolicyT>::template Use<LinearVector>;

      /// \brief Get the normal vector for this plane.
      /// \returns the normal vector for this plane.
      public: Normal GetNormal() const;

      /// \brief Get a point on the plane.
      /// \returns the offset of the plane.
      public: Point GetPoint() const;
    };

    public: template <typename PolicyT>
    class Implementation : public virtual Feature::Implementation<PolicyT>
    {
      public: using Normal =
          typename FromPolicy<PolicyT>::template Use<LinearVector>;

      public: using Point =
          typename FromPolicy<PolicyT>::template Use<LinearVector>;

      // See PlaneShape::GetNormal()
      public: virtual Normal GetPlaneShapeNormal(
          const Identity &_planeID) const = 0;

      // See PlaneShape::GetPoint()
      public: virtual Point GetPlaneShapePoint(
          const Identity &_planeID) const = 0;
    };
  };

  /////////////////////////////////////////////////
  class SetPlaneShapeProperties
      : public virtual FeatureWithRequirements<PlaneShapeCast>
  {
    public: template <typename PolicyT, typename FeaturesT>
    class PlaneShape : public virtual Entity<PolicyT, FeaturesT>
    {
      public: using Normal =
          typename FromPolicy<PolicyT>::template Use<LinearVector>;

      public: using Point =
          typename FromPolicy<PolicyT>::template Use<LinearVector>;

      /// \brief Set the normal vector of this plane
      /// \param[in] _normal
      ///   The new normal vector for this plane
      public: void SetNormal(const Normal &_normal);

      /// \brief Specify a point on this plane. The normal vector will remain
      /// fixed.
      /// \param[in] _point
      ///   A point which needs to be on the plane.
      public: void SetPoint(const Point &_point);
    };

    public: template <typename PolicyT>
    class Implementation : public virtual Feature::Implementation<PolicyT>
    {
      public: using Normal =
          typename FromPolicy<PolicyT>::template Use<LinearVector>;

      public: using Point =
          typename FromPolicy<PolicyT>::template Use<LinearVector>;

      public: virtual void SetPlaneShapeNormal(
          const Identity &_planeID,
          const Normal &_normal) = 0;

      public: virtual void SetPlaneShapePoint(
          const Identity &_planeID,
          const Point &_point) = 0;
    };
  };

  /////////////////////////////////////////////////
  /// \brief \brief This feature constructs a new plane shape and attaches the
  /// desired point, which the plane passes thorugh in the link frame. The
  /// default point is at zero coordinate.
  class AttachPlaneShapeFeature
      : public virtual FeatureWithRequirements<PlaneShapeCast>
  {
    public: template <typename PolicyT, typename FeaturesT>
    class Link : public virtual Feature::Link<PolicyT, FeaturesT>
    {
      public: using Normal =
          typename FromPolicy<PolicyT>::template Use<LinearVector>;

      public: using Point =
          typename FromPolicy<PolicyT>::template Use<LinearVector>;

      /// \brief Attach a PlaneShape to this link
      /// \param[in] _name
      ///   Name to give to the PlaneShape
      /// \param[in] _normal
      ///   Normal vector for the plane
      /// \param[in] _point
      ///   The point that the plane passes through (hence defining the plane)
      /// \returns the PlaneShapePtr that was just created.
      public: PlaneShapePtr<PolicyT, FeaturesT> AttachPlaneShape(
          const std::string &_name,
          const Normal &_normal,
          const Point &_point = Point::Zero());
    };

    public: template <typename PolicyT>
    class Implementation : public virtual Feature::Implementation<PolicyT>
    {
      public: using Normal =
          typename FromPolicy<PolicyT>::template Use<LinearVector>;

      public: using Point =
          typename FromPolicy<PolicyT>::template Use<LinearVector>;

      public: virtual Identity AttachPlaneShape(
          const Identity &_linkID,
          const std::string &_name,
          const Normal &_normal,
          const Point &_point) = 0;
    };
  };
}
}

#include <ignition/physics/detail/PlaneShape.hh>

#endif  // IGNITION_PHYSICS_PLANESHAPE_HH_
