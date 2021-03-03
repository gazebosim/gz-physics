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

#ifndef IGNITION_PHYSICS_HEIGHTMAP_HEIGHTMAPSHAPE_HH_
#define IGNITION_PHYSICS_HEIGHTMAP_HEIGHTMAPSHAPE_HH_

#include <string>

#include <ignition/common/HeightmapData.hh>

#include <ignition/physics/DeclareShapeType.hh>
#include <ignition/physics/Geometry.hh>

namespace ignition
{
namespace physics
{
namespace heightmap
{
  IGN_PHYSICS_DECLARE_SHAPE_TYPE(HeightmapShape)

  /////////////////////////////////////////////////
  class GetHeightmapShapeProperties
    : public virtual FeatureWithRequirements<HeightmapShapeCast>
  {
    public: template <typename PolicyT, typename FeaturesT>
    class HeightmapShape : public virtual Entity<PolicyT, FeaturesT>
    {
      public: using Dimensions =
          typename FromPolicy<PolicyT>::template Use<LinearVector>;

      /// \brief Get the size of the triangle heightmap.
      /// \returns the size of the triangle heightmap.
      public: Dimensions GetSize() const;

      /// \brief Get the scaling factor that is being applied to the heightmap.
      /// \returns the scaling factor that is being applied to the heightmap.
      public: Dimensions GetScale() const;
    };

    public: template <typename PolicyT>
    class Implementation : public virtual Feature::Implementation<PolicyT>
    {
      public: using Dimensions =
          typename FromPolicy<PolicyT>::template Use<LinearVector>;

      public: virtual Dimensions GetHeightmapShapeSize(
          const Identity &_heightmapID) const = 0;

      public: virtual Dimensions GetHeightmapShapeScale(
          const Identity &_heightmapID) const = 0;
    };
  };

  /////////////////////////////////////////////////
  class SetHeightmapShapeProperties
      : public virtual FeatureWithRequirements<HeightmapShapeCast>
  {
    public: template <typename PolicyT, typename FeaturesT>
    class HeightmapShape : public virtual Entity<PolicyT, FeaturesT>
    {
      public: using Dimensions =
          typename FromPolicy<PolicyT>::template Use<LinearVector>;

      public: void SetScale(const Dimensions &_dimensions);
    };

    public: template <typename PolicyT>
    class Implementation : public virtual Feature::Implementation<PolicyT>
    {
      public: using Dimensions =
          typename FromPolicy<PolicyT>::template Use<LinearVector>;

      public: void SetHeightmapShapeScale(
          const Identity &_heightmapID,
          const Dimensions &_dimensions) = 0;
    };
  };

  /////////////////////////////////////////////////
  class AttachHeightmapShapeFeature
      : public virtual FeatureWithRequirements<HeightmapShapeCast>
  {
    public: template <typename PolicyT, typename FeaturesT>
    class Link : public virtual Feature::Link<PolicyT, FeaturesT>
    {
      public: using PoseType =
          typename FromPolicy<PolicyT>::template Use<Pose>;

      public: using Dimensions =
          typename FromPolicy<PolicyT>::template Use<LinearVector>;

      public: using ShapePtrType = HeightmapShapePtr<PolicyT, FeaturesT>;

      public: ShapePtrType AttachHeightmapShape(
          const std::string &_name,
          const ignition::common::HeightmapData &_heightmapData,
          const PoseType &_pose = PoseType::Identity(),
          const Dimensions &_scale = Dimensions::Ones(),
          int _subSampling = 1);
    };

    public: template <typename PolicyT>
    class Implementation : public virtual Feature::Implementation<PolicyT>
    {
      public: using PoseType =
          typename FromPolicy<PolicyT>::template Use<Pose>;

      public: using Dimensions =
          typename FromPolicy<PolicyT>::template Use<LinearVector>;

      public: virtual Identity AttachHeightmapShape(
          const Identity &_linkID,
          const std::string &_name,
          const ignition::common::HeightmapData &_heightmapData,
          const PoseType &_pose,
          const Dimensions &_scale,
          int _subSampling) = 0;
    };
  };
}
}
}

#include <ignition/physics/heightmap/detail/HeightmapShape.hh>

#endif  // IGNITION_PHYSICS_HEIGHTMAP_HEIGHTMAPSHAPE_HH_
