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

#ifndef IGNITION_PHYSICS_MESH_MESHSHAPE_HH_
#define IGNITION_PHYSICS_MESH_MESHSHAPE_HH_

#include <string>

#include <ignition/common/Mesh.hh>

#include <ignition/physics/DeclareShapeType.hh>
#include <ignition/physics/Geometry.hh>

namespace ignition
{
namespace physics
{
namespace mesh
{
  IGN_PHYSICS_DECLARE_SHAPE_TYPE(MeshShape)

  /////////////////////////////////////////////////
  class GetMeshShapeProperties
    : public virtual FeatureWithRequirements<MeshShapeCast>
  {
    public: template <typename PolicyT, typename FeaturesT>
    class MeshShape : public virtual Entity<PolicyT, FeaturesT>
    {
      public: using Dimensions =
          typename FromPolicy<PolicyT>::template Use<LinearVector>;

      /// \brief Get the size of the triangle mesh.
      /// \returns the size of the triangle mesh.
      public: Dimensions GetSize() const;

      /// \brief Get the scaling factor that is being applied to the mesh.
      /// \returns the scaling factor that is being applied to the mesh.
      public: Dimensions GetScale() const;
    };

    public: template <typename PolicyT>
    class Implementation : public virtual Feature::Implementation<PolicyT>
    {
      public: using Dimensions =
          typename FromPolicy<PolicyT>::template Use<LinearVector>;

      public: virtual Dimensions GetMeshShapeSize(
          const Identity &_meshID) const = 0;

      public: virtual Dimensions GetMeshShapeScale(
          const Identity &_meshID) const = 0;
    };
  };

  /////////////////////////////////////////////////
  class SetMeshShapeProperties
      : public virtual FeatureWithRequirements<MeshShapeCast>
  {
    public: template <typename PolicyT, typename FeaturesT>
    class MeshShape : public virtual Entity<PolicyT, FeaturesT>
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

      public: void SetMeshShapeScale(
          const Identity &_meshID,
          const Dimensions &_dimensions) = 0;
    };
  };

  /////////////////////////////////////////////////
  class AttachMeshShapeFeature
      : public virtual FeatureWithRequirements<MeshShapeCast>
  {
    public: template <typename PolicyT, typename FeaturesT>
    class Link : public virtual Feature::Link<PolicyT, FeaturesT>
    {
      public: using PoseType =
          typename FromPolicy<PolicyT>::template Use<Pose>;

      public: using ShapePtrType = MeshShapePtr<PolicyT, FeaturesT>;

      public: ShapePtrType AttachMeshShape(
          const std::string &_name,
          const ignition::common::Mesh &_mesh,
          const PoseType &_pose = PoseType::Identity());
    };

    public: template <typename PolicyT>
    class Implementation : public virtual Feature::Implementation<PolicyT>
    {
      public: using PoseType =
          typename FromPolicy<PolicyT>::template Use<Pose>;

      public: virtual Identity AttachMeshShape(
          const Identity &_linkID,
          const std::string &_name,
          const ignition::common::Mesh &_mesh,
          const PoseType &_pose = PoseType::Identity()) = 0;
    };
  };
}
}
}

#include <ignition/physics/mesh/detail/MeshShape.hh>

#endif  // IGNITION_PHYSICS_MESH_MESHSHAPE_HH_
