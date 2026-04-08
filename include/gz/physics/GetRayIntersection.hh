/*
 * Copyright (C) 2024 Open Source Robotics Foundation
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

#ifndef GZ_PHYSICS_GETRAYINTERSECTION_HH_
#define GZ_PHYSICS_GETRAYINTERSECTION_HH_

#include <gz/physics/FeatureList.hh>
#include <gz/physics/ForwardStep.hh>
#include <gz/physics/Geometry.hh>
#include <gz/physics/RayIntersection.hh>
#include <gz/physics/SpecifyData.hh>
#include <gz/physics/Entity.hh>

namespace gz
{
namespace physics
{
/// \brief GetRayIntersectionFromLastStepFeature is a feature for retrieving
/// the ray intersection generated in the previous simulation step.
class GZ_PHYSICS_VISIBLE GetRayIntersectionFromLastStepFeature
    : public virtual FeatureWithRequirements<ForwardStep>
{
  public: template <typename PolicyT>
  struct ExtraRayIntersectionDataT
  {
    /// \brief The identity of the collision shape that was hit.
    std::size_t collisionShapeId{INVALID_ENTITY_ID};
  };

  /// \brief Alias to the shared ray intersection result type.
  public: template <typename PolicyT>
  using RayIntersectionT = gz::physics::RayIntersectionT<PolicyT>;

  public: template <typename PolicyT, typename FeaturesT>
  class World : public virtual Feature::World<PolicyT, FeaturesT>
  {
    public: using ExtraRayIntersectionData = ExtraRayIntersectionDataT<PolicyT>;
    public: using VectorType =
      typename FromPolicy<PolicyT>::template Use<LinearVector>;
    public: using RayIntersection = RayIntersectionT<PolicyT>;

    public: using RayIntersectionData =
        SpecifyData<RequireData<RayIntersection>,
                    ExpectData<ExtraRayIntersectionData>>;

    /// \brief Get ray intersection generated in the previous simulation step
    /// \param[in] _from The start point of the ray in world coordinates
    /// \param[in] _to The end point of the ray in world coordinates
    public: RayIntersectionData GetRayIntersectionFromLastStep(
      const VectorType &_from, const VectorType &_to) const;
  };

  public: template <typename PolicyT>
  class Implementation : public virtual Feature::Implementation<PolicyT>
  {
    public: using ExtraRayIntersectionData = ExtraRayIntersectionDataT<PolicyT>;
    public: using RayIntersection = RayIntersectionT<PolicyT>;
    public: using VectorType =
      typename FromPolicy<PolicyT>::template Use<LinearVector>;

    public: struct RayIntersectionInternal
    {
      RayIntersection intersection;
      CompositeData extraData;
    };

    public: virtual RayIntersectionInternal GetRayIntersectionFromLastStep(
      const Identity &_worldID,
      const VectorType &_from,
      const VectorType &_to) const = 0;
  };
};
}
}

#include "gz/physics/detail/GetRayIntersection.hh"

#endif /* end of include guard: GZ_PHYSICS_GETRAYINTERSECTION_HH_ */
