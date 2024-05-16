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

#include <vector>
#include <gz/physics/FeatureList.hh>
#include <gz/physics/ForwardStep.hh>
#include <gz/physics/Geometry.hh>
#include <gz/physics/SpecifyData.hh>

namespace gz
{
namespace physics
{
/// \brief GetRayIntersectionFromLastStepFeature is a feature for retrieving
/// the a ray intersection generated in the previous simulation step.
class GZ_PHYSICS_VISIBLE GetRayIntersectionFromLastStepFeature
    : public virtual FeatureWithRequirements<ForwardStep>
{
  public: template <typename PolicyT>
  struct RayIntersectionT
  {
    public: using Scalar = typename PolicyT::Scalar;
    public: using VectorType =
      typename FromPolicy<PolicyT>::template Use<LinearVector>;

    /// \brief The hit point in the world coordinates
    VectorType point;

    /// \brief The fraction of the ray length at the intersection/hit point.
    Scalar fraction;

    /// \brief The normal at the hit point in the world coordinates
    VectorType normal;
  };

  public: template <typename PolicyT, typename FeaturesT>
  class World : public virtual Feature::World<PolicyT, FeaturesT>
  {
    public: using VectorType =
      typename FromPolicy<PolicyT>::template Use<LinearVector>;
    public: using RayIntersection = RayIntersectionT<PolicyT>;
    public: using RayIntersectionData =
      SpecifyData<RequireData<RayIntersection>>;

    /// \brief Get ray intersection generated in the previous simulation step
    public: RayIntersectionData GetRayIntersectionFromLastStep(
      const VectorType &_from, const VectorType &_to) const;
  };

  public: template <typename PolicyT>
  class Implementation : public virtual Feature::Implementation<PolicyT>
  {
    public: using RayIntersection = RayIntersectionT<PolicyT>;
    public: using VectorType =
      typename FromPolicy<PolicyT>::template Use<LinearVector>;

    public: virtual RayIntersection GetRayIntersectionFromLastStep(
      const Identity &_worldID,
      const VectorType &_from,
      const VectorType &_to) const = 0;
  };
};
}
}

#include "gz/physics/detail/GetRayIntersection.hh"

#endif /* end of include guard: GZ_PHYSICS_GETRAYINTERSECTION_HH_ */
