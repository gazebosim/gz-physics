/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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

#ifndef GZ_PHYSICS_GETBATCHRAYINTERSECTION_HH_
#define GZ_PHYSICS_GETBATCHRAYINTERSECTION_HH_

#include <vector>

#include <gz/physics/FeatureList.hh>
#include <gz/physics/ForwardStep.hh>
#include <gz/physics/Geometry.hh>
#include <gz/physics/RayIntersection.hh>

namespace gz
{
namespace physics
{

/// \brief GetBatchRayIntersectionFromLastStepFeature is a feature for
/// retrieving multiple ray intersections generated in the previous simulation
/// step in a single call.
class GZ_PHYSICS_VISIBLE GetBatchRayIntersectionFromLastStepFeature
    : public virtual FeatureWithRequirements<ForwardStep>
{
  /// \brief Alias to the shared ray intersection result type.
  public: template <typename PolicyT>
  using RayIntersectionT = gz::physics::RayIntersectionT<PolicyT>;

  public: template <typename PolicyT>
  struct RayT
  {
    public: using VectorType =
      typename FromPolicy<PolicyT>::template Use<LinearVector>;

    /// \brief Ray start point in world coordinates
    VectorType origin;

    /// \brief Ray end point in world coordinates
    VectorType target;
  };

  public: template <typename PolicyT, typename FeaturesT>
  class World : public virtual Feature::World<PolicyT, FeaturesT>
  {
    public: using RayIntersection = RayIntersectionT<PolicyT>;
    public: using RayQuery = RayT<PolicyT>;

    /// \brief Cast multiple rays and return one result per ray.
    /// \param[in] _rays Ray queries (origin + target) in world coordinates.
    /// \return One RayIntersection per input ray, in the same order.
    ///   fraction is +INF for rays with no object in range (REP-117).
    public: std::vector<RayIntersection>
      GetBatchRayIntersectionFromLastStep(
        const std::vector<RayQuery> &_rays) const;
  };

  public: template <typename PolicyT>
  class Implementation : public virtual Feature::Implementation<PolicyT>
  {
    public: using RayIntersection = RayIntersectionT<PolicyT>;
    public: using RayQuery = RayT<PolicyT>;

    public: virtual std::vector<RayIntersection>
      GetBatchRayIntersectionFromLastStep(
        const Identity &_worldID,
        const std::vector<RayQuery> &_rays) const = 0;
  };
};

}  // namespace physics
}  // namespace gz

#include "gz/physics/detail/GetBatchRayIntersection.hh"

#endif  // GZ_PHYSICS_GETBATCHRAYINTERSECTION_HH_
