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

namespace gz
{
namespace physics
{

/// \brief GetBatchRayIntersectionFromLastStepFeature is a feature for
/// retrieving multiple ray intersections generated in the previous simulation
/// step in a single call.
///
/// This is the batched counterpart of GetRayIntersectionFromLastStepFeature.
/// Physics engines should prefer this path for lidar-like sensors where many
/// rays (e.g. 10,000+) are cast each step, as it allows the implementation to
/// amortise broadphase setup and virtual-dispatch overhead across the whole ray
/// set rather than paying it once per ray.
///
/// Implementations that do not support batched queries may return results
/// identical to calling GetRayIntersectionFromLastStep() N times; the value of
/// this feature is that the implementation is free to optimise the inner loop.
class GZ_PHYSICS_VISIBLE GetBatchRayIntersectionFromLastStepFeature
    : public virtual FeatureWithRequirements<ForwardStep>
{
  public: template <typename PolicyT>
  struct RayIntersectionT
  {
    public: using Scalar = typename PolicyT::Scalar;
    public: using VectorType =
      typename FromPolicy<PolicyT>::template Use<LinearVector>;

    /// \brief The hit point in world coordinates.
    /// Set to NaN when the ray misses (REP-117).
    VectorType point;

    /// \brief The fraction of the ray length at the intersection point.
    /// Set to NaN when the ray misses.
    Scalar fraction;

    /// \brief The surface normal at the hit point in world coordinates.
    /// Set to NaN when the ray misses.
    VectorType normal;
  };

  /// \brief A single ray query: an origin–target pair in world coordinates.
  public: template <typename PolicyT>
  struct RayT
  {
    public: using VectorType =
      typename FromPolicy<PolicyT>::template Use<LinearVector>;

    /// \brief Ray start point in world coordinates.
    VectorType origin;

    /// \brief Ray end point in world coordinates.
    VectorType target;
  };

  public: template <typename PolicyT, typename FeaturesT>
  class World : public virtual Feature::World<PolicyT, FeaturesT>
  {
    public: using VectorType =
      typename FromPolicy<PolicyT>::template Use<LinearVector>;
    public: using RayIntersection = RayIntersectionT<PolicyT>;
    public: using RayQuery = RayT<PolicyT>;

    /// \brief Cast multiple rays and return one result per ray.
    ///
    /// Results are returned in the same order as \p _rays. Rays that do not
    /// intersect any geometry have all fields set to NaN (REP-117). An empty
    /// input vector produces an empty output vector.
    ///
    /// \param[in] _rays  Ray queries (origin + target) in world coordinates.
    /// \return One RayIntersection per input ray, in the same order.
    public: std::vector<RayIntersection>
      GetBatchRayIntersectionFromLastStep(
        const std::vector<RayQuery> &_rays) const;
  };

  public: template <typename PolicyT>
  class Implementation : public virtual Feature::Implementation<PolicyT>
  {
    public: using RayIntersection = RayIntersectionT<PolicyT>;
    public: using RayQuery = RayT<PolicyT>;
    public: using VectorType =
      typename FromPolicy<PolicyT>::template Use<LinearVector>;

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
