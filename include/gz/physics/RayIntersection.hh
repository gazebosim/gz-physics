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

#ifndef GZ_PHYSICS_RAYINTERSECTION_HH_
#define GZ_PHYSICS_RAYINTERSECTION_HH_

#include <gz/physics/Geometry.hh>

namespace gz
{
namespace physics
{

/// \brief Result of a single ray intersection query, shared by both
/// GetRayIntersectionFromLastStepFeature and
/// GetBatchRayIntersectionFromLastStepFeature.
///
/// The \c fraction field encodes the outcome per REP-117:
///   - A finite value in [0, 1] indicates a hit at that fraction of the ray.
///   - +INF indicates no object was in range (a valid "no-range" reading).
///   - NaN indicates an erroneous or unsupported measurement.
///
/// \c point and \c normal are only meaningful when \c fraction is finite.
template <typename PolicyT>
struct RayIntersectionT
{
  public: using Scalar = typename PolicyT::Scalar;
  public: using VectorType =
    typename FromPolicy<PolicyT>::template Use<LinearVector>;

  /// \brief The hit point in world coordinates.
  VectorType point;

  /// \brief Fraction of the ray length at the intersection/hit point.
  /// +INF if no object was in range; NaN on error (see REP-117).
  Scalar fraction;

  /// \brief The normal at the hit point in world coordinates.
  VectorType normal;
};

}  // namespace physics
}  // namespace gz

#endif  // GZ_PHYSICS_RAYINTERSECTION_HH_
