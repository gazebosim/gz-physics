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

#ifndef IGNITION_PHYSICS_GEOMETRY_HH_
#define IGNITION_PHYSICS_GEOMETRY_HH_

#include <Eigen/Geometry>

#define DETAIL_IGN_PHYSICS_MAKE_BOTH_PRECISIONS(Type, Dim) \
  using Type ## Dim ## d = Type<double, Dim>; \
  using Type ## Dim ## f = Type<float, Dim>;

/// \brief This macro defines the following types:
/// Type2d // 2-dimensional version of Type with double precision
/// Type2f // 2-dimensional version of Type with float precision
/// Type3d // 3-dimensional version of Type with double precision
/// Type3f // 3-dimensional version of Type with float precision
#define IGN_PHYSICS_MAKE_ALL_TYPE_COMBOS(Type) \
  DETAIL_IGN_PHYSICS_MAKE_BOTH_PRECISIONS(Type, 2) \
  DETAIL_IGN_PHYSICS_MAKE_BOTH_PRECISIONS(Type, 3)

namespace ignition
{
  namespace physics
  {
    /// \brief This is used by ignition-physics to represent rigid body
    /// transforms in 2D or 3D simulations. The precision can be chosen as
    /// float or scalar.
    template <typename Scalar, std::size_t Dim>
    using Pose = Eigen::Transform<Scalar, Dim, Eigen::Isometry>;
    IGN_PHYSICS_MAKE_ALL_TYPE_COMBOS(Pose)

    template <typename Scalar, std::size_t Dim>
    using Vector = Eigen::Matrix<Scalar, Dim, 1>;
    IGN_PHYSICS_MAKE_ALL_TYPE_COMBOS(Vector)

    template <typename Scalar, std::size_t Dim>
    using LinearVector = Vector<Scalar, Dim>;
    IGN_PHYSICS_MAKE_ALL_TYPE_COMBOS(LinearVector)

    template <typename Scalar, std::size_t Dim>
    using AngularVector = Vector<Scalar, (Dim*(Dim-1))/2>;
    IGN_PHYSICS_MAKE_ALL_TYPE_COMBOS(AngularVector)

    template <typename Scalar, std::size_t Dim>
    using Wrench = Vector<Scalar, Dim + (Dim*(Dim-1))/2>;
    IGN_PHYSICS_MAKE_ALL_TYPE_COMBOS(Wrench)

    template <typename Scalar, std::size_t Dim>
    using AlignedBox = Eigen::AlignedBox<Scalar, Dim>;
    IGN_PHYSICS_MAKE_ALL_TYPE_COMBOS(AlignedBox)

    /////////////////////////////////////////////////
    /// \brief This struct is used to conveniently convert from a policy to a
    /// geometric type. Example usage:
    ///
    /// using AngularVector = FromPolicy<FeaturePolicy3d>::To<AngularVector>;
    template <typename PolicyT>
    struct FromPolicy
    {
      using Scalar = typename PolicyT::Scalar;
      enum { Dim = PolicyT::Dim };

      template<template <typename, std::size_t> class Type>
      using Use = Type<Scalar, Dim>;
    };

    template<typename Scalar>
    Eigen::Rotation2D<Scalar> Rotate(
        const Scalar &_angle,
        const AngularVector<Scalar, 2> &_axis)
    {
      return Eigen::Rotation2D<Scalar>(_angle*_axis[0]);
    }

    template <typename Scalar>
    Eigen::AngleAxis<Scalar> Rotate(
        const Scalar &_angle,
        const AngularVector<Scalar, 3> &_axis)
    {
      return Eigen::AngleAxis<Scalar>(_angle, _axis);
    }
  }
}

#endif
