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
    // cppcheck-suppress constStatement
    IGN_PHYSICS_MAKE_ALL_TYPE_COMBOS(Pose)

    template <typename Scalar, std::size_t Dim>
    using Vector = Eigen::Matrix<Scalar, Dim, 1>;
    // cppcheck-suppress constStatement
    IGN_PHYSICS_MAKE_ALL_TYPE_COMBOS(Vector)

    template <typename Scalar, std::size_t Dim>
    using LinearVector = Vector<Scalar, Dim>;
    // cppcheck-suppress constStatement
    IGN_PHYSICS_MAKE_ALL_TYPE_COMBOS(LinearVector)

    template <typename Scalar, std::size_t Dim>
    using AngularVector = Vector<Scalar, (Dim*(Dim-1))/2>;
    // cppcheck-suppress constStatement
    IGN_PHYSICS_MAKE_ALL_TYPE_COMBOS(AngularVector)

    /////////////////////////////////////////////////
    /// \brief This struct is used to conveniently convert from a policy to a
    /// geometric type. Example usage:
    ///
    /// using AngularVector = FromPolicy<FeaturePolicy3d>::To<AngularVector>;
    template<typename PolicyT>
    struct FromPolicy
    {
      using Scalar = typename PolicyT::Scalar;
      enum { Dim = PolicyT::Dim };

      template<template <typename, std::size_t> class Type>
      using Use = Type<Scalar, Dim>;
    };
  }
}

#endif
