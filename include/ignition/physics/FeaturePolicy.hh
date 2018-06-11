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

#ifndef IGNITION_PHYSICS_FEATUREPOLICY_HH_
#define IGNITION_PHYSICS_FEATUREPOLICY_HH_

#include <cstddef>

namespace ignition
{
  namespace physics
  {
    /////////////////////////////////////////////////
    /// \brief FeaturePolicy is a "policy class" used to provide metadata to
    /// features about what kind of simulation engine they are going to be used
    /// in.
    ///
    /// Currently, the information provided by the native FeaturePolicy includes
    ///
    ///     - Scalar: double or float. Determines the numerical precision used
    ///               by the simulation.
    ///
    ///     - Dim: 2 or 3. Determines whether the simulation is a 2D simulation
    ///            or a 3D simulation.
    ///
    /// Custom features may require additional metadata, which can be encoded
    /// into a custom FeaturePolicy. However, keep in mind that most features
    /// require at least the Scalar and Dim fields, so be sure to provide those
    /// in your feature policy or else you are likely to encounter compilation
    /// errors.
    ///
    /// Feature policies are typically composable, but you must be careful about
    /// resolving ambiguously defined fields. If two parent policies are each
    /// defining a field with the same name, then the child policy must
    /// explicitly define that field itself. Preferably, the child would define
    /// the field based on one of its parent's definitions, e.g.:
    ///
    /// \code
    ///     struct Child : public Parent1, public Parent2
    ///     {
    ///       using AmbiguousField = typename Parent1::AmbiguousField;
    ///     };
    /// \endcode
    ///
    /// This design pattern is known as "Policy-based design". For more
    /// information, see: https://en.wikipedia.org/wiki/Policy-based_design
    template <typename _Scalar, std::size_t _Dim>
    struct FeaturePolicy
    {
      public: using Scalar = _Scalar;
      public: enum { Dim = _Dim };
    };

    using FeaturePolicy3d = FeaturePolicy<double, 3>;
    using FeaturePolicy2d = FeaturePolicy<double, 2>;
    using FeaturePolicy3f = FeaturePolicy<float, 3>;
    using FeaturePolicy2f = FeaturePolicy<float, 2>;
  }
}

#endif
