/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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

#ifndef IGNITION_PHYSICS_FEATURE_HH_
#define IGNITION_PHYSICS_FEATURE_HH_

#include <ignition/physics/Export.hh>
#include <cstddef>

namespace ignition
{
  namespace physics
  {
    /////////////////////////////////////////////////
    /// \brief Placeholder class to be inherited by Feature types.
    class IGNITION_PHYSICS_VISIBLE Feature
    {
      public: virtual ~Feature() = default;

      /// \brief Placeholder class for the Engine API. Every Engine feature
      /// MUST inherit this class.
      public: template <typename>
      class Engine
      {
        public: virtual ~Engine() = default;
      };

      /// \brief Placeholder class in case a Feature does not define its own
      /// Link API
      public: template <typename>
      class Link
      {
        public: virtual ~Link() = default;
      };

      /// \brief Placeholder class in case a Feature does not define its own
      /// Joint API
      public: template <typename>
      class Joint
      {
        public: virtual ~Joint() = default;
      };

      /// \brief Placeholder class in case a Feature does not define its own
      /// Model API
      public: template <typename>
      class Model
      {
        public: virtual ~Model() = default;
      };

      /// \brief By default, a blank feature will not conflict with any other
      /// features. If your feature does conflict with some other set of
      /// features, then you should inherit the FeatureWithConflicts<...> class,
      /// and provide it a list of those conflicting features.
      template <typename SomeFeatureList, bool /*AssertNoConflict*/ = false>
      static constexpr bool ConflictsWith()
      {
        return false;
      }

      /// \brief By default, a blank feature will not require any other
      /// features. If your feature does require some other set of features,
      /// then you should inherit the FeatureWithRequirements class, and provide
      /// it with a list of the Features that you require.
      using RequiredFeatures = void;
    };

    /////////////////////////////////////////////////
    /// \brief If your feature is known to conflict with any other feature, then
    /// you should have your feature class inherit FeatureWithConflicts<...>,
    /// and pass it a list of the features that it conflicts with.
    ///
    /// Note: If your feature also has requirements, you should instead have
    /// your feature class inherit
    ///
    ///     FeatureList<
    ///         FeatureWithConflics<...conflicts...>,
    ///         FeatureWithRequirements<...requirements...>>
    ///
    /// The FeatureList class be used to compose conflicts and requirements.
    template <typename... ConflictingFeatures>
    struct FeatureWithConflicts;

    /////////////////////////////////////////////////
    /// \brief If your feature is known to require any other features, then you
    /// should have your feature class inherit FeatureWithRequirements<...>,
    /// and pass it a list of the features that it requires.
    ///
    /// Note: If your feature also has requirements, you should instead have
    /// your feature class inherit
    ///
    ///     FeatureList<
    ///         FeatureWithConflics<...conflicts...>,
    ///         FeatureWithRequirements<...requirements...>>
    ///
    /// The FeatureList class be used to compose conflicts and requirements.
    template <typename... RequiredFeatures>
    struct FeatureWithRequirements;

    /////////////////////////////////////////////////
    /// \brief Use a FeatureList to aggregate a list of Features.
    template <typename... Features>
    struct FeatureList;

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
    ///     struct Child : public Parent1, public Parent2
    ///     {
    ///       using AmbiguousField = typename Parent1::AmbiguousField;
    ///     };
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

//    template <typename... FeatureList>
//    using Features3d = Features<double, 3, FeatureList...>;

//    template <typename... FeatureList>
//    using Features2d = Features<double, 2, FeatureList...>;

//    template <typename... FeatureList>
//    using Features3f = Features<float, 3, FeatureList...>;

//    template <typename... FeatureList>
//    using Features2f = Features<float, 2, FeatureList...>;
  }
}

#include <ignition/physics/detail/Feature.hh>

#endif
