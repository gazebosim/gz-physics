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

#ifndef GZ_PHYSICS_TEMPLATEHELPERS_HH_
#define GZ_PHYSICS_TEMPLATEHELPERS_HH_

#include <type_traits>

namespace gz
{
  namespace physics
  {
    /// \brief Useful as a blank placeholder in template metaprogramming
    struct Empty { };

    /// \brief This can be used to turn a type into a function argument, which
    /// is useful for template metaprogramming.
    template <class... T> struct type { };

    /// TODO(MXG): Remove this and use std::void_t instead when migrating to
    /// C++17
    template <typename...>
    using void_t = void;

    /////////////////////////////////////////////////
    /// \brief Contains a static constexpr field named `value` which will be
    /// true if the type `From` has a const-quality less than or equal to the
    /// type `To`.
    ///
    /// The following expressions will return true:
    ///
    /// \code
    ///     ConstCompatible<T, T>::value
    ///     ConstCompatible<const T, T>::value
    /// \endcode
    ///
    /// The following expression will return false:
    ///
    /// \code
    ///     ConstCompatible<T, const T>::value
    /// \endcode
    ///
    template <typename To, typename From>
    struct ConstCompatible : std::true_type { };

    template <typename To, typename From>
    struct ConstCompatible<To, const From>
        : std::integral_constant<bool, std::is_const<To>::value> { };
  }
}

/// \brief Use this macro to create an API "selector" for a custom class.
///
/// Features may define APIs for class types that are not anticipated ahead of
/// time by the gz-physics library. When aggregating the API for that class
/// from a set of features, the Aggregator must be given a Selector that can
/// ignore features that don't mention the class (or else a compilation failure
/// would occur).
///
/// This macro creates a class that uses SFINAE (https://en.wikipedia.org/wiki/Substitution_failure_is_not_an_error)
/// to ignore features that don't define an API for the class of interest (X).
/// When using this macro, pass in the name of the class that you want the
/// selector to look for. If the class name is X, then this will create a
/// selector named SelectX. E.g. GZ_PHYSICS_CREATE_SELECTOR(RevoluteJoint) will
/// create a class named SelectRevoluteJoint which can be passed to an
/// Aggregator to extract the RevoluteJoint API from a list of features.
#define GZ_PHYSICS_CREATE_SELECTOR(X) \
  template<typename InFeature> \
  struct Select ## X \
  { \
    template<typename F, typename PolicyT, typename FeaturesT, \
             typename = ::gz::physics::void_t<>> \
    struct Implementation \
    { \
      using type = ::gz::physics::Empty; \
    }; \
    \
    template<typename F, typename PolicyT, typename FeaturesT> \
    struct Implementation<F, PolicyT, FeaturesT, \
                          ::gz::physics::void_t< \
                              typename F::template X <PolicyT, FeaturesT>>> \
    { \
      using type = typename F::template X <PolicyT, FeaturesT>; \
    }; \
    \
    template <typename PolicyT, typename FeaturesT> \
    using type = typename Implementation<InFeature, PolicyT, FeaturesT>::type; \
  };

#endif
