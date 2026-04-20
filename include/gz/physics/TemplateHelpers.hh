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

#include <tuple>
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

    /// \brief A minimal variadic template container for types.
    /// This is used instead of std::tuple for intermediate template
    /// metaprogramming to drastically reduce compiler memory and instantiation
    /// times.
    template <typename... Ts>
    struct TypeList {
      static constexpr std::size_t size = sizeof...(Ts);
    };

    /// \brief Concatenate multiple TypeLists into a single TypeList.
    ///
    /// This uses an O(log N) tree-based recursion instead of an O(N) linear
    /// recursion. When concatenating many lists (e.g., during feature
    /// flattening), a linear recursion would instantiate a deeply nested stack
    /// of templates, which can quickly exceed the compiler's maximum
    /// instantiation depth or heap space limits (especially on MSVC).
    /// By pairing elements and recursing down, we keep the instantiation stack
    /// shallow and dramatically reduce compiler memory consumption.
    template <typename... Lists>
    struct TypeListCat;

    template <>
    struct TypeListCat<> {
      using type = TypeList<>;
    };

    template <typename... Ts>
    struct TypeListCat<TypeList<Ts...>> {
      using type = TypeList<Ts...>;
    };

    template <typename... T1, typename... T2>
    struct TypeListCat<TypeList<T1...>, TypeList<T2...>> {
      using type = TypeList<T1..., T2...>;
    };

    // We explicitly require at least 3 lists (L1, L2, L3) to use this
    // recursive specialization. This prevents an ambiguity error with the
    // 2-element base case `TypeListCat<TypeList<T1...>, TypeList<T2...>>`
    // when exactly 2 lists are provided.
    template <typename L1, typename L2, typename L3, typename... Rest>
    struct TypeListCat<L1, L2, L3, Rest...> {
      using type = typename TypeListCat<
          typename TypeListCat<L1, L2>::type,
          typename TypeListCat<L3, Rest...>::type>::type;
    };

    namespace detail
    {
      /// \brief Convert a TypeList to a std::tuple
      template <typename T>
      struct ToTuple;

      template <typename... Ts>
      struct ToTuple<TypeList<Ts...>> {
        using type = std::tuple<Ts...>;
      };

      /// \brief Convert a std::tuple to a TypeList
      template <typename T>
      struct TupleToTypeList;

      template <typename... Ts>
      struct TupleToTypeList<std::tuple<Ts...>> {
        using type = TypeList<Ts...>;
      };
    }

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
             typename = std::void_t<>> \
    struct Implementation \
    { \
      using type = ::gz::physics::Empty; \
    }; \
    \
    template<typename F, typename PolicyT, typename FeaturesT> \
    struct Implementation<F, PolicyT, FeaturesT, \
                          std::void_t< \
                              typename F::template X <PolicyT, FeaturesT>>> \
    { \
      using type = typename F::template X <PolicyT, FeaturesT>; \
    }; \
    \
    template <typename PolicyT, typename FeaturesT> \
    using type = typename Implementation<InFeature, PolicyT, FeaturesT>::type; \
  };

#endif
