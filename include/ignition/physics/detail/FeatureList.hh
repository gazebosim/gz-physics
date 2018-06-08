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

#ifndef IGNITION_PHYSICS_DETAIL_FEATURELIST_HH_
#define IGNITION_PHYSICS_DETAIL_FEATURELIST_HH_

#include <tuple>
#include <type_traits>

#include <ignition/common/SpecializedPluginPtr.hh>
#include <ignition/physics/FeatureList.hh>
#include <ignition/physics/FeaturePolicy.hh>
#include <ignition/physics/TemplateHelpers.hh>

namespace ignition
{
  namespace physics
  {
    namespace detail
    {
      /////////////////////////////////////////////////
      /// \private This class provides a sanity check to make sure at compile
      /// time that each object that is being passed as a "feature" matches the
      /// concept of a feature. This can be replaced by Concepts once C++ has
      /// language-level support for Concepts.
      ///
      /// This default definition will only be used when F is empty.
      template <typename... F>
      class VerifyFeatures { };

      /// \private This definition will recursively check each item in the pack
      /// to make sure they are each features.
      template <typename F, typename... Others>
      class VerifyFeatures<F, Others...>
          : public VerifyFeatures<Others...>
      {
        static_assert(std::is_base_of<Feature, F>::value,
                      "FEATURELIST ERROR: YOU ARE ATTEMPTING TO ADD A CLASS "
                      "WHICH IS NOT A Feature TO A FeatureList! MAKE SURE THAT "
                      "YOUR CLASS VIRTUALLY INHERITS THE Feature BASE CLASS!");
      };

      /// \private If VerifyFeatures is given a tuple of features, this will
      /// unpack them so that each feature can be verified individually.
      template <typename... F>
      class VerifyFeatures<std::tuple<F...>>
          : public VerifyFeatures<F...> { };

      /////////////////////////////////////////////////
      /// \private ExtractFeatures is used to wipe out any potential containers
      /// that might be packing a set of features (such as a tuple or a
      /// FeatureList) and return a raw tuple of the features. This allows us to
      /// easily get a serialized list of features at compile time.
      ///
      /// This default implementation simply takes in a single feature and puts
      /// it into a tuple of size one. This allows us to use std::tuple_cat on
      /// it later to combine it with tuples that may contain multiple features.
      template <typename F>
      class ExtractFeatures
          : public VerifyFeatures<F>
      {
        public: using Result = std::tuple<F>;
      };

      /// \private This specialization of ExtractFeatures is used to wipe away
      /// a tuple that is currently holding a set of features, then verify those
      /// features, and finally repackage them as a tuple.
      template <typename... F>
      class ExtractFeatures<std::tuple<F...>>
          : public VerifyFeatures<F...>
      {
        public: using Result = std::tuple<F...>;
      };

      /// \private This specialization of ExtractFeatures is used to wipe away
      /// the FeatureList that is currently holding a set of features, then
      /// verify those features, and finally repackage them as the raw feature
      /// tuple that is being held by the FeatureList.
      template <typename... F>
      class ExtractFeatures<FeatureList<F...>>
          : public VerifyFeatures<typename FeatureList<F...>::Features>
      {
        public: using Result = typename FeatureList<F...>::Features;
      };

      /// \private This specialization skips over any void entries. This allows
      /// users or template metaprograms to place `void` into feature entries
      /// and have those entries be gracefully skipped over.
      template <>
      class ExtractFeatures<void>
      {
        public: using Result = std::tuple<>;
      };

      /////////////////////////////////////////////////
      /// \private CombineLists is used to take variadic lists of features,
      /// FeatureLists, or std::tuples of features, and collapse them into a
      /// serialized std::tuple of features.
      ///
      /// This class works recursively.
      template <typename... FeatureLists>
      class CombineLists;

      /// \private This specialization is the terminal of the class recursion.
      /// It gets called when only one type remains in the list.
      template <typename F>
      class CombineLists<F>
      {
        public: using Result = decltype(std::tuple_cat(
            typename ExtractFeatures<F>::Result(),
            typename ExtractFeatures<typename F::RequiredFeatures>::Result()));
      };

      /// \private This specialization peels away each type in the variadic set,
      /// ultimately collapsing the whole list into a single tuple of features.
      template <typename F1, typename... Others>
      class CombineLists<F1, Others...>
      {
        public: using Result =
          decltype(std::tuple_cat(
            typename ExtractFeatures<F1>::Result(),
            typename ExtractFeatures<typename F1::RequiredFeatures>::Result(),
            typename CombineLists<Others...>::Result()));
      };

      /////////////////////////////////////////////////
      /// \private Inspired by https://stackoverflow.com/a/26288164
      /// This class provides a static constexpr member named `value` which is
      /// true if T is one of the entries of Tuple, and false otherwise.
      template <typename T, typename Tuple>
      struct TupleContainsBase;

      /// \private This specialization implements TupleContainsBase. It only
      /// works if Tuple is a std::tuple; any other type for the second template
      /// argument will fail to compile.
      template <typename T, typename... Types>
      struct TupleContainsBase<T, std::tuple<Types...>>
          : std::integral_constant<bool,
              !std::is_same<
                std::tuple<typename std::conditional<
                  std::is_base_of<T, Types>::value, Empty, Types>::type...>,
                std::tuple<Types...>
              >::value> { };

      /////////////////////////////////////////////////
      /// \private This class implements FeatureList::ConflictsWith. Its
      /// implementation is conceptually similar to TupleContainsBase.
      template <typename SomeFeatureList, bool AssertNoConflict, typename Tuple>
      struct ConflictingLists;

      /// \private Implementation of ConflictingLists. If the Tuple argument is
      /// not a std::tuple, this class will be undefined.
      template <typename SomeFeatureList, bool AssertNoConflict,
                typename... Features>
      struct ConflictingLists<
          SomeFeatureList, AssertNoConflict, std::tuple<Features...>>
          : std::integral_constant<bool,
              !std::is_same<
                std::tuple<typename std::conditional<
                  Features::template ConflictsWith<
                    SomeFeatureList, AssertNoConflict>(),
                  Empty, Features>::type...>,
              std::tuple<Features...>
            >::value> { };

      /////////////////////////////////////////////////
      /// \private Check whether any feature within a std::tuple of features
      /// conflicts with any other feature in that tuple.
      template <bool AssertNoConflict, typename Tuple>
      struct SelfConflict;

      /// \private Recursive implementation of SelfConflict
      template <bool AssertNoConflict, typename Feature1,
                typename... OtherFeatures>
      struct SelfConflict<
            AssertNoConflict, std::tuple<Feature1, OtherFeatures...>>
          : std::integral_constant<bool,
          FeatureList<Feature1>::template ConflictsWith<
            FeatureList<OtherFeatures...>, AssertNoConflict>()
       || FeatureList<OtherFeatures...>::template ConflictsWith<
            Feature1, AssertNoConflict>()> {};

      /// \private Terminal implementation of SelfConflict
      template <bool AssertNoConflict, typename SingleFeature>
      struct SelfConflict<AssertNoConflict, std::tuple<SingleFeature>>
          : std::integral_constant<bool, false> {};

      /////////////////////////////////////////////////
      /// \private Extract the API out of a FeatureList
      template <template<typename> class, typename...>
      struct Aggregate;

      template <template<typename> class Selector, typename... FeaturesT>
      struct Aggregate<Selector, FeatureList<FeaturesT...>>
      {
        public: template<typename... T>
        using type =
          typename Aggregate<
              Selector, typename FeatureList<FeaturesT...>::Features>
                  ::template type<T...>;
      };

      /// \private Recursively extract the API out of a std::tuple of features
      template <template<typename> class Selector,
                typename F1, typename... Remaining>
      struct Aggregate<Selector, std::tuple<F1, Remaining...>>
      {
        public: template<typename... T>
        class type
            : public virtual Selector<F1>::template type<T...>,
              public virtual Aggregate<
                  Selector, std::tuple<Remaining...>>::template type<T...> { };
      };

      /// \private Terminate the recursion
      template <template<typename> class Extractor>
      struct Aggregate<Extractor, std::tuple<>>
      {
        public: template <typename... P>
        class type { };
      };

      /////////////////////////////////////////////////
      /// \private This class is used to determine what type of
      /// SpecializedPluginPtr should be used by the entities provided by a
      /// plugin.
      template <typename Policy, typename InterfaceTuple>
      struct DeterminePluginType;

      /// \private Implementation of DeterminePluginType
      template <typename Policy, typename... Interfaces>
      struct DeterminePluginType<Policy, std::tuple<Interfaces...>>
      {
        using type = common::SpecializedPluginPtr<
            typename Interfaces::template Implementation<Policy>...>;
      };

      /////////////////////////////////////////////////
      /// \private This class is used to inspect what features are provided by
      /// a plugin. It implements the API of RequestFeatures.
      template <typename Policy, typename InterfaceTuple>
      struct InspectFeatures;

      /// \private Implementation of InspectFeatures.
      template <typename PolicyT, typename Feature1, typename... Remaining>
      struct InspectFeatures<PolicyT, std::tuple<Feature1, Remaining...>>
      {
        using Interface = typename Feature1::template Implementation<PolicyT>;

        /// \brief Check that each feature is provided by the plugin.
        template <typename PtrT>
        static bool Verify(const PtrT &_pimpl)
        {
          // TODO(MXG): Replace with a fold expression when we migrate to C++17
          return _pimpl->template HasInterface<Interface>()
              && InspectFeatures<PolicyT, std::tuple<Remaining...>>::
                      Verify(_pimpl);
        }

        template <typename PtrT>
        static void MissingNames(const PtrT &_pimpl,
                                 std::set<std::string> &_names)
        {
          if (!_pimpl->template HasInterface<Interface>())
            _names.insert(typeid(Feature1).name());

          InspectFeatures<PolicyT, std::tuple<Remaining...>>::MissingNames(
                _pimpl, _names);
        }
      };

      template <typename PolicyT>
      struct InspectFeatures<PolicyT, std::tuple<>>
      {
        template <typename PtrT>
        static bool Verify(const PtrT&)
        {
          return true;
        }

        template <typename PtrT>
        static void MissingNames(const PtrT&, std::set<std::string>&)
        {
          // Do nothing
        }
      };
    }

    /////////////////////////////////////////////////
    template <typename... FeaturesT>
    template <typename F>
    constexpr bool FeatureList<FeaturesT...>::HasFeature()
    {
      return detail::TupleContainsBase<F, Features>::value;
    }

    /////////////////////////////////////////////////
    template <typename... FeaturesT>
    template <typename SomeFeatureList, bool AssertNoConflict>
    constexpr bool FeatureList<FeaturesT...>::ConflictsWith()
    {
      // TODO(MXG): Replace this with a simple fold expression once we use C++17
      return
          detail::ConflictingLists<
              SomeFeatureList, AssertNoConflict, Features>::value
       || detail::ConflictingLists<
              FeatureList<FeaturesT...>, AssertNoConflict,
              typename FeatureList<SomeFeatureList>::Features>::value;
    }
    /////////////////////////////////////////////////
    /// \private The default definition of FeatureWithConflicts only gets called
    /// when the ConflictingFeatures list is empty. It should simply fall back
    /// on the default behavior of a blank feature.
    template <typename... ConflictingFeatures>
    struct FeatureWithConflicts : public virtual Feature { };

    /////////////////////////////////////////////////
    /// \private This template specialization of FeatureWithConflicts will be
    /// called when one or more features are listed as conflicts.
    ///
    /// We implement this using recursion so that the compiler can indicate
    /// which feature specifically is causing a conflict. A more concise
    /// implementation would wash away that information, making it harder to
    /// track down what the specific conflict is.
    template <typename Conflict, typename... RemainingConflicts>
    struct FeatureWithConflicts<Conflict, RemainingConflicts...>
        : public virtual Feature
    {
      public: template <typename SomeFeatureList,
                        bool AssertNoConflict = false>
      static constexpr bool ConflictsWith()
      {
        constexpr bool conflict =
            FeatureList<SomeFeatureList>::template HasFeature<Conflict>();

        static_assert(!AssertNoConflict || !conflict,
                      "FEATURE CONFLICT DETECTED");

        return conflict
            || FeatureWithConflicts<RemainingConflicts...>::
                  template ConflictsWith<SomeFeatureList, AssertNoConflict>();
      }
    };

    /////////////////////////////////////////////////
    /// \brief The FeatureWithRequirements class simply wraps up its required
    /// features in a FeatureList and then sets the RequiredFeatures type.
    template <typename... Features>
    struct FeatureWithRequirements : public virtual Feature
    {
      public: using RequiredFeatures = FeatureList<Features...>;
    };


// Macros for generating EngineTemplate, LinkTemplate, etc
#define DETAIL_IGN_PHYSICS_MAKE_AGGREGATE_WITH_POLICY(X, P) \
  template <typename List> \
  using X ## P = X ## Template<FeaturePolicy ## P, List>;

#define DETAIL_IGN_PHYSICS_MAKE_AGGREGATE(X) \
  template <typename T> \
  struct X ## Selector \
  { \
    public: template<typename PolicyT, typename PimplT> \
    using type = typename T::template X<PolicyT, PimplT>; \
  }; \
  \
  template <typename PolicyT, typename List> \
  using X ## Template = \
      typename detail::Aggregate<X ## Selector, List>:: \
          template type< \
            PolicyT, detail::DeterminePluginType< \
                PolicyT, typename List::Features>>; \
  DETAIL_IGN_PHYSICS_MAKE_AGGREGATE_WITH_POLICY(X, 3d) \
  DETAIL_IGN_PHYSICS_MAKE_AGGREGATE_WITH_POLICY(X, 2d) \
  DETAIL_IGN_PHYSICS_MAKE_AGGREGATE_WITH_POLICY(X, 3f) \
  DETAIL_IGN_PHYSICS_MAKE_AGGREGATE_WITH_POLICY(X, 2f)


    // This macro expands to create the templates:
    // - Engine3d<List>
    // - Engine2d<List>
    // - Engine3f<List>
    // - Engine2f<List>
    // Each template accepts a FeatureList and results in an Engine object that
    // combines the Engine APIs of every feature in List.
    //
    // The dimensionality [3|2] and precision [double|float] of the object is
    // indicated by the suffix of the type name.
    //
    // This is repeated for each of the built-in feature objects (e.g. Link,
    // Joint, Model).
    DETAIL_IGN_PHYSICS_MAKE_AGGREGATE(Engine)
    DETAIL_IGN_PHYSICS_MAKE_AGGREGATE(World)
    DETAIL_IGN_PHYSICS_MAKE_AGGREGATE(Model)
    DETAIL_IGN_PHYSICS_MAKE_AGGREGATE(Link)
    DETAIL_IGN_PHYSICS_MAKE_AGGREGATE(Joint)
  }
}

#endif
