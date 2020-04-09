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

#include <memory>
#include <set>
#include <string>
#include <tuple>
#include <type_traits>
#include <utility>

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
      template <typename T>
      struct IterateTuple;

      template <typename Current, typename... T>
      struct IterateTuple<std::tuple<Current, T...>>
      {
        using CurrentTupleEntry = Current;
        using NextTupleEntry = IterateTuple<std::tuple<T...>>;
      };

      template <typename Current>
      struct IterateTuple<std::tuple<Current>>
      {
        using CurrentTupleEntry = Current;
      };

      template <>
      struct IterateTuple<std::tuple<>>
      {
        using CurrentTupleEntry = void;
      };

      template <typename Iterable, typename = void_t<>>
      struct GetNext
      {
        using n = void;
      };

      template <typename Iterable>
      struct GetNext<Iterable, void_t<typename Iterable::NextTupleEntry>>
      {
        // This struct is intentionally named with only one letter, because its
        // name will be repeated many times in the symbol names of aggregated
        // types. Do not change this name to anything longer than 1 letter or
        // else the compiled symbol names will be needlessly long, and that can
        // be costly to memory and performance.
        struct n : Iterable::NextTupleEntry { };
      };

      template <typename Policy, typename Feature, typename = void_t<>>
      struct ComposePlugin;

      template <typename Policy, typename Feature, bool HasRequirements>
      struct CheckRequirements
      {
        using type =
            ::ignition::plugin::SpecializedPlugin<
                typename Feature::template Implementation<Policy>>;
      };

      template <typename Policy, typename Feature>
      struct CheckRequirements<Policy, Feature, true>
      {
        struct type :
            ::ignition::plugin::SpecializedPlugin<
                typename Feature::template Implementation<Policy>>,
            ComposePlugin<Policy, typename Feature::RequiredFeatures>::type { };
      };

      template <typename Policy, typename Feature, typename>
      struct ComposePlugin
      {
        struct type : CheckRequirements<Policy, Feature,
            !std::is_void_v<typename Feature::RequiredFeatures>>::type { };
      };

      template <typename Policy, typename Iterable>
      struct ComposePlugin<Policy, Iterable,
          void_t<typename Iterable::CurrentTupleEntry>>
      {
        struct type :
            ComposePlugin<Policy, typename Iterable::CurrentTupleEntry>::type,
            ComposePlugin<Policy, typename GetNext<Iterable>::n> { };
      };

      template <typename Policy>
      struct ComposePlugin<Policy, void, void_t<>>
      {
        struct type { };
      };

      /////////////////////////////////////////////////
      /// \private This class is used to determine what type of
      /// SpecializedPluginPtr should be used by the entities provided by a
      /// plugin.
      template <typename Policy, typename FeaturesT>
      struct DeterminePlugin
      {
        struct Specializer
            : ::ignition::plugin::detail::SelectSpecializers<
              typename ComposePlugin<Policy, FeaturesT>::type> { };

        using type = ::ignition::plugin::TemplatePluginPtr<Specializer>;
      };

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
      template <typename F, typename = void_t<> >
      class ExtractFeatures
          : public VerifyFeatures<F>
      {
        public: using type = std::tuple<F>;
      };

      /// \private This specialization of ExtractFeatures is used to wipe away
      /// a tuple that is currently holding a set of features, then verify those
      /// features, and finally repackage them as a tuple.
      template <typename... F>
      class ExtractFeatures<std::tuple<F...>>
          : public VerifyFeatures<F...>
      {
        public: using type = std::tuple<F...>;
      };

      /// \private This specialization of ExtractFeatures is used to wipe away
      /// the FeatureList that is currently holding a set of features, then
      /// verify those features, and finally repackage them as the raw feature
      /// tuple that is being held by the FeatureList.
      template <typename SomeFeatureList>
      class ExtractFeatures<
              SomeFeatureList,
              void_t<typename SomeFeatureList::Features>>
          : public VerifyFeatures<typename SomeFeatureList::Features>
      {
        public: using type = typename SomeFeatureList::Features;
      };

      /// \private This specialization skips over any void entries. This allows
      /// users or template metaprograms to place `void` into feature entries
      /// and have those entries be gracefully skipped over.
      template <>
      class ExtractFeatures<void>
      {
        public: using type = std::tuple<>;
      };

      /////////////////////////////////////////////////
      template <typename DiscardTuple>
      struct RedundantTupleFilter
      {
        template <typename T>
        struct Apply : TupleContainsBase<T, DiscardTuple> { };
      };

      /////////////////////////////////////////////////
      template <template <typename> class Filter, typename InputTuple>
      struct FilterTuple;

      /////////////////////////////////////////////////
      template <template <typename> class Filter, typename... InputTypes>
      struct FilterTuple<Filter, std::tuple<InputTypes...>>
      {
        using type = decltype(std::tuple_cat(
            std::conditional_t<
              // If the input type is a base class of anything that should be
              // discared ...
              Filter<InputTypes>::value,
              // ... then we should leave it out of the final tuple ...
              std::tuple<>,
              // ... otherwise, include it.
              std::tuple<InputTypes>
            // Do this for each type in the InputTypes parameter pack.
            >()...));
      };

      /////////////////////////////////////////////////
      template <typename DiscardTuple>
      struct SubtractTuple
      {
        template <typename T>
        using Filter =
            typename RedundantTupleFilter<DiscardTuple>::template Apply<T>;

        template <typename FromTuple>
        struct From : FilterTuple<Filter, FromTuple> { };
      };

      /////////////////////////////////////////////////
      template <typename InputTuple>
      struct RemoveTupleRedundancies;

      template <typename... InputTupleArgs>
      struct RemoveTupleRedundancies<std::tuple<InputTupleArgs...>>
      {
        template <typename T>
        struct wrap { };

        template <typename PartialResultInput, typename...>
        struct Impl;

        template <typename PartialResultInput>
        struct Impl<PartialResultInput>
        {
          using type = std::tuple<>;
        };

        template <typename ParentResultInput, typename F1, typename... Others>
        struct Impl<ParentResultInput, F1, Others...>
        {
          using PartialResult =
              std::conditional_t<
                TupleContainsBase<wrap<F1>, ParentResultInput>::value,
                std::tuple<>,
                std::tuple<wrap<F1>>
              >;

          using AggregateResult = decltype(std::tuple_cat(
              ParentResultInput(), PartialResult()));

          using type = decltype(std::tuple_cat(
              PartialResult(),
              typename Impl<AggregateResult, Others...>::type()));
        };

        template <typename Tuple>
        struct unwrap;

        template <typename... T>
        struct unwrap<std::tuple<wrap<T>...>>
        {
          using type = std::tuple<T...>;
        };

        using type =
          typename unwrap<
            typename Impl<std::tuple<>, InputTupleArgs...>::type
          >::type;
      };
      /////////////////////////////////////////////////
      template <typename FeatureTuple, typename = void_t<>>
      struct FlattenFeatures;

      /////////////////////////////////////////////////
      template <typename FeatureOrList, typename = void_t<>>
      struct ExpandFeatures
      {
        using type = std::conditional_t<
            std::is_void_v<typename FeatureOrList::RequiredFeatures>,
            std::tuple<FeatureOrList>,
            decltype(std::tuple_cat(
              std::tuple<FeatureOrList>(),
              typename FlattenFeatures<
                typename FeatureOrList::RequiredFeatures>::type()))
        >;
      };

      /////////////////////////////////////////////////
      template <typename List>
      struct ExpandFeatures<List, void_t<typename List::Features>>
      {
        using type = typename FlattenFeatures<typename List::Features>::type;
      };

      /////////////////////////////////////////////////
      template <typename FeatureListT>
      struct FlattenFeatures<
          FeatureListT, void_t<typename FeatureListT::FeatureTuple>>
      {
        using type =
            typename FlattenFeatures<typename FeatureListT::FeatureTuple>::type;
      };

      /////////////////////////////////////////////////
      template <typename... Features>
      struct FlattenFeatures<std::tuple<Features...>, void_t<>>
      {
        using type = decltype(std::tuple_cat(
            typename ExpandFeatures<Features>::type()...));
      };

      /////////////////////////////////////////////////
      template <>
      struct FlattenFeatures<void>
      {
        using type = std::tuple<>;
      };

      /////////////////////////////////////////////////
      /// \private CombineListsImpl provides the implementation of CombineLists.
      /// This helper implementation structure allows us to filter out repeated
      /// features from the list.
      template <typename PartialResultInput, typename... FeatureLists>
      struct CombineListsImpl;

      template <typename PartialResultInput>
      struct CombineListsImpl<PartialResultInput>
      {
        using type = std::tuple<>;
      };

      template <typename ParentResultInput, typename F1, typename... Others>
      struct CombineListsImpl<ParentResultInput, F1, Others...>
      {
        // Add the features of the feature list F1, while filtering out any
        // repeated features.
        using InitialResult =
            typename SubtractTuple<ParentResultInput>
            ::template From<typename ExtractFeatures<F1>::type>::type;

        // Add the features that are required by F1, while filtering out any
        // repeated features.
        using PartialResult = decltype(std::tuple_cat(
            InitialResult(),
            typename SubtractTuple<
              decltype(std::tuple_cat(ParentResultInput(), InitialResult()))>
              ::template From<
                typename ExtractFeatures<typename F1::RequiredFeatures>::type
            >::type()));

        // Define the tuple that the child should use to filter its list
        using ChildFilter =
            decltype(std::tuple_cat(ParentResultInput(), PartialResult()));

        // Construct the final result
        using type = decltype(std::tuple_cat(
            PartialResult(),
            typename CombineListsImpl<ChildFilter, Others...>::type()));
      };

      /// \private CombineLists is used to take variadic lists of features,
      /// FeatureLists, or std::tuples of features, and collapse them into a
      /// serialized std::tuple of features.
      ///
      /// This class works recursively.
      template <typename... FeatureLists>
      struct CombineLists
      {
        public: using Result =
            typename CombineListsImpl<std::tuple<>, FeatureLists...>::type;
      };

      /////////////////////////////////////////////////
      /// \private This class helps to implement the function
      /// FeatureList::ConflictsWith(). Its implementation is conceptually
      /// similar to TupleContainsBase.
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
      template <bool AssertNoConflict, typename... FeatureLists>
      struct SelfConflict;

      /// \private Recursive implementation of SelfConflict
      template <bool AssertNoConflict, typename FeatureList1,
                typename... OtherFeatureLists>
      struct SelfConflict<
            AssertNoConflict, FeatureList1, OtherFeatureLists...>
          : std::integral_constant<bool,
          FeatureList<FeatureList1>::template ConflictsWith<
            FeatureList<OtherFeatureLists...>, AssertNoConflict>()> {};

      /// \private Terminal implementation of SelfConflict for 1 feature
      template <bool AssertNoConflict, typename SingleFeatureList>
      struct SelfConflict<AssertNoConflict, SingleFeatureList>
          : std::integral_constant<bool, false> {};

      /// \private Terminal implementation of SelfConflict for 0 features
      template <bool AssertNoConflict>
      struct SelfConflict<AssertNoConflict>
          : std::integral_constant<bool, false> {};

      /////////////////////////////////////////////////
      /// \private Extract the API out of a FeatureList
      template <template<typename> class Selector,
                typename FeatureT, typename = void_t<>>
      struct Aggregate
      {
        public: template<typename... T>
        struct type
            : public virtual Selector<FeatureT>::template type<T...>,
              public virtual Aggregate<Selector,
                typename FeatureT::RequiredFeatures>::template type<T...>
        { };
      };

      template <template<typename> class Selector>
      struct Aggregate<Selector, void, void_t<>>
      {
        public: template<typename... T>
        struct type { };
      };

      template <template<typename> class Selector, typename Iterable>
      struct Aggregate<Selector, Iterable,
            void_t<typename Iterable::CurrentTupleEntry>>
      {
        public: template <typename... T>
        struct type
            : public virtual Aggregate<Selector,
                  typename Iterable::CurrentTupleEntry>::template type<T...>,
              public virtual Aggregate<Selector,
                  typename GetNext<Iterable>::n>::template type<T...> { };
      };

      /////////////////////////////////////////////////
      template <template<typename> class Selector, typename FeatureListT>
      struct ExtractAPI
      {
        template <typename FeatureTuple, typename... T>
        struct Select;

        template <typename... Features, typename... T>
        struct Select<std::tuple<Features...>, T...>
        {
          using type =
            typename RemoveTupleRedundancies<
              std::tuple<typename Selector<Features>::template type<T...>...>
            >::type;
        };

        template <typename T>
        struct Filter : std::is_same<Selector<T>, Empty> { };

        template <typename... T>
        using Bases =
            typename Select<
              typename FilterTuple<
                Filter,
                typename FlattenFeatures<FeatureListT>::type
              >::type,
              T...
            >::type;

        template <typename... T>
        struct S : std::tuple_size<Bases<T...>> { };

        template <typename... T>
        using IndexSequence = std::make_index_sequence<S<T...>::value>;

        template <typename>
        struct Impl;

        template <std::size_t... I>
        struct Impl<std::index_sequence<I...>>
        {
          template<typename... T>
          class type
              : public virtual std::tuple_element<I, Bases<T...>>::type... { };
        };

        template <typename... T>
        using type = typename Impl<IndexSequence<T...>>::template type<T...>;
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
      public: struct RequiredFeatures : FeatureList<Features...> { };
    };


// Macros for generating EngineTemplate, LinkTemplate, etc
#define DETAIL_IGN_PHYSICS_DEFINE_ENTITY_WITH_POLICY(X, P) \
  template <typename List> \
  using X ## P = X <::ignition::physics::FeaturePolicy ## P, List>; \
  template <typename List> \
  using X ## P ## Ptr = X ## Ptr < \
    ::ignition::physics::FeaturePolicy ## P, List>; \
  template <typename List> \
  using Const ## X ## P ## Ptr = X ## Ptr < \
    ::ignition::physics::FeaturePolicy ## P, List>; \
  using Base ## X ## P ## Ptr = Base ## X ## Ptr < \
    ::ignition::physics::FeaturePolicy ## P>; \
  using ConstBase ## X ## P ## Ptr = ConstBase ## X ## Ptr <\
    ::ignition::physics::FeaturePolicy ## P>;


#define DETAIL_IGN_PHYSICS_DEFINE_ENTITY(X) \
  namespace detail { \
    IGN_PHYSICS_CREATE_SELECTOR(X) \
    /* Symbol used by X-types to identify other X-types */ \
    struct X ## Identifier { }; \
  } \
  template <typename PolicyT, typename FeaturesT> \
  class X : public ::ignition::physics::detail::ExtractAPI< \
        detail::Select ## X, FeaturesT> \
          ::template type<PolicyT, FeaturesT>, \
      public virtual Entity<PolicyT, FeaturesT> \
  { \
    public: using Identifier = detail:: X ## Identifier; \
    public: using UpcastIdentifiers = std::tuple<detail:: X ## Identifier>; \
    public: using Base = Entity<PolicyT, FeaturesT>; \
    \
    public: X(const X&) = default; \
    \
    public: X(const std::shared_ptr<typename Base::Pimpl> &_pimpl, \
              const Identity &_identity) \
      : Entity<PolicyT, FeaturesT>(_pimpl, _identity) { } \
    public: X(std::shared_ptr<typename Base::Pimpl> &&_pimpl, \
              const Identity &_identity) \
      : Entity<PolicyT, FeaturesT>(std::move(_pimpl), _identity) { } \
  }; \
  template <typename PolicyT, typename FeaturesT> \
  using X ## Ptr = ::ignition::physics::EntityPtr< \
    X <PolicyT, FeaturesT> >; \
  template <typename PolicyT, typename FeaturesT> \
  using Const ## X ## Ptr = ::ignition::physics::EntityPtr< \
    const X <PolicyT, FeaturesT> >; \
  template <typename PolicyT> \
  using Base ## X ## Ptr = ::ignition::physics::EntityPtr< \
    X <PolicyT, ::ignition::physics::FeatureList< \
        ::ignition::physics::Feature>>>; \
  template <typename PolicyT> \
  using ConstBase ## X ## Ptr = ::ignition::physics::EntityPtr< \
    const X <PolicyT, ::ignition::physics::FeatureList< \
        ::ignition::physics::Feature>>>; \
  DETAIL_IGN_PHYSICS_DEFINE_ENTITY_WITH_POLICY(X, 3d) \
  DETAIL_IGN_PHYSICS_DEFINE_ENTITY_WITH_POLICY(X, 2d) \
  DETAIL_IGN_PHYSICS_DEFINE_ENTITY_WITH_POLICY(X, 3f) \
  DETAIL_IGN_PHYSICS_DEFINE_ENTITY_WITH_POLICY(X, 2f)


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
    DETAIL_IGN_PHYSICS_DEFINE_ENTITY(Engine)
    DETAIL_IGN_PHYSICS_DEFINE_ENTITY(World)
    DETAIL_IGN_PHYSICS_DEFINE_ENTITY(Model)
    DETAIL_IGN_PHYSICS_DEFINE_ENTITY(Link)
    DETAIL_IGN_PHYSICS_DEFINE_ENTITY(Joint)
    DETAIL_IGN_PHYSICS_DEFINE_ENTITY(Shape)

    namespace detail
    {
      template <typename T>
      struct ImplementationSelector
      {
        public: template <typename PolicyT>
        using type = typename T::template Implementation<PolicyT>;
      };

      template <typename PolicyT, typename List>
      using ExtractImplementation =
        typename ExtractAPI<ImplementationSelector, List>
            ::template type<PolicyT>;
    }
  }
}

#endif
