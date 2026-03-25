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

#ifndef GZ_PHYSICS_DETAIL_FEATURELIST_HH_
#define GZ_PHYSICS_DETAIL_FEATURELIST_HH_

#include <memory>
#include <set>
#include <string>
#include <tuple>
#include <type_traits>
#include <utility>

#include <gz/physics/FeatureList.hh>
#include <gz/physics/FeaturePolicy.hh>
#include <gz/physics/TemplateHelpers.hh>

namespace gz
{
  namespace physics
  {
    namespace detail
    {
      /////////////////////////////////////////////////
      /// \private ComposePluginFromList takes a TypeList of fully-instantiated
      /// Feature Implementation classes (e.g., FeatureA::Implementation<Policy>,
      /// FeatureB::Implementation<Policy>) and generates a linear inheritance chain
      /// of SpecializedPlugin instances.
      ///
      /// This tail-recursive linear chain avoids the "Ambiguous Base Class"
      /// compiler error that would occur with flat multiple inheritance if
      /// multiple features happen to share the same underlying base class from
      /// gz-plugin.
      template <typename List>
      struct ComposePluginFromList;

      template <typename Impl, typename... Others>
      struct ComposePluginFromList<TypeList<Impl, Others...>>
      {
        struct type :
            ::gz::plugin::SpecializedPlugin<Impl>,
            ComposePluginFromList<TypeList<Others...>>::type { };
      };

      template <>
      struct ComposePluginFromList<TypeList<>>
      {
        struct type { };
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
      template <typename F, typename = std::void_t<> >
      class ExtractFeatures
          : public VerifyFeatures<F>
      {
        public: using type = TypeList<F>;
      };

      /// \private This specialization of ExtractFeatures is used to wipe away
      /// a tuple that is currently holding a set of features, then verify those
      /// features, and finally repackage them as a TypeList.
      template <typename... F>
      class ExtractFeatures<std::tuple<F...>>
          : public VerifyFeatures<F...>
      {
        public: using type = TypeList<F...>;
      };

      /// \private This specialization of ExtractFeatures is used to wipe away
      /// a TypeList that is currently holding a set of features, then verify
      /// those features, and finally repackage them as a TypeList.
      template <typename... F>
      class ExtractFeatures<TypeList<F...>>
          : public VerifyFeatures<F...>
      {
        public: using type = TypeList<F...>;
      };

      template <typename T> struct TupleToTypeList;
      template <typename... Ts>
      struct TupleToTypeList<std::tuple<Ts...>>
      {
        using type = TypeList<Ts...>;
      };

      /// \private This specialization of ExtractFeatures is used to wipe away
      /// the FeatureList that is currently holding a set of features, then
      /// verify those features, and finally repackage them as the raw feature
      /// TypeList that is being held by the FeatureList.
      template <typename SomeFeatureList>
      class ExtractFeatures<
              SomeFeatureList,
              std::void_t<typename SomeFeatureList::Features>>
          : public VerifyFeatures<typename SomeFeatureList::Features>
      {
        public: using type =
            typename TupleToTypeList<typename SomeFeatureList::Features>::type;
      };

      /// \private This specialization skips over any void entries. This allows
      /// users or template metaprograms to place `void` into feature entries
      /// and have those entries be gracefully skipped over.
      template <>
      class ExtractFeatures<void>
      {
        public: using type = TypeList<>;
      };

      /////////////////////////////////////////////////
      /// \private This struct wraps the TypeListContainsBase class to create a
      /// TypeList filter that can be passed to FilterList.
      template <typename DiscardList>
      struct RedundantListFilter
      {
        template <typename T>
        struct Apply : TypeListContainsBase<T, DiscardList> { };
      };

      /////////////////////////////////////////////////
      template <template <typename> class Filter, typename InputList>
      struct FilterList;

      /////////////////////////////////////////////////
      /// \private This class will apply a Filter to a TypeList and produce a
      /// new TypeList that only includes the TypeList elements which were
      /// ignored by the Filter.
      template <template <typename> class Filter, typename... InputTypes>
      struct FilterList<Filter, TypeList<InputTypes...>>
      {
        using type = typename TypeListCat<
            std::conditional_t<
              // If the input type is a base class of anything that should be
              // discared ...
              Filter<InputTypes>::value,
              // ... then we should leave it out of the final TypeList ...
              TypeList<>,
              // ... otherwise, include it.
              TypeList<InputTypes>
            // Do this for each type in the InputTypes parameter pack.
            >...
        >::type;
      };

      /////////////////////////////////////////////////
      /// \private Use this struct to remove the TypeList elements of one
      /// TypeList from another TypeList
      template <typename DiscardList>
      struct SubtractList
      {
        template <typename T>
        using Filter =
            typename RedundantListFilter<DiscardList>::template Apply<T>;

        /// This struct will contain a nested struct called `type` which will be
        /// a TypeList with all the elements of FromList that are not present in
        /// DiscardList
        template <typename FromList>
        struct From : FilterList<Filter, FromList> { };
      };

      /////////////////////////////////////////////////
      /// \private This template takes in a TypeList as InputList and gives back
      /// a TypeList where every element type is only present once.
      template <typename InputList, typename AccumulatedList = TypeList<>>
      struct RemoveListRedundancies;

      template <typename AccumulatedList>
      struct RemoveListRedundancies<TypeList<>, AccumulatedList>
      {
        using type = TypeList<>;
      };

      template <typename F1, typename... Others, typename... Acc>
      struct RemoveListRedundancies<TypeList<F1, Others...>, TypeList<Acc...>>
      {
        using PartialResult =
            std::conditional_t<
              TypeListContainsBase<F1, TypeList<Acc...>>::value,
              TypeList<>,
              TypeList<F1>
            >;

        using NextAcc =
            typename TypeListCat<TypeList<Acc...>, PartialResult>::type;

        using type = typename TypeListCat<
            PartialResult,
            typename RemoveListRedundancies<TypeList<Others...>, NextAcc>::type
        >::type;
      };

      template <typename InputTuple>
      struct RemoveTupleRedundancies;

      template <typename... InputTupleArgs>
      struct RemoveTupleRedundancies<std::tuple<InputTupleArgs...>>
      {
        using type = typename ToTuple<
            typename RemoveListRedundancies<TypeList<InputTupleArgs...>>::type
        >::type;
      };

      /////////////////////////////////////////////////
      /// \private This template is used to take a hierarchy of FeatureLists and
      /// flatten them into a single tuple.
      template <typename FeatureTuple, typename = std::void_t<>>
      struct FlattenFeatures;

      /////////////////////////////////////////////////
      /// \private This template is a helper for FlattenFeatures
      template <typename FeatureOrList, typename = std::void_t<>>
      struct ExpandFeatures
      {
        using type = std::conditional_t<
            std::is_void_v<typename FeatureOrList::RequiredFeatures>,
            TypeList<FeatureOrList>,
            typename TypeListCat<
              TypeList<FeatureOrList>,
              typename FlattenFeatures<
                typename FeatureOrList::RequiredFeatures>::type>::type
        >;
      };

      /////////////////////////////////////////////////
      template <typename List>
      struct ExpandFeatures<List, std::void_t<typename List::FeatureTuple>>
      {
        using type =
            typename FlattenFeatures<typename List::FeatureTuple>::type;
      };

      /////////////////////////////////////////////////
      template <typename FeatureListT>
      struct FlattenFeatures<
          FeatureListT, std::void_t<typename FeatureListT::FeatureTuple>>
      {
        using type =
            typename FlattenFeatures<typename FeatureListT::FeatureTuple>::type;
      };

      /////////////////////////////////////////////////
      template <typename... Features>
      struct FlattenFeatures<std::tuple<Features...>, std::void_t<>>
      {
        using type = typename TypeListCat<
            typename ExpandFeatures<Features>::type...>::type;
      };

      /////////////////////////////////////////////////
      template <typename... Features>
      struct FlattenFeatures<TypeList<Features...>, std::void_t<>>
      {
        using type = typename TypeListCat<
            typename ExpandFeatures<Features>::type...>::type;
      };

      /////////////////////////////////////////////////
      template <>
      struct FlattenFeatures<void>
      {
        using type = TypeList<>;
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
        using type = TypeList<>;
      };

      template <typename ParentResultInput, typename F1, typename... Others>
      struct CombineListsImpl<ParentResultInput, F1, Others...>
      {
        // Add the features of the feature list F1, while filtering out any
        // repeated features.
        using InitialResult =
            typename SubtractList<ParentResultInput>
            ::template From<typename ExtractFeatures<F1>::type>::type;

        // Add the features that are required by F1, while filtering out any
        // repeated features.
        using PartialResult = typename TypeListCat<
            InitialResult,
            typename SubtractList<
              typename TypeListCat<ParentResultInput, InitialResult>::type>
              ::template From<
                typename ExtractFeatures<typename F1::RequiredFeatures>::type
            >::type>::type;

        // Define the TypeList that the child should use to filter its list
        using ChildFilter =
            typename TypeListCat<ParentResultInput, PartialResult>::type;

        // Construct the final result
        using type = typename TypeListCat<
            PartialResult,
            typename CombineListsImpl<ChildFilter, Others...>::type>::type;
      };

      /// \private CombineLists is used to take variadic lists of features,
      /// FeatureLists, or TypeLists of features, and collapse them into a
      /// serialized std::tuple of features.
      ///
      /// This class works recursively.
      template <typename... FeatureLists>
      struct CombineLists
      {
        public: using Result = typename ToTuple<
            typename CombineListsImpl<TypeList<>, FeatureLists...>::type>::type;
      };

      /////////////////////////////////////////////////
      /// \private This class helps to implement the function
      /// FeatureList::ConflictsWith(). Its implementation is conceptually
      /// similar to TypeListContainsBase.
      template <typename SomeFeatureList, bool AssertNoConflict, typename Tuple>
      struct ConflictingLists;

      /// \private Implementation of ConflictingLists. If the Tuple argument is
      /// not a TypeList, this class will be undefined.
      template <typename SomeFeatureList, bool AssertNoConflict,
                typename... Features>
      struct ConflictingLists<
          SomeFeatureList, AssertNoConflict, TypeList<Features...>>
          : std::integral_constant<bool,
              (Features::template ConflictsWith<
                SomeFeatureList, AssertNoConflict>() || ...)> { };

      /////////////////////////////////////////////////
      /// \private Check whether any feature within a TypeList of features
      /// conflicts with any other feature in that TypeList.
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
      template <template<typename> class Selector, typename FeatureListT>
      struct ExtractAPI
      {
        template <typename FeatureTuple, typename... T>
        struct Select;

        template <typename... Features, typename... T>
        struct Select<TypeList<Features...>, T...>
        {
          using type = typename RemoveListRedundancies<
              TypeList<typename Selector<Features>::template type<T...>...>
            >::type;
        };

        template <typename T>
        struct Filter : std::is_same<Selector<T>, Empty> { };

        template <typename... T>
        using Bases =
            typename Select<
              typename FilterList<
                Filter,
                typename FlattenFeatures<FeatureListT>::type
              >::type,
              T...
            >::type;

        template <typename List>
        struct Impl;

        template <typename... B>
        struct Impl<TypeList<B...>>
        {
          class type : public virtual B... { };
        };

        template <typename... T>
        using type = typename Impl<Bases<T...>>::type;
      };
    }

    /////////////////////////////////////////////////
    template <typename... FeaturesT>
    template <typename F>
    constexpr bool FeatureList<FeaturesT...>::HasFeature()
    {
      return detail::TypeListContainsBase<F, FlatFeatureTypeList>::value;
    }

    /////////////////////////////////////////////////
    template <typename... FeaturesT>
    template <typename SomeFeatureList, bool AssertNoConflict>
    constexpr bool FeatureList<FeaturesT...>::ConflictsWith()
    {
      return
          detail::ConflictingLists<
              SomeFeatureList, AssertNoConflict, FlatFeatureTypeList>::value
       || detail::ConflictingLists<
              FeatureList<FeaturesT...>, AssertNoConflict,
              typename FeatureList<SomeFeatureList>::FlatFeatureTypeList>::
              value;
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
#define DETAIL_GZ_PHYSICS_DEFINE_ENTITY_WITH_POLICY(X, P) \
  template <typename List> \
  using X ## P = X <::gz::physics::FeaturePolicy ## P, List>; \
  template <typename List> \
  using X ## P ## Ptr = X ## Ptr < \
    ::gz::physics::FeaturePolicy ## P, List>; \
  template <typename List> \
  using Const ## X ## P ## Ptr = X ## Ptr < \
    ::gz::physics::FeaturePolicy ## P, List>; \
  using Base ## X ## P ## Ptr = Base ## X ## Ptr < \
    ::gz::physics::FeaturePolicy ## P>; \
  using ConstBase ## X ## P ## Ptr = ConstBase ## X ## Ptr <\
    ::gz::physics::FeaturePolicy ## P>;


#define DETAIL_GZ_PHYSICS_DEFINE_ENTITY(X) \
  namespace detail { \
    GZ_PHYSICS_CREATE_SELECTOR(X) \
    /* Symbol used by X-types to identify other X-types */ \
    struct X ## Identifier { }; \
    template <typename PolicyT, typename FeaturesT> \
    struct X ## _API : public ::gz::physics::detail::ExtractAPI< \
          detail::Select ## X, FeaturesT> \
            ::template type<PolicyT, FeaturesT> { }; \
  } \
  template <typename PolicyT, typename FeaturesT> \
  class X : public detail:: X ## _API <PolicyT, FeaturesT>, \
      public virtual Entity<PolicyT, FeaturesT> \
  { \
    public: using Identifier = detail:: X ## Identifier; \
    public: using UpcastIdentifiers = TypeList<detail:: X ## Identifier>; \
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
  using X ## Ptr = ::gz::physics::EntityPtr< \
    X <PolicyT, FeaturesT> >; \
  template <typename PolicyT, typename FeaturesT> \
  using Const ## X ## Ptr = ::gz::physics::EntityPtr< \
    const X <PolicyT, FeaturesT> >; \
  template <typename PolicyT> \
  using Base ## X ## Ptr = ::gz::physics::EntityPtr< \
    X <PolicyT, ::gz::physics::FeatureList< \
        ::gz::physics::Feature>>>; \
  template <typename PolicyT> \
  using ConstBase ## X ## Ptr = ::gz::physics::EntityPtr< \
    const X <PolicyT, ::gz::physics::FeatureList< \
        ::gz::physics::Feature>>>; \
  DETAIL_GZ_PHYSICS_DEFINE_ENTITY_WITH_POLICY(X, 3d) \
  DETAIL_GZ_PHYSICS_DEFINE_ENTITY_WITH_POLICY(X, 2d) \
  DETAIL_GZ_PHYSICS_DEFINE_ENTITY_WITH_POLICY(X, 3f) \
  DETAIL_GZ_PHYSICS_DEFINE_ENTITY_WITH_POLICY(X, 2f)


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
    DETAIL_GZ_PHYSICS_DEFINE_ENTITY(Engine)
    DETAIL_GZ_PHYSICS_DEFINE_ENTITY(World)
    DETAIL_GZ_PHYSICS_DEFINE_ENTITY(Model)
    DETAIL_GZ_PHYSICS_DEFINE_ENTITY(Link)
    DETAIL_GZ_PHYSICS_DEFINE_ENTITY(Joint)
    DETAIL_GZ_PHYSICS_DEFINE_ENTITY(Shape)

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

      /////////////////////////////////////////////////
      /// \private This class is used to determine what type of
      /// SpecializedPluginPtr should be used by the entities provided by a
      /// plugin.
      template <typename Policy, typename FeaturesT>
      struct DeterminePlugin
      {
        template <typename T>
        struct Selector
        {
          template <typename PolicyT>
          using type = typename T::template Implementation<PolicyT>;
        };

        struct Specializer
            : ::gz::plugin::detail::SelectSpecializers<
              typename ComposePluginFromList<
                  typename ExtractAPI<Selector, FeaturesT>
                      ::template Bases<Policy>
              >::type> { };

        using type = ::gz::plugin::TemplatePluginPtr<Specializer>;
      };
    }
  }
}

#endif
