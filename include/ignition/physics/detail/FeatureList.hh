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
      /// \private A specialization of DeterminePlugin that can accept a
      /// FeatureList
      template <typename Policy, typename... FeaturesT>
      struct DeterminePlugin<Policy, FeatureList<FeaturesT...>>
      {
        using type = typename DeterminePlugin<
            Policy, typename FeatureList<FeaturesT...>::Features>::type;
      };

      template <typename Policy, typename NamedFeatureList>
      struct DeterminePlugin
      {
        using type = typename DeterminePlugin<
            Policy, typename NamedFeatureList::Features>::type;
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
      template <typename SomeFeatureList>
      class ExtractFeatures<
              SomeFeatureList,
              void_t<typename SomeFeatureList::Features>>
          : public VerifyFeatures<typename SomeFeatureList::Features>
      {
        public: using Result = typename SomeFeatureList::Features;
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
      template <typename DiscardTuple, typename InputTuple>
      struct FilterTuple;

      /////////////////////////////////////////////////
      template <typename DiscardTuple, typename... InputTypes>
      struct FilterTuple<DiscardTuple, std::tuple<InputTypes...>>
      {
        using Result = decltype(std::tuple_cat(
            std::conditional_t<
              // If the input type is a base class of anything that should be
              // discared ...
              TupleContainsBase<InputTypes, DiscardTuple>::value,
              // ... then we should leave it out of the final tuple ...
              std::tuple<>,
              // ... otherwise, include it.
              std::tuple<InputTypes>
            // Do this for each type in the InputTypes parameter pack.
            >()...));
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
        using Result = std::tuple<>;
      };

      template <typename ParentResultInput, typename F1, typename... Others>
      struct CombineListsImpl<ParentResultInput, F1, Others...>
      {
        // Add the features of the feature list F1, while filtering out any
        // repeated features.
        using InitialResult =
            typename FilterTuple<
              ParentResultInput,
              typename ExtractFeatures<F1>::Result
            >::Result;

        // Add the features that are required by F1, while filtering out any
        // repeated features.
        using PartialResult = decltype(std::tuple_cat(
            InitialResult(),
            typename FilterTuple<
              decltype(std::tuple_cat(ParentResultInput(), InitialResult())),
              typename ExtractFeatures<typename F1::RequiredFeatures>::Result
            >::Result()));

        // Define the tuple that the child should use to filter its list
        using ChildFilter =
            decltype(std::tuple_cat(ParentResultInput(), PartialResult()));

        // Construct the final result
        using Result = decltype(std::tuple_cat(
            PartialResult(),
            typename CombineListsImpl<ChildFilter, Others...>::Result()));
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
            typename CombineListsImpl<std::tuple<>, FeatureLists...>::Result;
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

      template <template<typename> class Selector, typename FeatureListT>
      struct Aggregate<Selector, FeatureListT,
                        void_t<typename FeatureListT::FeatureTuple>>
      {
        public: template<typename... T>
        class type
          : public virtual Aggregate<Selector,
              typename FeatureListT::FeatureTuple>::template type<T...> { };
      };

      /// \private Recursively extract the API out of a std::tuple of features
      template <template<typename> class Selector,
                typename F1, typename... Remaining>
      struct Aggregate<Selector, std::tuple<F1, Remaining...>, void_t<>>
      {
        public: template<typename... T>
        class type
            : public virtual Aggregate<Selector, F1>::template type<T...>,
              public virtual Aggregate<
                  Selector, std::tuple<Remaining...>>::template type<T...> { };
      };

      /// \private Terminate the recursion
      template <template<typename> class Selector>
      struct Aggregate<Selector, std::tuple<>>
      {
        public: template <typename... P>
        class type { };
      };

      /////////////////////////////////////////////////
      template <template<typename> class Selector, typename FeatureListT>
      struct ExtractAPI
      {
        public: template<typename... T>
        class type :
            public Aggregate<Selector, FeatureListT>::template type<T...> { };
      };

      /////////////////////////////////////////////////
      /// \private This class is used to inspect what features are provided by
      /// a plugin. It implements the API of RequestEngine.
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
          // TODO(MXG): Consider replacing with a fold expression
          return _pimpl && _pimpl->template HasInterface<Interface>()
              && InspectFeatures<PolicyT, std::tuple<Remaining...>>::
                      Verify(_pimpl);
        }

        template <typename LoaderT, typename ContainerT>
        static void EraseIfMissing(
            const LoaderT &_loader,
            ContainerT &_plugins)
        {
          const auto acceptable =
              _loader.template PluginsImplementing<Interface>();

          std::set<std::string> unacceptable;
          for (const std::string &p : _plugins)
          {
            const auto it = std::find(acceptable.begin(), acceptable.end(), p);
            if (it == acceptable.end())
              unacceptable.insert(unacceptable.end(), p);
          }

          for (const std::string &u : unacceptable)
            _plugins.erase(u);

          InspectFeatures<PolicyT, std::tuple<Remaining...>>::EraseIfMissing(
                _loader, _plugins);
        }

        template <typename PtrT>
        static void MissingNames(const PtrT &_pimpl,
                                 std::set<std::string> &_names)
        {
          if (!_pimpl || !_pimpl->template HasInterface<Interface>())
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

        template <typename LoaderT, typename ContainerT>
        static void EraseIfMissing(const LoaderT &, ContainerT &)
        {
          // Do nothing
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
    X <PolicyT, ::ignition::physics::FeatureList<>>>; \
  template <typename PolicyT> \
  using ConstBase ## X ## Ptr = ::ignition::physics::EntityPtr< \
    const X <PolicyT, ::ignition::physics::FeatureList<>>>; \
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
        typename ExtractAPI<ImplementationSelector, typename List::Features>::
            template type<PolicyT>;
    }
  }
}

#endif
