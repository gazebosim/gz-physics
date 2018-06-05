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

#ifndef IGNITION_PHYSICS_DETAIL_FEATURE_HH_
#define IGNITION_PHYSICS_DETAIL_FEATURE_HH_

#include <type_traits>
#include <tuple>

#include <ignition/physics/Feature.hh>
#include <ignition/physics/TemplateHelpers.hh>

namespace ignition
{
  namespace physics
  {
    namespace detail
    {
      template <typename... F>
      class VerifyFeatures { };

      template <typename F, typename... Others>
      class VerifyFeatures<F, Others...>
          : public VerifyFeatures<Others...>
      {
        static_assert(std::is_base_of<Feature, F>::value,
                      "FEATURELIST ERROR: YOU ARE ATTEMPTING TO ADD A CLASS "
                      "WHICH IS NOT A Feature TO A FeatureList! MAKE SURE THAT "
                      "YOUR CLASS VIRTUALLY INHERITS THE Feature BASE CLASS!");
      };

      template <typename F>
      class ExtractFeatures
          : public VerifyFeatures<F>
      {
        public: using Result = std::tuple<F>;
      };

      template <typename... F>
      class ExtractFeatures<std::tuple<F...>>
          : public VerifyFeatures<F...>
      {
        public: using Result = std::tuple<F...>;
      };

      template <typename... F>
      class ExtractFeatures<FeatureList<F...>>
          : public VerifyFeatures<F...>
      {
        public: using Result = typename FeatureList<F...>::Features;
      };

      template <>
      class ExtractFeatures<void>
      {
        public: using Result = std::tuple<>;
      };


      template <typename... FeatureLists>
      class CombineLists;

      template <typename F>
      class CombineLists<F>
      {
        public: using Result = decltype(std::tuple_cat(
            typename ExtractFeatures<F>::Result(),
            typename ExtractFeatures<typename F::RequiredFeatures>::Result()));
      };


      template <typename F1, typename... Others>
      class CombineLists<F1, Others...>
      {
        public: using Result =
          decltype(std::tuple_cat(
            typename ExtractFeatures<F1>::Result(),
            typename ExtractFeatures<typename F1::RequiredFeatures>::Result(),
            typename CombineLists<Others...>::Result()));
      };

      // Inspired by https://stackoverflow.com/a/26288164
      template <typename T, typename Tuple>
      struct TupleContainsBase;

      template <typename T, typename... Types>
      struct TupleContainsBase<T, std::tuple<Types...>>
          : std::integral_constant<bool,
              !std::is_same<
                std::tuple<typename std::conditional<
                  std::is_base_of<T, Types>::value, Empty, Types>::type...>,
                std::tuple<Types...>
              >::value> { };
    }

    template <typename... FeaturesT>
    class FeatureList
    {
      public: using Features =
          typename detail::CombineLists<FeaturesT...>::Result;

      public: template <typename F>
      static constexpr bool HasFeature()
      {
        return detail::TupleContainsBase<F, Features>::value;
      }

//      public: template <typename SomeFeatureList,
//                        bool AssertNoConflict = false>
//      static constexpr bool ConflictsWith()
//      {
//        return
//      }

      public: using RequiredFeatures = void;
    };



//    /////////////////////////////////////////////////
//    /// \brief This default definition of FeatureList will only get called when
//    /// the list of Features is empty. Therefore, we just have it inherit the
//    /// completely generic Feature base class.
//    template <typename... Features>
//    struct FeatureList : public virtual Feature { };

//    namespace detail
//    {
//      /////////////////////////////////////////////////
//      /// \brief FeatureListHelper is used to help the FeatureList class to
//      /// pull in the RequiredFeatures of the Features that are given to it.
//      /// When a Feature has no RequiredFeatures, the RequiredFeatures field is
//      /// a void type. When that occurs, this default definition of
//      /// FeatureListHelper will be skipped for the next specialized definition.
//      template <typename F1, typename F2>
//      struct FeatureListHelper : public virtual FeatureList<F1, F2> { };

//      /////////////////////////////////////////////////
//      /// \brief This specialized definition of FeatureListHelper will be called
//      /// when the FeatureList class is given a Feature that does not have any
//      /// RequiredFeatures. This specialization will simply ignore the void type
//      /// so that it does not pollute the inheritance structure.
//      template <typename F2>
//      struct FeatureListHelper<void, F2> : public virtual F2 { };

//      template <bool same, typename ClassA, typename ClassB>
//      struct CombineIfDifferentImpl
//          : public virtual ClassA,
//            public virtual ClassB { };

//      template <typename ClassA, typename ClassB>
//      struct CombineIfDifferentImpl<true, ClassA, ClassB>
//          : public virtual ClassA { };


//      template <typename ClassA, typename ClassB>
//      struct CombineIfDifferent
//          : CombineIfDifferentImpl<
//              std::is_same<ClassA, ClassB>::value, ClassA, ClassB> { };
//    }

//    /////////////////////////////////////////////////
//    /// \brief This specialization of FeatureList will be called when one or
//    /// more features are provided as template arguments. We aggregate the APIs
//    /// of the Engine, Link, Joint, and Model of all the features which are
//    /// given.
//    template <typename _F1, typename... RemainingFeatures>
//    struct FeatureList<_F1, RemainingFeatures...>
//        : public virtual _F1,
//          public virtual detail::FeatureListHelper<
//                  typename _F1::RequiredFeatures,
//                  FeatureList<RemainingFeatures...> >
//    {
//      static_assert(std::is_base_of<Feature, _F1>::value,
//                    "FEATURELIST ERROR: YOU ARE ATTEMPTING TO ADD A CLASS "
//                    "WHICH IS NOT A Feature TO A FeatureList! MAKE SURE THAT "
//                    "YOUR CLASS VIRTUALLY INHERITS THE Feature BASE CLASS!");

//      public: using CurrentFeature = _F1;
//      public: using NextFeature =
//         detail::FeatureListHelper<
//            typename _F1::RequiredFeatures,
//            FeatureList<RemainingFeatures...>>;

//      public: template <typename FeatureType>
//      class Engine
//          : public virtual CurrentFeature::template Engine<FeatureType>,
//            public virtual NextFeature::template Engine<FeatureType> { };

//      public: template <typename FeatureType>
//      class Link : public detail::CombineIfDifferent<
//          typename CurrentFeature::template Link<FeatureType>,
//          typename NextFeature::template Link<FeatureType>> { };

//      public: template <typename FeatureType>
//      class Joint : public detail::CombineIfDifferent<
//          typename CurrentFeature::template Joint<FeatureType>,
//          typename NextFeature::template Joint<FeatureType>> { };

//      public: template <typename FeatureType>
//      class Model : public detail::CombineIfDifferent<
//          typename CurrentFeature::template Model<FeatureType>,
//          typename NextFeature::template Model<FeatureType>> { };

//      template <typename SomeFeatureList, bool AssertNoConflict = false>
//      static constexpr bool ConflictsWith()
//      {
//        return CurrentFeature::
//                template ConflictsWith<SomeFeatureList, AssertNoConflict>()
//            || NextFeature::
//                template ConflictsWith<SomeFeatureList, AssertNoConflict>();
//      }

//    // Make sure the requested FeatureList does not contain any self-conflicts.
//    static_assert(!CurrentFeature::template ConflictsWith<NextFeature, true>(),
//    "FEATURELIST ERROR: YOUR LIST OF FEATURES CONTAINS CONFLICTING FEATURES!");

//    static_assert(!NextFeature::template ConflictsWith<CurrentFeature, true>(),
//    "FEATURELIST ERROR: YOUR LIST OF FEATURES CONTAINS CONFLICTING FEATURES!");


//      // Dev Note (MXG): Whenever a feature gets added to a FeatureList, all of
//      // its required features will also be added to the FeatureList. Therefore,
//      // the FeatureList itself will never have any required features, because
//      // any requirements of the individual features in the list will already
//      // have been added to the list.
//      using RequiredFeatures = void;
//    };

    /////////////////////////////////////////////////
    /// \brief The default definition of FeatureWithConflicts only gets called
    /// when the ConflictingFeatures list is empty. It should simply fall back
    /// on the default behavior of a blank feature.
    template <typename... ConflictingFeatures>
    struct FeatureWithConflicts : public virtual Feature { };

    /////////////////////////////////////////////////
    /// \brief This template specialization of FeatureWithConflicts will be
    /// called when one or more features are listed as conflicts.
    template <typename Conflict, typename... RemainingConflicts>
    struct FeatureWithConflicts<Conflict, RemainingConflicts...>
        : public virtual Feature
    {
      public: template <typename SomeFeatureList,
                        bool AssertNoConflict = false>
      static constexpr bool ConflictsWith()
      {
        static_assert(   !AssertNoConflict
                      || !std::is_base_of<Conflict, SomeFeatureList>::value,
                      "FEATURE CONFLICT DETECTED");

        return std::is_base_of<Conflict, SomeFeatureList>::value
            || FeatureWithConflicts<RemainingConflicts...>
                ::template ConflictsWith<SomeFeatureList, AssertNoConflict>();
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
  }
}

#endif
