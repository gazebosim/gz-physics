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

#ifndef IGNITION_PHYSICS_FEATURELIST_HH_
#define IGNITION_PHYSICS_FEATURELIST_HH_

#include <ignition/physics/Feature.hh>

namespace ignition
{
  namespace physics
  {
    namespace detail
    {
      // Forward declarations
      template <typename...> class CombineLists;
      template <bool, typename> struct SelfConflict;
    }

    /////////////////////////////////////////////////
    /// \brief Use a FeatureList to aggregate a list of Features.
    ///
    /// FeatureLists can be constructed in hierarchies, e.g. a FeatureList can
    /// be passed into another FeatureList, and the set of all features in the
    /// new list will be the sum.
    ///
    /// \code
    /// // FeatureA, FeatureB, AdvancedA, and AdvancedB are all feature classes.
    ///
    /// using BasicList = FeatureList<FeatureA, FeatureB>;
    /// using AdvancedList = FeatureList<BasicList, AdvancedA, AdvancedB>;
    /// \endcode
    template <typename... FeaturesT>
    struct FeatureList
    {
      /// Features is a std::tuple containing all the feature classes that are
      /// bundled in this list. This list is fully seralialized; any hierarchy
      /// that was used to construct this FeatureList will be collapsed in this
      /// member.
      public: using Features =
          typename detail::CombineLists<FeaturesT...>::Result;

      /// \brief A static constexpr function which indicates whether a given
      /// Feature, F, is contained in this list.
      /// \tparam F
      ///   The feature class to check for in this FeatureList
      /// \return true if F is in this FeatureList; false otherwise.
      public: template <typename F>
      static constexpr bool HasFeature();

      /// \brief A static constexpr function which indicates whether any
      /// features in SomeFeatureList conflict with any features in
      /// SomeFeatureList.
      ///
      /// \tparam SomeFeatureList
      ///   The list to compare against for conflicts.
      /// \return true if any features in SomeFeatureList conflict with this
      /// list or vice versa.
      public: template <typename SomeFeatureList,
                        bool AssertNoConflict = false>
      static constexpr bool ConflictsWith();

      /// \brief A list has no additional required features
      public: using RequiredFeatures = void;

      // Check that this FeatureList does not contain any self-conflicts.
      static_assert(!detail::SelfConflict<true, Features>::value,
          "FeatureList ERROR: YOUR LIST CONTAINS CONFLICTING FEATURES!");
    };

    /////////////////////////////////////////////////
    /// \brief If your feature is known to conflict with any other feature, then
    /// you should have your feature class inherit FeatureWithConflicts<...>,
    /// and pass it a list of the features that it conflicts with.
    template <typename... ConflictingFeatures>
    struct FeatureWithConflicts;

    /////////////////////////////////////////////////
    /// \brief If your feature is known to require any other features, then you
    /// should have your feature class inherit FeatureWithRequirements<...>,
    /// and pass it a list of the features that it requires.
    template <typename... RequiredFeatures>
    struct FeatureWithRequirements;
  }
}

#include <ignition/physics/detail/FeatureList.hh>

#endif
