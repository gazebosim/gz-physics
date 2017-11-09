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

#include <ignition/physics/Feature.hh>

namespace ignition
{
  namespace physics
  {
    /// \brief This default definition of FeatureList will only get called when
    /// the list of Features is empty. Therefore, we just provide placeholder
    template <typename... Features>
    struct FeatureList : public virtual Feature { };

    /// \brief This specialization of FeatureList will be called when one or
    /// more features are provided as template arguments. We aggregate the APIs
    /// of the Engine, Link, Joint, and Model of all the features which are
    /// given.
    template <typename _Feature, typename... RemainingFeatures>
    struct FeatureList<_Feature, RemainingFeatures...>
        : public virtual _Feature,
          public virtual FeatureList<
            typename _Feature::RequiredFeatures,
            RemainingFeatures...>
    {
      static_assert(std::is_base_of<Feature, _Feature>::value,
                    "FEATURELIST ERROR: YOU ARE ATTEMPTING TO ADD A CLASS "
                    "WHICH IS NOT A Feature TO A FeatureList! MAKE SURE THAT "
                    "YOUR CLASS VIRTUALLY INHERITS THE Feature BASE CLASS!");

      public: using CurrentFeature = _Feature;
      public: using NextFeature = FeatureList<RemainingFeatures...>;

      public: template <typename FeatureType>
      class Engine
          : public virtual CurrentFeature::template Engine<FeatureType>,
            public virtual NextFeature::template Engine<FeatureType> { };

      public: template <typename FeatureType>
      class Link
          : public virtual CurrentFeature::template Link<FeatureType>,
            public virtual NextFeature::template Link<FeatureType> { };

      public: template <typename FeatureType>
      class Joint
          : public virtual CurrentFeature::template Joint<FeatureType>,
            public virtual NextFeature::template Joint<FeatureType> { };

      public: template <typename FeatureType>
      class Model
          : public virtual CurrentFeature::template Model<FeatureType>,
            public virtual NextFeature::template Model<FeatureType> { };

      template <typename SomeFeatureList>
      static constexpr bool ConflictsWith()
      {
        return CurrentFeature::template ConflictsWith<SomeFeatureList>()
               || NextFeature::template ConflictsWith<SomeFeatureList>();
      }

      static_assert(!CurrentFeature::template ConflictsWith<NextFeature>(),
        "FEATURELIST ERROR: YOUR FEATURE LIST CONTAINS CONFLICTING FEATURES!");

      static_assert(!NextFeature::template ConflictsWith<CurrentFeature>(),
        "FEATURELIST ERROR: YOUR FEATURE LIST CONTAINS CONFLICTING FEATURES!");

      using RequiredFeatures = FeatureList<
          typename CurrentFeature::RequiredFeatures,
          typename NextFeature::RequiredFeatures>;
    };

    template<>
    struct FeatureList<void, void> : public virtual Feature { };

    /// \brief This specialization of FeatureList will be called when a listed
    /// feature does not have any required features. This will also be called
    /// if someone puts "void" in their FeatureList for some reason.
    ///
    /// Effectively, this is gracefully filtering out any instances of "void"
    /// within a feature list.
    template <typename... RemainingFeatures>
    struct FeatureList<void, RemainingFeatures...>
        : public virtual FeatureList<RemainingFeatures...> { };

    /// \brief The default definition of FeatureWithConflicts only gets called
    /// when the ConflictingFeatures list is empty. It should simply fallback on
    /// the default behavior of a blank feature.
    template <typename... ConflictingFeatures>
    struct FeatureWithConflicts : public virtual Feature { };

    /// \brief This template specialization of FeatureWithConflicts will be
    /// called when one or more features are listed as conflicts.
    template <typename Conflict, typename... RemainingConflicts>
    struct FeatureWithConflicts<Conflict, RemainingConflicts...>
        : public virtual Feature
    {
      public: template <typename SomeFeatureList>
      static constexpr bool ConflictsWith()
      {
        return std::is_base_of<Conflict, SomeFeatureList>()
            || FeatureWithConflicts<RemainingConflicts...>
                ::template ConflictsWith<SomeFeatureList>();
      }
    };

    /// \brief The FeatureWithRequirements class simply wraps up its required
    /// features in a FeatureList and then sets the RequiredFeatures type.
    template <typename... Features>
    struct FeatureWithRequirements
    {
      public: using RequiredFeatures = FeatureList<Features...>;
    };
  }
}

#endif
