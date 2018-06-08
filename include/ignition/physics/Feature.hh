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

#include <cstddef>
#include <tuple>

#include <ignition/physics/Export.hh>
#include <ignition/physics/Entity.hh>

namespace ignition
{
  namespace physics
  {
    /////////////////////////////////////////////////
    /// \brief This class defines the concept of a Feature. It should be
    /// inherited by classes that define some plugin feature.
    class IGNITION_PHYSICS_VISIBLE Feature
    {
      /// \brief Placeholder class for the Engine API. Every Engine feature
      /// MUST inherit this class.
      public: template <typename Policy, typename PimplT>
      class Engine : public virtual Entity<Policy, PimplT>
      {
        /// \brief Virtual destructor
        public: virtual ~Engine() = default;
      };

      /// \brief Placeholder class in case a Feature does not define its own
      /// World API
      public: template <typename Policy, typename PimplT>
      class World : public virtual Entity<Policy, PimplT>
      {
        /// \brief Virtual destructor
        public: virtual ~World() = default;
      };

      /// \brief Placeholder class in case a Feature does not define its own
      /// Model API
      public: template <typename Policy, typename PimplT>
      class Model : public virtual Entity<Policy, PimplT>
      {
        /// \brief Virtual destructor
        public: virtual ~Model() = default;
      };

      /// \brief Placeholder class in case a Feature does not define its own
      /// Link API
      public: template <typename Policy, typename PimplT>
      class Link : public virtual Entity<Policy, PimplT>
      {
        /// \brief Virtual destructor
        public: virtual ~Link() = default;
      };

      /// \brief Placeholder class in case a Feature does not define its own
      /// Joint API
      public: template <typename Policy, typename PimplT>
      class Joint : public virtual Entity<Policy, PimplT>
      {
        /// \brief Virtual destructor
        public: virtual ~Joint() = default;
      };

      public: template <typename Policy>
      class Implementation
      {
        /// \brief Tell the physics plugin to initiate a physics engine.
        ///
        /// Some physics plugins might be able to provide multiple simultaneous
        /// physics engines, in which case engineID might vary. The meaning of
        /// engineID is implementation-defined, but every physics plugins must
        /// support at least engineID==0 to be well-formed. If a physics plugin
        /// only supports having one engine at a time, then it may only support
        /// engineID==0.
        ///
        /// \return The Entity ID of the physics engine. For engineID==0 this
        /// should also return 0. In the event of an error (or an invalid
        /// engineID), this should return INVALID_ENTITY_ID.
        public: virtual std::size_t InitiateEngine(
            std::size_t engineID = 0) = 0;

        /// \brief Ask for a reference-counting pointer to the specified engine.
        ///
        /// Note to physics engine implementers: return a nullptr if your
        /// physics engine does not support reference-counting.
        ///
        /// \return A reference-counting pointer to the requested engine.
        public: virtual std::shared_ptr<const void> EngineRef(
            std::size_t engineID) = 0;

        /// \brief Virtual destructor
        public: virtual ~Implementation() = default;
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
  }
}

#endif
