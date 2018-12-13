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

#ifndef IGNITION_PHYSICS_FINDFEATURES_HH_
#define IGNITION_PHYSICS_FINDFEATURES_HH_

#include <set>
#include <string>

#include <ignition/physics/FeatureList.hh>

namespace ignition
{
  namespace physics
  {
    template <typename FeaturePolicyT, typename FeatureListT>
    struct FindFeatures
    {
      // Tuple of features that are being requested
      using Features = typename FeatureListT::Features;

      /// \brief Find a set of plugins that satisfy the requested list of
      /// features.
      ///
      /// \tparam LoaderT
      ///   The type of plugin loader to use. Typically this will be an
      ///   ignition::plugin::Loader object, but the function can be used on any
      ///   type that has a function `C<std::string> PluginsImplementing<T>()`
      ///   and `C<std::string> AllPlugins()` where `T` is an interface class
      ///   type and `C<std::string>` is a container of `std::strings`.
      ///
      /// \param[in] _loader
      ///   The loader object to search for plugins.
      ///
      /// \return The names of plugins in _loader that provide the requested set
      /// of features.
      template <typename LoaderT>
      static std::set<std::string> From(const LoaderT &_loader);
    };

    template <typename FeatureListT>
    using FindFeatures3d = FindFeatures<FeaturePolicy3d, FeatureListT>;

    template <typename FeatureListT>
    using FindFeatures2d = FindFeatures<FeaturePolicy2d, FeatureListT>;

    template <typename FeatureListT>
    using FindFeatures3f = FindFeatures<FeaturePolicy3f, FeatureListT>;

    template <typename FeatureListT>
    using FindFeatures2f = FindFeatures<FeaturePolicy2f, FeatureListT>;
  }
}

#include <ignition/physics/detail/FindFeatures.hh>

#endif
