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

#ifndef GZ_PHYSICS_REQUESTENGINE_HH_
#define GZ_PHYSICS_REQUESTENGINE_HH_

#include <memory>
#include <set>
#include <string>

#include <ignition/physics/FeatureList.hh>

namespace gz
{
  namespace physics
  {
    /// \brief This class provides utilities for inspecting what features are
    /// available in a plugin.
    template <typename FeaturePolicyT, typename FeatureListT>
    struct RequestEngine
    {
      // Forward declaration of the Engine type that will be returned
      using EnginePtrType = EnginePtr<FeaturePolicyT, FeatureListT>;

      // Tuple of features that are being requested
      using Features = FeatureListT;

      /// \brief Get an Engine from the given physics plugin.
      ///
      /// \param[in] _pimpl
      ///   PluginPtr to the physics plugin
      /// \param[in] _engineID
      ///   The ID of the engine that you want to receive from the plugin.
      ///   Usually this will be 0. The result of requesting a value other than
      ///   0 is determined by the plugin that you are using.
      /// \tparam PtrT
      ///   The type of PluginPtr that you are providing. This will typically be
      ///   inferred from the input argument.
      ///
      /// \return A pointer to a physics engine with the requested features. If
      /// any of the requested features aren't available in the given plugin,
      /// this will be a nullptr. MissingFeatureNames() can be used to inspect
      /// which features are missing.
      template <typename PtrT>
      static EnginePtrType From(
          const PtrT &_pimpl,
          const std::size_t _engineID = 0);

      /// \brief Check that a physics plugin has all the requested features.
      ///
      /// \param[in] _pimpl
      ///   The PluginPtr to the plugin.
      /// \tparam PtrT
      ///   The type of PluginPtr that you are providing. This will typically be
      ///   inferred from the input argument.
      ///
      /// \return True if all the requested features are available from the
      /// plugin.
      template <typename PtrT>
      static bool Verify(const PtrT &_pimpl);

      /// \brief Get a set of the names of the requested features that are
      /// missing from this plugin.
      ///
      /// \param[in] _pimpl
      ///   The PluginPtr to the plugin.
      /// \tparam PtrT
      ///   The type of PluginPtr that you are providing. This will typically be
      ///   inferred from the input argument.
      ///
      /// \return A set of all the names of the requested features that are
      /// missing from the plugin.
      template <typename PtrT>
      static std::set<std::string> MissingFeatureNames(const PtrT &_pimpl);
    };

#define IGN_PHYSICS_REQUEST_FEATURES_MACRO(X) \
  template <typename FeatureList> \
  using RequestEngine ## X = \
      RequestEngine<FeaturePolicy ## X, FeatureList>;

    IGN_PHYSICS_REQUEST_FEATURES_MACRO(3d)
    IGN_PHYSICS_REQUEST_FEATURES_MACRO(2d)
    IGN_PHYSICS_REQUEST_FEATURES_MACRO(3f)
    IGN_PHYSICS_REQUEST_FEATURES_MACRO(2f)
  }
}

#include <gz/physics/detail/RequestEngine.hh>

#endif
