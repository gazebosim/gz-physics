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

#ifndef IGNITION_PHYSICS_DETAIL_REGISTER_HH_
#define IGNITION_PHYSICS_DETAIL_REGISTER_HH_

#include <tuple>

#include <ignition/plugin/Register.hh>
#include <ignition/physics/Feature.hh>

namespace ignition
{
  namespace physics
  {
    namespace detail
    {
      template <typename PluginT, typename FeaturePolicyT,
                typename FeatureListOrTuple>
      struct Registrar;

      template <typename PluginT, typename FeaturePolicyT,
                typename FeatureListT>
      struct Registrar
      {
        static void RegisterPlugin()
        {
          Registrar<PluginT, FeaturePolicyT, typename FeatureListT::Features>::
              RegisterPlugin();
        }
      };

      template <typename PluginT, typename FeaturePolicyT,
                typename... Features>
      struct Registrar<PluginT, FeaturePolicyT, std::tuple<Features...>>
      {
        static void RegisterPlugin()
        {
          ignition::plugin::detail::Registrar<
                PluginT, Feature::Implementation<FeaturePolicyT>,
                typename Features::template Implementation<FeaturePolicyT>...>::
              Register();
        }
      };
    }
  }
}

// Dev Note (MXG): Using a namespace called detail_ignition_physics avoids
// confusion with the ignition::physics namespace. This is important because
// users might call this macro within their own namespace scope, which can
// create unexpected and confusing namespace hierarchies.
#define DETAIL_IGN_PHYSICS_ADD_PLUGIN_HELPER( \
  UniqueID, PluginType, FeaturePolicyT, FeatureListT) \
  namespace detail_ignition_physics \
  { \
  namespace \
  { \
    struct ExecuteWhenLoadingLibrary##UniqueID \
    { \
      ExecuteWhenLoadingLibrary##UniqueID() \
      { \
        ::ignition::physics::detail::Registrar< \
            PluginType, FeaturePolicyT, FeatureListT>:: \
            RegisterPlugin(); \
      } \
    }; \
  \
    static ExecuteWhenLoadingLibrary##UniqueID execute##UniqueID; \
  }  /* namespace */ \
  }

#define DETAIL_IGN_PHYSICS_ADD_PLUGIN_WITH_COUNTER( \
  UniqueID, PluginType, FeaturePolicyT, FeatureListT) \
  DETAIL_IGN_PHYSICS_ADD_PLUGIN_HELPER( \
    UniqueID, PluginType, FeaturePolicyT, FeatureListT)

#define DETAIL_IGN_PHYSICS_ADD_PLUGIN( \
  PluginType, FeaturePolicyT, FeatureListT) \
  DETAIL_IGN_PHYSICS_ADD_PLUGIN_WITH_COUNTER( \
  __COUNTER__, PluginType, FeaturePolicyT, FeatureListT)

#endif
