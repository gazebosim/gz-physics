/*
 * Copyright (C) 2025 Open Source Robotics Foundation
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

#ifndef GZ_PHYSICS_DETAIL_REGISTER_STATIC_HH_
#define GZ_PHYSICS_DETAIL_REGISTER_STATIC_HH_

#include <tuple>

#include <gz/plugin/RegisterStatic.hh>
#include <gz/physics/Feature.hh>

namespace gz
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
        static void StaticRegisterPlugin()
        {
          StaticRegistrar<PluginT, FeaturePolicyT,
              typename FeatureListT::Features>::RegisterPlugin();
        }
      };

      template <typename PluginT, typename FeaturePolicyT,
                typename... Features>
      struct StaticRegistrar<PluginT, FeaturePolicyT, std::tuple<Features...>>
      {
        static void StaticRegisterPlugin()
        {
          gz::plugin::detail::StaticRegistrar<
                PluginT, Feature::Implementation<FeaturePolicyT>,
                typename Features::template Implementation<FeaturePolicyT>...>::
              Register();
        }
      };
    }
  }
}

// Dev Note (MXG): Using a namespace called detail_gz_physics avoids
// confusion with the gz::physics namespace. This is important because
// users might call this macro within their own namespace scope, which can
// create unexpected and confusing namespace hierarchies.
#define DETAIL_GZ_PHYSICS_ADD_STATIC_PLUGIN_HELPER( \
  UniqueID, PluginType, FeaturePolicyT, FeatureListT) \
  namespace detail_gz_physics \
  { \
  namespace \
  { \
    struct ExecuteWhenLoadingLibrary##UniqueID \
    { \
      ExecuteWhenLoadingLibrary##UniqueID() \
      { \
        ::gz::physics::detail::StaticRegistrar< \
            PluginType, FeaturePolicyT, FeatureListT>:: \
            StaticRegisterPlugin(); \
      } \
    }; \
  \
    static ExecuteWhenLoadingLibrary##UniqueID execute##UniqueID; \
  }  /* namespace */ \
  }

#define DETAIL_GZ_PHYSICS_ADD_STATIC_PLUGIN_WITH_COUNTER( \
  UniqueID, PluginType, FeaturePolicyT, FeatureListT) \
  DETAIL_GZ_PHYSICS_ADD_STATIC_PLUGIN_HELPER( \
    UniqueID, PluginType, FeaturePolicyT, FeatureListT)

#define DETAIL_GZ_PHYSICS_ADD_STATIC_PLUGIN( \
  PluginType, FeaturePolicyT, FeatureListT) \
  DETAIL_GZ_PHYSICS_ADD_STATIC_PLUGIN_WITH_COUNTER( \
  __COUNTER__, PluginType, FeaturePolicyT, FeatureListT)

#endif
