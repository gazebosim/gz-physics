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

#ifndef GZ_PHYSICS_DETAIL_FINDFEATURES_HH_
#define GZ_PHYSICS_DETAIL_FINDFEATURES_HH_

#include <set>
#include <string>

#include <gz/physics/FindFeatures.hh>
#include <gz/physics/detail/InspectFeatures.hh>

namespace ignition
{
  namespace physics
  {
    template <typename FeaturePolicyT, typename FeatureListT>
    template <typename LoaderT>
    std::set<std::string> FindFeatures<FeaturePolicyT, FeatureListT>::From(
        const LoaderT &_loader)
    {
      auto plugins = _loader.AllPlugins();
      detail::InspectFeatures<FeaturePolicyT, Features>::EraseIfMissing(
            _loader, plugins);

      std::set<std::string> result;
      for (const std::string &p : plugins)
        result.insert(result.end(), p);

      return result;
    }
  }
}

#endif
