/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#ifndef IGNITION_PHYSICS_DETAIL_REQUESTFEATURES_HH_
#define IGNITION_PHYSICS_DETAIL_REQUESTFEATURES_HH_

#include <memory>
#include <set>
#include <string>
#include <utility>

#include <ignition/physics/RequestFeatures.hh>
#include <ignition/physics/detail/InspectFeatures.hh>

namespace ignition
{
  namespace physics
  {
    /////////////////////////////////////////////////
    template <typename FeatureListT>
    template <
      typename PolicyT,
      typename FromFeatureList,
      template <typename, typename> class EntityT>
    EntityPtr<EntityT<PolicyT, FeatureListT>>
    RequestFeatures<FeatureListT>::From(
        const EntityPtr<EntityT<PolicyT, FromFeatureList>>& _from)
    {
      using ToPluginType =
          typename detail::DeterminePlugin<PolicyT, FeatureListT>::type;

      // If there is no underlying entity, then just return a null EntityPtr.
      if (!_from.entity)
        return nullptr;

      // If there is no underlying pimpl, then just return a null EntityPtr.
      // TODO(MXG): Should this ever be possible? Maybe this should be an
      // assertion instead?
      if (!_from.entity->pimpl)
        return nullptr;

      ToPluginType toPlugin(*_from.entity->pimpl);
      if (!detail::InspectFeatures<
              PolicyT,
              typename FeatureListT::Features
          >::Verify(toPlugin))
      {
        // The physics plugin does not implement all of the features that were
        // requested, so we will return a nullptr.
        return nullptr;
      }

      return EntityPtr<EntityT<PolicyT, FeatureListT>>(
            std::make_shared<ToPluginType>(std::move(toPlugin)),
            _from.entity->identity);
    }

    /////////////////////////////////////////////////
    template <typename FeatureListT>
    template <
        typename PolicyT,
        typename FromFeatureList,
        template <typename, typename> class EntityT>
    std::set<std::string> RequestFeatures<FeatureListT>::MissingFeatureNames(
        const EntityPtr<EntityT<PolicyT, FromFeatureList>> &_entity)
    {
      std::set<std::string> names;
      if (!_entity.entity || !_entity.entity->pimpl)
      {
        detail::InspectFeatures<PolicyT, Features>::MissingNames(
              plugin::PluginPtr(), names);
        return names;
      }

      detail::InspectFeatures<PolicyT, Features>::MissingNames(
            *_entity.entity->pimpl, names);
      return names;
    }
  }
}

#endif
