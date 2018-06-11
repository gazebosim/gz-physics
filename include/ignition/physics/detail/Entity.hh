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

#ifndef IGNITION_PHYSICS_DETAIL_ENTITY_HH_
#define IGNITION_PHYSICS_DETAIL_ENTITY_HH_

#include <memory>

#include <ignition/common/SpecializedPluginPtr.hh>
#include <ignition/physics/Entity.hh>

namespace ignition
{
  namespace physics
  {
    namespace detail
    {
      /////////////////////////////////////////////////
      /// \private This class is used to determine what type of
      /// SpecializedPluginPtr should be used by the entities provided by a
      /// plugin.
      template <typename Policy, typename Features>
      struct DeterminePlugin;

      /// \private Implementation of DeterminePluginType
      template <typename Policy, typename... Features>
      struct DeterminePlugin<Policy, std::tuple<Features...>>
      {
        using type = common::SpecializedPluginPtr<
            typename Features::template Implementation<Policy>...>;
      };
    }

    /////////////////////////////////////////////////
    template <typename Policy, typename Features>
    std::size_t Entity<Policy, Features>::EntityID() const
    {
      return this->id;
    }

    /////////////////////////////////////////////////
    template <typename Policy, typename Features>
    const std::shared_ptr<const void> &
    Entity<Policy, Features>::EntityReference() const
    {
      return this->ref;
    }

    /////////////////////////////////////////////////
    template <typename Policy, typename Features>
    Entity<Policy, Features>::Entity(
        const std::shared_ptr<Pimpl> &_pimpl,
        const Identity &_identity)
      : pimpl(_pimpl),
        identity(_identity)
    {
      // Do nothing
    }

    /////////////////////////////////////////////////
    template <typename Policy, typename Features>
    template <typename FeatureT>
    typename FeatureT::template Implementation<Policy>*
    Entity<Policy, Features>::Interface()
    {
      return (*this->pimpl)->template QueryInterface<
          typename FeatureT::template Implementation<Policy>>();
    }

    /////////////////////////////////////////////////
    template <typename Policy, typename Features>
    template <typename FeatureT>
    const typename FeatureT::template Implementation<Policy>*
    Entity<Policy, Features>::Interface() const
    {
      return (*this->pimpl)->template QueryInterface<
          typename FeatureT::template Implementation<Policy>>();
    }
  }
}

#endif
