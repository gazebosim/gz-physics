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

#include <ignition/physics/Entity.hh>
#include <ignition/common/Console.hh>

namespace ignition
{
  namespace physics
  {
    /////////////////////////////////////////////////
    template <typename FeatureType, typename PimplT>
    std::size_t Entity<FeatureType, PimplT>::EntityID() const
    {
      return this->id;
    }

    /////////////////////////////////////////////////
    template <typename FeatureType, typename PimplT>
    const std::shared_ptr<const void> &
    Entity<FeatureType, PimplT>::EntityReference() const
    {
      return this->ref;
    }

    /////////////////////////////////////////////////
    template <typename FeatureType, typename PimplT>
    Entity<FeatureType, PimplT>::Entity(
        const std::shared_ptr<PimplT> &_pimpl,
        const std::size_t _id,
        const std::shared_ptr<const void> &_ref)
      : pimpl(_pimpl),
        id(_id),
        ref(_ref)
    {
      // Do nothing
    }
  }
}

#endif
