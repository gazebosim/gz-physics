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

#ifndef IGNITION_PHYSICS_DETAIL_REMOVEENTITIES_HH_
#define IGNITION_PHYSICS_DETAIL_REMOVEENTITIES_HH_

#include <string>
#include <ignition/physics/RemoveEntities.hh>

namespace ignition
{
  namespace physics
  {
    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    bool RemoveModelFromWorld::World<PolicyT, FeaturesT>::RemoveModel(
        const std::size_t _index)
    {
      return this->template Interface<RemoveModelFromWorld>()
          ->RemoveModelByIndex(this->identity, _index);
    }


    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    bool RemoveModelFromWorld::World<PolicyT, FeaturesT>::RemoveModel(
        const std::string &_name)
    {
      return this->template Interface<RemoveModelFromWorld>()
                          ->RemoveModelByName(this->identity, _name);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    bool RemoveModelFromWorld::Model<PolicyT, FeaturesT>::Remove()
    {
      return this->template Interface<RemoveModelFromWorld>()
              ->RemoveModel(this->identity);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    bool RemoveModelFromWorld::Model<PolicyT, FeaturesT>::Removed() const
    {
      return this->template Interface<RemoveModelFromWorld>()
              ->ModelRemoved(this->identity);
    }
  }
}

#endif
