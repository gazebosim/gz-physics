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

#ifndef GZ_PHYSICS_DETAIL_CONSTRUCTEMPTY_HH_
#define GZ_PHYSICS_DETAIL_CONSTRUCTEMPTY_HH_

#include <string>

#include <gz/physics/ConstructEmpty.hh>

namespace gz {
namespace physics {

/////////////////////////////////////////////////
template <typename PolicyT, typename FeaturesT>
auto ConstructEmptyWorldFeature::Engine<PolicyT, FeaturesT>
::ConstructEmptyWorld(const std::string &_name) -> WorldPtrType
{
  return WorldPtrType(this->pimpl,
        this->template Interface<ConstructEmptyWorldFeature>()
                      ->ConstructEmptyWorld(this->identity, _name));
}

/////////////////////////////////////////////////
template <typename PolicyT, typename FeaturesT>
auto ConstructEmptyModelFeature::World<PolicyT, FeaturesT>
::ConstructEmptyModel(const std::string &_name) -> ModelPtrType
{
  return ModelPtrType(this->pimpl,
        this->template Interface<ConstructEmptyModelFeature>()
                      ->ConstructEmptyModel(this->identity, _name));
}

/////////////////////////////////////////////////
template <typename PolicyT, typename FeaturesT>
auto ConstructEmptyNestedModelFeature::Model<PolicyT, FeaturesT>
::ConstructEmptyNestedModel(const std::string &_name) -> ModelPtrType
{
  return ModelPtrType(this->pimpl,
        this->template Interface<ConstructEmptyNestedModelFeature>()
                      ->ConstructEmptyNestedModel(this->identity, _name));
}

/////////////////////////////////////////////////
template <typename PolicyT, typename FeaturesT>
auto ConstructEmptyLinkFeature::Model<PolicyT, FeaturesT>
::ConstructEmptyLink(const std::string &_name) -> LinkPtrType
{
  return LinkPtrType(this->pimpl,
        this->template Interface<ConstructEmptyLinkFeature>()
                     ->ConstructEmptyLink(this->identity, _name));
}

}
}

#endif
