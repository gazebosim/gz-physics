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

#ifndef IGNITION_PHYSICS_DETAIL_GETENTITIES_HH_
#define IGNITION_PHYSICS_DETAIL_GETENTITIES_HH_

#include <ignition/physics/GetEntities.hh>

namespace ignition
{
  namespace physics
  {
    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    const std::string &GetEntities::Engine<PolicyT, FeaturesT>::GetName() const
    {
      return this->template Interface<GetEntities>()
          ->GetEngineName(this->identity);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    std::size_t GetEntities::Engine<PolicyT, FeaturesT>::GetIndex() const
    {
      return this->template Interface<GetEntities>()
          ->GetEngineIndex(this->identity);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    std::size_t GetEntities::Engine<PolicyT, FeaturesT>::GetWorldCount() const
    {
      return this->template Interface<GetEntities>()
          ->GetWorldCount(this->identity);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto GetEntities::Engine<PolicyT, FeaturesT>::GetWorld(
        const std::size_t _index) -> WorldPtr
    {
      return WorldPtr(this->pimpl,
                      this->template Interface<GetEntities>()
                          ->GetWorld(this->identity, _index));
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto GetEntities::Engine<PolicyT, FeaturesT>::GetWorld(
        const std::size_t _index) const -> ConstWorldPtr
    {
      return ConstWorldPtr(this->pimpl,
                           this->template Interface<GetEntities>()
                              ->GetWorld(this->identity, _index));
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto GetEntities::Engine<PolicyT, FeaturesT>::GetWorld(
        const std::string &_name) -> WorldPtr
    {
      return WorldPtr(this->pimpl,
                      this->template Interface<GetEntities>()
                          ->GetWorld(this->identity, _name));
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto GetEntities::Engine<PolicyT, FeaturesT>::GetWorld(
        const std::string &_name) const -> ConstWorldPtr
    {
      return ConstWorldPtr(this->pimpl,
                           this->template Interface<GetEntities>()
                              ->GetWorld(this->identity, _name));
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    const std::string &GetEntities::World<PolicyT, FeaturesT>::GetName() const
    {
      return this->template Interface<GetEntities>()
          ->GetWorldName(this->identity);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    std::size_t GetEntities::World<PolicyT, FeaturesT>::GetIndex() const
    {
      return this->template Interface<GetEntities>()
          ->GetWorldIndex(this->identity);
    }


    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    std::size_t GetEntities::World<PolicyT, FeaturesT>::GetModelCount() const
    {
      return this->template Interface<GetEntities>()
          ->GetModelCount(this->identity);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto GetEntities::World<PolicyT, FeaturesT>::GetModel(
        const std::size_t _index) -> ModelPtr
    {
      return ModelPtr(this->pimpl,
                      this->template Interface<GetEntities>()
                          ->GetModel(this->identity, _index));
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto GetEntities::World<PolicyT, FeaturesT>::GetModel(
        const std::size_t _index) const -> ConstModelPtr
    {
      return ConstModelPtr(this->pimpl,
                           this->template Interface<GetEntities>()
                               ->GetModel(this->identity, _index));
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto GetEntities::World<PolicyT, FeaturesT>::GetModel(
        const std::string &_name) -> ModelPtr
    {
      return ModelPtr(this->pimpl,
                      this->template Interface<GetEntities>()
                          ->GetModel(this->identity, _name));
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto GetEntities::World<PolicyT, FeaturesT>::GetModel(
        const std::string &_name) const -> ConstModelPtr
    {
      return ConstModelPtr(this->pimpl,
                           this->template Interface<GetEntities>()
                               ->GetModel(this->identity, _name));
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    const std::string &GetEntities::Model<PolicyT, FeaturesT>::GetName() const
    {
      return this->template Interface<GetEntities>()
          ->GetModelName(this->identity);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    std::size_t GetEntities::Model<PolicyT, FeaturesT>::GetIndex() const
    {
      return this->template Interface<GetEntities>()
          ->GetModelIndex(this->identity);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    std::size_t GetEntities::Model<PolicyT, FeaturesT>::GetLinkCount() const
    {
      return this->template Interface<GetEntities>()
          ->GetLinkCount(this->identity);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto GetEntities::Model<PolicyT, FeaturesT>::GetLink(
        const std::size_t _index) -> LinkPtr
    {
      return LinkPtr(this->pimpl,
                     this->template Interface<GetEntities>()
                        ->GetLink(this->identity, _index));
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto GetEntities::Model<PolicyT, FeaturesT>::GetLink(
        const std::size_t _index) const -> ConstLinkPtr
    {
      return ConstLinkPtr(this->pimpl,
                          this->template Interface<GetEntities>()
                              ->GetLink(this->identity, _index));
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto GetEntities::Model<PolicyT, FeaturesT>::GetLink(
        const std::string &_name) -> LinkPtr
    {
      return LinkPtr(this->pimpl,
                     this->template Interface<GetEntities>()
                         ->GetLink(this->identity, _name));
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto GetEntities::Model<PolicyT, FeaturesT>::GetLink(
        const std::string &_name) const -> ConstLinkPtr
    {
      return ConstLinkPtr(this->pimpl,
                          this->template Interface<GetEntities>()
                              ->GetLink(this->identity, _name));
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    std::size_t GetEntities::Model<PolicyT, FeaturesT>::GetJointCount() const
    {
      return this->template Interface<GetEntities>()
          ->GetJointCount(this->identity);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto GetEntities::Model<PolicyT, FeaturesT>::GetJoint(
        const std::size_t _index) -> JointPtr
    {
      return JointPtr(this->pimpl,
                     this->template Interface<GetEntities>()
                        ->GetJoint(this->identity, _index));
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto GetEntities::Model<PolicyT, FeaturesT>::GetJoint(
        const std::size_t _index) const -> ConstJointPtr
    {
      return ConstJointPtr(this->pimpl,
                          this->template Interface<GetEntities>()
                              ->GetJoint(this->identity, _index));
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto GetEntities::Model<PolicyT, FeaturesT>::GetJoint(
        const std::string &_name) -> JointPtr
    {
      return JointPtr(this->pimpl,
                     this->template Interface<GetEntities>()
                         ->GetJoint(this->identity, _name));
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto GetEntities::Model<PolicyT, FeaturesT>::GetJoint(
        const std::string &_name) const -> ConstJointPtr
    {
      return ConstJointPtr(this->pimpl,
                          this->template Interface<GetEntities>()
                              ->GetJoint(this->identity, _name));
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    const std::string &GetEntities::Link<PolicyT, FeaturesT>::GetName() const
    {
      return this->template Interface<GetEntities>()
          ->GetLinkName(this->identity);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    std::size_t GetEntities::Link<PolicyT, FeaturesT>::GetIndex() const
    {
      return this->template Interface<GetEntities>()
          ->GetLinkIndex(this->identity);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    const std::string &GetEntities::Joint<PolicyT, FeaturesT>::GetName() const
    {
      return this->template Interface<GetEntities>()
          ->GetJointName(this->identity);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    std::size_t GetEntities::Joint<PolicyT, FeaturesT>::GetIndex() const
    {
      return this->template Interface<GetEntities>()
          ->GetJointIndex(this->identity);
    }
  }
}

#endif
