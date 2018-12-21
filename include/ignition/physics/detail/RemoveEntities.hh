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
    // template <typename PolicyT, typename FeaturesT>
    // std::size_t RemoveWorldFromEngine::Engine<PolicyT, FeaturesT>::RemoveWorldCount()
    //     const
    // {
    //   return this->template Interface<RemoveWorldFromEngine>()
    //       ->RemoveWorldCount(this->identity);
    // }

    // /////////////////////////////////////////////////
    // template <typename PolicyT, typename FeaturesT>
    // auto RemoveWorldFromEngine::Engine<PolicyT, FeaturesT>::RemoveWorld(
    //     const std::size_t _index) -> WorldPtrType
    // {
    //   return WorldPtrType(this->pimpl,
    //                   this->template Interface<RemoveWorldFromEngine>()
    //                       ->RemoveWorld(this->identity, _index));
    // }

    // /////////////////////////////////////////////////
    // template <typename PolicyT, typename FeaturesT>
    // auto RemoveWorldFromEngine::Engine<PolicyT, FeaturesT>::RemoveWorld(
    //     const std::size_t _index) const -> ConstWorldPtrType
    // {
    //   return ConstWorldPtrType(this->pimpl,
    //                        this->template Interface<RemoveWorldFromEngine>()
    //                           ->RemoveWorld(this->identity, _index));
    // }

    // /////////////////////////////////////////////////
    // template <typename PolicyT, typename FeaturesT>
    // auto RemoveWorldFromEngine::Engine<PolicyT, FeaturesT>::RemoveWorld(
    //     const std::string &_name) -> WorldPtrType
    // {
    //   return WorldPtrType(this->pimpl,
    //                   this->template Interface<RemoveWorldFromEngine>()
    //                       ->RemoveWorld(this->identity, _name));
    // }

    /////////////////////////////////////////////////
    // template <typename PolicyT, typename FeaturesT>
    // auto RemoveWorldFromEngine::Engine<PolicyT, FeaturesT>::RemoveWorld(
    //     const std::string &_name) const -> ConstWorldPtrType
    // {
    //   return ConstWorldPtrType(this->pimpl,
    //                        this->template Interface<RemoveWorldFromEngine>()
    //                           ->RemoveWorld(this->identity, _name));
    // }

    // /////////////////////////////////////////////////
    // template <typename PolicyT, typename FeaturesT>
    // const std::string &RemoveWorldFromEngine::World<PolicyT, FeaturesT>::GetName()
    //     const
    // {
    //   return this->template Interface<RemoveWorldFromEngine>()
    //       ->RemoveWorldName(this->identity);
    // }

    // /////////////////////////////////////////////////
    // template <typename PolicyT, typename FeaturesT>
    // std::size_t RemoveWorldFromEngine::World<PolicyT, FeaturesT>::GetIndex() const
    // {
    //   return this->template Interface<RemoveWorldFromEngine>()
    //       ->RemoveWorldIndex(this->identity);
    // }

    // /////////////////////////////////////////////////
    // template <typename PolicyT, typename FeaturesT>
    // auto RemoveWorldFromEngine::World<PolicyT, FeaturesT>::GetEngine()
    //     -> EnginePtrType
    // {
    //   return EnginePtrType(this->pimpl,
    //         this->template Interface<RemoveWorldFromEngine>()
    //           ->GetEngineOfWorld(this->identity));
    // }

    // /////////////////////////////////////////////////
    // template <typename PolicyT, typename FeaturesT>
    // auto RemoveWorldFromEngine::World<PolicyT, FeaturesT>::GetEngine() const
    //   -> ConstEnginePtrType
    // {
    //   return ConstEnginePtrType(this->pimpl,
    //         this->template Interface<RemoveWorldFromEngine>()
    //           ->GetEngineOfWorld(this->identity));
    // }

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
    void RemoveModelFromWorld::Model<PolicyT, FeaturesT>::Remove()
    {
      return this->template Interface<RemoveModelFromWorld>()
              ->RemoveModel(this->identity);
    }

    // /////////////////////////////////////////////////
    // template <typename PolicyT, typename FeaturesT>
    // std::size_t RemoveLinkFromModel::Model<PolicyT, FeaturesT>::RemoveLinkCount()
    //     const
    // {
    //   return this->template Interface<RemoveLinkFromModel>()
    //       ->RemoveLinkCount(this->identity);
    // }

    // /////////////////////////////////////////////////
    // template <typename PolicyT, typename FeaturesT>
    // auto RemoveLinkFromModel::Model<PolicyT, FeaturesT>::RemoveLink(
    //     const std::size_t _index) -> LinkPtrType
    // {
    //   return LinkPtrType(this->pimpl,
    //                  this->template Interface<RemoveLinkFromModel>()
    //                     ->RemoveLink(this->identity, _index));
    // }

    // /////////////////////////////////////////////////
    // template <typename PolicyT, typename FeaturesT>
    // auto RemoveLinkFromModel::Model<PolicyT, FeaturesT>::RemoveLink(
    //     const std::size_t _index) const -> ConstLinkPtrType
    // {
    //   return ConstLinkPtrType(this->pimpl,
    //                       this->template Interface<RemoveLinkFromModel>()
    //                           ->RemoveLink(this->identity, _index));
    // }

    // /////////////////////////////////////////////////
    // template <typename PolicyT, typename FeaturesT>
    // auto RemoveLinkFromModel::Model<PolicyT, FeaturesT>::RemoveLink(
    //     const std::string &_name) -> LinkPtrType
    // {
    //   return LinkPtrType(this->pimpl,
    //                  this->template Interface<RemoveLinkFromModel>()
    //                      ->RemoveLink(this->identity, _name));
    // }

    // /////////////////////////////////////////////////
    // template <typename PolicyT, typename FeaturesT>
    // auto RemoveLinkFromModel::Model<PolicyT, FeaturesT>::RemoveLink(
    //     const std::string &_name) const -> ConstLinkPtrType
    // {
    //   return ConstLinkPtrType(this->pimpl,
    //                       this->template Interface<RemoveLinkFromModel>()
    //                           ->RemoveLink(this->identity, _name));
    // }

    // /////////////////////////////////////////////////
    // template <typename PolicyT, typename FeaturesT>
    // std::size_t RemoveJointFromModel::Model<PolicyT, FeaturesT>::RemoveJointCount()
    //     const
    // {
    //   return this->template Interface<RemoveJointFromModel>()
    //       ->RemoveJointCount(this->identity);
    // }

    // /////////////////////////////////////////////////
    // template <typename PolicyT, typename FeaturesT>
    // auto RemoveJointFromModel::Model<PolicyT, FeaturesT>::RemoveJoint(
    //     const std::size_t _index) -> JointPtrType
    // {
    //   return JointPtrType(this->pimpl,
    //                  this->template Interface<RemoveJointFromModel>()
    //                     ->RemoveJoint(this->identity, _index));
    // }

    // /////////////////////////////////////////////////
    // template <typename PolicyT, typename FeaturesT>
    // auto RemoveJointFromModel::Model<PolicyT, FeaturesT>::RemoveJoint(
    //     const std::size_t _index) const -> ConstJointPtrType
    // {
    //   return ConstJointPtrType(this->pimpl,
    //                       this->template Interface<RemoveJointFromModel>()
    //                           ->RemoveJoint(this->identity, _index));
    // }

    // /////////////////////////////////////////////////
    // template <typename PolicyT, typename FeaturesT>
    // auto RemoveJointFromModel::Model<PolicyT, FeaturesT>::RemoveJoint(
    //     const std::string &_name) -> JointPtrType
    // {
    //   return JointPtrType(this->pimpl,
    //                  this->template Interface<RemoveJointFromModel>()
    //                      ->RemoveJoint(this->identity, _name));
    // }

    // /////////////////////////////////////////////////
    // template <typename PolicyT, typename FeaturesT>
    // auto RemoveJointFromModel::Model<PolicyT, FeaturesT>::RemoveJoint(
    //     const std::string &_name) const -> ConstJointPtrType
    // {
    //   return ConstJointPtrType(this->pimpl,
    //                       this->template Interface<RemoveJointFromModel>()
    //                           ->RemoveJoint(this->identity, _name));
    // }

    // /////////////////////////////////////////////////
    // template <typename PolicyT, typename FeaturesT>
    // const std::string &RemoveLinkFromModel::Link<PolicyT, FeaturesT>::GetName()
    //     const
    // {
    //   return this->template Interface<RemoveLinkFromModel>()
    //       ->RemoveLinkName(this->identity);
    // }

    // /////////////////////////////////////////////////
    // template <typename PolicyT, typename FeaturesT>
    // std::size_t RemoveLinkFromModel::Link<PolicyT, FeaturesT>::GetIndex() const
    // {
    //   return this->template Interface<RemoveLinkFromModel>()
    //       ->RemoveLinkIndex(this->identity);
    // }

    // /////////////////////////////////////////////////
    // template <typename PolicyT, typename FeaturesT>
    // auto RemoveLinkFromModel::Link<PolicyT, FeaturesT>::RemoveModel() -> ModelPtrType
    // {
    //   return ModelPtrType(this->pimpl,
    //         this->template Interface<RemoveLinkFromModel>()
    //           ->RemoveModelOfLink(this->identity));
    // }

    // /////////////////////////////////////////////////
    // template <typename PolicyT, typename FeaturesT>
    // auto RemoveLinkFromModel::Link<PolicyT, FeaturesT>::RemoveModel() const
    //     -> ConstModelPtrType
    // {
    //   return ConstModelPtrType(this->pimpl,
    //         this->template Interface<RemoveLinkFromModel>()
    //           ->RemoveModelOfLink(this->identity));
    // }

    // /////////////////////////////////////////////////
    // template <typename PolicyT, typename FeaturesT>
    // std::size_t RemoveShapeFromLink::Link<PolicyT, FeaturesT>::RemoveShapeCount()
    //     const
    // {
    //   return this->template Interface<RemoveShapeFromLink>()
    //       ->RemoveShapeCount(this->identity);
    // }

    // /////////////////////////////////////////////////
    // template <typename PolicyT, typename FeaturesT>
    // auto RemoveShapeFromLink::Link<PolicyT, FeaturesT>::RemoveShape(
    //     const std::size_t _index) -> ShapePtrType
    // {
    //   return ShapePtrType(this->pimpl,
    //         this->template Interface<RemoveShapeFromLink>()->RemoveShape(
    //                         this->identity, _index));
    // }

    // /////////////////////////////////////////////////
    // template <typename PolicyT, typename FeaturesT>
    // auto RemoveShapeFromLink::Link<PolicyT, FeaturesT>::RemoveShape(
    //     const std::size_t _index) const -> ConstShapePtrType
    // {
    //   return ConstShapePtrType(this->pimpl,
    //         this->template Interface<RemoveShapeFromLink>()->RemoveShape(
    //                         this->identity, _index));
    // }

    // /////////////////////////////////////////////////
    // template <typename PolicyT, typename FeaturesT>
    // auto RemoveShapeFromLink::Link<PolicyT, FeaturesT>::RemoveShape(
    //     const std::string &_name) -> ShapePtrType
    // {
    //   return ShapePtrType(this->pimpl,
    //         this->template Interface<RemoveShapeFromLink>()->RemoveShape(
    //                         this->identity, _name));
    // }

    // /////////////////////////////////////////////////
    // template <typename PolicyT, typename FeaturesT>
    // auto RemoveShapeFromLink::Link<PolicyT, FeaturesT>::RemoveShape(
    //     const std::string &_name) const -> ConstShapePtrType
    // {
    //   return ConstShapePtrType(this->pimpl,
    //         this->template Interface<RemoveShapeFromLink>()->RemoveShape(
    //                         this->identity, _name));
    // }

    // /////////////////////////////////////////////////
    // template <typename PolicyT, typename FeaturesT>
    // const std::string &RemoveShapeFromLink::Shape<PolicyT, FeaturesT>::GetName()
    //     const
    // {
    //   return this->template Interface<RemoveShapeFromLink>()->RemoveShapeName(
    //         this->identity);
    // }

    // /////////////////////////////////////////////////
    // template <typename PolicyT, typename FeaturesT>
    // std::size_t RemoveShapeFromLink::Shape<PolicyT, FeaturesT>::GetIndex() const
    // {
    //   return this->template Interface<RemoveShapeFromLink>()->RemoveShapeIndex(
    //         this->identity);
    // }

    // /////////////////////////////////////////////////
    // template <typename PolicyT, typename FeaturesT>
    // auto RemoveShapeFromLink::Shape<PolicyT, FeaturesT>::RemoveLink() -> LinkPtrType
    // {
    //   return LinkPtrType(this->pimpl,
    //         this->template Interface<RemoveShapeFromLink>()->RemoveLinkOfShape(
    //                        this->identity));
    // }

    // /////////////////////////////////////////////////
    // template <typename PolicyT, typename FeaturesT>
    // auto RemoveShapeFromLink::Shape<PolicyT, FeaturesT>::RemoveLink() const
    // -> ConstLinkPtrType
    // {
    //   return ConstLinkPtrType(this->pimpl,
    //         this->template Interface<RemoveShapeFromLink>()->RemoveLinkOfShape(
    //                             this->identity));
    // }

    // /////////////////////////////////////////////////
    // template <typename PolicyT, typename FeaturesT>
    // const std::string &RemoveJointFromModel::Joint<PolicyT, FeaturesT>::GetName()
    //     const
    // {
    //   return this->template Interface<RemoveJointFromModel>()
    //       ->RemoveJointName(this->identity);
    // }

    // /////////////////////////////////////////////////
    // template <typename PolicyT, typename FeaturesT>
    // std::size_t RemoveJointFromModel::Joint<PolicyT, FeaturesT>::GetIndex() const
    // {
    //   return this->template Interface<RemoveJointFromModel>()
    //       ->RemoveJointIndex(this->identity);
    // }

    // /////////////////////////////////////////////////
    // template <typename PolicyT, typename FeaturesT>
    // auto RemoveJointFromModel::Joint<PolicyT, FeaturesT>::RemoveModel()
    //     -> ModelPtrType
    // {
    //   return ModelPtrType(this->pimpl,
    //         this->template Interface<RemoveJointFromModel>()
    //           ->RemoveModelOfJoint(this->identity));
    // }

    // /////////////////////////////////////////////////
    // template <typename PolicyT, typename FeaturesT>
    // auto RemoveJointFromModel::Joint<PolicyT, FeaturesT>::RemoveModel() const
    //     -> ConstModelPtrType
    // {
    //   return ConstModelPtrType(this->pimpl,
    //         this->template Interface<RemoveJointFromModel>()
    //           ->RemoveModelOfJoint(this->identity));
    // }
  }
}

#endif
