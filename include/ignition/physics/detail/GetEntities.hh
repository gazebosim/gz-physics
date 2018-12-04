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

#include <string>
#include <ignition/physics/GetEntities.hh>

namespace ignition
{
  namespace physics
  {
    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    const std::string &GetEngine::Engine<PolicyT, FeaturesT>::GetName() const
    {
      return this->template Interface<GetEngine>()
          ->GetEngineName(this->identity);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    std::size_t GetEngine::Engine<PolicyT, FeaturesT>::GetIndex() const
    {
      return this->template Interface<GetEngine>()
          ->GetEngineIndex(this->identity);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    std::size_t EngineGetWorld::Engine<PolicyT, FeaturesT>::GetWorldCount()
                const
    {
      return this->template Interface<EngineGetWorld>()
          ->GetWorldCount(this->identity);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto EngineGetWorld::Engine<PolicyT, FeaturesT>::GetWorld(
        const std::size_t _index) -> WorldPtrType
    {
      return WorldPtrType(this->pimpl,
                      this->template Interface<EngineGetWorld>()
                          ->GetWorld(this->identity, _index));
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto EngineGetWorld::Engine<PolicyT, FeaturesT>::GetWorld(
        const std::size_t _index) const -> ConstWorldPtrType
    {
      return ConstWorldPtrType(this->pimpl,
                           this->template Interface<EngineGetWorld>()
                              ->GetWorld(this->identity, _index));
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto EngineGetWorld::Engine<PolicyT, FeaturesT>::GetWorld(
        const std::string &_name) -> WorldPtrType
    {
      return WorldPtrType(this->pimpl,
                      this->template Interface<EngineGetWorld>()
                          ->GetWorld(this->identity, _name));
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto EngineGetWorld::Engine<PolicyT, FeaturesT>::GetWorld(
        const std::string &_name) const -> ConstWorldPtrType
    {
      return ConstWorldPtrType(this->pimpl,
                           this->template Interface<EngineGetWorld>()
                              ->GetWorld(this->identity, _name));
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    const std::string &EngineGetWorld::World<PolicyT, FeaturesT>::GetName()
                      const
    {
      return this->template Interface<EngineGetWorld>()
          ->GetWorldName(this->identity);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    std::size_t EngineGetWorld::World<PolicyT, FeaturesT>::GetIndex() const
    {
      return this->template Interface<EngineGetWorld>()
          ->GetWorldIndex(this->identity);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto EngineGetWorld::World<PolicyT, FeaturesT>::GetEngine() -> EnginePtrType
    {
      return EnginePtrType(this->pimpl,
            this->template Interface<EngineGetWorld>()
              ->GetEngineOfWorld(this->identity));
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto EngineGetWorld::World<PolicyT, FeaturesT>::GetEngine() const
      -> ConstEnginePtrType
    {
      return ConstEnginePtrType(this->pimpl,
            this->template Interface<EngineGetWorld>()
              ->GetEngineOfWorld(this->identity));
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    std::size_t WorldGetModel::World<PolicyT, FeaturesT>::GetModelCount() const
    {
      return this->template Interface<WorldGetModel>()
          ->GetModelCount(this->identity);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto WorldGetModel::World<PolicyT, FeaturesT>::GetModel(
        const std::size_t _index) -> ModelPtrType
    {
      return ModelPtrType(this->pimpl,
                      this->template Interface<WorldGetModel>()
                          ->GetModel(this->identity, _index));
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto WorldGetModel::World<PolicyT, FeaturesT>::GetModel(
        const std::size_t _index) const -> ConstModelPtrType
    {
      return ConstModelPtrType(this->pimpl,
                           this->template Interface<WorldGetModel>()
                               ->GetModel(this->identity, _index));
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto WorldGetModel::World<PolicyT, FeaturesT>::GetModel(
        const std::string &_name) -> ModelPtrType
    {
      return ModelPtrType(this->pimpl,
                      this->template Interface<WorldGetModel>()
                          ->GetModel(this->identity, _name));
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto WorldGetModel::World<PolicyT, FeaturesT>::GetModel(
        const std::string &_name) const -> ConstModelPtrType
    {
      return ConstModelPtrType(this->pimpl,
                           this->template Interface<WorldGetModel>()
                               ->GetModel(this->identity, _name));
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    const std::string &WorldGetModel::Model<PolicyT, FeaturesT>::GetName() const
    {
      return this->template Interface<WorldGetModel>()
          ->GetModelName(this->identity);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    std::size_t WorldGetModel::Model<PolicyT, FeaturesT>::GetIndex() const
    {
      return this->template Interface<WorldGetModel>()
          ->GetModelIndex(this->identity);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto WorldGetModel::Model<PolicyT, FeaturesT>::GetWorld() -> WorldPtrType
    {
      return WorldPtrType(this->pimpl,
            this->template Interface<WorldGetModel>()
              ->GetWorldOfModel(this->identity));
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto WorldGetModel::Model<PolicyT, FeaturesT>::GetWorld() const
      -> ConstWorldPtrType
    {
      return ConstWorldPtrType(this->pimpl,
            this->template Interface<WorldGetModel>()
              ->GetWorldOfModel(this->identity));
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    std::size_t ModelGetLink::Model<PolicyT, FeaturesT>::GetLinkCount() const
    {
      return this->template Interface<ModelGetLink>()
          ->GetLinkCount(this->identity);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto ModelGetLink::Model<PolicyT, FeaturesT>::GetLink(
        const std::size_t _index) -> LinkPtrType
    {
      return LinkPtrType(this->pimpl,
                     this->template Interface<ModelGetLink>()
                        ->GetLink(this->identity, _index));
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto ModelGetLink::Model<PolicyT, FeaturesT>::GetLink(
        const std::size_t _index) const -> ConstLinkPtrType
    {
      return ConstLinkPtrType(this->pimpl,
                          this->template Interface<ModelGetLink>()
                              ->GetLink(this->identity, _index));
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto ModelGetLink::Model<PolicyT, FeaturesT>::GetLink(
        const std::string &_name) -> LinkPtrType
    {
      return LinkPtrType(this->pimpl,
                     this->template Interface<ModelGetLink>()
                         ->GetLink(this->identity, _name));
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto ModelGetLink::Model<PolicyT, FeaturesT>::GetLink(
        const std::string &_name) const -> ConstLinkPtrType
    {
      return ConstLinkPtrType(this->pimpl,
                          this->template Interface<ModelGetLink>()
                              ->GetLink(this->identity, _name));
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    std::size_t ModelGetJoint::Model<PolicyT, FeaturesT>::GetJointCount() const
    {
      return this->template Interface<ModelGetJoint>()
          ->GetJointCount(this->identity);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto ModelGetJoint::Model<PolicyT, FeaturesT>::GetJoint(
        const std::size_t _index) -> JointPtrType
    {
      return JointPtrType(this->pimpl,
                     this->template Interface<ModelGetJoint>()
                        ->GetJoint(this->identity, _index));
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto ModelGetJoint::Model<PolicyT, FeaturesT>::GetJoint(
        const std::size_t _index) const -> ConstJointPtrType
    {
      return ConstJointPtrType(this->pimpl,
                          this->template Interface<ModelGetJoint>()
                              ->GetJoint(this->identity, _index));
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto ModelGetJoint::Model<PolicyT, FeaturesT>::GetJoint(
        const std::string &_name) -> JointPtrType
    {
      return JointPtrType(this->pimpl,
                     this->template Interface<ModelGetJoint>()
                         ->GetJoint(this->identity, _name));
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto ModelGetJoint::Model<PolicyT, FeaturesT>::GetJoint(
        const std::string &_name) const -> ConstJointPtrType
    {
      return ConstJointPtrType(this->pimpl,
                          this->template Interface<ModelGetJoint>()
                              ->GetJoint(this->identity, _name));
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    const std::string &ModelGetLink::Link<PolicyT, FeaturesT>::GetName() const
    {
      return this->template Interface<ModelGetLink>()
          ->GetLinkName(this->identity);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    std::size_t ModelGetLink::Link<PolicyT, FeaturesT>::GetIndex() const
    {
      return this->template Interface<ModelGetLink>()
          ->GetLinkIndex(this->identity);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto ModelGetLink::Link<PolicyT, FeaturesT>::GetModel() -> ModelPtrType
    {
      return ModelPtrType(this->pimpl,
            this->template Interface<ModelGetLink>()
              ->GetModelOfLink(this->identity));
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto ModelGetLink::Link<PolicyT, FeaturesT>::GetModel() const
        -> ConstModelPtrType
    {
      return ConstModelPtrType(this->pimpl,
            this->template Interface<ModelGetLink>()
              ->GetModelOfLink(this->identity));
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    std::size_t LinkGetShape::Link<PolicyT, FeaturesT>::GetShapeCount() const
    {
      return this->template Interface<LinkGetShape>()
          ->GetShapeCount(this->identity);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto LinkGetShape::Link<PolicyT, FeaturesT>::GetShape(
        const std::size_t _index) -> ShapePtrType
    {
      return ShapePtrType(this->pimpl,
            this->template Interface<LinkGetShape>()->GetShape(
                            this->identity, _index));
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto LinkGetShape::Link<PolicyT, FeaturesT>::GetShape(
        const std::size_t _index) const -> ConstShapePtrType
    {
      return ConstShapePtrType(this->pimpl,
            this->template Interface<LinkGetShape>()->GetShape(
                            this->identity, _index));
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto LinkGetShape::Link<PolicyT, FeaturesT>::GetShape(
        const std::string &_name) -> ShapePtrType
    {
      return ShapePtrType(this->pimpl,
            this->template Interface<LinkGetShape>()->GetShape(
                            this->identity, _name));
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto LinkGetShape::Link<PolicyT, FeaturesT>::GetShape(
        const std::string &_name) const -> ConstShapePtrType
    {
      return ConstShapePtrType(this->pimpl,
            this->template Interface<LinkGetShape>()->GetShape(
                            this->identity, _name));
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    const std::string &LinkGetShape::Shape<PolicyT, FeaturesT>::GetName() const
    {
      return this->template Interface<LinkGetShape>()->GetShapeName(
            this->identity);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    std::size_t LinkGetShape::Shape<PolicyT, FeaturesT>::GetIndex() const
    {
      return this->template Interface<LinkGetShape>()->GetShapeIndex(
            this->identity);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto LinkGetShape::Shape<PolicyT, FeaturesT>::GetLink() -> LinkPtrType
    {
      return LinkPtrType(this->pimpl,
            this->template Interface<LinkGetShape>()->GetLinkOfShape(
                           this->identity));
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto LinkGetShape::Shape<PolicyT, FeaturesT>::GetLink() const
    -> ConstLinkPtrType
    {
      return ConstLinkPtrType(this->pimpl,
            this->template Interface<LinkGetShape>()->GetLinkOfShape(
                                this->identity));
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    const std::string &ModelGetJoint::Joint<PolicyT, FeaturesT>::GetName() const
    {
      return this->template Interface<ModelGetJoint>()
          ->GetJointName(this->identity);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    std::size_t ModelGetJoint::Joint<PolicyT, FeaturesT>::GetIndex() const
    {
      return this->template Interface<ModelGetJoint>()
          ->GetJointIndex(this->identity);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto ModelGetJoint::Joint<PolicyT, FeaturesT>::GetModel() -> ModelPtrType
    {
      return ModelPtrType(this->pimpl,
            this->template Interface<ModelGetJoint>()
              ->GetModelOfJoint(this->identity));
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto ModelGetJoint::Joint<PolicyT, FeaturesT>::GetModel() const
        -> ConstModelPtrType
    {
      return ConstModelPtrType(this->pimpl,
            this->template Interface<ModelGetJoint>()
              ->GetModelOfJoint(this->identity));
    }
  }
}

#endif
