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

#ifndef GZ_PHYSICS_DETAIL_GETENTITIES_HH_
#define GZ_PHYSICS_DETAIL_GETENTITIES_HH_

#include <string>
#include <gz/physics/GetEntities.hh>

namespace gz
{
  namespace physics
  {
    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    const std::string &GetEngineInfo::Engine<PolicyT, FeaturesT>::GetName()
        const
    {
      return this->template Interface<GetEngineInfo>()
          ->GetEngineName(this->identity);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    std::size_t GetEngineInfo::Engine<PolicyT, FeaturesT>::GetIndex() const
    {
      return this->template Interface<GetEngineInfo>()
          ->GetEngineIndex(this->identity);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    std::size_t GetWorldFromEngine::Engine<PolicyT, FeaturesT>::GetWorldCount()
        const
    {
      return this->template Interface<GetWorldFromEngine>()
          ->GetWorldCount(this->identity);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto GetWorldFromEngine::Engine<PolicyT, FeaturesT>::GetWorld(
        const std::size_t _index) -> WorldPtrType
    {
      return WorldPtrType(this->pimpl,
                      this->template Interface<GetWorldFromEngine>()
                          ->GetWorld(this->identity, _index));
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto GetWorldFromEngine::Engine<PolicyT, FeaturesT>::GetWorld(
        const std::size_t _index) const -> ConstWorldPtrType
    {
      return ConstWorldPtrType(this->pimpl,
                           this->template Interface<GetWorldFromEngine>()
                              ->GetWorld(this->identity, _index));
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto GetWorldFromEngine::Engine<PolicyT, FeaturesT>::GetWorld(
        const std::string &_name) -> WorldPtrType
    {
      return WorldPtrType(this->pimpl,
                      this->template Interface<GetWorldFromEngine>()
                          ->GetWorld(this->identity, _name));
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto GetWorldFromEngine::Engine<PolicyT, FeaturesT>::GetWorld(
        const std::string &_name) const -> ConstWorldPtrType
    {
      return ConstWorldPtrType(this->pimpl,
                           this->template Interface<GetWorldFromEngine>()
                              ->GetWorld(this->identity, _name));
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    const std::string &GetWorldFromEngine::World<PolicyT, FeaturesT>::GetName()
        const
    {
      return this->template Interface<GetWorldFromEngine>()
          ->GetWorldName(this->identity);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    std::size_t GetWorldFromEngine::World<PolicyT, FeaturesT>::GetIndex() const
    {
      return this->template Interface<GetWorldFromEngine>()
          ->GetWorldIndex(this->identity);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto GetWorldFromEngine::World<PolicyT, FeaturesT>::GetEngine()
        -> EnginePtrType
    {
      return EnginePtrType(this->pimpl,
            this->template Interface<GetWorldFromEngine>()
              ->GetEngineOfWorld(this->identity));
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto GetWorldFromEngine::World<PolicyT, FeaturesT>::GetEngine() const
      -> ConstEnginePtrType
    {
      return ConstEnginePtrType(this->pimpl,
            this->template Interface<GetWorldFromEngine>()
              ->GetEngineOfWorld(this->identity));
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    std::size_t GetModelFromWorld::World<PolicyT, FeaturesT>::GetModelCount()
        const
    {
      return this->template Interface<GetModelFromWorld>()
          ->GetModelCount(this->identity);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto GetModelFromWorld::World<PolicyT, FeaturesT>::GetModel(
        const std::size_t _index) -> ModelPtrType
    {
      return ModelPtrType(this->pimpl,
                      this->template Interface<GetModelFromWorld>()
                          ->GetModel(this->identity, _index));
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto GetModelFromWorld::World<PolicyT, FeaturesT>::GetModel(
        const std::size_t _index) const -> ConstModelPtrType
    {
      return ConstModelPtrType(this->pimpl,
                           this->template Interface<GetModelFromWorld>()
                               ->GetModel(this->identity, _index));
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto GetModelFromWorld::World<PolicyT, FeaturesT>::GetModel(
        const std::string &_name) -> ModelPtrType
    {
      return ModelPtrType(this->pimpl,
                      this->template Interface<GetModelFromWorld>()
                          ->GetModel(this->identity, _name));
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto GetModelFromWorld::World<PolicyT, FeaturesT>::GetModel(
        const std::string &_name) const -> ConstModelPtrType
    {
      return ConstModelPtrType(this->pimpl,
                           this->template Interface<GetModelFromWorld>()
                               ->GetModel(this->identity, _name));
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    const std::string &GetModelFromWorld::Model<PolicyT, FeaturesT>::GetName()
        const
    {
      return this->template Interface<GetModelFromWorld>()
          ->GetModelName(this->identity);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    std::size_t GetModelFromWorld::Model<PolicyT, FeaturesT>::GetIndex() const
    {
      return this->template Interface<GetModelFromWorld>()
          ->GetModelIndex(this->identity);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto GetModelFromWorld::Model<PolicyT, FeaturesT>::GetWorld()
        -> WorldPtrType
    {
      return WorldPtrType(this->pimpl,
            this->template Interface<GetModelFromWorld>()
              ->GetWorldOfModel(this->identity));
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto GetModelFromWorld::Model<PolicyT, FeaturesT>::GetWorld() const
      -> ConstWorldPtrType
    {
      return ConstWorldPtrType(this->pimpl,
            this->template Interface<GetModelFromWorld>()
              ->GetWorldOfModel(this->identity));
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    std::size_t GetNestedModelFromModel::Model<
        PolicyT, FeaturesT>::GetNestedModelCount() const
    {
      return this->template Interface<GetNestedModelFromModel>()
          ->GetNestedModelCount(this->identity);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto GetNestedModelFromModel::Model<PolicyT, FeaturesT>::GetNestedModel(
        const std::size_t _index) -> ModelPtrType
    {
      return ModelPtrType(
          this->pimpl,
          this->template Interface<GetNestedModelFromModel>()->GetNestedModel(
              this->identity, _index));
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto GetNestedModelFromModel::Model<PolicyT, FeaturesT>::GetNestedModel(
        const std::size_t _index) const -> ConstModelPtrType
    {
      return ConstModelPtrType(
          this->pimpl,
          this->template Interface<GetNestedModelFromModel>()->GetNestedModel(
              this->identity, _index));
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto GetNestedModelFromModel::Model<PolicyT, FeaturesT>::GetNestedModel(
        const std::string &_name) -> ModelPtrType
    {
      return ModelPtrType(
          this->pimpl,
          this->template Interface<GetNestedModelFromModel>()->GetNestedModel(
              this->identity, _name));
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto GetNestedModelFromModel::Model<PolicyT, FeaturesT>::GetNestedModel(
        const std::string &_name) const -> ConstModelPtrType
    {
      return ConstModelPtrType(this->pimpl,
          this->template Interface<GetNestedModelFromModel>()->GetNestedModel(
              this->identity, _name));
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    std::size_t GetLinkFromModel::Model<PolicyT, FeaturesT>::GetLinkCount()
        const
    {
      return this->template Interface<GetLinkFromModel>()
          ->GetLinkCount(this->identity);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto GetLinkFromModel::Model<PolicyT, FeaturesT>::GetLink(
        const std::size_t _index) -> LinkPtrType
    {
      return LinkPtrType(this->pimpl,
                     this->template Interface<GetLinkFromModel>()
                        ->GetLink(this->identity, _index));
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto GetLinkFromModel::Model<PolicyT, FeaturesT>::GetLink(
        const std::size_t _index) const -> ConstLinkPtrType
    {
      return ConstLinkPtrType(this->pimpl,
                          this->template Interface<GetLinkFromModel>()
                              ->GetLink(this->identity, _index));
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto GetLinkFromModel::Model<PolicyT, FeaturesT>::GetLink(
        const std::string &_name) -> LinkPtrType
    {
      return LinkPtrType(this->pimpl,
                     this->template Interface<GetLinkFromModel>()
                         ->GetLink(this->identity, _name));
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto GetLinkFromModel::Model<PolicyT, FeaturesT>::GetLink(
        const std::string &_name) const -> ConstLinkPtrType
    {
      return ConstLinkPtrType(this->pimpl,
                          this->template Interface<GetLinkFromModel>()
                              ->GetLink(this->identity, _name));
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    std::size_t GetJointFromModel::Model<PolicyT, FeaturesT>::GetJointCount()
        const
    {
      return this->template Interface<GetJointFromModel>()
          ->GetJointCount(this->identity);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto GetJointFromModel::Model<PolicyT, FeaturesT>::GetJoint(
        const std::size_t _index) -> JointPtrType
    {
      return JointPtrType(this->pimpl,
                     this->template Interface<GetJointFromModel>()
                        ->GetJoint(this->identity, _index));
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto GetJointFromModel::Model<PolicyT, FeaturesT>::GetJoint(
        const std::size_t _index) const -> ConstJointPtrType
    {
      return ConstJointPtrType(this->pimpl,
                          this->template Interface<GetJointFromModel>()
                              ->GetJoint(this->identity, _index));
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto GetJointFromModel::Model<PolicyT, FeaturesT>::GetJoint(
        const std::string &_name) -> JointPtrType
    {
      return JointPtrType(this->pimpl,
                     this->template Interface<GetJointFromModel>()
                         ->GetJoint(this->identity, _name));
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto GetJointFromModel::Model<PolicyT, FeaturesT>::GetJoint(
        const std::string &_name) const -> ConstJointPtrType
    {
      return ConstJointPtrType(this->pimpl,
                          this->template Interface<GetJointFromModel>()
                              ->GetJoint(this->identity, _name));
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    const std::string &GetLinkFromModel::Link<PolicyT, FeaturesT>::GetName()
        const
    {
      return this->template Interface<GetLinkFromModel>()
          ->GetLinkName(this->identity);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    std::size_t GetLinkFromModel::Link<PolicyT, FeaturesT>::GetIndex() const
    {
      return this->template Interface<GetLinkFromModel>()
          ->GetLinkIndex(this->identity);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto GetLinkFromModel::Link<PolicyT, FeaturesT>::GetModel() -> ModelPtrType
    {
      return ModelPtrType(this->pimpl,
            this->template Interface<GetLinkFromModel>()
              ->GetModelOfLink(this->identity));
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto GetLinkFromModel::Link<PolicyT, FeaturesT>::GetModel() const
        -> ConstModelPtrType
    {
      return ConstModelPtrType(this->pimpl,
            this->template Interface<GetLinkFromModel>()
              ->GetModelOfLink(this->identity));
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    std::size_t GetShapeFromLink::Link<PolicyT, FeaturesT>::GetShapeCount()
        const
    {
      return this->template Interface<GetShapeFromLink>()
          ->GetShapeCount(this->identity);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto GetShapeFromLink::Link<PolicyT, FeaturesT>::GetShape(
        const std::size_t _index) -> ShapePtrType
    {
      return ShapePtrType(this->pimpl,
            this->template Interface<GetShapeFromLink>()->GetShape(
                            this->identity, _index));
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto GetShapeFromLink::Link<PolicyT, FeaturesT>::GetShape(
        const std::size_t _index) const -> ConstShapePtrType
    {
      return ConstShapePtrType(this->pimpl,
            this->template Interface<GetShapeFromLink>()->GetShape(
                            this->identity, _index));
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto GetShapeFromLink::Link<PolicyT, FeaturesT>::GetShape(
        const std::string &_name) -> ShapePtrType
    {
      return ShapePtrType(this->pimpl,
            this->template Interface<GetShapeFromLink>()->GetShape(
                            this->identity, _name));
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto GetShapeFromLink::Link<PolicyT, FeaturesT>::GetShape(
        const std::string &_name) const -> ConstShapePtrType
    {
      return ConstShapePtrType(this->pimpl,
            this->template Interface<GetShapeFromLink>()->GetShape(
                            this->identity, _name));
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    const std::string &GetShapeFromLink::Shape<PolicyT, FeaturesT>::GetName()
        const
    {
      return this->template Interface<GetShapeFromLink>()->GetShapeName(
            this->identity);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    std::size_t GetShapeFromLink::Shape<PolicyT, FeaturesT>::GetIndex() const
    {
      return this->template Interface<GetShapeFromLink>()->GetShapeIndex(
            this->identity);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto GetShapeFromLink::Shape<PolicyT, FeaturesT>::GetLink() -> LinkPtrType
    {
      return LinkPtrType(this->pimpl,
            this->template Interface<GetShapeFromLink>()->GetLinkOfShape(
                           this->identity));
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto GetShapeFromLink::Shape<PolicyT, FeaturesT>::GetLink() const
    -> ConstLinkPtrType
    {
      return ConstLinkPtrType(this->pimpl,
            this->template Interface<GetShapeFromLink>()->GetLinkOfShape(
                                this->identity));
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    const std::string &GetJointFromModel::Joint<PolicyT, FeaturesT>::GetName()
        const
    {
      return this->template Interface<GetJointFromModel>()
          ->GetJointName(this->identity);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    std::size_t GetJointFromModel::Joint<PolicyT, FeaturesT>::GetIndex() const
    {
      return this->template Interface<GetJointFromModel>()
          ->GetJointIndex(this->identity);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto GetJointFromModel::Joint<PolicyT, FeaturesT>::GetModel()
        -> ModelPtrType
    {
      return ModelPtrType(this->pimpl,
            this->template Interface<GetJointFromModel>()
              ->GetModelOfJoint(this->identity));
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto GetJointFromModel::Joint<PolicyT, FeaturesT>::GetModel() const
        -> ConstModelPtrType
    {
      return ConstModelPtrType(this->pimpl,
            this->template Interface<GetJointFromModel>()
              ->GetModelOfJoint(this->identity));
    }
  }
}

#endif
