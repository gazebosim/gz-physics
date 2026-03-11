/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#ifndef GZ_PHYSICS_BULLET_FEATHERSTONE_SRC_GETENTITIESFEATURE_HH_
#define GZ_PHYSICS_BULLET_FEATHERSTONE_SRC_GETENTITIESFEATURE_HH_

#include <string>

#include <gz/physics/ConstructEmpty.hh>
#include <gz/physics/GetEntities.hh>
#include <gz/physics/Implements.hh>
#include <gz/physics/RemoveEntities.hh>
#include <gz/physics/Shape.hh>

#include "Base.hh"

namespace gz {
namespace physics {
namespace bullet_featherstone {

struct EntityManagementFeatureList : gz::physics::FeatureList<
  CollisionFilterMaskFeature,
  ConstructEmptyWorldFeature,
  GetEngineInfo,
  GetJointFromModel,
  GetLinkFromModel,
  GetModelFromWorld,
  GetNestedModelFromModel,
  GetShapeFromLink,
  GetWorldFromEngine,
  RemoveEntities,
  WorldModelFeature
> { };

class EntityManagementFeatures :
    public virtual Base,
    public virtual Implements3d<EntityManagementFeatureList>
{
  // ----- GetEngineInfo -----
  public: const std::string &GetEngineName(
      const Identity &_engineID) const override;

  public: std::size_t GetEngineIndex(
      const Identity &_engineID) const override;

  // ----- GetModelFromWorld -----
  public: virtual std::size_t GetModelCount(
      const Identity &_worldID) const override;

  public: virtual Identity GetModel(
      const Identity &_worldID, std::size_t _modelIndex) const override;

  public: virtual Identity GetModel(
      const Identity &_worldID, const std::string &_modelName) const override;

  public: virtual const std::string &GetModelName(
      const Identity &_modelID) const override;

  public: virtual std::size_t GetModelIndex(
      const Identity &_modelID) const override;

  public: virtual Identity GetWorldOfModel(
      const Identity &_modelID) const override;

  // ----- GetWorldFromEngine -----
  public: virtual std::size_t GetWorldCount(
      const Identity &_engineID) const override;

  public: virtual Identity GetWorld(
      const Identity &_engineID, std::size_t _worldIndex) const override;

  public: virtual Identity GetWorld(
      const Identity &_engineID, const std::string &_worldName) const override;

  public: virtual const std::string &GetWorldName(
      const Identity &_worldID) const override;

  public: virtual std::size_t GetWorldIndex(
      const Identity &_worldID) const override;

  public: virtual Identity GetEngineOfWorld(
      const Identity &_worldID) const override;

  // ----- GetLinkFromModel -----
  public: std::size_t GetLinkCount(
      const Identity &_modelID) const override;

  public: Identity GetLink(
      const Identity &_modelID, std::size_t _linkIndex) const override;

  public: Identity GetLink(
      const Identity &_modelID, const std::string &_linkName) const override;

  public: const std::string &GetLinkName(
      const Identity &_linkID) const override;

  public: std::size_t GetLinkIndex(const Identity &_linkID) const override;

  public: Identity GetModelOfLink(const Identity &_linkID) const override;

  // ----- GetJointFromModel -----
  public: std::size_t GetJointCount(
      const Identity &_modelID) const override;

  public: Identity GetJoint(
      const Identity &_modelID, std::size_t _jointIndex) const override;

  public: Identity GetJoint(
      const Identity &_modelID, const std::string &_jointName) const override;

  public: const std::string &GetJointName(
      const Identity &_jointID) const override;

  public: std::size_t GetJointIndex(
      const Identity &_jointID) const override;

  public: Identity GetModelOfJoint(
      const Identity &_jointID) const override;

  // ----- GetShapeFromLink -----
  public: std::size_t GetShapeCount(const Identity &_linkID) const override;

  public: Identity GetShape(
      const Identity &_linkID, std::size_t _shapeIndex) const override;

  public: Identity GetShape(
      const Identity &_linkID, const std::string &_shapeName) const override;

  const std::string &GetShapeName(
      const Identity &_shapeID) const override;

  std::size_t GetShapeIndex(const Identity &_shapeID) const override;

  Identity GetLinkOfShape(const Identity &_shapeID) const override;

  // ----- Remove entities -----
  public: bool RemoveModelByIndex(
      const Identity &_worldID, std::size_t _modelIndex) override;

  public: bool RemoveModelByName(
      const Identity &_worldID,
      const std::string &_modelName) override;

  public: bool RemoveModel(const Identity &_modelID) override;

  public: bool ModelRemoved(const Identity &_modelID) const override;

  public: bool RemoveNestedModelByIndex(
     const Identity &_modelID, std::size_t _nestedModelIndex) override;

  public: bool RemoveNestedModelByName(
      const Identity &_modelID, const std::string &_modelName) override;

  // ----- Manage collision filter masks -----
  public: void SetCollisionFilterMask(
      const Identity &_shapeID, uint16_t _mask) override;

  public: uint16_t GetCollisionFilterMask(
      const Identity &_shapeID) const override;

  public: void RemoveCollisionFilterMask(const Identity &_shapeID) override;

  // ----- Construct empty entities -----
  public: Identity ConstructEmptyWorld(
      const Identity &_engineID, const std::string & _name) override;

  // ----- GetNestedModelFromModel -----
  public: std::size_t GetNestedModelCount(
    const Identity &_modelID) const override;

  public: Identity GetNestedModel(
    const Identity &_modelID, std::size_t _modelIndex) const override;

  public: Identity GetNestedModel(
    const Identity &_modelID, const std::string &_modelName) const override;

  // ----- World model feature -----
  public: Identity GetWorldModel(const Identity &_worldID) const override;
};

}  // namespace bullet_featherstone
}  // namespace physics
}  // namespace gz

#endif
