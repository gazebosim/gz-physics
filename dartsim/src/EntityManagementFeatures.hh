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

#ifndef IGNITION_PHYSICS_DARTSIM_SRC_GETENTITIESFEATURE_HH_
#define IGNITION_PHYSICS_DARTSIM_SRC_GETENTITIESFEATURE_HH_

#include <ignition/physics/ConstructEmpty.hh>
#include <ignition/physics/GetEntities.hh>
#include <ignition/physics/Implements.hh>

#include "Base.hh"

namespace ignition {
namespace physics {
namespace dartsim {

using EntityManagementFeatureList = FeatureList<
  GetEntities,
  ConstructEmptyWorldFeature,
  ConstructEmptyModelFeature,
  ConstructEmptyLinkFeature
>;

class EntityManagementFeatures :
    public virtual Base,
    public virtual Implements3d<EntityManagementFeatureList>
{
  public: const std::string &GetEngineName(const std::size_t) const override;

  public: std::size_t GetEngineIndex(const std::size_t) const override;

  public: std::size_t GetWorldCount(const std::size_t) const override;

  public: Identity GetWorld(
      const std::size_t, std::size_t _worldIndex) const override;

  public: Identity GetWorld(
      const std::size_t, const std::string &_worldName) const override;

  public: const std::string &GetWorldName(std::size_t _worldID) const override;

  public: std::size_t GetWorldIndex(std::size_t _worldID) const override;

  public: Identity GetEngineOfWorld(std::size_t _worldID) const override;

  public: std::size_t GetModelCount(
      std::size_t _worldID) const override;

  public: Identity GetModel(
      std::size_t _worldID, std::size_t _modelIndex) const override;

  public: Identity GetModel(
      std::size_t _worldID, const std::string &_modelName) const override;

  public: const std::string &GetModelName(std::size_t _modelID) const override;

  public: std::size_t GetModelIndex(std::size_t _modelID) const override;

  public: Identity GetWorldOfModel(std::size_t _modelID) const override;

  public: std::size_t GetLinkCount(std::size_t _modelID) const override;

  public: Identity GetLink(
      std::size_t _modelID, std::size_t _linkIndex) const override;

  public: Identity GetLink(
      std::size_t _modelID, const std::string &_linkName) const override;

  public: std::size_t GetJointCount(std::size_t _modelID) const override;

  public: Identity GetJoint(
      std::size_t _modelID, std::size_t _jointIndex) const override;

  public: Identity GetJoint(
      std::size_t _modelID, const std::string &_jointName) const override;

  public: const std::string &GetLinkName(std::size_t _linkID) const override;

  public: std::size_t GetLinkIndex(std::size_t _linkID) const override;

  public: Identity GetModelOfLink(std::size_t _linkID) const override;

  public: const std::string &GetJointName(std::size_t _jointID) const override;

  public: std::size_t GetJointIndex(std::size_t _jointID) const override;

  public: Identity GetModelOfJoint(std::size_t _jointID) const override;

  public: Identity ConstructEmptyWorld(
      std::size_t _engineID, const std::string &_name) override;

  public: Identity ConstructEmptyModel(
      std::size_t _worldID, const std::string &_name) override;

  public: Identity ConstructEmptyLink(
      std::size_t _modelID, const std::string &_name) override;
};

}
}
}

#endif
