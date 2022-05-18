/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#ifndef GZ_PHYSICS_TPE_PLUGIN_SRC_BASE_HH_
#define GZ_PHYSICS_TPE_PLUGIN_SRC_BASE_HH_

#include <ignition/physics/Implements.hh>

#include <map>
#include <memory>
#include <string>

#include "lib/src/World.hh"
#include "lib/src/Engine.hh"
#include "lib/src/Model.hh"
#include "lib/src/Link.hh"
#include "lib/src/Collision.hh"
#include "lib/src/Shape.hh"

namespace gz {
namespace physics {
namespace tpeplugin {

/// \brief The structs tpelib::WorldInfo,
/// tpelib::ModelInfo, LinkInfo, and CollisionInfo are used
/// to provide easy access to tpelib structures in the plugin library

struct WorldInfo
{
  std::shared_ptr<tpelib::World> world;
};

struct ModelInfo
{
  tpelib::Model *model;
};

struct LinkInfo
{
  tpelib::Link *link;
};

struct CollisionInfo
{
  tpelib::Collision *collision;
};

class Base : public Implements3d<FeatureList<Feature>>
{
  public: inline Identity InitiateEngine(std::size_t /*_engineID*/) override
  {
    return this->GenerateIdentity(0);
  }

  public: inline std::size_t idToIndexInContainer(std::size_t _id) const
  {
    std::size_t index = 0;
    auto it = this->childIdToParentId.find(_id);
    if (it != this->childIdToParentId.end())
    {
      for (const auto &pair : this->childIdToParentId)
      {
        if (pair.first == _id && pair.second == it->second)
        {
          return index;
        }
        else if (pair.second == it->second)
        {
          ++index;
        }
      }
    }
    // return invalid index if not found in id map
    return -1;
  }

  public: template <typename EntityType>
  inline std::pair<std::size_t, EntityType> indexInContainerToId(
      const std::size_t _containerId,
      const std::size_t _index,
      const std::map<std::size_t, EntityType> &_idMap) const
  {
    std::size_t counter = 0;
    auto it = this->childIdToParentId.begin();

    while (counter <= _index && it != this->childIdToParentId.end())
    {
      if (it->second == _containerId)
      {
        auto idMapIt = _idMap.find(it->first);
        // only count if the entity is found in the idMap. This makes sure we're
        // counting only the entities with the correct EntityType
        if (idMapIt != _idMap.end())
        {
          if (counter == _index)
          {
            return *idMapIt;
          }
          else
          {
            ++counter;
          }
        }
      }
      ++it;
    }
    // return invalid id if entity not found
    return {INVALID_ENTITY_ID, nullptr};
  }

  public: inline Identity AddWorld(std::shared_ptr<tpelib::World> _world)
  {
    size_t worldId = _world->GetId();
    auto worldPtr = std::make_shared<WorldInfo>();
    worldPtr->world = _world;
    this->worlds.insert({worldId, worldPtr});
    this->childIdToParentId.insert({worldId, -1});
    return this->GenerateIdentity(worldId, worldPtr);
  }

  public: inline Identity AddModel(std::size_t _parentId, tpelib::Model &_model)
  {
    auto modelPtr = std::make_shared<ModelInfo>();
    modelPtr->model = &_model;
    size_t modelId = _model.GetId();
    this->models.insert({modelId, modelPtr});
    // keep track of model's corresponding world
    this->childIdToParentId.insert({modelId, _parentId});

    return this->GenerateIdentity(modelId, modelPtr);
  }

  public: inline Identity AddLink(std::size_t _modelId, tpelib::Link &_link)
  {
    auto linkPtr = std::make_shared<LinkInfo>();
    linkPtr->link = &_link;
    size_t linkId = _link.GetId();
    this->links.insert({linkId, linkPtr});
    // keep track of link's corresponding model
    this->childIdToParentId.insert({linkId, _modelId});

    return this->GenerateIdentity(linkId, linkPtr);
  }

  public: inline Identity AddCollision(std::size_t _linkId,
    tpelib::Collision &_collision)
  {
    auto collisionPtr = std::make_shared<CollisionInfo>();
    collisionPtr->collision = &_collision;
    size_t collisionId = _collision.GetId();
    this->collisions.insert({collisionId, collisionPtr});
    // keep track of collision's corresponding link
    this->childIdToParentId.insert({collisionId, _linkId});

    return this->GenerateIdentity(collisionId, collisionPtr);
  }

  public: bool RemoveModelImpl(std::size_t _modelID)
  {
    auto parentIt = this->childIdToParentId.find(_modelID);
    if (parentIt == this->childIdToParentId.end())
      return false;

    auto modelInfoIt = this->models.find(_modelID);
    if (modelInfoIt == this->models.end())
      return false;

    tpelib::Entity *parentEntity = nullptr;
    auto worldIt = this->worlds.find(parentIt->second);
    if (worldIt != this->worlds.end())
      parentEntity = worldIt->second->world.get();

    if (nullptr == parentEntity)
    {
      // If the parent entity is not a world, try a parent model of a nested
      // model
      auto modelIt = this->models.find(parentIt->second);
      if (modelIt != this->models.end())
        parentEntity = modelIt->second->model;
    }

    if (nullptr == parentEntity)
      return false;

    bool result = true;
    for (std::size_t i = 0; i < modelInfoIt->second->model->GetChildCount();
         ++i)
    {
      // Get a reference so we can dynamic cast
      auto &ent = modelInfoIt->second->model->GetChildByIndex(i);
      auto modelEnt = dynamic_cast<tpelib::Model *>(&ent);
      if (modelEnt)
      {
        result &= this->RemoveModelImpl(modelEnt->GetId());
      }
    }
    result &= this->RemoveModelFromParent(_modelID, parentEntity);
    return result;
  }

  public: bool RemoveModelFromParent(std::size_t _modelID,
                                     tpelib::Entity *_parentEntity)
  {
    if (nullptr == _parentEntity)
      return false;
    bool result = this->models.erase(_modelID) == 1;
    result &= this->childIdToParentId.erase(_modelID) == 1;
    result &= _parentEntity->RemoveChildById(_modelID);
    return result;
  }

  public: std::map<std::size_t, std::shared_ptr<WorldInfo>> worlds;
  public: std::map<std::size_t, std::shared_ptr<ModelInfo>> models;
  public: std::map<std::size_t, std::shared_ptr<LinkInfo>> links;
  public: std::map<std::size_t, std::shared_ptr<CollisionInfo>> collisions;
  public: std::map<std::size_t, std::size_t> childIdToParentId;
};

}
}
}

#endif
