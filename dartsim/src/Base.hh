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

#ifndef IGNITION_PHYSICS_DARTSIM_BASE_HH_
#define IGNITION_PHYSICS_DARTSIM_BASE_HH_

#include <unordered_map>

#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/SimpleFrame.hpp>
#include <dart/dynamics/Skeleton.hpp>
#include <dart/simulation/World.hpp>

#include <ignition/physics/Implements.hh>

namespace ignition {
namespace physics {
namespace dartsim {

struct ModelInfo
{
  dart::dynamics::SkeletonPtr model;
  dart::dynamics::SimpleFramePtr frame;
};

struct ShapeInfo
{
  dart::dynamics::ShapeNodePtr node;
  Eigen::Isometry3d tf_offset;
};

template <typename Value1, typename Key2 = Value1>
struct EntityStorage
{
  /// \brief Map from an entity ID to its corresponding object
  std::unordered_map<std::size_t, Value1> idToObject;

  /// \brief Map from an object pointer (or other unique key) to its entity ID
  std::unordered_map<Key2, std::size_t> objectToID;

  using IndexMap = std::unordered_map<std::size_t, std::vector<std::size_t>>;
  /// \brief The key represents the parent ID. The value represents a vector of
  /// the objects' IDs. The key of the vector is the object's index within its
  /// container. This is used by World and Model objects, which don't know their
  /// own indices within their containers.
  ///
  /// The container type for World is Engine.
  /// The container type for Model is World.
  ///
  /// Links and Joints are contained in Models, but Links and Joints know their
  /// own indices within their Models, so we do not need to use this field for
  /// either of those types.
  IndexMap indexInContainerToID;

  /// \brief Map from an entity ID to its index within its container
  std::unordered_map<std::size_t, std::size_t> idToIndexInContainer;

  /// \brief Map from an entity ID to the ID of its container
  std::unordered_map<std::size_t, std::size_t> idToContainerID;

  Value1 &operator[](const std::size_t _id)
  {
    return idToObject[_id];
  }

  Value1 &at(const std::size_t _id)
  {
    return idToObject.at(_id);
  }

  const Value1 &at(const std::size_t _id) const
  {
    return idToObject.at(_id);
  }

  std::size_t size() const
  {
    return idToObject.size();
  }

  std::size_t IdentityOf(const Key2 &_key) const
  {
    return objectToID.at(_key);
  }
};

class Base : public Implements3d<FeatureList<Feature>>
{
  public: using DartWorldPtr = dart::simulation::WorldPtr;
  public: using DartSkeletonPtr = dart::dynamics::SkeletonPtr;
  public: using DartBodyNode = dart::dynamics::BodyNode;
  public: using DartBodyNodePtr = dart::dynamics::BodyNodePtr;
  public: using DartJoint = dart::dynamics::Joint;
  public: using DartJointPtr = dart::dynamics::JointPtr;
  public: using DartShapeNode = dart::dynamics::ShapeNode;

  public: inline Identity InitiateEngine(std::size_t /*_engineID*/) override
  {
    this->GetNextEntity();

    // Create a 0th entry in this map
    this->worlds.indexInContainerToID.insert(
          std::make_pair(0u, std::vector<std::size_t>()));

    // dartsim does not have multiple "engines"
    return this->GenerateIdentity(0);
  }

  public: inline std::size_t GetNextEntity()
  {
    return entityCount++;
  }

  public: std::size_t entityCount = 0;

  public: inline std::size_t AddWorld(
      const DartWorldPtr &_world, const std::string &_name)
  {
    const std::size_t id = this->GetNextEntity();

    this->worlds.idToObject[id] = _world;
    this->worlds.objectToID[_name] = id;

    std::vector<std::size_t> &indexInContainerToID =
        this->worlds.indexInContainerToID.at(0);

    this->worlds.idToIndexInContainer[id] = indexInContainerToID.size();
    indexInContainerToID.push_back(id);

    this->worlds.idToContainerID[id] = 0;

    _world->setName(_name);

    return id;
  }

  public: inline std::tuple<std::size_t, ModelInfo&> AddModel(
      const ModelInfo &_info, const std::size_t _worldID)
  {
    const std::size_t id = this->GetNextEntity();
    ModelInfo &entry = this->models.idToObject[id];
    entry = _info;
    this->models.objectToID[_info.model] = id;

    const dart::simulation::WorldPtr &world = worlds[_worldID];

    const std::size_t indexInWorld = world->getNumSkeletons();
    this->models.idToIndexInContainer[id] = indexInWorld;
    std::vector<std::size_t> &indexInContainerToID =
        this->models.indexInContainerToID[_worldID];
    indexInContainerToID.push_back(id);
    world->addSkeleton(entry.model);

    this->models.idToContainerID[id] = _worldID;

    assert(indexInContainerToID.size() == world->getNumSkeletons());

    return std::forward_as_tuple(id, entry);
  }

  public: inline std::size_t AddLink(DartBodyNode *_bn)
  {
    const std::size_t id = this->GetNextEntity();
    this->links.idToObject[id] = _bn;
    this->links.objectToID[_bn] = id;

    return id;
  }

  public: inline std::size_t AddJoint(DartJoint *_joint)
  {
    const std::size_t id = this->GetNextEntity();
    this->joints.idToObject[id] = _joint;
    this->joints.objectToID[_joint] = id;

    return id;
  }

  public: inline std::size_t AddShape(
      const ShapeInfo &_info)
  {
    const std::size_t id = this->GetNextEntity();
    this->shapes.idToObject[id] = _info;
    this->shapes.objectToID[_info.node] = id;

    return id;
  }

  public: EntityStorage<DartWorldPtr, std::string> worlds;
  public: EntityStorage<ModelInfo, DartSkeletonPtr> models;
  public: EntityStorage<DartBodyNodePtr, DartBodyNode*> links;
  public: EntityStorage<DartJointPtr, DartJoint*> joints;
  public: EntityStorage<ShapeInfo, DartShapeNode*> shapes;
};

}
}
}

#endif
