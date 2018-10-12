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

#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/SimpleFrame.hpp>
#include <dart/dynamics/Skeleton.hpp>
#include <dart/simulation/World.hpp>

#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

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

  /// \brief dartsim has more strict name uniqueness rules than Gazebo, so we
  /// store the Gazebo-specified name of the Shape here.
  std::string name;

  /// \brief This is added because sometimes the relative transform of a shape
  /// according to Gazebo specifications may be different than the relative
  /// transform of a shape within the dartsim specifications. This is the offset
  /// from the Gazebo specs to the dartsim specs, i.e. T_g * tf_offset = T_d
  /// where T_g is the relative transform according to Gazebo and T_d is the
  /// relative transform according to dartsim.
  Eigen::Isometry3d tf_offset = Eigen::Isometry3d::Identity();
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
    this->frames[id] = _bn;

    this->UpdateSkeletonInWorld(_bn->getSkeleton());

    return id;
  }

  public: inline std::size_t AddJoint(DartJoint *_joint)
  {
    const std::size_t id = this->GetNextEntity();
    this->joints.idToObject[id] = _joint;
    this->joints.objectToID[_joint] = id;

    this->UpdateSkeletonInWorld(_joint->getSkeleton());

    return id;
  }

  public: inline std::size_t AddShape(
      const ShapeInfo &_info)
  {
    const std::size_t id = this->GetNextEntity();
    this->shapes.idToObject[id] = _info;
    this->shapes.objectToID[_info.node] = id;
    this->frames[id] = _info.node.get();

    this->UpdateSkeletonInWorld(_info.node->getSkeleton());

    return id;
  }

  private: void UpdateSkeletonInWorld(const DartSkeletonPtr &_skel)
  {
    // If the skeleton is not added to the world, add it. Otherwise, remove and
    // add it again.

    // Find the world the skeleton belongs to by finding the model first
    const std::size_t modelID = this->models.objectToID.at(_skel);
    const std::size_t worldID = this->models.idToContainerID.at(modelID);
    const DartWorldPtr &world = this->worlds.at(worldID);

    if (world->hasSkeleton(_skel))
    {
      world->removeSkeleton(_skel);
      world->addSkeleton(_skel);
    }
    else
    {
      std::cerr << "[ignition::physics::dartsim::Base] Given a "
                << "skeleton to update, but skeleton was not found in world. "
                << "This should not be possible! Please report this bug!\n";
      assert(false);
    }
  }

  public: EntityStorage<DartWorldPtr, std::string> worlds;
  public: EntityStorage<ModelInfo, DartSkeletonPtr> models;
  public: EntityStorage<DartBodyNodePtr, DartBodyNode*> links;
  public: EntityStorage<DartJointPtr, DartJoint*> joints;
  public: EntityStorage<ShapeInfo, DartShapeNode*> shapes;
  public: std::unordered_map<std::size_t, dart::dynamics::Frame*> frames;
};

}
}
}

#endif
