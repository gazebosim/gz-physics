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

#include <memory>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include <ignition/common/Console.hh>
#include <ignition/physics/Implements.hh>

namespace ignition {
namespace physics {
namespace dartsim {

/// \brief The structs ModelInfo, LinkInfo, JointInfo, and ShapeInfo are used
/// for two reasons:
/// 1) Holding extra information such as the name or offset
///    that will be different from the underlying engine
/// 2) Wrap shared pointers to DART entities. Since these shared pointers (eg.
///    dart::dynamics::BodyNodePtr) are different from std::shared_ptr, we
///    cannot use them directly as parameters to GenerateIdentity. Instead we
///    create a std::shared_ptr of the struct that wraps the corresponding DART
///    shared pointer.

struct ModelInfo
{
  dart::dynamics::SkeletonPtr model;
  dart::dynamics::SimpleFramePtr frame;
  std::string canonicalLinkName;
};

struct LinkInfo
{
  dart::dynamics::BodyNodePtr link;
  /// \brief It may be necessary for dartsim to rename a BodyNode (eg. when
  /// moving the BodyNode to a new skeleton), so we store the Gazebo-specified
  /// name of the Link here.
  std::string name;
};

struct JointInfo
{
  dart::dynamics::JointPtr joint;
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

  Value1 &at(const Key2 &_key)
  {
    return idToObject.at(objectToID.at(_key));
  }

  const Value1 &at(const Key2 &_key) const
  {
    return idToObject.at(objectToID.at(_key));
  }

  std::size_t size() const
  {
    return idToObject.size();
  }

  std::size_t IdentityOf(const Key2 &_key) const
  {
    return objectToID.at(_key);
  }

  bool HasEntity(const Key2 &_key) const
  {
    return objectToID.find(_key) != objectToID.end();
  }

  bool HasEntity(const std::size_t _id) const
  {
    return idToObject.find(_id) != idToObject.end();
  }

  bool RemoveEntity(const Key2 &_key)
  {
    auto entIter = this->objectToID.find(_key);
    if (entIter!= this->objectToID.end())
    {
      std::size_t entId = entIter->second;

      // Check if we are keeping track of the index of this entity in its
      // container
      auto contIter = this->idToContainerID.find(entId);
      if (contIter != this->idToContainerID.end())
      {
        std::size_t contId = contIter->second;
        std::size_t entIndex = this->idToIndexInContainer.at(entId);

        // house keeping
        // The key in indexInContainerToID is the index of the vector so erasing
        // the element automatically decrements the index of the rest of the
        // elements of the vector. The indices in idToIndexInContainer, however,
        // are stored as numbers (as values in the map). We need to decrement
        // all the indices greater than the index of the model we are removing.
        for (auto indIter =
                 this->indexInContainerToID[contId].begin() + entIndex + 1;
             indIter != this->indexInContainerToID[contId].end(); ++indIter)
        {
          // decrement the index (the value of the map)
          --this->idToIndexInContainer[*indIter];
        }

        this->idToIndexInContainer.erase(entId);
        this->indexInContainerToID[contId].erase(
            this->indexInContainerToID[contId].begin() + entIndex);
        this->idToContainerID.erase(entId);
      }

      this->objectToID.erase(entIter);
      this->idToObject.erase(entId);
      return true;
    }
    return false;
  }
};

class Base : public Implements3d<FeatureList<Feature>>
{
  public: using DartWorld = dart::simulation::World;
  public: using DartWorldPtr = dart::simulation::WorldPtr;
  public: using DartSkeletonPtr = dart::dynamics::SkeletonPtr;
  public: using DartConstSkeletonPtr = dart::dynamics::ConstSkeletonPtr;
  public: using DartSkeleton = dart::dynamics::Skeleton;
  public: using DartBodyNode = dart::dynamics::BodyNode;
  public: using DartBodyNodePtr = dart::dynamics::BodyNodePtr;
  public: using DartJoint = dart::dynamics::Joint;
  public: using DartJointPtr = dart::dynamics::JointPtr;
  public: using DartShapeNode = dart::dynamics::ShapeNode;
  public: using DartShapeNodePtr = std::shared_ptr<DartShapeNode>;
  public: using ModelInfoPtr = std::shared_ptr<ModelInfo>;
  public: using LinkInfoPtr = std::shared_ptr<LinkInfo>;
  public: using JointInfoPtr = std::shared_ptr<JointInfo>;
  public: using ShapeInfoPtr = std::shared_ptr<ShapeInfo>;

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
    this->models.idToObject[id] = std::make_shared<ModelInfo>(_info);
    ModelInfo &entry = *this->models.idToObject[id];
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
    this->links.idToObject[id] = std::make_shared<LinkInfo>();
    this->links.idToObject[id]->link = _bn;
    // The name of the BodyNode during creation is assumed to be the
    // Gazebo-specified name.
    this->links.idToObject[id]->name = _bn->getName();
    this->links.objectToID[_bn] = id;
    this->frames[id] = _bn;

    return id;
  }

  public: inline std::size_t AddJoint(DartJoint *_joint)
  {
    const std::size_t id = this->GetNextEntity();
    this->joints.idToObject[id] = std::make_shared<JointInfo>();
    this->joints.idToObject[id]->joint = _joint;
    this->joints.objectToID[_joint] = id;

    return id;
  }

  public: inline std::size_t AddShape(
      const ShapeInfo &_info)
  {
    const std::size_t id = this->GetNextEntity();
    this->shapes.idToObject[id] = std::make_shared<ShapeInfo>(_info);
    this->shapes.objectToID[_info.node] = id;
    this->frames[id] = _info.node.get();

    return id;
  }

  public: void RemoveModelImpl(const std::size_t _worldID,
                               const std::size_t _modelID)
  {
    const auto &world = this->worlds.at(_worldID);
    auto skel = this->models.at(_modelID)->model;
    // Remove the contents of the skeleton from local entity storage containers
    for (auto &jt : skel->getJoints())
    {
      this->joints.RemoveEntity(jt);
    }
    for (auto &bn : skel->getBodyNodes())
    {
      for (auto &sn : bn->getShapeNodes())
      {
        this->shapes.RemoveEntity(sn);
      }
      this->links.RemoveEntity(bn);
    }
    this->models.RemoveEntity(skel);
    this->skeletonsWithInverseDynamics.erase(skel);
    world->removeSkeleton(skel);
  }

  public: EntityStorage<DartWorldPtr, std::string> worlds;
  public: EntityStorage<ModelInfoPtr, DartConstSkeletonPtr> models;
  public: EntityStorage<LinkInfoPtr, const DartBodyNode*> links;
  public: EntityStorage<JointInfoPtr, const DartJoint*> joints;
  public: EntityStorage<ShapeInfoPtr, const DartShapeNode*> shapes;
  public: std::unordered_map<std::size_t, const dart::dynamics::Frame*> frames;
  public: std::set<dart::dynamics::SkeletonPtr> skeletonsWithInverseDynamics;
};

}
}
}

#endif
