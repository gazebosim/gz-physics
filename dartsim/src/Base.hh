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
struct TwoWayMap
{
  std::unordered_map<std::size_t, Value1> idToObject;
  std::unordered_map<Key2, std::size_t> objectToID;

  // Currently these two fields are only used by World
  std::vector<std::size_t> indexInParentToID;
  std::unordered_map<std::size_t, std::size_t> idToIndexInParent;

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
    this->worlds.idToIndexInParent[id] = this->worlds.indexInParentToID.size();
    this->worlds.indexInParentToID.push_back(id);

    _world->setName(_name);

    return id;
  }

  public: inline std::tuple<std::size_t, ModelInfo&> AddModel(
      const ModelInfo &_info)
  {
    const std::size_t id = this->GetNextEntity();
    ModelInfo &entry = this->models.idToObject[id];
    entry = _info;
    this->models.objectToID[_info.model] = id;

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

  public: TwoWayMap<DartWorldPtr, std::string> worlds;
  public: TwoWayMap<ModelInfo, DartSkeletonPtr> models;
  public: TwoWayMap<DartBodyNodePtr, DartBodyNode*> links;
  public: TwoWayMap<DartJointPtr, DartJoint*> joints;
  public: TwoWayMap<ShapeInfo, DartShapeNode*> shapes;
};

}
}
}

#endif
