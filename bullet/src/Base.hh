/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#ifndef IGNITION_PHYSICS_BULLET_BASE_HH_
#define IGNITION_PHYSICS_BULLET_BASE_HH_

#include <btBulletDynamicsCommon.h>
#include <Eigen/Geometry>

#include <assert.h>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <map>

#include <ignition/common/Console.hh>
#include <ignition/physics/Implements.hh>
#include <ignition/math/eigen3/Conversions.hh>

namespace ignition {
namespace physics {
namespace bullet {

/// \brief The Info structs are used for three reasons:
/// 1) Holding extra information such as the name
///    that will be different from the underlying engine
/// 2) Wrap shared pointers to Bullet entities to use as parameters to
///    GenerateIdentity.
/// 3) Hold explicit copies of raw pointers that can be deallocated

// TO-DO(): Consider using unique_ptrs instead of shared pointers
// for Bullet internal objects

// Note: For Bullet library it's important the order in which the elements
// are destroyed. The current implementation relies on C++ destroying the
// elements in the opposite order stated in the structure
struct WorldInfo
{
  std::string name;
  std::shared_ptr<btDefaultCollisionConfiguration> collisionConfiguration;
  std::shared_ptr<btCollisionDispatcher> dispatcher;
  std::shared_ptr<btBroadphaseInterface> broadphase;
  std::shared_ptr<btConstraintSolver> solver;
  std::shared_ptr<btDiscreteDynamicsWorld> world;
};

struct ModelInfo
{
  std::string name;
  Identity world;
  // cppcheck-suppress unusedStructMember
  bool fixed;
  math::Pose3d pose;
  std::vector<std::size_t> links = {};
};

struct LinkInfo
{
  std::string name;
  Identity model;
  math::Pose3d pose;
  math::Pose3d inertialPose;
  // cppcheck-suppress unusedStructMember
  double mass;
  btVector3 inertia;
  std::shared_ptr<btDefaultMotionState> motionState;
  std::shared_ptr<btCompoundShape> collisionShape;
  std::shared_ptr<btRigidBody> link;
};

struct CollisionInfo
{
  std::string name;
  std::shared_ptr<btCollisionShape> shape;
  Identity link;
  Identity model;
  math::Pose3d pose;
  // cppcheck-suppress unusedStructMember
  bool isMesh;
  std::shared_ptr<btTriangleMesh> mesh;
};

struct JointInfo
{
  std::string name;
  // Base class for all the constraint objects,
  std::shared_ptr<btTypedConstraint> joint;
  // cppcheck-suppress unusedStructMember
  std::size_t childLinkId;
  // cppcheck-suppress unusedStructMember
  std::size_t parentLinkId;
  // cppcheck-suppress unusedStructMember
  int constraintType;
  ignition::math::Vector3d axis;
};

inline btMatrix3x3 convertMat(Eigen::Matrix3d mat)
{
  return btMatrix3x3(mat(0, 0), mat(0, 1), mat(0, 2),
                     mat(1, 0), mat(1, 1), mat(1, 2),
                     mat(2, 0), mat(2, 1), mat(2, 2));
}

inline btVector3 convertVec(Eigen::Vector3d vec)
{
  return btVector3(vec(0), vec(1), vec(2));
}

inline Eigen::Matrix3d convert(btMatrix3x3 mat)
{
  Eigen::Matrix3d val;
  val << mat[0][0], mat[0][1], mat[0][2],
         mat[1][0], mat[1][1], mat[1][2],
         mat[2][0], mat[2][1], mat[2][2];
  return val;
}

inline Eigen::Vector3d convert(btVector3 vec)
{
  Eigen::Vector3d val;
  val << vec[0], vec[1], vec[2];
  return val;
}

class Base : public Implements3d<FeatureList<Feature>>
{
  public: std::size_t entityCount = 0;

  public: inline std::size_t GetNextEntity()
  {
    return entityCount++;
  }

  public: inline Identity InitiateEngine(std::size_t /*_engineID*/) override
  {
    const auto id = this->GetNextEntity();
    assert(id == 0);

    return this->GenerateIdentity(0);
  }

  public: inline std::size_t idToIndexInContainer(std::size_t _id) const
  {
    auto it = this->childIdToParentId.find(_id);
    if (it != this->childIdToParentId.end())
    {
      std::size_t index = 0;
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

  public: inline std::size_t indexInContainerToId(
    const std::size_t _containerId, const std::size_t _index) const
  {
    std::size_t counter = 0;
    auto it = this->childIdToParentId.begin();

    while (counter <= _index && it != this->childIdToParentId.end())
    {
      if (it->second == _containerId && counter == _index)
      {
        return it->first;
      }
      else if (it->second == _containerId)
      {
        ++counter;
      }
      ++it;
    }
    // return invalid id if entity not found
    return -1;
  }

  public: inline Identity AddWorld(WorldInfo _worldInfo)
  {
    const auto id = this->GetNextEntity();
    this->worlds[id] = std::make_shared<WorldInfo>(_worldInfo);
    this->childIdToParentId.insert({id, -1});
    return this->GenerateIdentity(id, this->worlds.at(id));
  }

  public: inline Identity AddModel(std::size_t _worldId, ModelInfo _modelInfo)
  {
    const auto id = this->GetNextEntity();
    this->models[id] = std::make_shared<ModelInfo>(_modelInfo);
    this->childIdToParentId.insert({id, _worldId});
    return this->GenerateIdentity(id, this->models.at(id));
  }

  public: inline Identity AddLink(std::size_t _modelId, LinkInfo _linkInfo)
  {
    const auto id = this->GetNextEntity();
    this->links[id] = std::make_shared<LinkInfo>(_linkInfo);

    auto model = this->models.at(_linkInfo.model);
    model->links.push_back(id);

    this->childIdToParentId.insert({id, _modelId});
    return this->GenerateIdentity(id, this->links.at(id));
  }
  public: inline Identity AddCollision(
    std::size_t _linkId, CollisionInfo _collisionInfo)
  {
   const auto id = this->GetNextEntity();
   this->collisions[id] = std::make_shared<CollisionInfo>(_collisionInfo);
   this->childIdToParentId.insert({id, _linkId});
   return this->GenerateIdentity(id, this->collisions.at(id));
  }

  public: inline Identity AddJoint(JointInfo _jointInfo)
  {
    const auto id = this->GetNextEntity();
    this->joints[id] = std::make_shared<JointInfo>(_jointInfo);

    return this->GenerateIdentity(id, this->joints.at(id));
  }

  public: using WorldInfoPtr = std::shared_ptr<WorldInfo>;
  public: using ModelInfoPtr = std::shared_ptr<ModelInfo>;
  public: using LinkInfoPtr  = std::shared_ptr<LinkInfo>;
  public: using CollisionInfoPtr = std::shared_ptr<CollisionInfo>;
  public: using JointInfoPtr  = std::shared_ptr<JointInfo>;

  public: std::unordered_map<std::size_t, WorldInfoPtr> worlds;
  public: std::unordered_map<std::size_t, ModelInfoPtr> models;
  public: std::unordered_map<std::size_t, LinkInfoPtr> links;
  public: std::unordered_map<std::size_t, CollisionInfoPtr> collisions;
  public: std::unordered_map<std::size_t, JointInfoPtr> joints;

  // childIdToParentId needs to be an ordered map so this iteration proceeds
  // in ascending order of the keys of that map. Do not change.
  public: std::map<std::size_t, std::size_t> childIdToParentId;
};

}  // namespace bullet
}  // namespace physics
}  // namespace ignition

#endif
