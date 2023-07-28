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

#ifndef GZ_PHYSICS_BULLET_BASE_HH_
#define GZ_PHYSICS_BULLET_BASE_HH_

#include <btBulletDynamicsCommon.h>
#include <Eigen/Geometry>

#include <assert.h>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <map>

#include <gz/common/Console.hh>
#include <gz/physics/Implements.hh>
#include <gz/math/eigen3/Conversions.hh>

namespace gz {
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
  std::vector<std::size_t> models = {};
  std::unordered_map<std::string, std::size_t> modelsByName = {};
};

struct ModelInfo
{
  std::string name;
  Identity world;
  // cppcheck-suppress unusedStructMember
  bool fixed;
  math::Pose3d pose;
  std::vector<std::size_t> links = {};
  std::unordered_map<std::string, std::size_t> linksByName = {};
  std::vector<std::size_t> joints = {};
  std::unordered_map<std::string, std::size_t> jointsByName = {};
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
  std::vector<std::size_t> shapes = {};
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
  gz::math::Vector3d axis;
};

inline btMatrix3x3 convertMat(Eigen::Matrix3d mat)
{
  return btMatrix3x3(
      static_cast<btScalar>(mat(0, 0)), static_cast<btScalar>(mat(0, 1)),
      static_cast<btScalar>(mat(0, 2)), static_cast<btScalar>(mat(1, 0)),
      static_cast<btScalar>(mat(1, 1)), static_cast<btScalar>(mat(1, 2)),
      static_cast<btScalar>(mat(2, 0)), static_cast<btScalar>(mat(2, 1)),
      static_cast<btScalar>(mat(2, 2)));
}

inline btVector3 convertVec(Eigen::Vector3d vec)
{
  return btVector3(static_cast<btScalar>(vec(0)), static_cast<btScalar>(vec(1)),
                   static_cast<btScalar>(vec(2)));
}

inline btVector3 convertVec(const math::Vector3d& vec)
{
  return btVector3(static_cast<btScalar>(vec[0]), static_cast<btScalar>(vec[1]),
                   static_cast<btScalar>(vec[2]));
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

inline math::Vector3d convertToGz(const btVector3 &_vec)
{
  return math::Vector3d(_vec[0], _vec[1], _vec[2]);
}

inline math::Quaterniond convertToGz(const btMatrix3x3 &_mat)
{
  return math::Quaterniond(math::Matrix3d(_mat[0][0], _mat[0][1], _mat[0][2],
                                          _mat[1][0], _mat[1][1], _mat[1][2],
                                          _mat[2][0], _mat[2][1], _mat[2][2]));
}

inline math::Pose3d convertToGz(const btTransform &_pose)
{
  return math::Pose3d(convertToGz(_pose.getOrigin()),
                      convertToGz(_pose.getBasis()));
}


class Base : public Implements3d<FeatureList<Feature>>
{
  public: std::size_t entityCount = 0;

  public: inline std::size_t GetNextEntity()
  {
    return entityCount++;
  }

  public: inline Identity AddWorld(WorldInfo _worldInfo)
  {
    const auto id = this->GetNextEntity();
    const auto world = std::make_shared<WorldInfo>(_worldInfo);
    this->worlds[id] = world;
    this->worldsByName[world->name] = id;
    this->worldsByIndex.push_back(id);
    return this->GenerateIdentity(id, this->worlds.at(id));
  }

  public: inline Identity AddModel(std::size_t _worldId, ModelInfo _modelInfo)
  {
    const auto id = this->GetNextEntity();
    const auto model = std::make_shared<ModelInfo>(_modelInfo);
    this->models[id] = model;
    const auto world = this->worlds.at(_worldId);
    world->models.push_back(id);
    world->modelsByName[model->name] = id;
    return this->GenerateIdentity(id, this->models.at(id));
  }

  public: inline Identity AddLink(LinkInfo _linkInfo)
  {
    const auto id = this->GetNextEntity();
    const auto link = std::make_shared<LinkInfo>(_linkInfo);
    this->links[id] = link;

    auto model = this->models.at(link->model);
    model->links.push_back(id);
    model->linksByName[link->name] = id;
    return this->GenerateIdentity(id, this->links.at(id));
  }
  public: inline Identity AddCollision(
    std::size_t _linkId, CollisionInfo _collisionInfo)
  {
   const auto id = this->GetNextEntity();
   this->collisions[id] = std::make_shared<CollisionInfo>(_collisionInfo);
   this->links.at(_linkId)->shapes.push_back(id);
   return this->GenerateIdentity(id, this->collisions.at(id));
  }

  public: inline Identity AddJoint(
    std::optional<std::size_t> _modelId, JointInfo _jointInfo)
  {
    const auto id = this->GetNextEntity();
    const auto joint = std::make_shared<JointInfo>(_jointInfo);
    this->joints[id] = joint;
    if (_modelId.has_value())
    {
      const auto model = this->models.at(*_modelId);
      model->joints.push_back(id);
      model->jointsByName[joint->name] = id;
    }

    return this->GenerateIdentity(id, this->joints.at(id));
  }

  public: using WorldInfoPtr = std::shared_ptr<WorldInfo>;
  public: using ModelInfoPtr = std::shared_ptr<ModelInfo>;
  public: using LinkInfoPtr  = std::shared_ptr<LinkInfo>;
  public: using CollisionInfoPtr = std::shared_ptr<CollisionInfo>;
  public: using JointInfoPtr  = std::shared_ptr<JointInfo>;

  public: std::unordered_map<std::size_t, WorldInfoPtr> worlds;
  public: std::vector<std::size_t> worldsByIndex;
  public: std::unordered_map<std::string, std::size_t> worldsByName;
  public: std::unordered_map<std::size_t, ModelInfoPtr> models;
  public: std::unordered_map<std::size_t, LinkInfoPtr> links;
  public: std::unordered_map<std::size_t, CollisionInfoPtr> collisions;
  public: std::unordered_map<std::size_t, JointInfoPtr> joints;
};

}  // namespace bullet
}  // namespace physics
}  // namespace gz

#endif
