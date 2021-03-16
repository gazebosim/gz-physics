/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include <ignition/common/Console.hh>
#include <ignition/physics/Implements.hh>
#include <ignition/math/eigen3/Conversions.hh>

namespace ignition {
namespace physics {
namespace bullet {

/// \brief The structs WorldInfo are used
/// for three reasons:
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
};

struct JointInfo
{
  std::string name;
  // Base class for all the constraint objects,
  std::shared_ptr<btTypedConstraint> joint;
  std::size_t childLinkId;
  std::size_t parentLinkId;
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

  public: inline Identity AddWorld(WorldInfo _worldInfo)
  {
    const auto id = this->GetNextEntity();
    this->worlds[id] = std::make_shared<WorldInfo>(_worldInfo);
    return this->GenerateIdentity(id, this->worlds.at(id));
  }

  public: inline Identity AddModel(ModelInfo _modelInfo)
  {
    const auto id = this->GetNextEntity();
    this->models[id] = std::make_shared<ModelInfo>(_modelInfo);
    return this->GenerateIdentity(id, this->models.at(id));
  }

  public: inline Identity AddLink(LinkInfo _linkInfo)
  {
    const auto id = this->GetNextEntity();
    this->links[id] = std::make_shared<LinkInfo>(_linkInfo);

    auto model = this->models.at(_linkInfo.model);
    model->links.push_back(id);

    return this->GenerateIdentity(id, this->links.at(id));
  }
  public: inline Identity AddCollision(CollisionInfo _collisionInfo)
  {
   const auto id = this->GetNextEntity();
   this->collisions[id] = std::make_shared<CollisionInfo>(_collisionInfo);
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
};

}  // namespace bullet
}  // namespace physics
}  // namespace ignition

#endif
