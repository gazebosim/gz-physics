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

#ifndef GZ_PHYSICS_BULLET_FEATHERSTONE_BASE_HH_
#define GZ_PHYSICS_BULLET_FEATHERSTONE_BASE_HH_

#include <BulletCollision/CollisionShapes/btCollisionShape.h>
#include <BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h>
#include <LinearMath/btVector3.h>
#include <LinearMath/btVector3.h>
#include <btBulletDynamicsCommon.h>
#include <BulletDynamics/Featherstone/btMultiBody.h>
#include <BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h>
#include <BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h>

#include <Eigen/Geometry>
#include <algorithm>

#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <variant>

#include <gz/common/Console.hh>
#include <gz/physics/Implements.hh>
#include <gz/math/eigen3/Conversions.hh>

namespace gz {
namespace physics {
namespace bullet_featherstone {

/// \brief The Info structs are used for three reasons:
/// 1) Holding extra information such as the name
///    that will be different from the underlying engine
/// 2) Wrap shared pointers to Bullet entities to use as parameters to
///    GenerateIdentity.
/// 3) Hold explicit copies of raw pointers that can be deallocated

// Note: For Bullet library it's important the order in which the elements
// are destroyed. The current implementation relies on C++ destroying the
// elements in the opposite order stated in the structure
struct WorldInfo
{
  std::string name;
  std::unique_ptr<btDefaultCollisionConfiguration> collisionConfiguration;
  std::unique_ptr<btCollisionDispatcher> dispatcher;
  std::unique_ptr<btBroadphaseInterface> broadphase;
  std::unique_ptr<btMultiBodyConstraintSolver> solver;
  std::unique_ptr<btMultiBodyDynamicsWorld> world;

  std::unordered_map<std::size_t, std::size_t> modelIndexToEntityId;
  std::unordered_map<std::string, std::size_t> modelNameToEntityId;
  std::size_t nextModelIndex = 0;

  explicit WorldInfo(std::string name);
};

struct ModelInfo
{
  std::string name;
  Identity world;
  std::size_t indexInWorld;
  // cppcheck-suppress unusedStructMember
  bool fixed;
  math::Pose3d pose;
  std::unique_ptr<btMultiBody> body;

  std::vector<std::size_t> linkEntityIds;

  /// These are joints that connect this model to other models, e.g. fixed
  /// constraints.
  std::unordered_set<std::size_t> external_constraints;
};

/// Link information is embedded inside the model, so all we need to store here
/// is a reference to the model and the index of this link inside of it.
struct LinkInfo
{
  std::string name;
  std::size_t indexInModel;
  Identity model;
};

struct CollisionInfo
{
  std::string name;
  std::unique_ptr<btCollisionShape> shape;
  Identity link;
  math::Pose3d relative_pose;
  // cppcheck-suppress unusedStructMember
  bool isMesh;
  std::unique_ptr<btTriangleMesh> mesh;
};

struct InternalJoint
{
  std::size_t indexInModel;
  Identity model;
};

struct JointInfo
{
  std::string name;

  // The joint might be identified by an index within a model or by a constraint
  // in the world.
  std::variant<
    std::monostate,
    InternalJoint,
    std::unique_ptr<btMultiBodyConstraint>> identifier;

  // cppcheck-suppress unusedStructMember
  Identity childLink;
  // cppcheck-suppress unusedStructMember
  Identity parentLink;
  // cppcheck-suppress unusedStructMember
  int constraintType;
  gz::math::Vector3d axis;
};

inline btMatrix3x3 convertMat(const Eigen::Matrix3d& mat)
{
  return btMatrix3x3(mat(0, 0), mat(0, 1), mat(0, 2),
                     mat(1, 0), mat(1, 1), mat(1, 2),
                     mat(2, 0), mat(2, 1), mat(2, 2));
}

inline btVector3 convertVec(const Eigen::Vector3d& vec)
{
  return btVector3(vec(0), vec(1), vec(2));
}

inline btTransform convertTf(const Eigen::Isometry3d& tf)
{
  return btTransform(
    convertMat(tf.linear()),
    convertVec(tf.translation()));
}

inline Eigen::Matrix3d convert(const btMatrix3x3& mat)
{
  Eigen::Matrix3d val;
  val << mat[0][0], mat[0][1], mat[0][2],
         mat[1][0], mat[1][1], mat[1][2],
         mat[2][0], mat[2][1], mat[2][2];
  return val;
}

inline Eigen::Vector3d convert(const btVector3& vec)
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
    auto world = std::make_shared<WorldInfo>(std::move(_worldInfo));
    this->worlds[id] = world;
    return this->GenerateIdentity(id, world);
  }

  public: inline Identity AddModel(ModelInfo _modelInfo)
  {
    const auto id = this->GetNextEntity();
    auto model = std::make_shared<ModelInfo>(std::move(_modelInfo));
    this->models[id] = model;
    auto *world = this->ReferenceInterface<WorldInfo>(model->world);
    world->world->addMultiBody(model->body.get());
    world->modelNameToEntityId[model->name] = id;
    model->indexInWorld = world->nextModelIndex++;
    world->modelIndexToEntityId[model->indexInWorld] = id;
    return this->GenerateIdentity(id, model);
  }

  public: inline Identity AddLink(LinkInfo _linkInfo)
  {
    const auto id = this->GetNextEntity();
    auto link = std::make_shared<LinkInfo>(std::move(_linkInfo));
    this->links[id] = link;
    return this->GenerateIdentity(id, link);
  }

  public: inline Identity AddCollision(CollisionInfo _collisionInfo)
  {
   const auto id = this->GetNextEntity();
   auto collision = std::make_shared<CollisionInfo>(std::move(_collisionInfo));
   this->collisions[id] = collision;
   return this->GenerateIdentity(id, collision);
  }

  public: inline Identity AddJoint(JointInfo _jointInfo)
  {
    const auto id = this->GetNextEntity();
    auto joint = std::make_shared<JointInfo>(std::move(_jointInfo));
    this->joints[id] = joint;
    return this->GenerateIdentity(id, joint);
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

}  // namespace bullet_featherstone
}  // namespace physics
}  // namespace gz

#endif
