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

#include "ignition/physics/Geometry.hh"
#include "ignition/physics/Geometry.hh"
#include <BulletCollision/CollisionShapes/btCollisionShape.h>
#include <BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h>
#include <LinearMath/btQuaternion.h>
#include <LinearMath/btVector3.h>
#include <btBulletDynamicsCommon.h>
#include <BulletDynamics/Featherstone/btMultiBody.h>
#include <BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h>
#include <BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h>

#include <Eigen/Geometry>
#include <algorithm>
#include <ignition/common/Console.hh>
#include <ignition/math/Matrix3.hh>
#include <ignition/math/graph/Graph.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/eigen3/Conversions.hh>
#include <ignition/physics/Implements.hh>
#include <ignition/physics/detail/Identity.hh>
#include <memory>
#include <sdf/Collision.hh>
#include <sdf/Joint.hh>
#include <sdf/Link.hh>
#include <string>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

namespace ignition {
namespace physics {
namespace bullet {

struct World;
struct RootModel;
struct Model;
struct Link;
struct Joint;
struct Collision;

/////////////////////////////////////////////////
struct World {
  explicit World(std::string _name) : name(std::move(_name)) {
    collisionConfiguration = std::make_unique<btDefaultCollisionConfiguration>();
    dispatcher =
        std::make_unique<btCollisionDispatcher>(collisionConfiguration.get());
    broadphase = std::make_unique<btDbvtBroadphase>();
    solver = std::make_unique<btMultiBodyConstraintSolver>();
    btWorld = std::make_unique<btMultiBodyDynamicsWorld>(
        dispatcher.get(), broadphase.get(), solver.get(),
        collisionConfiguration.get());
    // TO-DO(Lobotuerk): figure out what this line does
    btWorld->getSolverInfo().m_globalCfm = 0;
    btGImpactCollisionAlgorithm::registerAlgorithm(dispatcher.get());
  }

  std::string name;
  std::unique_ptr<btDefaultCollisionConfiguration> collisionConfiguration;
  std::unique_ptr<btCollisionDispatcher> dispatcher;
  std::unique_ptr<btBroadphaseInterface> broadphase;
  std::unique_ptr<btMultiBodyConstraintSolver> solver;
  std::unique_ptr<btMultiBodyDynamicsWorld> btWorld;
  std::vector<std::unique_ptr<Model>> models;
  std::vector<std::unique_ptr<RootModel>> rootModels;
};

/////////////////////////////////////////////////
struct RootModel {
  RootModel(std::string _name, btMultiBodyDynamicsWorld* _world, bool _isStatic, const math::Pose3d& _sdfPose)
      : name(std::move(_name)), isStatic(_isStatic), sdfPose(_sdfPose), world(_world) {};

  struct MeshWithPose {
    std::unique_ptr<btTriangleMesh> mesh;
    Pose3d pose;
  };

  std::string name;
  bool isStatic;
  math::Pose3d sdfPose;

  // A model can either be single-link or multi-link. We represent a single-link model with
  // an instance of btRigidBody and a multi-link model with an instance of btMultiBody.
  // Since we don't know a priori what type of model this will be, the std::variant object
  // is initialized with a std::monostate type.
  std::variant<std::monostate, btMultiBody, btRigidBody> body;

  math::graph::DirectedGraph<::sdf::Link, ::sdf::Joint> skeleton;
  btMultiBodyDynamicsWorld* world;
  std::unordered_map<math::graph::VertexId, int> vertexIdToLinkIndex;
  std::unordered_map<math::graph::EdgeId, int> edgeIdToJointIndex;
  std::unordered_map<math::graph::VertexId, std::vector<::sdf::Collision>> vertexIdToSdfCollisions;
  std::unordered_map<math::graph::VertexId, std::vector<MeshWithPose>> vertexIdToMeshes;
  std::unordered_map<math::graph::VertexId, math::Pose3d> vertexIdToLinkPoseFromPivot;

  // Collision objects
  std::vector<std::unique_ptr<btCollisionShape>> collisionShapes;
};

/////////////////////////////////////////////////
struct Model {
  Model(std::string _name,
        const math::Pose3d& _sdfPose,
        RootModel* _rootModel)
      : name(std::move(_name)), sdfPose(_sdfPose), rootModel(_rootModel) {};

  std::string name;
  math::Pose3d sdfPose;
  RootModel* rootModel;
  std::vector<std::unique_ptr<Model>> models;
  std::vector<std::unique_ptr<Link>> links;
  std::vector<std::unique_ptr<Joint>> joints;
};

/////////////////////////////////////////////////
struct Link {
  Link(std::string _name, RootModel* _rootModel, math::graph::VertexId _vertexId) : name(std::move(_name)), rootModel(_rootModel), vertexId(_vertexId) {}
  std::string name;
  RootModel* rootModel;
  math::graph::VertexId vertexId;
};

/////////////////////////////////////////////////
struct Collision {
  Collision(std::string _name, RootModel* _rootModel, math::graph::VertexId _vertexId) : name(_name), rootModel(_rootModel), vertexId(_vertexId) {}
  std::string name;
  RootModel* rootModel;
  math::graph::VertexId vertexId;
};

/////////////////////////////////////////////////
struct Joint {
  Joint(std::string _name, ::sdf::JointType _type, RootModel *_rootModel, math::graph::EdgeId _edgeId) : name(std::move(_name)),
  type(_type), rootModel(_rootModel), edgeId(_edgeId) {}
  std::string name;
  ::sdf::JointType type;
  RootModel* rootModel;
  math::graph::EdgeId edgeId;
};

/////////////////////////////////////////////////
class Base : public Implements3d<FeatureList<Feature>> {
  using EntityPtr =
      std::variant<World*, Model*, Link*, Joint*, Collision*>;

  /// \brief Only one world allowed
 public:
  std::unique_ptr<World> world;

  /// \ brief The World ID
 public:
  std::size_t worldId;

  /// \brief Hashmap for fast access to an Element given the ID
 public:
  std::unordered_map<std::size_t, EntityPtr> entities;

  /// \brief Hashmap for fast access to a Container given the child ID
 public:
  std::unordered_map<std::size_t, EntityPtr> container;

  /// \brief Counter for generating IDs
 private:
  std::size_t entityCount = 0;

  /// \brief Gets the next available ID
 public:
  inline std::size_t GetNextEntity() { return entityCount++; }

  /// \brief Add a World Entity to this Base object.
 public:
  inline Identity InitiateEngine(std::size_t /*_engineID*/) override {
    const auto id = this->GetNextEntity();
    assert(id == 0);
    return this->GenerateIdentity(0);
  }

  /// \brief Add a World Entity to this Base object.
 public:
  inline Identity AddWorld(std::unique_ptr<World>&& _world) {
    auto id = this->GetNextEntity();
    this->world = std::move(_world);
    auto identity = this->GenerateIdentity(id, nullptr);
    this->worldId = identity.id;
    return identity;
  }

  /// \brief Add a Model Entity. It will be contained by the World Entity.
 public:
  inline Identity AddModel(std::unique_ptr<RootModel>&& _rootModel,
                           std::unique_ptr<Model>&& _model) {
    auto id = this->GetNextEntity();
    this->world->rootModels.push_back(std::move(_rootModel));
    this->world->models.push_back(std::move(_model));
    auto& modelPtr = world->models.back();
    this->entities[id] = modelPtr.get();
    this->container[id] = this->world.get();
    return this->GenerateIdentity(id, nullptr);
  }

  /// \brief Add a nested Model Entity contained by another Model
 public:
  inline Identity AddNestedModel(std::size_t _modelId,
                                 std::unique_ptr<Model>&& _model) {
    auto parentModel = std::get<Model*>(this->entities.at(_modelId));
    auto id = this->GetNextEntity();
    parentModel->models.push_back(std::move(_model));
    auto& nestedModelPtr = parentModel->models.back();
    this->entities[id] = nestedModelPtr.get();
    this->container[id] = parentModel;
    return this->GenerateIdentity(id, nullptr);
  }

  /// \brief Add a Link Entity
 public:
  inline Identity AddLink(std::size_t _modelId, std::unique_ptr<Link>&& _link) {
    auto id = this->GetNextEntity();
    auto model = std::get<Model*>(this->entities.at(_modelId));
    model->links.push_back(std::move(_link));
    const auto& linkPtr = model->links.back();
    this->entities[id] = linkPtr.get();
    this->container[id] = model;
    return this->GenerateIdentity(id, nullptr);
  }

  /// \brief Add a Collision Entity
 public:
  inline Identity AddCollision(std::size_t _linkId,
                               std::unique_ptr<Collision>&& _collision) {
    auto id = this->GetNextEntity();
    (void) _linkId;
    (void) _collision;
    // auto link = std::get<Link*>(this->entities.at(_linkId));
    // link->collisions.push_back(std::move(_collision));
    // const auto& collisionPtr = link->collisions.back();
    // this->entities[id] = collisionPtr.get();
    // this->container[id] = link;
    return this->GenerateIdentity(id, nullptr);
  }

  /// \brief Add a Joint Entity
 public:
  inline Identity AddJoint(std::size_t _modelId,
                           std::unique_ptr<Joint>&& _joint) {
    auto id = this->GetNextEntity();
    auto model = std::get<Model*>(this->entities.at(_modelId));
    model->joints.push_back(std::move(_joint));
    const auto& jointPtr = model->joints.back();
    this->entities[id] = jointPtr.get();
    this->container[id] = model;
    return this->GenerateIdentity(id, nullptr);
  }
};  // namespace physics

/////////////////////////////////////////////////
inline btQuaternion convertQuat(const math::Quaterniond& _q) {
  return btQuaternion(_q.X(), _q.Y(), _q.Z(), _q.W());
}

/////////////////////////////////////////////////
inline btMatrix3x3 convertMat(const Eigen::Matrix3d& mat) {
  return btMatrix3x3(mat(0, 0), mat(0, 1), mat(0, 2), mat(1, 0), mat(1, 1),
                     mat(1, 2), mat(2, 0), mat(2, 1), mat(2, 2));
}

/////////////////////////////////////////////////
inline btVector3 convertVec(const Eigen::Vector3d& vec) {
  return btVector3(vec(0), vec(1), vec(2));
}

/////////////////////////////////////////////////
inline Eigen::Matrix3d convert(const btMatrix3x3& mat) {
  Eigen::Matrix3d val;
  val << mat[0][0], mat[0][1], mat[0][2], mat[1][0], mat[1][1], mat[1][2],
      mat[2][0], mat[2][1], mat[2][2];
  return val;
}

/////////////////////////////////////////////////
inline Eigen::Vector3d convert(const btVector3& vec) {
  Eigen::Vector3d val;
  val << vec[0], vec[1], vec[2];
  return val;
}

}  // namespace bullet
}  // namespace physics
}  // namespace ignition

#endif
