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

#include <BulletCollision/CollisionShapes/btCollisionShape.h>
#include <BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h>
#include <LinearMath/btVector3.h>
#include <btBulletDynamicsCommon.h>

#include <Eigen/Geometry>
#include <algorithm>
#include <ignition/common/Console.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/eigen3/Conversions.hh>
#include <ignition/physics/Implements.hh>
#include <ignition/physics/detail/Identity.hh>
#include <memory>
#include <sdf/Joint.hh>
#include <string>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

///
/// \brief The structs World, Model, Link, Joint and Collision are used to
/// manage and store the required Bullet related objects (e.g. btRigidBody is
/// the underlying Bullet object for a Link). An object of any of the types
/// mentioned above is considered an Entity. We use the constructors/destructors
/// of these Entities not only to initialize/clean their internal data
/// structures, but also to perform the required computations to add/remove
/// their related Bullet objects from the Bullet World.
///
/// An Entity is considered a Container if itself manages other Entities. For
/// example, a Model is a Container for Link, Joints, and other (nested) Models.
/// A Container keeps a dynamic array of `unique_ptrs` to its child entities,
/// which means that deleting a Container Entity will automatically delete all
/// its childs. The hierarchy can be described as:
///
/// | Entity      | Contains             |
/// | -----------------------------------|
/// | World       | Models               |
/// | Model       | Links, Joints, Models|
/// | Link        | Collisions           |
/// | Joint       | -                    |
/// | Colllision  | -                    |
///
/// For example, a possible Entities configuration could be represented visually
/// as:
///                          ┌───────┐
///                          │ world │
///              ┌───────────┴───────┴───────────┐
///              │                               │
///              ▼                               ▼
///          ┌───────┐                       ┌───────┐
///          │model_A│                       │model_B│
///     ┌────┴───────┴────┐             ┌────┴───────┴────┐
///     ▼        ▼        ▼             ▼                 ▼
/// ┌──────┐ ┌──────┐ ┌───────┐     ┌───────┐          ┌──────┐
/// │link_A│ │link_B│ │joint_A│     │model_C│          │link_C│
/// └──────┘ └──────┘ └───────┘     └───────┘          └──────┘
///                                     ▼                  ▼
///                                 ┌────────┐       ┌───────────┐
///                                 │ link_D │       │collision_A│
///                                 └────────┘       └───────────┘
///                                     ▼
///                               ┌────────────┐
///                               │collision_B │
///                               └────────────┘
///
/// For fast O(1) access, we also mantain an index that maps each Entity ID to
/// the Entity pointer.
///

namespace ignition {
namespace physics {
namespace bullet {

struct World;
struct Model;
struct Link;
struct Joint;
struct Collision;

/////////////////////////////////////////////////
struct World {
  explicit World(std::string _name);
  std::string name;
  std::unique_ptr<btDefaultCollisionConfiguration> collisionConfiguration;
  std::unique_ptr<btCollisionDispatcher> dispatcher;
  std::unique_ptr<btBroadphaseInterface> broadphase;
  std::unique_ptr<btConstraintSolver> solver;
  std::unique_ptr<btDiscreteDynamicsWorld> btWorld;
  std::vector<std::unique_ptr<Model>> models;
};

/////////////////////////////////////////////////
struct Model {
  Model(std::string _name,
        const math::Pose3d& _sdfPose,
        const math::Pose3d& _worldPose,
        bool _static,
        btDiscreteDynamicsWorld* _btWorld);
  std::string name;
  math::Pose3d sdfPose;
  math::Pose3d worldPose;
  bool fixed;
  btDiscreteDynamicsWorld* world;
  std::vector<std::unique_ptr<Link>> links;
  std::vector<std::unique_ptr<Joint>> joints;
  std::vector<std::unique_ptr<Model>> models;
};

/////////////////////////////////////////////////
struct Link {
  Link(std::string _name,
       const math::Pose3d& _sdfPose,
       const math::Pose3d& _sdfInertialPose,
       btDiscreteDynamicsWorld* _btWorld,
       const btTransform& _baseTransform,
       double _mass,
       const btVector3& _linkInertiaDiag);

  ~Link();

  std::string name;
  math::Pose3d sdfPose;
  math::Pose3d sdfInertialPose;
  btDiscreteDynamicsWorld* world;
  std::unique_ptr<btDefaultMotionState> motionState;
  std::unique_ptr<btCompoundShape> collisionShape;
  std::unique_ptr<btRigidBody> body;
  std::vector<std::unique_ptr<Collision>> collisions;
};

/////////////////////////////////////////////////
struct Collision {
  Collision(std::string _name,
            btRigidBody* _body,
            bool _isMesh,
            std::unique_ptr<btCollisionShape>&& _shape,
            std::unique_ptr<btTriangleMesh>&& _mesh,
            const btTransform& _localPose,
            double _mu = .0,
            double _mu2 = .0);

  ~Collision();

  std::string name;
  btRigidBody* body;
  btTransform localPose;
  bool isMesh;
  std::unique_ptr<btGImpactMeshShape> gImpactMeshShape;
  std::unique_ptr<btCollisionShape> shape;
  std::unique_ptr<btTriangleMesh> mesh;
};

/////////////////////////////////////////////////
struct Joint {
  Joint(std::string _name,
        btDiscreteDynamicsWorld* _world,
        btRigidBody* _childBody,
        btRigidBody* _parentBody,
        const btVector3& _pivotChild,
        const btVector3& _pivotParent,
        const btVector3& _axisChild,
        const btVector3& _axisParent,
        ::sdf::JointType _type);

  void SetPosition(double _value);
  double GetPosition();

  ~Joint();

  std::string name;
  Model* model;
  btDiscreteDynamicsWorld* world;
  btRigidBody* childBody;
  btRigidBody* parentBody;
  ::sdf::JointType type;
  ignition::math::Vector3d axis;
  std::unique_ptr<btTypedConstraint> constraint;
};

/////////////////////////////////////////////////
class Base : public Implements3d<FeatureList<Feature>> {
  using EntityPtr = std::variant<World*, Model*, Link*, Joint*, Collision*>;

  /// \brief Only one world allowed
 public:
  std::unique_ptr<World> world;

  /// \ brief The World ID
 public:
  std::size_t worldId;

  /// \brief Hashmap for fast access to an Element given the ID
 public:
  std::unordered_map<std::size_t, EntityPtr> entities;

  /// \brief Hashmap for fast access to a Container given its child ID
 public:
  std::unordered_map<std::size_t, EntityPtr> containers;

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
  inline Identity AddModel(std::unique_ptr<Model>&& _model) {
    auto id = this->GetNextEntity();
    this->world->models.push_back(std::move(_model));
    const auto& modelPtr = world->models.back();
    this->entities[id] = modelPtr.get();
    this->containers[id] = this->world.get();
    return this->GenerateIdentity(id, nullptr);
  }

  /// \brief Add a nested Model Entity contained by another Model
 public:
  inline Identity AddNestedModel(std::size_t _modelId,
                                 std::unique_ptr<Model>&& _model) {
    auto id = this->GetNextEntity();
    auto model = std::get<Model*>(this->entities.at(_modelId));
    model->models.push_back(std::move(_model));
    const auto& nestedModelPtr = model->models.back();
    this->entities[id] = nestedModelPtr.get();
    this->containers[id] = model;
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
    this->containers[id] = model;
    return this->GenerateIdentity(id, nullptr);
  }

  /// \brief Add a Collision Entity
 public:
  inline Identity AddCollision(std::size_t _linkId,
                               std::unique_ptr<Collision>&& _collision) {
    auto id = this->GetNextEntity();
    auto link = std::get<Link*>(this->entities.at(_linkId));
    link->collisions.push_back(std::move(_collision));
    const auto& collisionPtr = link->collisions.back();
    this->entities[id] = collisionPtr.get();
    this->containers[id] = link;
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
    this->containers[id] = model;
    return this->GenerateIdentity(id, nullptr);
  }
};

/////////////////////////////////////////////////
inline btMatrix3x3 convertMat(Eigen::Matrix3d mat) {
  return btMatrix3x3(mat(0, 0), mat(0, 1), mat(0, 2), mat(1, 0), mat(1, 1),
                     mat(1, 2), mat(2, 0), mat(2, 1), mat(2, 2));
}

/////////////////////////////////////////////////
inline btVector3 convertVec(Eigen::Vector3d vec) {
  return btVector3(vec(0), vec(1), vec(2));
}

/////////////////////////////////////////////////
inline Eigen::Matrix3d convert(btMatrix3x3 mat) {
  Eigen::Matrix3d val;
  val << mat[0][0], mat[0][1], mat[0][2], mat[1][0], mat[1][1], mat[1][2],
      mat[2][0], mat[2][1], mat[2][2];
  return val;
}

/////////////////////////////////////////////////
inline Eigen::Vector3d convert(btVector3 vec) {
  Eigen::Vector3d val;
  val << vec[0], vec[1], vec[2];
  return val;
}

}  // namespace bullet
}  // namespace physics
}  // namespace ignition

#endif
