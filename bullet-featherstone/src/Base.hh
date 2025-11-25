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
#include <BulletCollision/CollisionShapes/btCompoundShape.h>
#include <BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h>
#include <BulletDynamics/Featherstone/btMultiBody.h>
#include <BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h>
#include <BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h>
#include <BulletDynamics/Featherstone/btMultiBodyGearConstraint.h>
#include <BulletDynamics/Featherstone/btMultiBodyJointFeedback.h>
#include <BulletDynamics/Featherstone/btMultiBodyJointMotor.h>
#include <BulletDynamics/Featherstone/btMultiBodyJointLimitConstraint.h>
#include <BulletDynamics/Featherstone/btMultiBodyLinkCollider.h>
#include <BulletDynamics/Featherstone/btMultiBodyFixedConstraint.h>
#include <LinearMath/btVector3.h>
#include <btBulletDynamicsCommon.h>
#include "BulletCollision/Gimpact/btGImpactShape.h"

#include <Eigen/Geometry>

#include <algorithm>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <variant>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/math/eigen3/Conversions.hh>
#include <gz/physics/Implements.hh>

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

  std::unordered_map<int, std::size_t> modelIndexToEntityId;
  std::unordered_map<std::string, std::size_t> modelNameToEntityId;
  int nextModelIndex = 0;

  double stepSize = 0.001;

  explicit WorldInfo(std::string name);
};

/// \brief Custom `GzMultiBody` wrapper for `btMultiBody` used for the following
/// purposes where the btMultiBody API falls short:
/// - to ensure that a flag is set whenever joint position is set or base
/// transform is set indicating that collision transforms need to be updated.
/// - to apply explicit joint damping torques before stepping simulation.
class GzMultiBody: public btMultiBody
{
  using btMultiBody::btMultiBody;

  /// \brief Set position for a particular joint dof and set a flag that the
  /// collision world transforms need to be updated before performing collision
  /// queries. Use this method to set a joint dof position instead of the base
  /// class methods `setJointPos`/ `setJointPosMultiDof`/
  /// (non-const)`getJointPosMultiDof` to ensure that collision queries after
  /// setting joint positions are accurate.
  public: void SetJointPosForDof(
    int _jointIndex,
    std::size_t _dof,
    btScalar _value);
  private: using btMultiBody::setJointPos;
  private: using btMultiBody::setJointPosMultiDof;
  private: using btMultiBody::getJointPosMultiDof;

  /// \brief Get the position of a particular joint dof.
  /// This method is needed because the private using-declaration above hides
  /// both the const and non-const overloads of
  /// `btMultiBody::getJointPosMultiDof`.
  public: btScalar GetJointPosForDof(int _jointIndex, std::size_t _dof) const;

  /// \brief Set base transform in world.
  /// Use this method to set the transform for a moving base model instead of
  /// the base class method `setBaseWorldTransform` to ensure that collision
  /// queries after setting the transform are accurate.
  public: void SetBaseWorldTransform(const btTransform &_pose);
  private: using btMultiBody::setBaseWorldTransform;

  /// \brief Update collision transforms if `needsCollisionTransformsUpdate` is
  /// set and reset the flag.
  public: void UpdateCollisionTransformsIfNeeded();

  /// \brief Add joint damping and spring stiffness torque to the specified
  /// joint index on all dofs.
  /// \param[in] _jointIndex Joint index
  /// \param[in] _damping Joint damping coefficient
  /// \param[in] _springStiffness Joint spring stiffness
  /// \param[in] _springReference Joint spring reference
  public: void AddJointDampingStiffnessTorque(int _jointIndex, double _damping,
      double _springStiffness, double _springReference);

  private: bool needsCollisionTransformsUpdate = false;
};

struct ModelInfo
{
  std::string name;
  Identity world;
  int indexInWorld;
  Eigen::Isometry3d rootLinkToModelTf;
  Eigen::Isometry3d baseInertiaToLinkFrame;
  std::shared_ptr<GzMultiBody> body;

  bool isNestedModel = false;

  std::vector<std::size_t> linkEntityIds;
  std::vector<std::size_t> jointEntityIds;
  std::vector<std::size_t> nestedModelEntityIds;
  std::unordered_map<std::string, std::size_t> linkNameToEntityId;
  std::unordered_map<std::string, std::size_t> jointNameToEntityId;
  std::unordered_map<std::string, std::size_t> nestedModelNameToEntityId;

  /// These are joints that connect this model to other models, e.g. fixed
  /// constraints.
  std::unordered_set<std::size_t> external_constraints;

  ModelInfo(
    std::string _name,
    Identity _world,
    Eigen::Isometry3d _rootLinkToModelTf,
    Eigen::Isometry3d _baseInertiaToLinkFrame,
    std::shared_ptr<GzMultiBody> _body)
    : name(std::move(_name)),
      world(std::move(_world)),
      rootLinkToModelTf(_rootLinkToModelTf),
      baseInertiaToLinkFrame(_baseInertiaToLinkFrame),
      body(std::move(_body))
  {
    // Do nothing
  }
};

/// \brief Custom GzMultiBodyLinkCollider class
class GzMultiBodyLinkCollider: public btMultiBodyLinkCollider {
  using btMultiBodyLinkCollider::btMultiBodyLinkCollider;

  /// \brief Overrides base function to enable support for ignoring
  /// collision with objects from other bodies if
  /// btCollisionObject::setIgnoreCollisionCheck is called.
  /// Note: originally btMultiBodyLinkCollider::checkCollideWithOverride
  /// just returns true if the input collision object is from a
  /// different body and disregards any setIgnoreCollisionCheck calls.
  public: bool checkCollideWithOverride(const btCollisionObject *_co) const
          override
  {
    return btMultiBodyLinkCollider::checkCollideWithOverride(_co) &&
           btCollisionObject::checkCollideWithOverride(_co);
  }
};

/// Link information is embedded inside the model, so all we need to store here
/// is a reference to the model and the index of this link inside of it.
struct LinkInfo
{
  std::string name;
  std::optional<int> indexInModel;
  Identity model;
  Eigen::Isometry3d inertiaToLinkFrame;
  std::unique_ptr<GzMultiBodyLinkCollider> collider = nullptr;
  std::unique_ptr<btCompoundShape> shape = nullptr;
  std::vector<std::size_t> collisionEntityIds = {};
  std::unordered_map<std::string, std::size_t> collisionNameToEntityId = {};
  // Link is either static, fixed to world, or has zero dofs
  bool isStaticOrFixed = false;
};

struct CollisionInfo
{
  std::string name;
  std::unique_ptr<btCollisionShape> collider;
  Identity link;
  Eigen::Isometry3d linkToCollision;
  int indexInLink = 0;
};

struct InternalJoint
{
  int indexInBtModel;
};

struct RootJoint {};

struct FixedConstraintJoint {};

struct JointInfo
{
  std::string name;

  // The joint might be identified by an index within a model or by a constraint
  // in the world.
  std::variant<
    std::monostate,
    RootJoint,
    InternalJoint,
    FixedConstraintJoint> identifier;

  /// If the parent link is nullopt then the joint attaches its child to the
  /// world
  std::optional<std::size_t> parentLinkID;
  Identity childLinkID;

  // These properties are difficult to back out of the bullet API, so we save
  // them here. This violates the single-source-of-truth principle, but we do
  // not currently support modifying the kinematics of a model after it is
  // constructed.
  Eigen::Isometry3d tf_from_parent;
  Eigen::Isometry3d tf_to_child;

  Identity model;
  // This field gets set by AddJoint
  std::size_t indexInGzModel = 0;

  // joint limits
  // \todo(iche033) Extend to support joints with more than 1 dof
  double minEffort = 0.0;
  double maxEffort = 0.0;
  double minVelocity = 0.0;
  double maxVelocity = 0.0;
  double axisLower = 0.0;
  double axisUpper = 0.0;

  // joint damping, spring stiffness and reference
  double damping = 0.0;
  double springStiffness = 0.0;
  double springReference = 0.0;

  std::shared_ptr<btMultiBodyJointMotor> motor = nullptr;
  std::shared_ptr<btMultiBodyJointLimitConstraint> jointLimits = nullptr;
  std::shared_ptr<btMultiBodyFixedConstraint> fixedConstraint = nullptr;
  std::shared_ptr<btMultiBodyGearConstraint> gearConstraint = nullptr;
  std::shared_ptr<btMultiBodyJointFeedback> jointFeedback = nullptr;
};

inline btMatrix3x3 convertMat(const Eigen::Matrix3d& mat)
{
  return btMatrix3x3(
      static_cast<btScalar>(mat(0, 0)), static_cast<btScalar>(mat(0, 1)),
      static_cast<btScalar>(mat(0, 2)), static_cast<btScalar>(mat(1, 0)),
      static_cast<btScalar>(mat(1, 1)), static_cast<btScalar>(mat(1, 2)),
      static_cast<btScalar>(mat(2, 0)), static_cast<btScalar>(mat(2, 1)),
      static_cast<btScalar>(mat(2, 2)));
}

inline btVector3 convertVec(const Eigen::Vector3d& vec)
{
  return btVector3(static_cast<btScalar>(vec(0)), static_cast<btScalar>(vec(1)),
                   static_cast<btScalar>(vec(2)));
}

inline btVector3 convertVec(const math::Vector3d& vec)
{
  return btVector3(static_cast<btScalar>(vec[0]), static_cast<btScalar>(vec[1]),
                   static_cast<btScalar>(vec[2]));
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

inline Eigen::Isometry3d convert(const btTransform& tf)
{
  Eigen::Isometry3d output;
  output.translation() = convert(tf.getOrigin());
  output.linear() = convert(btMatrix3x3(tf.getRotation()));
  return output;
}

inline btTransform GetWorldTransformOfLinkInertiaFrame(
    const btMultiBody &body,
    const int linkIndexInModel)
{
  const auto p = body.localPosToWorld(
    linkIndexInModel, btVector3(0, 0, 0));
  const auto rot = body.localFrameToWorld(
    linkIndexInModel, btMatrix3x3::getIdentity());
  return btTransform(rot, p);
}

inline Eigen::Isometry3d GetWorldTransformOfLink(
    const ModelInfo &model,
    const LinkInfo &linkInfo)
{
  const auto &body = *model.body;
  const auto indexOpt = linkInfo.indexInModel;
  if (indexOpt.has_value())
  {
    return convert(GetWorldTransformOfLinkInertiaFrame(body, *indexOpt))
        * linkInfo.inertiaToLinkFrame;
  }

  return convert(body.getBaseWorldTransform()) * model.baseInertiaToLinkFrame;
}

class Base : public Implements3d<FeatureList<Feature>>
{
  // Note: Entity ID 0 is reserved for the "engine"
  public: std::size_t entityCount = 1;

  public: inline std::size_t GetNextEntity()
  {
    return entityCount++;
  }

  public: inline Identity InitiateEngine(std::size_t /*_engineID*/) override
  {
    return this->GenerateIdentity(0);
  }

  public: inline Identity AddWorld(WorldInfo _worldInfo)
  {
    const auto id = this->GetNextEntity();
    auto world = std::make_shared<WorldInfo>(std::move(_worldInfo));
    this->worlds[id] = world;
    auto worldID = this->GenerateIdentity(id, world);

    auto worldModel = std::make_shared<ModelInfo>(
      world->name, worldID, Eigen::Isometry3d::Identity(),
      Eigen::Isometry3d::Identity(), nullptr);
    this->models[id] = worldModel;
    world->modelNameToEntityId[worldModel->name] = id;
    worldModel->indexInWorld = -1;
    world->modelIndexToEntityId[worldModel->indexInWorld] = id;

    return worldID;
  }

  public: inline Identity AddModel(
    std::string _name,
    Identity _worldID,
    Eigen::Isometry3d _rootLinkToModelTf,
    Eigen::Isometry3d _baseInertialToLinkFrame,
    std::shared_ptr<GzMultiBody> _body)
  {
    const auto id = this->GetNextEntity();
    auto model = std::make_shared<ModelInfo>(
      std::move(_name), std::move(_worldID),
      std::move(_rootLinkToModelTf),
      std::move(_baseInertialToLinkFrame),
      std::move(_body));

    this->models[id] = model;
    auto *world = this->ReferenceInterface<WorldInfo>(model->world);
    world->modelNameToEntityId[model->name] = id;
    model->indexInWorld = world->nextModelIndex++;
    world->modelIndexToEntityId[model->indexInWorld] = id;

    auto worldModel = this->models.at(model->world);
    worldModel->nestedModelEntityIds.push_back(id);
    worldModel->nestedModelNameToEntityId[model->name] = id;

    return this->GenerateIdentity(id, model);
  }

  public: inline Identity AddNestedModel(
    std::string _name,
    Identity _parentID,
    Identity _worldID,
    Eigen::Isometry3d _rootLinkToModelTf,
    Eigen::Isometry3d _baseInertialToLinkFrame,
    std::shared_ptr<GzMultiBody> _body)
  {
    const auto id = this->GetNextEntity();
    auto model = std::make_shared<ModelInfo>(
      std::move(_name), std::move(_worldID),
      std::move(_rootLinkToModelTf),
      std::move(_baseInertialToLinkFrame),
      std::move(_body));

    model->isNestedModel = true;
    this->models[id] = model;
    const auto parentModel = this->models.at(_parentID);
    parentModel->nestedModelEntityIds.push_back(id);
    parentModel->nestedModelNameToEntityId[model->name] = id;
    return this->GenerateIdentity(id, model);
  }

  public: inline Identity AddLink(LinkInfo _linkInfo)
  {
    const auto id = this->GetNextEntity();
    auto link = std::make_shared<LinkInfo>(std::move(_linkInfo));
    this->links[id] = link;

    auto *model = this->ReferenceInterface<ModelInfo>(_linkInfo.model);
    model->linkNameToEntityId[link->name] = id;
    if (!link->indexInModel.has_value())
    {
      // We are adding the root link. This means the model should not already
      // have a root link
      // This check makes `ConstructEmptyLink` to fail
      // assert(model->linkEntityIds.empty());
    }
    model->linkEntityIds.push_back(id);

    return this->GenerateIdentity(id, link);
  }

  public: inline Identity AddCollision(CollisionInfo _collisionInfo)
  {
   const auto id = this->GetNextEntity();
   auto collision = std::make_shared<CollisionInfo>(std::move(_collisionInfo));
   this->collisions[id] = collision;
   auto *link = this->ReferenceInterface<LinkInfo>(_collisionInfo.link);
   collision->indexInLink = static_cast<int>(link->collisionEntityIds.size());
   link->collisionEntityIds.push_back(id);
   link->collisionNameToEntityId[collision->name] = id;

   return this->GenerateIdentity(id, collision);
  }

  public: inline Identity AddJoint(JointInfo _jointInfo)
  {
    const auto id = this->GetNextEntity();
    auto joint = std::make_shared<JointInfo>(std::move(_jointInfo));
    this->joints[id] = joint;

    auto *model = this->ReferenceInterface<ModelInfo>(joint->model);
    joint->indexInGzModel = model->jointEntityIds.size();
    model->jointEntityIds.push_back(id);
    model->jointNameToEntityId[joint->name] = id;

    return this->GenerateIdentity(id, joint);
  }

  public: inline Identity addConstraint(JointInfo _jointInfo)
  {
    const auto id = this->GetNextEntity();
    auto joint = std::make_shared<JointInfo>(std::move(_jointInfo));
    this->joints[id] = joint;

    return this->GenerateIdentity(id, joint);
  }

  public: bool RemoveModelImpl(const Identity &_parentID,
                               const Identity &_modelID)
  {
    auto *model = this->ReferenceInterface<ModelInfo>(_modelID);
    if (!model)
      return false;

    // Remove nested models
    for (auto &nestedModelID : model->nestedModelEntityIds)
    {
      this->RemoveModelImpl(_modelID, this->GenerateIdentity(nestedModelID,
                            this->models.at(nestedModelID)));
    }
    model->nestedModelEntityIds.clear();

    // remove references in parent model or world model
    auto parentModelIt = this->models.find(_parentID);
    if (parentModelIt != this->models.end())
    {
      auto parentModel = parentModelIt->second;
      auto nestedModelIt =
          parentModel->nestedModelNameToEntityId.find(model->name);
      if (nestedModelIt !=
          parentModel->nestedModelNameToEntityId.end())
      {
        std::size_t nestedModelID = nestedModelIt->second;
        parentModel->nestedModelNameToEntityId.erase(nestedModelIt);
        parentModel->nestedModelEntityIds.erase(std::remove(
            parentModel->nestedModelEntityIds.begin(),
            parentModel->nestedModelEntityIds.end(), nestedModelID),
            parentModel->nestedModelEntityIds.end());
      }
    }

    // If nested, no need to remove multibody
    // \todo(iche033) Remove links and joints in nested model
    bool isNested =  this->worlds.find(_parentID) == this->worlds.end();
    if (isNested)
    {
      return true;
    }

    // Remove model from world
    auto *world = this->ReferenceInterface<WorldInfo>(model->world);
    if (!world)
      return false;
    if (world->modelIndexToEntityId.erase(model->indexInWorld) == 0)
    {
      // The model has already been removed at some point
      return false;
    }
    world->modelNameToEntityId.erase(model->name);

    // Remove all constraints related to this model
    for (const auto jointID : model->jointEntityIds)
    {
      const auto joint = this->joints.at(jointID);
      if (joint->motor)
      {
        world->world->removeMultiBodyConstraint(joint->motor.get());
      }
      if (joint->fixedConstraint)
      {
        world->world->removeMultiBodyConstraint(joint->fixedConstraint.get());
      }
      if (joint->jointLimits)
      {
        world->world->removeMultiBodyConstraint(joint->jointLimits.get());
      }
      this->joints.erase(jointID);
    }
    // \todo(iche033) Remove external constraints related to this model
    // (model->external_constraints) once this is supported

    world->world->removeMultiBody(model->body.get());
    for (const auto linkID : model->linkEntityIds)
    {
      const auto &link = this->links.at(linkID);
      if (link->collider)
      {
        world->world->removeCollisionObject(link->collider.get());
        for (const auto shapeID : link->collisionEntityIds)
          this->collisions.erase(shapeID);
      }

      this->links.erase(linkID);
    }

    this->models.erase(_modelID);

    return true;
  }

  public: ~Base() override {
    // The order of destruction between meshesGImpact and triangleMeshes is
    // important.
    this->meshesGImpact.clear();
    this->triangleMeshes.clear();
    this->meshesConvex.clear();

    this->joints.clear();

    for (const auto &[k, link] : links)
    {
        auto *model = this->ReferenceInterface<ModelInfo>(link->model);
        auto *world = this->ReferenceInterface<WorldInfo>(model->world);
        if (link->collider != nullptr)
        {
          world->world->removeCollisionObject(link->collider.get());
        }
    }

    this->collisions.clear();
    this->links.clear();
    this->models.clear();
    this->worlds.clear();
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

  public: std::vector<std::unique_ptr<btTriangleMesh>> triangleMeshes;
  public: std::vector<std::unique_ptr<btGImpactMeshShape>> meshesGImpact;
  public: std::vector<std::unique_ptr<btConvexHullShape>> meshesConvex;
};

}  // namespace bullet_featherstone
}  // namespace physics
}  // namespace gz

#endif
