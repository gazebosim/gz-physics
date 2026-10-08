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

#include "SimulationFeatures.hh"

#include <LinearMath/btTransformUtil.h>

#include <gz/math/eigen3/Conversions.hh>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <unordered_map>
#include <utility>
#include <vector>

#include <gz/common/Profiler.hh>

namespace gz {
namespace physics {
namespace bullet_featherstone {

/////////////////////////////////////////////////
bool hasConvexHullChildShapes(
    const btCollisionShape *_shape)
{
  if (!_shape || !_shape->isCompound())
    return false;

  const btCompoundShape *compoundShape =
      static_cast<const btCompoundShape *>(_shape);
  return (compoundShape->getNumChildShapes() > 0 &&
      compoundShape->getChildShape(0)->getShapeType() ==
      CONVEX_HULL_SHAPE_PROXYTYPE);
}

/////////////////////////////////////////////////
const btCollisionShape *findCollisionShape(
    const btCompoundShape *_compoundShape, int _childIndex)
{
  GZ_PROFILE("bullet_featherstone::findCollisionShape");
  // _childIndex should give us the index of the child shape within
  // _compoundShape which represents the collision.
  // One exception is when the collision is a convex decomposed mesh.
  // In this case, the child shape is another btCompoundShape (nested), and
  // _childIndex is the index of one of the decomposed convex hulls
  // in the nested compound shape. The nested compound shape is the collision.
  int childCount = _compoundShape->getNumChildShapes();
  if (childCount > 0)
  {
    if (_childIndex >= 0 && _childIndex < childCount)
    {
      // todo(iche033) We do not have sufficient info to determine which
      // child shape is the collision if the link has convex decomposed mesh
      // collisions alongside of other collisions. See following example:
      // parentLink -> boxShape0
      //            -> boxShape1
      //            -> compoundShape -> convexShape0
      //                             -> convexShape1
      // A _childIndex of 1 is ambiguous as it could refer to either
      // boxShape1 or convexShape1
      // return nullptr in this case to indicate ambiguity
      if (childCount > 1)
      {
        for (int i = 0; i < childCount; ++i)
        {
          const btCollisionShape *shape = _compoundShape->getChildShape(i);
          if (hasConvexHullChildShapes(shape))
          {
            static bool informed{false};
            if (!informed)
            {
              gzwarn << "Unable to determine the collision id for a link with "
                     << "both simple primitive and convex shape collisions."
                     << std::endl;
              informed = true;
            }
            return nullptr;
          }
        }
      }

      const btCollisionShape *shape =
          _compoundShape->getChildShape(_childIndex);
      return shape;
    }
    else
    {
      return _compoundShape->getChildShape(0);
    }
  }
  return nullptr;
}

/////////////////////////////////////////////////
void enforceFixedConstraint(
    btMultiBodyFixedConstraint *_fixedConstraint)
{
  GZ_PROFILE("bullet_featherstone::enforceFixedConstraint");
  // Update fixed constraint's child link pose to maintain a fixed transform
  // from the parent link.
  GzMultiBody *parent =
      dynamic_cast<GzMultiBody*> (_fixedConstraint->getMultiBodyA());
  if (parent == nullptr)
  {
    std::cerr << "Internal error: Failed to cast parent btMultiBody to "
                 "GzMultiBody!" << std::endl;
    return;
  }

  GzMultiBody *child =
      dynamic_cast<GzMultiBody*> (_fixedConstraint->getMultiBodyB());
  if (child == nullptr)
  {
    std::cerr << "Internal error: Failed to cast child btMultiBody to "
                  "GzMultiBody!" << std::endl;
    return;
  }

  btTransform parentToChildTf;
  parentToChildTf.setOrigin(_fixedConstraint->getPivotInA());
  parentToChildTf.setBasis(_fixedConstraint->getFrameInA());

  int parentLinkIndex = _fixedConstraint->getLinkA();
  int childLinkIndex = _fixedConstraint->getLinkB();

  btTransform parentLinkTf;
  btTransform childLinkTf;
  if (parentLinkIndex == -1)
  {
    parentLinkTf = parent->getBaseWorldTransform();
  }
  else
  {
    btMultiBodyLinkCollider *collider =
        parent->getLinkCollider(parentLinkIndex);
    parentLinkTf = collider->getWorldTransform();
  }
  if (childLinkIndex == -1)
  {
    childLinkTf = child->getBaseWorldTransform();
  }
  else
  {
    btMultiBodyLinkCollider *collider =
        child->getLinkCollider(childLinkIndex);
    childLinkTf = collider->getWorldTransform();
  }

  btTransform expectedChildLinkTf = parentLinkTf * parentToChildTf;
  btTransform childBaseTf =  child->getBaseWorldTransform();
  btTransform childBaseToLink =
      childBaseTf.inverse() * childLinkTf;
  btTransform newChildBaseTf =
      expectedChildLinkTf * childBaseToLink.inverse();
  child->SetBaseWorldTransform(newChildBaseTf);
}

void clearCollisionCache(btMultiBodyDynamicsWorld *_world)
{
  GZ_PROFILE("bullet_featherstone::clearCollisionCache");
  btDispatcher* dispatcher = _world->getDispatcher();
  btOverlappingPairCache* pairCache =
      _world->getBroadphase()->getOverlappingPairCache();
  btBroadphasePairArray &pairArray = pairCache->getOverlappingPairArray();
  // Iterate backwards to safely handle removals from the array
  for (int i = pairArray.size() - 1; i >= 0; --i)
  {
    pairCache->cleanOverlappingPair(pairArray[i], dispatcher);
  }
}

#if BT_BULLET_VERSION >= 307
/// \brief Joint velocity of a kinematic child joint that needs to be
/// restored after a step.
struct KinematicJointVel
{
  /// \brief Multibody the joint belongs to.
  GzMultiBody *body;

  /// \brief Index of the joint's child link in the multibody.
  int indexInBtModel;

  /// \brief Number of dofs of the joint.
  int dofCount;

  /// \brief Velocity of the first dof. All other dofs are held at zero
  /// velocity.
  btScalar vel;
};

/////////////////////////////////////////////////
/// \brief Integrate and lock / drive a joint whose child link is kinematic.
/// Bullet does not integrate the position of kinematic links and does not
/// lock the joint of a kinematic link that has a dynamic ancestor, so the
/// joint position is integrated here from the joint velocity (or one-step
/// velocity command) and a motor is used to lock or drive the joint when the
/// link has a dynamic ancestor. Must be called before stepping the world.
/// \param[in] _world Bullet world the multibody belongs to.
/// \param[in] _joint Joint to update.
/// \param[in] _body Multibody the joint belongs to.
/// \param[in] _stepSize Step size in seconds.
/// \return Joint velocity to restore after the step with
/// restoreKinematicJointVelocities, or nullopt if the joint's child link is
/// not kinematic.
std::optional<KinematicJointVel> updateKinematicJoint(
    btMultiBodyDynamicsWorld *_world, JointInfo *_joint, GzMultiBody *_body,
    double _stepSize)
{
  GZ_PROFILE("bullet_featherstone::updateKinematicJoint");
  const auto *identifier = std::get_if<InternalJoint>(&_joint->identifier);
  if (!identifier)
    return std::nullopt;

  const int idx = identifier->indexInBtModel;
  const int dofCount = _body->getLink(idx).m_dofCount;
  if (!_body->isLinkKinematic(idx) || dofCount == 0)
    return std::nullopt;

  double targetVel = _joint->kinematicJointVelCmd.value_or(
      _joint->kinematicJointVel);
  _joint->kinematicJointVelCmd = std::nullopt;

  if (dofCount == 1)
  {
    // Bullet does not integrate the position of kinematic links, so
    // integrate the joint position here. The joint stops at its position
    // limits instead of moving further into them.
    if (std::abs(targetVel) > 0.0 && _stepSize > 0.0)
    {
      const double curPos = _body->GetJointPosForDof(idx, 0);
      double newPos = curPos + targetVel * _stepSize;
      if (targetVel > 0.0 && newPos > _joint->axisUpper)
      {
        newPos = std::max(curPos, _joint->axisUpper);
        targetVel = (newPos - curPos) / _stepSize;
      }
      else if (targetVel < 0.0 && newPos < _joint->axisLower)
      {
        newPos = std::min(curPos, _joint->axisLower);
        targetVel = (newPos - curPos) / _stepSize;
      }
      if (std::abs(targetVel) > 0.0)
      {
        _body->SetJointPosForDof(idx, 0, static_cast<btScalar>(newPos));
        _body->wakeUp();
      }
    }

    if (!_body->isLinkAndAllAncestorsKinematic(idx))
    {
      // Bullet does not lock the joint of a kinematic link that has a
      // dynamic ancestor, so use a motor to lock or drive the joint.
      if (!_joint->kinematicMotor)
      {
        _joint->kinematicMotor = std::make_shared<btMultiBodyJointMotor>(
            _body, idx, 0, static_cast<btScalar>(targetVel),
            static_cast<btScalar>(1e9));
        _world->addMultiBodyConstraint(_joint->kinematicMotor.get());
      }
      _joint->kinematicMotor->setVelocityTarget(
          static_cast<btScalar>(targetVel));
    }

    _body->getJointVelMultiDof(idx)[0] = static_cast<btScalar>(targetVel);
  }
  else
  {
    // \todo(iche033) Driving multi-dof kinematic child joints (e.g. ball
    // joints) is not supported, which matches the dof support of
    // JointFeatures::SetJointVelocityCommand. Hold the joint at its
    // current position instead.
    targetVel = 0.0;
    for (int d = 0; d < dofCount; ++d)
    {
      _body->getJointVelMultiDof(idx)[d] = btScalar(0);
    }
  }
  return KinematicJointVel{_body, idx, dofCount,
      static_cast<btScalar>(targetVel)};
}

/////////////////////////////////////////////////
/// \brief Pin the joint velocities of kinematic child joints to their
/// commanded values after a step. Bullet does not reset the joint velocities
/// of kinematic links and the motor used to lock a kinematic link under a
/// dynamic ancestor leaves small residuals.
/// \param[in] _jointVels Joint velocities returned by updateKinematicJoint.
void restoreKinematicJointVelocities(
    const std::vector<KinematicJointVel> &_jointVels)
{
  for (const auto &entry : _jointVels)
  {
    btScalar *jointVel = entry.body->getJointVelMultiDof(entry.indexInBtModel);
    jointVel[0] = entry.vel;
    for (int d = 1; d < entry.dofCount; ++d)
    {
      jointVel[d] = btScalar(0);
    }
  }
}
#endif

/////////////////////////////////////////////////
void SimulationFeatures::WorldForwardStep(
    const Identity &_worldID,
    ForwardStep::Output & _h,
    ForwardStep::State & /*_x*/,
    const ForwardStep::Input & _u)
{
  GZ_PROFILE("SimulationFeatures::WorldForwardStep");
  const auto worldInfo = this->ReferenceInterface<WorldInfo>(_worldID);
  auto *dtDur =
    _u.Query<std::chrono::steady_clock::duration>();
  double stepSize = 0.001;
  if (dtDur)
  {
    std::chrono::duration<double> dt = *dtDur;
    stepSize = dt.count();
  }

#if BT_BULLET_VERSION >= 307
  struct KinematicBaseVel
  {
    GzMultiBody *body;
    btVector3 linVel;
    btVector3 angVel;
  };
  std::vector<KinematicBaseVel> kinematicBaseVels;

  // Integrate base transform for kinematic root links that have velocity.
  for (int i = 0; i < worldInfo->world->getNumMultibodies(); ++i)
  {
    // All multibodies are created as GzMultiBody in SDFFeatures
    auto *body = static_cast<GzMultiBody *>(worldInfo->world->getMultiBody(i));
    if (!body || !body->isBaseKinematic())
      continue;

    const btVector3 linVel = body->getBaseVel();
    const btVector3 angVel = body->getBaseOmega();
    if (!linVel.isZero() || !angVel.isZero())
    {
      btTransform predictedTrans;
      btTransformUtil::integrateTransform(
          body->getBaseWorldTransform(),
          linVel, angVel, static_cast<btScalar>(stepSize),
          predictedTrans);
      body->SetBaseWorldTransform(predictedTrans);
      kinematicBaseVels.push_back({body, linVel, angVel});
    }
  }

  // Integrate and lock/control joints whose child link is kinematic.
  std::vector<KinematicJointVel> kinematicJointVels;
  for (auto & joint : this->joints)
  {
    const auto *model =
        this->ReferenceInterface<ModelInfo>(joint.second->model);
    if (!model || !model->body ||
        std::size_t(model->world) != std::size_t(_worldID))
    {
      continue;
    }
    const auto jointVel = updateKinematicJoint(worldInfo->world.get(),
        joint.second.get(), model->body.get(), stepSize);
    if (jointVel.has_value())
      kinematicJointVels.push_back(*jointVel);
  }
#endif

  // Update fixed constraint behavior to weld child to parent.
  // Do this before stepping, i.e. before physics engine tries to solve and
  // enforce the constraint
  for (auto & joint : this->joints)
  {
    if (joint.second->fixedConstraint &&
        joint.second->fixedConstraintWeldChildToParent)
    {
      enforceFixedConstraint(joint.second->fixedConstraint.get());
    }
  }

  // Bullet updates collision transforms *after* forward integration. But in
  // some case (e.g. if joint positions were updated), collision transforms may
  // need to be manually updated before stepping the Bullet simulation.
  for (auto & model : this->models)
  {
    if (model.second->body)
    {
      model.second->body->UpdateCollisionTransformsIfNeeded();
    }
  }

  // Add joint damping and spring stiffness torque.
  // TODO(https://github.com/bulletphysics/bullet3/issues/4709) Remove this
  // once upstream Bullet supports internal joint damping and spring stiffness.
  // e.g. set `model->body->getLink(i).m_jointDamping` directly in
  // SDFFeatures.cc. Note: there is currently no `m_jointSpringStiffness`
  // property.
  for (auto & joint : this->joints)
  {
    const auto *model =
        this->ReferenceInterface<ModelInfo>(joint.second->model);
    const auto *identifier =
        std::get_if<InternalJoint>(&joint.second->identifier);
    if (model != nullptr && model->body != nullptr && identifier != nullptr)
    {
      model->body->AddJointDampingStiffnessTorque(identifier->indexInBtModel,
          joint.second->damping, joint.second->springStiffness,
          joint.second->springReference);
    }
  }

  // Regenerate the cache if the collision masks have been updated.
  if (worldInfo->collisionMasksDirty)
  {
    clearCollisionCache(worldInfo->world.get());
  }

  // \todo(iche033) Stepping sim with varying dt may not work properly.
  // One example is the motor constraint that's created in
  // JointFeatures::SetJointVelocityCommand which assumes a fixed step
  // size.
  worldInfo->world->stepSimulation(static_cast<btScalar>(stepSize), 1,
                                   static_cast<btScalar>(stepSize));

#if BT_BULLET_VERSION >= 307
  // In Featherstone's spatial algebra, setting the base spatial acceleration
  // to zero (for a kinematic base) yields a classical world-frame linear
  // acceleration of (omega x v), which Bullet adds to m_realBuf[3..5] each
  // step in computeAccelerationsArticulatedBodyAlgorithmMultiDof. Restore the
  // world-frame base velocities so they remain constant in the world frame.
  for (const auto &entry : kinematicBaseVels)
  {
    entry.body->setBaseVel(entry.linVel);
    entry.body->setBaseOmega(entry.angVel);
  }
  // Pin the joint velocities of kinematic child joints to the commanded
  // values.
  restoreKinematicJointVelocities(kinematicJointVels);
#endif

  // Reset joint velocity target after each step to be consistent with dart's
  // joint velocity command behavior
  for (auto & joint : this->joints)
  {
    if (joint.second->motor)
    {
      joint.second->motor->setVelocityTarget(btScalar(0));
    }
  }

  if (worldInfo->collisionMasksDirty)
  {
    // manual sync to get up-to-date contacts for the current frame
    // by forcing collision detection again
    worldInfo->world->getCollisionWorld()->performDiscreteCollisionDetection();
    worldInfo->collisionMasksDirty = false;
  }

  this->WriteRequiredData(_h);
  this->Write(_h.Get<ChangedWorldPoses>());
}

/////////////////////////////////////////////////
std::vector<SimulationFeatures::ContactInternal>
SimulationFeatures::GetContactsFromLastStep(const Identity &_worldID) const
{
  GZ_PROFILE("SimulationFeatures::GetContactsFromLastStep");
  std::vector<SimulationFeatures::ContactInternal> outContacts;
  auto *const world = this->ReferenceInterface<WorldInfo>(_worldID);
  if (!world)
  {
    return outContacts;
  }

  int numManifolds = world->world->getDispatcher()->getNumManifolds();
  for (int i = 0; i < numManifolds; i++)
  {
    btPersistentManifold* contactManifold =
      world->world->getDispatcher()->getManifoldByIndexInternal(i);
    const btMultiBodyLinkCollider* ob0 =
      dynamic_cast<const btMultiBodyLinkCollider*>(contactManifold->getBody0());
    const btMultiBodyLinkCollider* ob1 =
      dynamic_cast<const btMultiBodyLinkCollider*>(contactManifold->getBody1());

    if (!ob0 || !ob1)
      continue;

    const btCollisionShape *linkShape0 = ob0->getCollisionShape();
    const btCollisionShape *linkShape1 = ob1->getCollisionShape();

    if (!linkShape0 || !linkShape1 ||
        !linkShape0->isCompound() || !linkShape1->isCompound())
      continue;

    const btCompoundShape *compoundShape0 =
        static_cast<const btCompoundShape *>(linkShape0);
    const btCompoundShape *compoundShape1 =
        static_cast<const btCompoundShape *>(linkShape1);

    int numContacts = contactManifold->getNumContacts();
    for (int j = 0; j < numContacts; j++)
    {
      btManifoldPoint& pt = contactManifold->getContactPoint(j);

      const btCollisionShape *colShape0 = findCollisionShape(
          compoundShape0, pt.m_index0);
      const btCollisionShape *colShape1 = findCollisionShape(
          compoundShape1, pt.m_index1);

      std::size_t collision0ID = std::numeric_limits<std::size_t>::max();
      std::size_t collision1ID = std::numeric_limits<std::size_t>::max();
      if (colShape0)
        collision0ID = colShape0->getUserIndex();
      else if (compoundShape0->getNumChildShapes() > 0)
        collision0ID = compoundShape0->getChildShape(0)->getUserIndex();
      if (colShape1)
        collision1ID = colShape1->getUserIndex();
      else if (compoundShape1->getNumChildShapes() > 0)
        collision1ID = compoundShape1->getChildShape(0)->getUserIndex();

      CompositeData extraData;

      // Add normal, depth and force to extraData.
      auto& extraContactData =
        extraData.Get<SimulationFeatures::ExtraContactData>();

      const Eigen::Vector3d normal = convert(pt.m_normalWorldOnB);
      extraContactData.force =
          normal * (pt.m_appliedImpulse / world->stepSize);
      extraContactData.normal = normal;
      extraContactData.depth = -pt.getDistance();

      outContacts.push_back(SimulationFeatures::ContactInternal {
        this->GenerateIdentity(collision0ID, this->collisions.at(collision0ID)),
        this->GenerateIdentity(collision1ID, this->collisions.at(collision1ID)),
        convert(pt.getPositionWorldOnA()), extraData});
      }
  }
  return outContacts;
}

/////////////////////////////////////////////////
void SimulationFeatures::Write(WorldPoses &_worldPoses) const
{
  GZ_PROFILE("SimulationFeatures::Write");
  // remove link poses from the previous iteration
  _worldPoses.entries.clear();
  _worldPoses.entries.reserve(this->links.size());

  for (const auto &[id, info] : this->links)
  {
    const auto &model = this->ReferenceInterface<ModelInfo>(info->model);
    WorldPose wp;
    wp.pose = gz::math::eigen3::convert(GetWorldTransformOfLink(*model, *info));
    wp.body = id;
    _worldPoses.entries.push_back(wp);
  }
}
/////////////////////////////////////////////////
void SimulationFeatures::Write(ChangedWorldPoses &_changedPoses) const
{
  GZ_PROFILE("SimulationFeatures::Write_changed");
  // remove link poses from the previous iteration
  _changedPoses.entries.clear();
  _changedPoses.entries.reserve(this->links.size());

  for (const auto &[id, info] : this->links)
  {
    const auto &model = this->ReferenceInterface<ModelInfo>(info->model);
    WorldPose wp;
    wp.pose = gz::math::eigen3::convert(GetWorldTransformOfLink(*model, *info));
    wp.body = id;

    if (!info->prevPose.has_value() ||
        !info->prevPose->Pos().Equal(wp.pose.Pos(), 1e-6) ||
        !info->prevPose->Rot().Equal(wp.pose.Rot(), 1e-6))
    {
      _changedPoses.entries.push_back(wp);
      info->prevPose = wp.pose;
    }
  }
}
}  // namespace bullet_featherstone
}  // namespace physics
}  // namespace gz
