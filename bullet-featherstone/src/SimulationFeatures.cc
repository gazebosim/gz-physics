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
/// \brief Velocity of a kinematic base that needs to be restored after a
/// step.
struct KinematicBaseVel
{
  /// \brief Multibody with the kinematic base.
  GzMultiBody *body;

  /// \brief World linear velocity of the center of mass of the base.
  btVector3 linVel;

  /// \brief World angular velocity of the base.
  btVector3 angVel;
};

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

  /// \brief Velocity of the first dof after clamping to the position limits
  /// of the joint. All other dofs are held at zero velocity.
  btScalar vel;
};

/// \brief Max impulse of the motor that locks or drives the joint of a
/// kinematic link under a dynamic ancestor. Effectively unlimited so that the
/// joint follows the commanded velocity exactly.
constexpr btScalar kKinematicJointMotorMaxImpulse = 1e9;

/// \brief Joint motor that locks or drives the joint of a kinematic link.
/// Bullet solves a multibody constraint in the simulation island of the
/// colliders of its links, but kinematic colliders belong to no island. A
/// plain joint motor between two kinematic links, e.g. a kinematic joint
/// above a dynamic link in an otherwise kinematic chain, would never be
/// solved. This motor falls back to the island of the dynamic links of its
/// multibody, which bullet merges into a single island.
class KinematicJointMotor : public btMultiBodyJointMotor
{
  public: using btMultiBodyJointMotor::btMultiBodyJointMotor;

  // Documentation inherited
  public: int getIslandIdA() const override
  {
    const int island = btMultiBodyJointMotor::getIslandIdA();
    return island >= 0 ? island : this->DynamicIsland();
  }

  // Documentation inherited
  public: int getIslandIdB() const override
  {
    const int island = btMultiBodyJointMotor::getIslandIdB();
    return island >= 0 ? island : this->DynamicIsland();
  }

  /// \brief Island of the first dynamic collider of the multibody.
  /// \return Island tag, or -1 if the multibody has no dynamic collider.
  private: int DynamicIsland() const
  {
    const auto island = [](const btMultiBodyLinkCollider *_collider)
    {
      return (_collider && !_collider->isStaticOrKinematicObject()) ?
          _collider->getIslandTag() : -1;
    };
    int tag = island(this->m_bodyA->getBaseCollider());
    for (int i = 0; tag < 0 && i < this->m_bodyA->getNumLinks(); ++i)
      tag = island(this->m_bodyA->getLink(i).m_collider);
    return tag;
  }
};

/////////////////////////////////////////////////
/// \brief Set the velocity of the first dof of a joint and zero the velocity
/// of its other dofs.
void setKinematicJointVel(GzMultiBody *_body, int _indexInBtModel,
    int _dofCount, btScalar _vel)
{
  btScalar *jointVel = _body->getJointVelMultiDof(_indexInBtModel);
  jointVel[0] = _vel;
  for (int d = 1; d < _dofCount; ++d)
  {
    jointVel[d] = btScalar(0);
  }
}

/////////////////////////////////////////////////
/// \brief Integrate and lock / drive a joint whose child link is kinematic.
/// Bullet does not integrate the position of kinematic links and does not
/// lock the joint of a kinematic link that has a dynamic ancestor, so the
/// joint position is integrated here from the joint velocity (or one-step
/// velocity command) and the joint motor is used to lock or drive the joint
/// when the link has a dynamic ancestor. The joint stops at its position
/// limits: the velocity is reduced to what is needed to reach a limit and is
/// zero while the joint sits at a limit and is driven into it. Must be called
/// before stepping the world.
/// \param[in] _world Bullet world the multibody belongs to.
/// \param[in] _joint Joint to update.
/// \param[in] _body Multibody the joint belongs to.
/// \param[in] _stepSize Step size in seconds.
/// \return Joint velocity, clamped to the position limits, to restore after
/// the step with restoreKinematicJointVelocities, or nullopt if the joint's
/// child link is not kinematic.
std::optional<KinematicJointVel> updateKinematicJoint(
    btMultiBodyDynamicsWorld *_world, JointInfo *_joint, GzMultiBody *_body,
    double _stepSize)
{
  const auto *identifier = std::get_if<InternalJoint>(&_joint->identifier);
  if (!identifier)
    return std::nullopt;

  const int idx = identifier->indexInBtModel;
  const int dofCount = _body->getLink(idx).m_dofCount;
  // The collider flags are checked rather than LinkInfo::isKinematic: the
  // collider of a kinematic link is static while the link is welded to a
  // static link, and the joint must not be driven against that weld. The
  // recorded velocity / command is applied once the link is detached.
  if (!_body->isLinkKinematic(idx) || dofCount == 0)
    return std::nullopt;

  double targetVel = _joint->kinematicJointVelCmd.value_or(
      _joint->kinematicJointVel);
  _joint->kinematicJointVelCmd = std::nullopt;

  // The position limits of a kinematic child joint are enforced below by
  // clamping the integrated position, so disable bullet's joint limit
  // constraint while the link is kinematic. The constraint is active as soon
  // as the joint sits on a limit and would stop the joint on its own in the
  // step the joint reaches the limit, in addition to the motor in the next
  // step, which pushes a dynamic ancestor twice. SetLinkKinematic restores
  // the constraint when the link becomes dynamic.
  if (_joint->jointLimits && _joint->jointLimits->getMaxAppliedImpulse() > 0)
  {
    _joint->jointLimitsMaxImpulse =
        _joint->jointLimits->getMaxAppliedImpulse();
    _joint->jointLimits->setMaxAppliedImpulse(0);
  }

  if (dofCount == 1)
  {
    const double curPos = _body->GetJointPosForDof(idx, 0);
    // The joint velocity was set from outside (SetJointVelocity) if it differs
    // from the velocity written after the last step.
    const bool velOverridden = std::abs(
        _body->getJointVelMultiDof(idx)[0] - _joint->kinematicJointVelPinned)
        > 0;

    // Bullet does not integrate the position of kinematic links, so
    // integrate the joint position here. The joint stops at its position
    // limits instead of moving further into them. A joint that already sits
    // beyond a limit is not snapped back, but it can move back in.
    bool clamped = false;
    if ((targetVel > 0.0 && curPos >= _joint->axisUpper) ||
        (targetVel < 0.0 && curPos <= _joint->axisLower))
    {
      // The joint cannot move into a limit it sits at, whatever the step size
      targetVel = 0.0;
      clamped = true;
    }
    else if (std::abs(targetVel) > 0.0 && _stepSize > 0.0)
    {
      const double newPos = curPos + targetVel * _stepSize;
      const double clampedPos = std::clamp(newPos,
          std::min(curPos, _joint->axisLower),
          std::max(curPos, _joint->axisUpper));
      if (std::abs(clampedPos - newPos) > 0.0)
      {
        targetVel = (clampedPos - curPos) / _stepSize;
        clamped = true;
      }
      _body->SetJointPosForDof(idx, 0, static_cast<btScalar>(clampedPos));
      _body->wakeUp();
    }

    if (!_body->isLinkAndAllAncestorsKinematic(idx))
    {
      // Bullet does not lock the joint of a kinematic link that has a
      // dynamic ancestor, so use the joint motor to lock or drive the joint.
      // The joint velocity is deliberately not set here: the motor changes it
      // during the step so that the dynamic ancestor feels the reaction of a
      // change in the commanded velocity, including a joint that is stopped
      // by its limit while moving. The same motor is used by
      // JointFeatures::SetJointVelocityCommand, which only records the
      // command while the link is kinematic.
      if (!dynamic_cast<KinematicJointMotor *>(_joint->motor.get()))
      {
        // Replace a motor created by a velocity command while the link was
        // dynamic: it has the joint effort limit and no island fallback.
        if (_joint->motor)
          _world->removeMultiBodyConstraint(_joint->motor.get());
        _joint->motor = std::make_shared<KinematicJointMotor>(
            _body, idx, 0, static_cast<btScalar>(targetVel),
            kKinematicJointMotorMaxImpulse);
        _world->addMultiBodyConstraint(_joint->motor.get());
      }
      _joint->motor->setVelocityTarget(static_cast<btScalar>(targetVel));

      if (velOverridden && clamped)
      {
        // The joint velocity was set from outside to a value the joint cannot
        // follow because of its limits. Correct it before the step: the joint
        // never moved with that velocity, so the motor must not brake it and
        // push the dynamic ancestor.
        setKinematicJointVel(_body, idx, dofCount,
            static_cast<btScalar>(targetVel));
      }
    }
    else
    {
      // Fully kinematic chain: there are no dynamics, so set the joint
      // velocity directly. A motor left over from a velocity command or from
      // the time an ancestor was dynamic has no effect on a fully kinematic
      // chain, remove it.
      if (_joint->motor)
      {
        _world->removeMultiBodyConstraint(_joint->motor.get());
        _joint->motor.reset();
      }
      setKinematicJointVel(_body, idx, dofCount,
          static_cast<btScalar>(targetVel));
    }
    // restoreKinematicJointVelocities writes this velocity after the step
    _joint->kinematicJointVelPinned = static_cast<btScalar>(targetVel);
  }
  else
  {
    // \todo(iche033) Driving multi-dof kinematic child joints (e.g. ball
    // joints) is not supported, which matches the dof support of
    // JointFeatures::SetJointVelocityCommand. Hold the joint at its
    // current position instead.
    targetVel = 0.0;
    setKinematicJointVel(_body, idx, dofCount, btScalar(0));
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
    setKinematicJointVel(entry.body, entry.indexInBtModel, entry.dofCount,
        entry.vel);
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

  // Bullet keeps no link accelerations, so record the velocities to derive
  // them from the velocity change of the step. This is done before the
  // kinematic bases and joints below advance, so that the change includes the
  // motion they are driven with.
  for (int i = 0; i < worldInfo->world->getNumMultibodies(); ++i)
  {
    // All multibodies are created as GzMultiBody in SDFFeatures
    static_cast<GzMultiBody *>(worldInfo->world->getMultiBody(i))
        ->RecordVelocitiesBeforeStep();
  }

#if BT_BULLET_VERSION >= 307
  std::vector<KinematicBaseVel> kinematicBaseVels;

  // Integrate base transform for kinematic root links that have velocity.
  for (int i = 0; i < worldInfo->world->getNumMultibodies(); ++i)
  {
    // All multibodies are created as GzMultiBody in SDFFeatures
    auto *body = static_cast<GzMultiBody *>(worldInfo->world->getMultiBody(i));
    if (!body || !body->isBaseKinematic())
      continue;

    // The velocity of a kinematic base is the velocity of its root link
    // frame, stored in double precision in the multibody by the free group
    // velocity setters. Bullet stores the velocity of the center of mass of
    // the root link, which is derived from the stored velocity at every pose
    // and never fed back into it, so that a root link with an offset center
    // of mass moves at exactly the requested velocity instead of orbiting
    // its center of mass or drifting with bullet's single precision state.
    const Eigen::Vector3d &linkLinVelCmd =
        body->GetBaseLinkFrameLinearVelocity();
    const Eigen::Vector3d &angVelCmd = body->GetBaseLinkFrameAngularVelocity();
    if (linkLinVelCmd.isZero(0.0) && angVelCmd.isZero(0.0))
      continue;

    // btMultiBody user index stores the gz-physics model root link id
    const std::size_t rootID =
        static_cast<std::size_t>(body->getUserIndex());
    const auto *model = this->ReferenceInterface<ModelInfo>(
        this->links.at(rootID)->model);
    const btTransform baseInertiaToLinkBt =
        convertTf(model->baseInertiaToLinkFrame);
    const btTransform linkTf =
        body->getBaseWorldTransform() * baseInertiaToLinkBt;
    const btVector3 linkLinVel = convertVec(linkLinVelCmd);
    const btVector3 angVel = convertVec(angVelCmd);

    btTransform predictedLinkTf;
    btTransformUtil::integrateTransform(
        linkTf, linkLinVel, angVel, static_cast<btScalar>(stepSize),
        predictedLinkTf);
    const btTransform predictedBaseTf =
        predictedLinkTf * baseInertiaToLinkBt.inverse();
    const btVector3 comToLink = getBaseComToLinkFrame(*model, predictedBaseTf);
    const btVector3 baseLinVel = linkLinVel - angVel.cross(comToLink);

    body->SetBaseWorldTransform(predictedBaseTf);
    body->setBaseVel(baseLinVel);
    body->setBaseOmega(angVel);
    kinematicBaseVels.push_back({body, baseLinVel, angVel});
  }

  // Integrate and lock/control joints whose child link is kinematic.
  std::vector<KinematicJointVel> kinematicJointVels;
  {
    GZ_PROFILE("bullet_featherstone::updateKinematicJoints");
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
    // A kinematic base moving slower than bullet's sleep threshold would be
    // put to sleep after the sleep timeout even though it is moving, which
    // would freeze the dynamic links attached to it.
    entry.body->wakeUp();
  }
  // Pin the joint velocities of kinematic child joints to the commanded
  // values.
  restoreKinematicJointVelocities(kinematicJointVels);
#endif

  for (int i = 0; i < worldInfo->world->getNumMultibodies(); ++i)
  {
    static_cast<GzMultiBody *>(worldInfo->world->getMultiBody(i))
        ->UpdateAccelerations(stepSize);
  }

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
