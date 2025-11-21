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

#include "Base.hh"

#include <LinearMath/btVector3.h>
#include <LinearMath/btQuaternion.h>

#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>

#include <utility>

namespace gz {
namespace physics {
namespace bullet_featherstone {

/////////////////////////////////////////////////
bool GzCollisionFilterCallback::needBroadphaseCollision(
    btBroadphaseProxy *_proxy0, btBroadphaseProxy *_proxy1) const
{
  GzMultiBodyLinkCollider *col0 =
          static_cast<GzMultiBodyLinkCollider *>(
          _proxy0->m_clientObject);
  GzMultiBodyLinkCollider *col1 =
          static_cast<GzMultiBodyLinkCollider *>(
          _proxy1->m_clientObject);

  // Early out if collide bitmask test fails
  if ((col0 && col1) && !(col0->collideBitmask & col1->collideBitmask))
  {
    return false;
  }

  // Continue filtering collision based on logic in
  // btOverlappingPairCache::needsBroadphaseCollision
  bool collides = (_proxy0->m_collisionFilterGroup &
                   _proxy1->m_collisionFilterMask) != 0;
  collides = collides && (_proxy1->m_collisionFilterGroup &
                          _proxy0->m_collisionFilterMask);
  return collides;
}

/////////////////////////////////////////////////
GzCollisionDispatcher::GzCollisionDispatcher(
    btCollisionConfiguration *_collisionConfiguration)
    : btCollisionDispatcher(_collisionConfiguration)
{
}

/////////////////////////////////////////////////
bool GzCollisionDispatcher::needsCollision(const btCollisionObject *_body0,
    const btCollisionObject *_body1)
{
  const GzMultiBodyLinkCollider *col0 =
          static_cast<const GzMultiBodyLinkCollider *>(_body0);
  const GzMultiBodyLinkCollider *col1 =
          static_cast<const GzMultiBodyLinkCollider *>(_body1);

  // Collision filtering in narrow phase.
  // Early out if collide bitmask test fails
  if ((col0 && col1) && !(col0->collideBitmask & col1->collideBitmask))
  {
    return false;
  }
  return btCollisionDispatcher::needsCollision(_body0, _body1);
}

/////////////////////////////////////////////////
WorldInfo::WorldInfo(std::string name_)
  : name(std::move(name_))
{
  this->collisionConfiguration =
    std::make_unique<btDefaultCollisionConfiguration>();
  this->dispatcher =
    std::make_unique<GzCollisionDispatcher>(collisionConfiguration.get());
  this->broadphase = std::make_unique<btDbvtBroadphase>();
  this->solver = std::make_unique<btMultiBodyConstraintSolver>();
  this->world = std::make_unique<btMultiBodyDynamicsWorld>(
    dispatcher.get(), broadphase.get(), solver.get(),
    collisionConfiguration.get());

  // Set custom collision filter callback for filtering based on
  // surface contact parameters
  this->collisionFilterCallback = std::make_unique<GzCollisionFilterCallback>();
  btOverlappingPairCache* pairCache = this->world->getPairCache();
  pairCache->setOverlapFilterCallback(this->collisionFilterCallback.get());

  btGImpactCollisionAlgorithm::registerAlgorithm(dispatcher.get());

  // Needed for force-torque sensor
  this->world->getSolverInfo().m_jointFeedbackInJointFrame = true;
  this->world->getSolverInfo().m_jointFeedbackInWorldSpace = false;

  // By default a large impulse is applied when collisions penetrate
  // which causes unstable behavior. Bullet featherstone does not support
  // configuring split impulse and penetration threshold parameters. Instead
  // the penentration impulse depends on the erp2 parameter so set to a small
  // value (default in bullet is 0.2).
  this->world->getSolverInfo().m_erp2 = btScalar(0.02);

  // Set solver iterations to the same as the default value in SDF,
  // //world/physics/solver/bullet/iters
  // (default in bullet is 10)
  this->world->getSolverInfo().m_numIterations = 50u;
}

void GzMultiBody::SetJointPosForDof(
  int _jointIndex,
  std::size_t _dof,
  btScalar _value)
{
  btScalar *positions =
      this->getJointPosMultiDof(_jointIndex);

  // Ball joints store joint pos as a quaternion (4 floats) instead of one
  // joint angle in radians
  if (this->getLink(_jointIndex).m_jointType == btMultibodyLink::eSpherical)
  {
    math::Quaterniond quat(positions[3], positions[0], positions[1],
                           positions[2]);
    math::Vector3d axis;
    double angle;
    quat.AxisAngle(axis, angle);
    math::Vector3 posVec = axis * angle;
    posVec[_dof] = _value;
    angle = posVec.Length();
    if (math::equal(angle, 0.0, 1e-10))
    {
      positions[0] = 0;
      positions[1] = 0;
      positions[2] = 0;
      positions[3] = 1;
    }
    else
    {
      posVec /= angle;
      math::Quaterniond newQuat(posVec, angle);
      positions[0] = newQuat.X();
      positions[1] = newQuat.Y();
      positions[2] = newQuat.Z();
      positions[3] = newQuat.W();
    }
  }
  else
  {
    positions[_dof] = static_cast<btScalar>(_value);
  }
  // Call setJointPosMultiDof to ensure that the link pose cache is updated.
  this->setJointPosMultiDof(_jointIndex, positions);

  this->needsCollisionTransformsUpdate = true;
}

btScalar GzMultiBody::GetJointPosForDof(int _jointIndex, std::size_t _dof)
    const
{
  if (this->getLink(_jointIndex).m_jointType == btMultibodyLink::eSpherical)
  {
    // Ball joints store joint pos as a quaternion (4 floats) instead of one
    // joint angle in radians
    const btScalar *jointPos = this->getJointPosMultiDof(_jointIndex);
    math::Quaterniond quat(jointPos[3], jointPos[0], jointPos[1], jointPos[2]);
    math::Vector3d axis;
    double angle;
    quat.AxisAngle(axis, angle);
    return axis[_dof] * angle;
  }

  return this->getJointPosMultiDof(_jointIndex)[_dof];
}

void GzMultiBody::SetBaseWorldTransform(const btTransform &_pose)
{
  this->setBaseWorldTransform(_pose);
  this->needsCollisionTransformsUpdate = true;
}

void GzMultiBody::UpdateCollisionTransformsIfNeeded()
{
  if (this->needsCollisionTransformsUpdate)
  {
    btAlignedObjectArray<btQuaternion> scratchWorldToLocal;
    btAlignedObjectArray<btVector3> scratchLocalOrigin;
    this->updateCollisionObjectWorldTransforms(
      scratchWorldToLocal, scratchLocalOrigin);
    this->needsCollisionTransformsUpdate = false;
  }
}

void GzMultiBody::AddJointDampingStiffnessTorque(int _jointIndex,
    double _damping, double _springStiffness, double _springReference)
{
  const btMultibodyLink& link = this->getLink(_jointIndex);

  for (int dof = 0; dof < link.m_dofCount; ++dof)
  {
    btScalar currVel = this->getJointVelMultiDof(_jointIndex)[dof];

    // Apply damping
    btScalar torque = -static_cast<btScalar>(_damping) * currVel;

    // Apply spring stiffness
    torque += -_springStiffness *
        (this->GetJointPosForDof(_jointIndex, dof) - _springReference);

    this->addJointTorqueMultiDof(_jointIndex, dof, torque);
  }
}

}  // namespace bullet_featherstone
}  // namespace physics
}  // namespace gz
