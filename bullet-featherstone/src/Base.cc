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

#include <BulletCollision/CollisionDispatch/btManifoldResult.h>
#include <LinearMath/btVector3.h>
#include <LinearMath/btQuaternion.h>

#include <gz/common/Profiler.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>

#include <utility>

namespace
{

bool doCollide(uint32_t _categoryBitmask0, uint32_t _collideBitmask0,
               uint32_t _categoryBitmask1, uint32_t _collideBitmask1)
{
  return (_categoryBitmask0 & _collideBitmask1) |
         (_categoryBitmask1 & _collideBitmask0);
}

/////////////////////////////////////////////////
// Bullet combines contact stiffness using the harmonic mean, but by default
// sums contact damping (d0 + d1), causing the larger damping to dominate.
// This callback uses the configured object's damping directly when only one
// object specifies <kp>/<kd>, and combines damping via the harmonic mean
// (1 / (1/d0 + 1/d1)) when both objects specify it. This allows compliant
// objects to compress naturally when contacting stiff surfaces.
btScalar calculateCombinedContactDamping(
    const btCollisionObject *_body0, const btCollisionObject *_body1)
{
  const bool has0 = (_body0->getCollisionFlags() &
      btCollisionObject::CF_HAS_CONTACT_STIFFNESS_DAMPING) != 0;
  const bool has1 = (_body1->getCollisionFlags() &
      btCollisionObject::CF_HAS_CONTACT_STIFFNESS_DAMPING) != 0;
  if (has0 && !has1)
    return _body0->getContactDamping();
  if (has1 && !has0)
    return _body1->getContactDamping();
  const btScalar d0 = _body0->getContactDamping();
  const btScalar d1 = _body1->getContactDamping();
  if (d0 <= btScalar(0) || d1 <= btScalar(0))
    return btScalar(0);
  return btScalar(1) / (btScalar(1) / d0 + btScalar(1) / d1);
}

}  // namespace

namespace gz {
namespace physics {
namespace bullet_featherstone {

/////////////////////////////////////////////////
bool GzCollisionFilterCallback::needBroadphaseCollision(
    btBroadphaseProxy *_proxy0, btBroadphaseProxy *_proxy1) const
{
  GZ_PROFILE("GzCollisionFilterCallback::needBroadphaseCollision");
  GzMultiBodyLinkCollider *col0 =
          static_cast<GzMultiBodyLinkCollider *>(
          _proxy0->m_clientObject);
  GzMultiBodyLinkCollider *col1 =
          static_cast<GzMultiBodyLinkCollider *>(
          _proxy1->m_clientObject);

  if (col0 && col1)
  {
    // For backward compatibility, if category bitmask is not set, it
    // defaults to the same value as collide bitmask.
    uint32_t col0CategoryBitmask = col0->categoryBitmask.has_value() ?
        col0->categoryBitmask.value() : col0->collideBitmask;
    uint32_t col1CategoryBitmask = col1->categoryBitmask.has_value() ?
        col1->categoryBitmask.value() : col1->collideBitmask;
    // Early out if collide bitmask test fails
    if (!doCollide(col0CategoryBitmask, col0->collideBitmask,
        col1CategoryBitmask, col1->collideBitmask))
    {
      return false;
    }
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
GzCollisionDispatcher::~GzCollisionDispatcher()
{
  std::vector<btPersistentManifold *> manifolds(
      this->customManifolds.begin(), this->customManifolds.end());
  for (auto *manifold : manifolds)
  {
    this->releaseManifold(manifold);
  }
  this->customManifolds.clear();
  this->shapeManifoldMap.clear();
}

/////////////////////////////////////////////////
bool GzCollisionDispatcher::needsCollision(const btCollisionObject *_body0,
    const btCollisionObject *_body1)
{
  GZ_PROFILE("GzCollisionDispatcher::needsCollision");
  const GzMultiBodyLinkCollider *col0 =
          static_cast<const GzMultiBodyLinkCollider *>(_body0);
  const GzMultiBodyLinkCollider *col1 =
          static_cast<const GzMultiBodyLinkCollider *>(_body1);

  // Collision filtering in narrow phase.
  if (col0 && col1)
  {
    // For backward compatibility, if category bitmask is not set, it
    // defaults to the same value as collide bitmask.
    uint32_t col0CategoryBitmask = col0->categoryBitmask.has_value() ?
        col0->categoryBitmask.value() : col0->collideBitmask;
    uint32_t col1CategoryBitmask = col1->categoryBitmask.has_value() ?
        col1->categoryBitmask.value() : col1->collideBitmask;
    // Early out if collide bitmask test fails
    if (!doCollide(col0CategoryBitmask, col0->collideBitmask,
        col1CategoryBitmask, col1->collideBitmask))
    {
      return false;
    }
  }

  return btCollisionDispatcher::needsCollision(_body0, _body1);
}

/////////////////////////////////////////////////
bool GzCollisionDispatcher::HasConvexHullChildShapes(
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
const btCollisionShape *GzCollisionDispatcher::FindCollisionShape(
    const btCompoundShape *_compoundShape, int _childIndex) const
{
  GZ_PROFILE("GzCollisionDispatcher::FindCollisionShape");
  if (!_compoundShape)
    return nullptr;

  int childCount = _compoundShape->getNumChildShapes();
  if (childCount <= 0)
    return nullptr;

  const btCollisionShape *shape =
      (_childIndex >= 0 && _childIndex < childCount) ?
      _compoundShape->getChildShape(_childIndex) :
      _compoundShape->getChildShape(0);

  // When a convex decomposed mesh collision is added to a link's compound
  // shape, its decomposed btConvexHullShape children are added directly to
  // the link's compound shape and their userPointer is set to the parent
  // collision btCompoundShape.
  if (shape && shape->getUserPointer() != nullptr)
  {
    return static_cast<const btCollisionShape *>(shape->getUserPointer());
  }

  return shape;
}

/////////////////////////////////////////////////
void GzCollisionDispatcher::RemoveManifoldByCollisionObject(
    const btCollisionObject *_obj)
{
  std::vector<btPersistentManifold *> toRemove;
  for (auto *manifold : this->customManifolds)
  {
    if (manifold->getBody0() == _obj || manifold->getBody1() == _obj)
      toRemove.push_back(manifold);
  }

  for (auto *manifold : toRemove)
  {
    this->releaseManifold(manifold);
  }
}

/////////////////////////////////////////////////
void GzCollisionDispatcher::releaseManifold(btPersistentManifold *_manifold)
{
  if (this->customManifolds.erase(_manifold) > 0)
  {
    for (auto it = this->shapeManifoldMap.begin();
         it != this->shapeManifoldMap.end();)
    {
      if (it->second == _manifold)
        it = this->shapeManifoldMap.erase(it);
      else
        ++it;
    }
  }
  btCollisionDispatcher::releaseManifold(_manifold);
}

/////////////////////////////////////////////////
void GzCollisionDispatcher::dispatchAllCollisionPairs(
    btOverlappingPairCache *_pairCache,
    const btDispatcherInfo &_dispatchInfo,
    btDispatcher *_dispatcher)
{
  GZ_PROFILE("GzCollisionDispatcher::dispatchAllCollisionPairs");
  btCollisionDispatcher::dispatchAllCollisionPairs(_pairCache, _dispatchInfo,
      _dispatcher);

  // Clear custom manifolds for active collision pairs before aggregating
  // fresh contacts from the narrowphase manifolds, or refresh cached points
  // if both colliders are sleeping (e.g. if a sleeping model's pose was
  // updated before activation state is synced).
  for (auto *customManifold : this->customManifolds)
  {
    const btCollisionObject *ob0 = customManifold->getBody0();
    const btCollisionObject *ob1 = customManifold->getBody1();
    if (!ob0 || !ob1 || ob0->isActive() || ob1->isActive())
    {
      customManifold->clearManifold();
    }
    else
    {
      customManifold->refreshContactPoints(
          ob0->getWorldTransform(), ob1->getWorldTransform());
    }
  }

  int numManifolds = this->getNumManifolds();
  for (int i = 0; i < numManifolds; ++i)
  {
    btPersistentManifold *contactManifold =
        this->getManifoldByIndexInternal(i);

    if (this->customManifolds.find(contactManifold) !=
        this->customManifolds.end())
    {
      continue;
    }

    int numContacts = contactManifold->getNumContacts();
    if (numContacts <= 0)
      continue;

    const btMultiBodyLinkCollider *ob0 =
        btMultiBodyLinkCollider::upcast(contactManifold->getBody0());
    const btMultiBodyLinkCollider *ob1 =
        btMultiBodyLinkCollider::upcast(contactManifold->getBody1());
    if (!ob0 || !ob1)
      continue;

    const btCollisionShape *linkShape0 = ob0->getCollisionShape();
    const btCollisionShape *linkShape1 = ob1->getCollisionShape();
    if (!linkShape0 || !linkShape1 ||
        !linkShape0->isCompound() || !linkShape1->isCompound())
    {
      continue;
    }

    const btCompoundShape *compoundShape0 =
        static_cast<const btCompoundShape *>(linkShape0);
    const btCompoundShape *compoundShape1 =
        static_cast<const btCompoundShape *>(linkShape1);

    btManifoldPoint &pt0 = contactManifold->getContactPoint(0);
    const btCollisionShape *colShape0 =
        this->FindCollisionShape(compoundShape0, pt0.m_index0);
    const btCollisionShape *colShape1 =
        this->FindCollisionShape(compoundShape1, pt0.m_index1);
    if (!colShape0 || !colShape1)
      continue;

    // Only merge manifolds when at least one collision shape is a convex
    // decomposed mesh.
    if (!HasConvexHullChildShapes(colShape0) &&
        !HasConvexHullChildShapes(colShape1))
    {
      continue;
    }

    btPersistentManifold *colManifold = nullptr;
    auto colManifoldIt = this->shapeManifoldMap.find(
        std::make_pair(colShape0, colShape1));
    if (colManifoldIt == this->shapeManifoldMap.end())
    {
      colManifoldIt = this->shapeManifoldMap.find(
          std::make_pair(colShape1, colShape0));
    }
    if (colManifoldIt != this->shapeManifoldMap.end())
    {
      colManifold = colManifoldIt->second;
    }
    else
    {
      colManifold = this->getNewManifold(ob0, ob1);
      this->shapeManifoldMap[std::make_pair(colShape0, colShape1)] =
          colManifold;
      this->customManifolds.insert(colManifold);
    }

    if (contactManifold->getContactBreakingThreshold() >
        colManifold->getContactBreakingThreshold())
    {
      colManifold->setContactBreakingThreshold(
          contactManifold->getContactBreakingThreshold());
    }

    // Add contacts from this convex hull manifold into the shared collision
    // manifold, which automatically reduces contacts to at most 4 points.
    for (int j = 0; j < numContacts; ++j)
    {
      btManifoldPoint ptToAdd = contactManifold->getContactPoint(j);
      if (colManifold->getBody0() != ob0)
      {
        std::swap(ptToAdd.m_localPointA, ptToAdd.m_localPointB);
        std::swap(ptToAdd.m_positionWorldOnA, ptToAdd.m_positionWorldOnB);
        ptToAdd.m_normalWorldOnB = -ptToAdd.m_normalWorldOnB;
        std::swap(ptToAdd.m_index0, ptToAdd.m_index1);
        std::swap(ptToAdd.m_partId0, ptToAdd.m_partId1);
      }

      bool duplicate = false;
      for (int k = 0; k < colManifold->getNumContacts(); ++k)
      {
        btManifoldPoint &existingPt = colManifold->getContactPoint(k);
        if ((existingPt.m_localPointA - ptToAdd.m_localPointA).length2() <
            btScalar(1e-6) &&
            (existingPt.m_localPointB - ptToAdd.m_localPointB).length2() <
            btScalar(1e-6))
        {
          if (ptToAdd.getDistance() < existingPt.getDistance())
          {
            colManifold->replaceContactPoint(ptToAdd, k);
          }
          duplicate = true;
          break;
        }
      }

      if (!duplicate)
      {
        colManifold->addManifoldPoint(ptToAdd);
      }
    }
    contactManifold->clearManifold();
  }
}

/////////////////////////////////////////////////
WorldInfo::WorldInfo(std::string name_)
  : name(std::move(name_))
{
  GZ_PROFILE("WorldInfo::WorldInfo");
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
  gCalculateCombinedContactDampingCallback = &calculateCombinedContactDamping;

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
  GZ_PROFILE("GzMultiBody::SetJointPosForDof");
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
  GZ_PROFILE("GzMultiBody::GetJointPosForDof");
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
  GZ_PROFILE("GzMultiBody::SetBaseWorldTransform");
  this->setBaseWorldTransform(_pose);
  this->needsCollisionTransformsUpdate = true;
}

void GzMultiBody::UpdateCollisionTransformsIfNeeded()
{
  GZ_PROFILE("GzMultiBody::UpdateCollisionTransformsIfNeeded");
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
  GZ_PROFILE("GzMultiBody::AddJointDampingStiffnessTorque");
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

void GzMultiBody::SetBaseLinkFrameVelocity(const Eigen::Vector3d &_linVel,
    const Eigen::Vector3d &_angVel)
{
  this->baseLinkFrameLinVel = _linVel;
  this->baseLinkFrameAngVel = _angVel;
}

const Eigen::Vector3d &GzMultiBody::GetBaseLinkFrameLinearVelocity() const
{
  return this->baseLinkFrameLinVel;
}

const Eigen::Vector3d &GzMultiBody::GetBaseLinkFrameAngularVelocity() const
{
  return this->baseLinkFrameAngVel;
}

}  // namespace bullet_featherstone
}  // namespace physics
}  // namespace gz
