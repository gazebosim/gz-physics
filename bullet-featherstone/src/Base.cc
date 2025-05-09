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

#include <utility>

namespace gz {
namespace physics {
namespace bullet_featherstone {

/////////////////////////////////////////////////
GzCollisionDispatcher::GzCollisionDispatcher(
    btCollisionConfiguration *_collisionConfiguration)
    : btCollisionDispatcher(_collisionConfiguration)
{
}

/////////////////////////////////////////////////
GzCollisionDispatcher::~GzCollisionDispatcher()
{
  for (auto& manifold : this->customManifolds)
  {
    btCollisionDispatcher::releaseManifold(manifold);
  }

  this->customManifolds.clear();
  this->colPairManifolds.clear();
}

/////////////////////////////////////////////////
void GzCollisionDispatcher::RemoveManifoldByCollisionObject(
    btCollisionObject *_colObj)
{
  std::unordered_set<btPersistentManifold *> manifoldsToRemove;
  for (const auto& manifold : this->customManifolds)
  {
    if (manifold->getBody0() == _colObj ||
        manifold->getBody1() == _colObj)
    {
      manifoldsToRemove.insert(manifold);
    }
  }

  for (auto& manifold : manifoldsToRemove)
  {
    btCollisionDispatcher::releaseManifold(manifold);
    this->customManifolds.erase(manifold);
  }
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
    const btCompoundShape *_compoundShape, int _childIndex)
{
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
      //            -> comopundShape -> convexShape0
      //                             -> convexShape1
      // A _childIndex of 1 is ambiguous as it could refer to either
      // boxShape1 or convexShape1
      // return nullptr in this case to indicate ambiguity
      if (childCount > 1)
      {
        for (int i = 0; i < childCount; ++i)
        {
          const btCollisionShape *shape = _compoundShape->getChildShape(i);
          if (this->HasConvexHullChildShapes(shape))
            return nullptr;
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
void GzCollisionDispatcher::dispatchAllCollisionPairs(
    btOverlappingPairCache* pairCache,
    const btDispatcherInfo& dispatchInfo,
    btDispatcher* dispatcher)
{
  btCollisionDispatcher::dispatchAllCollisionPairs(
      pairCache, dispatchInfo, dispatcher);

  // Loop through all the contact manifolds.
  // Find convex decomposed mesh collision shapes.
  // Create a shared contact manifold for all decomposed shapes.
  // This is so that we can limit the number of contact points to 4
  // for the collision shape. Otherwise it will generate up to
  // (decomposed col count * 4) contact points.
  int numManifolds = this->getNumManifolds();
  for (int i = 0; i < numManifolds; ++i)
  {
    btPersistentManifold* contactManifold =
        this->getManifoldByIndexInternal(i);

    const btMultiBodyLinkCollider* ob0 =
        dynamic_cast<const btMultiBodyLinkCollider *>(
        contactManifold->getBody0());
    const btMultiBodyLinkCollider* ob1 =
        dynamic_cast<const btMultiBodyLinkCollider *>(
        contactManifold->getBody1());

    // if it's a custom manifold, just refresh contacts, no need to
    // loop through and check them.
    if (this->customManifolds.find(contactManifold) !=
        this->customManifolds.end())
    {
      contactManifold->refreshContactPoints(ob0->getWorldTransform(),
                                            ob1->getWorldTransform());
      continue;
    }

    if (this->manifoldsToKeep.find(contactManifold) !=
        this->manifoldsToKeep.end())
    {
      continue;
    }

    const btCompoundShape *compoundShape0 =
        static_cast<const btCompoundShape *>(ob0->getCollisionShape());
    const btCompoundShape *compoundShape1 =
        static_cast<const btCompoundShape *>(ob1->getCollisionShape());

    int numContacts = contactManifold->getNumContacts();
    for (int j = 0; j < numContacts; ++j)
    {
      btManifoldPoint& pt = contactManifold->getContactPoint(j);
      const btCollisionShape *colShape0 = this->FindCollisionShape(
          compoundShape0, pt.m_index0);
      const btCollisionShape *colShape1 = this->FindCollisionShape(
          compoundShape1, pt.m_index1);

      if (!colShape0 || !colShape1 ||
         (!this->HasConvexHullChildShapes(colShape0) &&
          !this->HasConvexHullChildShapes(colShape1)))
      {
        this->manifoldsToKeep.insert(contactManifold);
        continue;
      }

      btPersistentManifold* colManifold =
          this->colPairManifolds[colShape0][colShape1];
      if (!colManifold)
      {
        // create new custom manifold for the collision pair
        colManifold = this->getNewManifold(ob0, ob1);
        this->colPairManifolds[colShape0][colShape1] = colManifold;
        this->colPairManifolds[colShape1][colShape0] = colManifold;
        this->customManifolds.insert(colManifold);
      }
      colManifold->addManifoldPoint(pt);
      colManifold->refreshContactPoints(ob0->getWorldTransform(),
                                        ob1->getWorldTransform());
    }

    // clear manifolds that are replaced by custom ones
    if (this->manifoldsToKeep.find(contactManifold) ==
        this->manifoldsToKeep.end())
      contactManifold->clearManifold();
  }
}

/////////////////////////////////////////////////
void GzCollisionDispatcher::releaseManifold(btPersistentManifold *_manifold)
{
  auto manifoldIt = this->manifoldsToKeep.find(_manifold);
  if (manifoldIt != this->manifoldsToKeep.end())
    this->manifoldsToKeep.erase(manifoldIt);

  btCollisionDispatcher::releaseManifold(_manifold);
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
  positions[_dof] = static_cast<btScalar>(_value);

  // Call setJointPosMultiDof to ensure that the link pose cache is updated.
  this->setJointPosMultiDof(_jointIndex, positions);

  this->needsCollisionTransformsUpdate = true;
}

btScalar GzMultiBody::GetJointPosForDof(int _jointIndex, std::size_t _dof) const
{
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

void GzMultiBody::AddJointDampingTorque(int _jointIndex, double _damping)
{
  const btMultibodyLink& link = this->getLink(_jointIndex);
  for (int dof = 0; dof < link.m_dofCount; ++dof)
  {
    btScalar currVel = this->getJointVelMultiDof(_jointIndex)[dof];
    btScalar torque = -static_cast<btScalar>(_damping) * currVel;
    this->addJointTorqueMultiDof(_jointIndex, dof, torque);
  }
}

}  // namespace bullet_featherstone
}  // namespace physics
}  // namespace gz
