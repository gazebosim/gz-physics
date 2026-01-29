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

#include <gz/math/eigen3/Conversions.hh>

#include <limits>
#include <unordered_map>
#include <utility>

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

/////////////////////////////////////////////////
void SimulationFeatures::WorldForwardStep(
    const Identity &_worldID,
    ForwardStep::Output & _h,
    ForwardStep::State & /*_x*/,
    const ForwardStep::Input & _u)
{
  const auto worldInfo = this->ReferenceInterface<WorldInfo>(_worldID);
  auto *dtDur =
    _u.Query<std::chrono::steady_clock::duration>();
  double stepSize = 0.001;
  if (dtDur)
  {
    std::chrono::duration<double> dt = *dtDur;
    stepSize = dt.count();
  }

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

  // \todo(iche033) Stepping sim with varying dt may not work properly.
  // One example is the motor constraint that's created in
  // JointFeatures::SetJointVelocityCommand which assumes a fixed step
  // size.
  worldInfo->world->stepSimulation(static_cast<btScalar>(stepSize), 1,
                                   static_cast<btScalar>(stepSize));

  // Reset joint velocity target after each step to be consistent with dart's
  // joint velocity command behavior
  for (auto & joint : this->joints)
  {
    if (joint.second->motor)
    {
      joint.second->motor->setVelocityTarget(btScalar(0));
    }
  }

  this->WriteRequiredData(_h);
  this->Write(_h.Get<ChangedWorldPoses>());
}

/////////////////////////////////////////////////
std::vector<SimulationFeatures::ContactInternal>
SimulationFeatures::GetContactsFromLastStep(const Identity &_worldID) const
{
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

      // Add normal, depth and wrench to extraData.
      auto& extraContactData =
        extraData.Get<SimulationFeatures::ExtraContactData>();
      extraContactData.force =
        convert(btVector3(pt.m_appliedImpulse,
                          pt.m_appliedImpulse,
                          pt.m_appliedImpulse));
      extraContactData.normal = convert(pt.m_normalWorldOnB);
      extraContactData.depth = pt.getDistance();

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
  // remove link poses from the previous iteration
  _changedPoses.entries.clear();
  _changedPoses.entries.reserve(this->links.size());

  std::unordered_map<std::size_t, math::Pose3d> newPoses;

  for (const auto &[id, info] : this->links)
  {
    const auto &model = this->ReferenceInterface<ModelInfo>(info->model);
    WorldPose wp;
    wp.pose = gz::math::eigen3::convert(GetWorldTransformOfLink(*model, *info));
    wp.body = id;

    auto iter = this->prevLinkPoses.find(id);
    if ((iter == this->prevLinkPoses.end()) ||
        !iter->second.Pos().Equal(wp.pose.Pos(), 1e-6) ||
        !iter->second.Rot().Equal(wp.pose.Rot(), 1e-6))
    {
      _changedPoses.entries.push_back(wp);
      newPoses[id] = wp.pose;
    }
    else
      newPoses[id] = iter->second;
  }

  // Save the new poses so that they can be used to check for updates in the
  // next iteration. Re-setting this->prevLinkPoses with the contents of
  // newPoses ensures that we aren't caching data for links that were removed
  this->prevLinkPoses = std::move(newPoses);
}
}  // namespace bullet_featherstone
}  // namespace physics
}  // namespace gz
