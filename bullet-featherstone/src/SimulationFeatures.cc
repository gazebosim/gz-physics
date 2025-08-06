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

  // Add joint damping torque.
  // TODO(https://github.com/bulletphysics/bullet3/issues/4709) Remove this
  // once upstream Bullet supports internal joint damping and set
  // `model->body->getLink(i).m_jointDamping` directly in SDFFeatures.cc.
  for (auto & joint : this->joints)
  {
    const auto *model =
        this->ReferenceInterface<ModelInfo>(joint.second->model);
    const auto *identifier =
        std::get_if<InternalJoint>(&joint.second->identifier);
    if (model != nullptr && model->body != nullptr && identifier != nullptr)
    {
      model->body->AddJointDampingTorque(identifier->indexInBtModel,
                                         joint.second->damping);
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
    const btMultiBodyLinkCollider* obA =
      dynamic_cast<const btMultiBodyLinkCollider*>(contactManifold->getBody0());
    const btMultiBodyLinkCollider* obB =
      dynamic_cast<const btMultiBodyLinkCollider*>(contactManifold->getBody1());
    std::size_t collision1ID = std::numeric_limits<std::size_t>::max();
    std::size_t collision2ID = std::numeric_limits<std::size_t>::max();

    for (const auto & link : this->links)
    {
      if (obA == link.second->collider.get())
      {
        for (const auto &v : link.second->collisionNameToEntityId)
        {
          collision1ID = v.second;
        }
      }
      if (obB == link.second->collider.get())
      {
        for (const auto &v : link.second->collisionNameToEntityId)
        {
          collision2ID = v.second;
        }
      }
    }
    int numContacts = contactManifold->getNumContacts();
    for (int j = 0; j < numContacts; j++)
    {
      btManifoldPoint& pt = contactManifold->getContactPoint(j);
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
        this->GenerateIdentity(collision1ID, this->collisions.at(collision1ID)),
        this->GenerateIdentity(collision2ID, this->collisions.at(collision2ID)),
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
