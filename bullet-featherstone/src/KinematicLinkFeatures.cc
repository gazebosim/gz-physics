/*
 * Copyright (C) 2024 Open Source Robotics Foundation
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

#include "KinematicLinkFeatures.hh"

namespace gz {
namespace physics {
namespace bullet_featherstone {

#if BT_BULLET_VERSION >= 307
/////////////////////////////////////////////////
void KinematicLinkFeatures::SetLinkKinematic(
    const Identity &_id, bool _kinematic)
{
  auto *link = this->ReferenceInterface<LinkInfo>(_id);
  auto *model = this->ReferenceInterface<ModelInfo>(link->model);
  auto *world = this->ReferenceInterface<WorldInfo>(model->world);

  link->isKinematic = _kinematic;
  const int collisionFlags = _kinematic ?
      btCollisionObject::CF_KINEMATIC_OBJECT :
      btCollisionObject::CF_DYNAMIC_OBJECT;

  // A link that is static, fixed to the world or currently welded to a static
  // link by AttachFixedJoint keeps its static collider flags. The kinematic
  // flag is applied by makeColliderDynamic when the link is made non-static
  // or detached.
  const bool colliderStatic = link->isStaticOrFixed || isColliderStatic(link);

  btBroadphaseProxy *proxy = link->collider ?
      link->collider->getBroadphaseHandle() : nullptr;
  if (!colliderStatic && proxy &&
      proxy->m_collisionFilterGroup != colliderFilterGroup(_kinematic))
  {
    // A kinematic collider does not collide with static or other kinematic
    // colliders, see colliderFilterGroup. The broadphase applies the filter
    // only when it creates pairs, so re-create the proxy: this drops the
    // pairs of a link made kinematic with static colliders right away and
    // creates them for a link made dynamic, which would otherwise only
    // happen once the link has moved.
    proxy->m_collisionFilterGroup = colliderFilterGroup(_kinematic);
    proxy->m_collisionFilterMask = colliderFilterMask(_kinematic);
    world->world->refreshBroadphaseProxy(link->collider.get());
  }

  if (link->indexInModel.has_value())
  {
    const int idx = link->indexInModel.value();
    if (!colliderStatic)
      model->body->setLinkDynamicType(idx, collisionFlags);
    if (_kinematic)
    {
      // Bullet stops integrating the joint position of a kinematic link but
      // keeps its joint velocity. Zero the velocity so that the link, which
      // no longer moves, does not report a stale velocity.
      for (int d = 0; d < model->body->getLink(idx).m_dofCount; ++d)
      {
        model->body->getJointVelMultiDof(idx)[d] = 0;
      }
    }
    // Note: besides the joint of the link, detachable fixed joints that were
    // attached to the link are also in this map, so do not stop at the first
    // match.
    for (auto &jointPair : this->joints)
    {
      auto &joint = *jointPair.second;
      if (std::size_t(joint.childLinkID) != std::size_t(_id))
        continue;

      if (_kinematic)
      {
        joint.kinematicJointVel = 0.0;
        joint.kinematicJointVelCmd = std::nullopt;
        joint.kinematicJointVelPinned = 0;
      }
      else
      {
        if (joint.motor)
        {
          // WorldForwardStep may have turned the joint motor into the lock
          // of the kinematic link, with an unlimited impulse. Remove it so
          // that the dynamic link moves freely. The next velocity command
          // recreates it with the joint effort limit.
          world->world->removeMultiBodyConstraint(joint.motor.get());
          joint.motor.reset();
        }
        if (joint.jointLimits && joint.jointLimitsMaxImpulse > 0)
        {
          // Re-enable the joint limit constraint that WorldForwardStep
          // disabled while the link was kinematic.
          joint.jointLimits->setMaxAppliedImpulse(joint.jointLimitsMaxImpulse);
          joint.jointLimitsMaxImpulse = 0;
        }
      }
    }
  }
  else
  {
    if (!colliderStatic)
      model->body->setBaseDynamicType(collisionFlags);
    if (_kinematic)
    {
      // A kinematic base is moved with the velocity of its root link frame
      // stored in the multibody, see SimulationFeatures::WorldForwardStep.
      // Start from the current velocity so that the base keeps moving as it
      // did while it was dynamic.
      model->body->SetBaseLinkFrameVelocity(
          convert(getBaseLinkFrameLinearVelocity(*model)),
          convert(model->body->getBaseOmega()));
    }
  }
  // Changing the dynamic type does not wake up a sleeping body in bullet, so
  // a body that fell asleep while kinematic would otherwise not start moving
  // when it is made dynamic.
  model->body->wakeUp();
}

/////////////////////////////////////////////////
bool KinematicLinkFeatures::GetLinkKinematic(const Identity &_id) const
{
  // Report the requested state rather than the collider flags: the collider
  // of a kinematic link is temporarily static while the link is welded to a
  // static link, see SetLinkKinematic.
  return this->ReferenceInterface<LinkInfo>(_id)->isKinematic;
}
#endif

}  // namespace bullet_featherstone
}  // namespace physics
}  // namespace gz
