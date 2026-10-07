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

  link->isKinematic = _kinematic;
  int collisionFlags = _kinematic ? btCollisionObject::CF_KINEMATIC_OBJECT :
      btCollisionObject::CF_DYNAMIC_OBJECT;

  if (link->indexInModel.has_value())
  {
    const int idx = link->indexInModel.value();
    model->body->setLinkDynamicType(idx, collisionFlags);
    if (_kinematic)
    {
      for (int d = 0; d < model->body->getLink(idx).m_dofCount; ++d)
      {
        model->body->getJointVelMultiDof(idx)[d] = 0;
      }
      for (auto &jointPair : this->joints)
      {
        if (std::size_t(jointPair.second->childLinkID) == std::size_t(_id))
        {
          jointPair.second->kinematicJointVel = 0.0;
          jointPair.second->kinematicJointVelCmd = std::nullopt;
        }
      }
    }
    else
    {
      auto *world = this->ReferenceInterface<WorldInfo>(model->world);
      for (auto &jointPair : this->joints)
      {
        if (std::size_t(jointPair.second->childLinkID) == std::size_t(_id) &&
            jointPair.second->kinematicMotor)
        {
          world->world->removeMultiBodyConstraint(
              jointPair.second->kinematicMotor.get());
          jointPair.second->kinematicMotor.reset();
        }
      }
    }
  }
  else
  {
    model->body->setBaseDynamicType(collisionFlags);
  }
  model->body->wakeUp();
}

/////////////////////////////////////////////////
bool KinematicLinkFeatures::GetLinkKinematic(const Identity &_id) const
{
  auto *link = this->ReferenceInterface<LinkInfo>(_id);
  auto *model = this->ReferenceInterface<ModelInfo>(link->model);

  int indexInModel = link->indexInModel.value_or(-1);
  return model->body->isLinkKinematic(indexInModel);
}
#endif

}  // namespace bullet_featherstone
}  // namespace physics
}  // namespace gz
