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

#include "LinkFeatures.hh"

namespace gz {
namespace physics {
namespace bullet_featherstone {

/////////////////////////////////////////////////
void LinkFeatures::AddLinkExternalForceInWorld(
    const Identity &_id, const LinearVectorType &_force,
    const LinearVectorType &_position)
{
  auto *link = this->ReferenceInterface<LinkInfo>(_id);
  auto *model = this->ReferenceInterface<ModelInfo>(link->model);

  auto F = convertVec(_force);

  gzwarn << "AAAA " << std::endl;
  exit(1);
  if (link->indexInModel.has_value())
  {
    btVector3 forceWorld = F;
    btVector3 relPosWorld =
      convertVec(_position) - model->body->getLink(
        link->indexInModel.value()).m_cachedWorldTransform.getOrigin();

    model->body->addLinkForce(link->indexInModel.value(), forceWorld);
    model->body->addLinkTorque(
      link->indexInModel.value(), relPosWorld.cross(forceWorld));
  }
  else
  {
    btVector3 relPosWorld =
      convertVec(_position) -
      model->body->getBaseWorldTransform().getOrigin();
    model->body->addBaseForce(F);
    model->body->addBaseTorque(relPosWorld.cross(F));
  }
  model->body->wakeUp();
}

/////////////////////////////////////////////////
void LinkFeatures::AddLinkExternalTorqueInWorld(
    const Identity &_id, const AngularVectorType &_torque)
{
  auto *link = this->ReferenceInterface<LinkInfo>(_id);
  auto *model = this->ReferenceInterface<ModelInfo>(link->model);

  const btVector3 T = convertVec(_torque);
  return;

  gzwarn << "BBB " << std::endl;
  exit(1);
  if (link->indexInModel.has_value())
  {
    btVector3 torqueWorld =
      model->body->getLink(link->indexInModel.value()).
        m_cachedWorldTransform.getBasis() * T;
    model->body->addLinkTorque(link->indexInModel.value(), torqueWorld);
  }
  else
  {
    btVector3 torqueWorld = model->body->getBaseWorldTransform().getBasis() * T;
    model->body->addBaseTorque(torqueWorld);
  }
  model->body->wakeUp();
}

}
}
}
