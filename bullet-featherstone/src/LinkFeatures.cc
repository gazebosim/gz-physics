/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

  auto F = btVector3(_force[0], _force[1], _force[2]);

  if (link->indexInModel.has_value())
  {
    gzdbg << "Adding force to link: " << F.x() << " " << F.y() << " " << F.z() <<  std::endl;
    model->body->addLinkForce(link->indexInModel.value(), F);
  }
  else
  {
    gzdbg << "Adding force to body: " << F.x() << " " << F.y() << " " << F.z() <<  std::endl;
    model->body->addBaseForce(F);
  }
}

/////////////////////////////////////////////////
void LinkFeatures::AddLinkExternalTorqueInWorld(
    const Identity &_id, const AngularVectorType &_torque)
{
  auto *link = this->ReferenceInterface<LinkInfo>(_id);
  auto *model = this->ReferenceInterface<ModelInfo>(link->model);

  auto T = btVector3(_torque[0], _torque[1], _torque[2]);

  if (link->indexInModel.has_value())
  {
    gzdbg << "Adding torque to link: " << T.x() << " " << T.y() << " " << T.z() <<  std::endl;
    model->body->addLinkTorque(link->indexInModel.value(), T);
  }
  else
  {
    gzdbg << "Adding torque to body: " << T.x() << " " << T.y() << " " << T.z() <<  std::endl;
    model->body->addBaseTorque(T);
  }
}

}
}
}
