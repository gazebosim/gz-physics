/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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

#include <Eigen/Geometry>

namespace gz {
namespace physics {
namespace mujoco {

/////////////////////////////////////////////////
void LinkFeatures::AddLinkExternalForceInWorld(
    const Identity &_id,
    const LinearVectorType &_force,
    const LinearVectorType &_position)
{
  auto linkInfo = this->ReferenceInterface<LinkInfo>(_id);
  if (!linkInfo || !linkInfo->body)
    return;

  auto worldInfo = linkInfo->worldInfo;
  if (!worldInfo || !worldInfo->mjModelObj || !worldInfo->mjDataObj)
    return;

  int bodyId = mjs_getId(linkInfo->body->element);
  if (bodyId < 0 || bodyId >= worldInfo->mjModelObj->nbody)
    return;

  auto *d = worldInfo->mjDataObj;
  Eigen::Vector3d com = convertPos(&d->xipos[3 * bodyId]);
  Eigen::Vector3d torque = (_position - com).cross(_force);

  Eigen::Vector3d currentForce = convertPos(&d->xfrc_applied[6 * bodyId]);
  copyPos(currentForce + _force, &d->xfrc_applied[6 * bodyId]);

  Eigen::Vector3d currentTorque = convertPos(&d->xfrc_applied[6 * bodyId + 3]);
  copyPos(currentTorque + torque, &d->xfrc_applied[6 * bodyId + 3]);
}

/////////////////////////////////////////////////
void LinkFeatures::AddLinkExternalTorqueInWorld(
    const Identity &_id,
    const AngularVectorType &_torque)
{
  auto linkInfo = this->ReferenceInterface<LinkInfo>(_id);
  if (!linkInfo || !linkInfo->body)
    return;

  auto worldInfo = linkInfo->worldInfo;
  if (!worldInfo || !worldInfo->mjModelObj || !worldInfo->mjDataObj)
    return;

  int bodyId = mjs_getId(linkInfo->body->element);
  if (bodyId < 0 || bodyId >= worldInfo->mjModelObj->nbody)
    return;

  auto *d = worldInfo->mjDataObj;

  Eigen::Vector3d currentTorque = convertPos(&d->xfrc_applied[6 * bodyId + 3]);
  copyPos(currentTorque + _torque, &d->xfrc_applied[6 * bodyId + 3]);
}

}
}
}
