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

#include "JointFeatures.hh"

namespace ignition {
namespace physics {
namespace bullet {

/////////////////////////////////////////////////
double JointFeatures::GetJointPosition(
      const Identity &_id, const std::size_t _dof) const
{
    const auto &jointInfo = this->joints.at(_id);
    const auto &modelInfo = this->models.at(jointInfo->model);
    const auto pos = modelInfo->model->getJointPosMultiDof
        (jointInfo->childIndex)[_dof];
    return pos;
}

////////////////////////////////////////////////
double JointFeatures::GetJointVelocity(
      const Identity &_id, const std::size_t _dof) const
{
    const auto &jointInfo = this->joints.at(_id);
    const auto &modelInfo = this->models.at(jointInfo->model);
    const auto vel = modelInfo->model->getJointVelMultiDof
        (jointInfo->childIndex)[_dof];
    return vel;
}

////////////////////////////////////////////////
double JointFeatures::GetJointForce(
      const Identity &_id, const std::size_t _dof) const
{
    const auto &jointInfo = this->joints.at(_id);
    const auto &modelInfo = this->models.at(jointInfo->model);
    const auto force = modelInfo->model->getJointTorqueMultiDof
        (jointInfo->childIndex)[_dof];
    return force;
}

/////////////////////////////////////////////////
void JointFeatures::SetJointVelocity(
      const Identity &_id, const std::size_t _dof,
      const double _value)
{
    const auto &jointInfo = this->joints.at(_id);
    const auto &model = this->models.at(jointInfo->model)->model;
    const auto jointIndex = jointInfo->childIndex;
    auto currentVel = model->getJointVelMultiDof(jointIndex);
    currentVel[_dof] = _value;
    model->setJointVelMultiDof(jointIndex, currentVel);
}

/////////////////////////////////////////////////
void JointFeatures::SetJointForce(
      const Identity &_id, const std::size_t _dof,
      const double _value)
{
    const auto &jointInfo = this->joints.at(_id);
    const auto &model = this->models.at(jointInfo->model)->model;
    const auto jointIndex = jointInfo->childIndex;
    const auto currentTorque = model->getJointTorqueMultiDof(jointIndex)[_dof];
    model->addJointTorqueMultiDof(jointIndex, _dof, _value-currentTorque);
}

/////////////////////////////////////////////////
std::size_t JointFeatures::GetJointDegreesOfFreedom(
    const Identity &_id) const
{
    const auto &jointInfo = this->joints.at(_id);
    const auto &modelInfo = this->models.at(jointInfo->model);
    const auto dof = modelInfo->model->getLink(
        jointInfo->childIndex).m_dofCount;
    return dof;
}

}
}
}
