/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/FreeJoint.hpp>

#include "LinkFeatures.hh"

namespace ignition {
namespace physics {
namespace dartsim {

/////////////////////////////////////////////////
LinearVector3d LinkFeatures::GetLinkExternalForceInWorld(
    std::size_t _id) const
{
  // DART uses spatial forces which express the force in the last three
  // components of the 6D vector
  Eigen::Vector6d f = this->links.at(_id)->getExternalForceGlobal();
  return f.tail<3>();
}

/////////////////////////////////////////////////
AngularVector3d LinkFeatures::GetLinkExternalTorqueInWorld(
    std::size_t _id) const
{
  // DART uses spatial forces which express the torque in the first three
  // components of the 6D vector
  Eigen::Vector6d f = this->links.at(_id)->getExternalForceGlobal();

  Eigen::Isometry3d tfCom = Eigen::Isometry3d::Identity();
  // We use the negative of the global COM location in order to transform the
  // spatial vector to be with respect to the the body's center of mass.
  tfCom.translation() = -this->links.at(_id)->getCOM(
        dart::dynamics::Frame::World());

  const Eigen::Vector6d fCom = dart::math::dAdInvT(tfCom, f);

  return fCom.head<3>();
}

/////////////////////////////////////////////////
void LinkFeatures::SetLinkExternalForceInWorld(
    std::size_t _id, const LinearVector3d &_force)
{
  this->links.at(_id)->setExtForce(_force);
}

/////////////////////////////////////////////////
void LinkFeatures::SetLinkExternalTorqueInWorld(
      std::size_t _id, const AngularVector3d &_torque)
{
  this->links.at(_id)->setExtTorque(_torque);
}
}
}
}
