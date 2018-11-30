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
void LinkFeatures::SetLinkLinearVelocity(
    std::size_t _id, const LinearVector3d &_vel)
{
  dart::dynamics::BodyNode *dtBodyNode = this->links.at(_id);
  // DART body node always has a parent joint.
  dart::dynamics::Joint *joint = dtBodyNode->getParentJoint();

  // Check if the parent joint is a free joint
  dart::dynamics::FreeJoint *freeJoint =
      dynamic_cast<dart::dynamics::FreeJoint*>(joint);

  // If the parent joint is a free joint, set the proper generalized velocity to
  // fit the linear velocity of the link
  if (freeJoint)
  {
    Eigen::Vector3d genVel = _vel;
    // If this link has parent link then subtract the effect of parent link's
    // linear and angular velocities
    if (dtBodyNode->getParentBodyNode())
    {
      // Local transformation from the parent link frame to this link frame
      Eigen::Isometry3d T = freeJoint->getRelativeTransform();

      // Parent link's linear and angular velocities
      Eigen::Vector3d parentLinVel =
          dtBodyNode->getParentBodyNode()->getLinearVelocity();
      Eigen::Vector3d parentAngVel =
          dtBodyNode->getParentBodyNode()->getAngularVelocity();

      // The effect of the parent link's velocities
      Eigen::Vector3d propagatedLinVel =
          T.linear().transpose() *
          (parentAngVel.cross(T.translation()) + parentLinVel);

      // Subtract the effect
      genVel -= propagatedLinVel;
    }

    // Rotation matrix from world frame to this link frame
    Eigen::Matrix3d R = dtBodyNode->getTransform().linear();

    // Change the reference frame to world
    genVel = R * genVel;

    // Set the generalized velocities
    freeJoint->setVelocity(3, genVel[0]);
    freeJoint->setVelocity(4, genVel[1]);
    freeJoint->setVelocity(5, genVel[2]);
  }
  else
  {
    igndbg << "dartsim:SetLinkLinearVelocity() doesn't make sense if the "
           << "parent joint is not free joint (6-dof).\n";
  }
}

/////////////////////////////////////////////////
void LinkFeatures::SetLinkAngularVelocity(
    std::size_t _id, const AngularVector3d &_vel)
{
  dart::dynamics::BodyNode *dtBodyNode = this->links.at(_id);
  // DART body node always has a parent joint.
  dart::dynamics::Joint *joint = dtBodyNode->getParentJoint();

  // Check if the parent joint is free joint
  dart::dynamics::FreeJoint *freeJoint =
      dynamic_cast<dart::dynamics::FreeJoint*>(joint);

  // If the parent joint is free joint, set the proper generalized velocity to
  // fit the linear velocity of the link
  if (freeJoint)
  {
    // Generalized velocities
    Eigen::Vector3d genVel = _vel;

    // If this link has parent link then subtract the effect of parent link's
    // linear and angular velocities
    if (dtBodyNode->getParentBodyNode())
    {
      // Local transformation from the parent link frame to this link frame
      Eigen::Isometry3d T = freeJoint->getRelativeTransform();

      // Parent link's linear and angular velocities
      Eigen::Vector3d parentAngVel =
          dtBodyNode->getParentBodyNode()->getAngularVelocity();

      // The effect of the parent link's velocities
      Eigen::Vector3d propagatedAngVel = T.linear().transpose() * parentAngVel;

      // Subtract the effect
      genVel -= propagatedAngVel;
    }

    // Rotation matrix from world frame to this link frame
    Eigen::Matrix3d R = dtBodyNode->getTransform().linear();

    // Change the reference frame to world
    genVel = R * genVel;

    // Set the generalized velocities
    freeJoint->setVelocity(0, genVel[0]);
    freeJoint->setVelocity(1, genVel[1]);
    freeJoint->setVelocity(2, genVel[2]);
  }
  else
  {
    igndbg << "dartsim::SetLinkAngularVelcity() doesn't make sense if the "
           << "parent joint is not free joint (6-dof).\n";
  }
}

/////////////////////////////////////////////////
void LinkFeatures::SetLinkForce(
    std::size_t _id, const LinearVector3d &_force)
{
  this->links.at(_id)->setExtForce(_force);
}

/////////////////////////////////////////////////
void LinkFeatures::SetLinkTorque(
      std::size_t _id, const AngularVector3d &_torque)
{
  this->links.at(_id)->setExtTorque(_torque);
}
}
}
}
