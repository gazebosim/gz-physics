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

#include <ignition/common/Console.hh>
#include "KinematicsFeatures.hh"

namespace ignition {
namespace physics {
namespace bullet {

Eigen::Matrix3d convert(btMatrix3x3 mat)
{
  Eigen::Matrix3d val;
  val << mat[0][0], mat[0][1], mat[0][2],
         mat[1][0], mat[1][1], mat[1][2],
         mat[2][0], mat[2][1], mat[2][2];
  return val;
}

Eigen::Vector3d convert(btVector3 vec)
{
  Eigen::Vector3d val;
  val << vec[0], vec[1], vec[2];
  return val;
}

/////////////////////////////////////////////////
FrameData3d KinematicsFeatures::FrameDataRelativeToWorld(
    const FrameID &_id) const
{
  FrameData3d data;

  // The feature system should never send us the world ID.
  if (_id.IsWorld())
  {
    ignerr << "Given a FrameID belonging to the world. This should not be "
           << "possible! Please report this bug!\n";
    assert(false);
    return data;
  }

  const auto linkID = _id.ID();
  const auto &linkInfo = this->links.at(linkID);
  const auto modelIdentity = linkInfo->model;
  const auto &modelInfo = this->models.at(modelIdentity);
  const auto &model = modelInfo->model;

  // Get pose
  const btVector3 pos = model->localPosToWorld(
      linkInfo->linkIndex, btVector3(0, 0, 0));
  const btMatrix3x3 mat =
      model->localFrameToWorld(linkInfo->linkIndex,
                          btMatrix3x3::getIdentity());

  auto eigenMat = convert(mat);
  auto eigenVec = convert(pos);

  data.pose.linear() = eigenMat;
  data.pose.translation() = eigenVec;

  // Get frame velocities
  // Note: only support revolute/fixed joints
  btVector3 vel = btVector3(0, 0, 0);
  btVector3 omega = btVector3(0, 0, 0);
  btMatrix3x3 matChildToThis = btMatrix3x3().getIdentity();
  btVector3 rThisToChild = btVector3(0, 0, 0);
  auto linkIndex = linkInfo->linkIndex;
  while (linkIndex != -1)
  {
    const auto &link = model->getLink(linkIndex);
    btScalar jointVel;
    if (link.m_jointType == btMultibodyLink::eFixed)
      jointVel = 0;
    else if (link.m_jointType == btMultibodyLink::eRevolute)
      jointVel = model->getJointVel(linkIndex);
    else
    {
      ignerr << "Unregistered joint type for frame velocity computation.\n";
      return data;
    }

    // Get target link's velocities in current link frame
    omega += jointVel * link.getAxisTop(0);
    vel += jointVel * link.getAxisBottom(0) + omega.cross(
      matChildToThis * rThisToChild);

    // Express in parent frame
    const auto matThisToParent = btMatrix3x3(
      link.m_cachedRotParentToThis).inverse();
    omega = matThisToParent * omega;
    vel = matThisToParent * vel;

    // Info for next computation
    matChildToThis = matThisToParent;
    rThisToChild = link.m_cachedRVector;

    // Move on to parent
    linkIndex = link.m_parent;
  }

  // Add base velocities
  omega += model->getBaseOmega();
  vel += model->getBaseVel() + omega.cross(matChildToThis * rThisToChild);
  // Transform to world frame
  const auto matBaseToWorld = btMatrix3x3(model->getWorldToBaseRot()).inverse();
  omega = matBaseToWorld * omega;
  vel = matBaseToWorld * vel;

  data.linearVelocity = convert(vel);
  data.angularVelocity = convert(omega);

  // ToDo: compute frame accelerations

  return data;
}

}
}
}
