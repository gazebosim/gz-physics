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

#include <gz/common/Console.hh>
#include <gz/common/Profiler.hh>
#include "KinematicsFeatures.hh"

namespace gz {
namespace physics {
namespace bullet_featherstone {


FrameData3d getNonBaseLinkFrameData(
    const ModelInfo *_modelInfo,
    const LinkInfo *_linkInfo,
    const Eigen::Isometry3d &_poseOffset = Eigen::Isometry3d::Identity())
{
  GZ_PROFILE("bullet_featherstone::getNonBaseLinkFrameData");
  const auto index = _linkInfo->indexInModel.value();
  const auto &body = *_modelInfo->body;

  // World transform of the link's center of mass frame (the bullet link
  // frame). Everything below is derived from it so that the kinematic tree
  // is traversed only once.
  const Eigen::Isometry3d comWorldTf =
      convert(GetWorldTransformOfLinkInertiaFrame(body, index));
  const Eigen::Matrix3d &comWorldRot = comWorldTf.linear();

  FrameData3d data;
  data.pose = comWorldTf * _linkInfo->inertiaToLinkFrame * _poseOffset;

  // Velocities of the center of mass frame of every link, expressed in the
  // respective link frame.
  auto &localOmega = _modelInfo->linkOmega;
  auto &localVel = _modelInfo->linkVel;
  const std::size_t numLinks = static_cast<std::size_t>(body.getNumLinks());
  localOmega.resize(numLinks + 1);
  localVel.resize(numLinks + 1);
  body.compTreeLinkVelocities(localOmega.data(), localVel.data());

  const Eigen::Vector3d comAngVel =
      comWorldRot * convert(localOmega[index + 1]);
  const Eigen::Vector3d comLinVel =
      comWorldRot * convert(localVel[index + 1]);
  const Eigen::Vector3d comToFrame =
      data.pose.translation() - comWorldTf.translation();

  data.angularVelocity = comAngVel;
  data.linearVelocity = comLinVel + comAngVel.cross(comToFrame);

  // \todo(iche033) Link accelerations are not computed. Bullet does not
  // expose joint or link accelerations.
  return data;
}

/////////////////////////////////////////////////
FrameData3d KinematicsFeatures::FrameDataRelativeToWorld(
    const FrameID &_id) const
{
  GZ_PROFILE("KinematicsFeatures::FrameDataRelativeToWorld");
  bool isModel = false;
  bool isCollision = false;
  bool isJoint = false;
  const ModelInfo *model = nullptr;
  Eigen::Isometry3d collisionPoseOffset = Eigen::Isometry3d();
  Eigen::Isometry3d jointPoseOffset = Eigen::Isometry3d();

  const auto linkIt = this->links.find(_id.ID());
  if (linkIt != this->links.end())
  {
    const auto &linkInfo = linkIt->second;
    model = this->ReferenceInterface<ModelInfo>(linkInfo->model);

    // If indexInModel has value, then this is a non-base link
    // otherwise, it is a base link and the calculations will be performed
    // later below in this function
    if (linkInfo->indexInModel.has_value())
    {
      return getNonBaseLinkFrameData(model, linkInfo.get());
    }
  }
  else
  {
    auto jointIt = this->joints.find(_id.ID());
    if (jointIt != this->joints.end())
    {
      isJoint = true;
      const auto &jointInfo = jointIt->second;

      const auto linkIt2 = this->links.find(jointInfo->childLinkID);
      if (linkIt2 != this->links.end())
      {
        const auto &linkInfo2 = linkIt2->second;
        model = this->ReferenceInterface<ModelInfo>(linkInfo2->model);
        jointPoseOffset = jointInfo->tf_to_child.inverse();
        // If indexInModel has value, then this is a joint connected to
        // non-base link
        if (linkInfo2->indexInModel.has_value())
        {
          return getNonBaseLinkFrameData(
              model, linkInfo2.get(), jointPoseOffset);
        }
      }
    }
    else
    {
      auto collisionIt = this->collisions.find(_id.ID());
      if (collisionIt != this->collisions.end())
      {
        isCollision = true;
        const auto &collisionInfo = collisionIt->second;

        const auto linkIt2 = this->links.find(collisionInfo->link);
        if (linkIt2 != this->links.end())
        {
          const auto &linkInfo2 = linkIt2->second;
          model = this->ReferenceInterface<ModelInfo>(linkInfo2->model);
          collisionPoseOffset = collisionInfo->linkToCollision;
          // If indexInModel has value, then this is a collision in a
          // non-base link
          if (linkInfo2->indexInModel.has_value())
          {
            return getNonBaseLinkFrameData(
                model, linkInfo2.get(), collisionPoseOffset);
          }
        }
      }
      else
      {
        auto modelIt = this->models.find(_id.ID());
        if (modelIt != this->models.end())
        {
          model = modelIt->second.get();
          isModel = true;
        }
      }
    }
  }

  // The function reached here so that means the entity is either
  // a model, a base link, a collision in the base link, or a joint
  // connected to a base link.
  FrameData data;
  if (model && model->body)
  {
    data.pose = convert(model->body->getBaseWorldTransform())
        * model->baseInertiaToLinkFrame;
    if (isModel)
      data.pose = data.pose * model->rootLinkToModelTf;
    else if (isCollision)
      data.pose = data.pose * collisionPoseOffset;
    else if (isJoint)
      data.pose = data.pose * jointPoseOffset;
    data.linearVelocity = convert(model->body->getBaseVel());
    data.angularVelocity = convert(model->body->getBaseOmega());
  }
  return data;
}

}  // namespace bullet_featherstone
}  // namespace physics
}  // namespace gz
