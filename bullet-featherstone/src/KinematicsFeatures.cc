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
#include "KinematicsFeatures.hh"

namespace gz {
namespace physics {
namespace bullet_featherstone {


FrameData3d getNonBaseLinkFrameData(const ModelInfo *_modelInfo,
                                    const LinkInfo *_linkInfo)
{
  const auto index = _linkInfo->indexInModel.value();
  FrameData3d data;
  data.pose = GetWorldTransformOfLink(*_modelInfo, *_linkInfo);
  const auto &link = _modelInfo->body->getLink(index);
  data.linearVelocity = convert(link.m_absFrameTotVelocity.getLinear());
  data.angularVelocity = convert(link.m_absFrameTotVelocity.getAngular());
  return data;
}

/////////////////////////////////////////////////
FrameData3d KinematicsFeatures::FrameDataRelativeToWorld(
    const FrameID &_id) const
{
  bool isModel = false;
  bool isCollision = false;
  const ModelInfo *model = nullptr;
  Eigen::Isometry3d collisionPoseOffset = Eigen::Isometry3d();

  const auto linkIt = this->links.find(_id.ID());
  if (linkIt != this->links.end())
  {
    const auto &linkInfo = linkIt->second;
    model = this->ReferenceInterface<ModelInfo>(linkInfo->model);

    if (linkInfo->indexInModel.has_value())
    {
      return getNonBaseLinkFrameData(model, linkInfo.get());
    }

    // If indexInModel is nullopt then the link is the base link which will be
    // calculated below.
  }
  else
  {
    auto jointIt = this->joints.find(_id.ID());
    if (jointIt != this->joints.end())
    {
      const auto &jointInfo = jointIt->second;

      const auto linkIt2 = this->links.find(jointInfo->childLinkID);
      if (linkIt2 != this->links.end())
      {
        const auto &linkInfo2 = linkIt2->second;
        model = this->ReferenceInterface<ModelInfo>(linkInfo2->model);
        if (linkInfo2->indexInModel.has_value())
        {
          return getNonBaseLinkFrameData(model, linkInfo2.get());
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
          if (linkInfo2->indexInModel.has_value())
          {
            auto data = getNonBaseLinkFrameData(model, linkInfo2.get());
            data.pose = data.pose * collisionPoseOffset;
            return data;
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

  FrameData data;
  if (model && model->body)
  {
    data.pose = convert(model->body->getBaseWorldTransform());
    if (!isModel)
      data.pose = data.pose * model->baseInertiaToLinkFrame;
    if (isCollision)
      data.pose = data.pose * collisionPoseOffset;
    data.linearVelocity = convert(model->body->getBaseVel());
    data.angularVelocity = convert(model->body->getBaseOmega());
  }
  return data;
}

}  // namespace bullet_featherstone
}  // namespace physics
}  // namespace gz
