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

/////////////////////////////////////////////////
FrameData3d KinematicsFeatures::FrameDataRelativeToWorld(
    const FrameID &_id) const
{
  const auto linkIt = this->links.find(_id.ID());
  const ModelInfo *model = nullptr;
  if (linkIt != this->links.end())
  {
    const auto &linkInfo = linkIt->second;
    const auto indexOpt = linkInfo->indexInModel;
    model = this->ReferenceInterface<ModelInfo>(linkInfo->model);

    if (indexOpt.has_value())
    {
      const auto index = *indexOpt;
      FrameData data;
      data.pose = GetWorldTransformOfLink(*model, *linkInfo);
      const auto &link = model->body->getLink(index);
      data.linearVelocity = convert(link.m_absFrameTotVelocity.getLinear());
      data.angularVelocity = convert(link.m_absFrameTotVelocity.getAngular());
      return data;
    }

    // If indexOpt is nullopt then the link is the base link which will be
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
        const auto indexOpt2 = linkInfo2->indexInModel;
        model = this->ReferenceInterface<ModelInfo>(linkInfo2->model);

        if (indexOpt2.has_value())
        {
          const auto index2 = *indexOpt2;
          FrameData data;
          data.pose = GetWorldTransformOfLink(*model, *linkInfo2);
          const auto &link = model->body->getLink(index2);
          data.linearVelocity = convert(
            link.m_absFrameTotVelocity.getLinear());
          data.angularVelocity = convert(
            link.m_absFrameTotVelocity.getAngular());
          return data;
        }
      }
    }

    auto collisionIt = this->collisions.find(_id.ID());
    if (collisionIt != this->collisions.end())
    {
      const auto &collisionInfo = collisionIt->second;

      const auto linkIt2 = this->links.find(collisionInfo->link);
      if (linkIt2 != this->links.end())
      {
        const auto &linkInfo2 = linkIt2->second;
        const auto indexOpt2 = linkInfo2->indexInModel;
        model = this->ReferenceInterface<ModelInfo>(linkInfo2->model);

        if (indexOpt2.has_value())
        {
          const auto index2 = *indexOpt2;
          FrameData data;
          data.pose = GetWorldTransformOfLink(*model, *linkInfo2);
          const auto &link = model->body->getLink(index2);
          data.linearVelocity = convert(
            link.m_absFrameTotVelocity.getLinear());
          data.angularVelocity = convert(
            link.m_absFrameTotVelocity.getAngular());
          return data;
        }
      }
    }

    if (model->body == nullptr)
      model = this->FrameInterface<ModelInfo>(_id);
  }

  FrameData data;
  if(model && model->body)
  {
    data.pose = convert(model->body->getBaseWorldTransform())
      * model->baseInertiaToLinkFrame;
    data.linearVelocity = convert(model->body->getBaseVel());
    data.angularVelocity = convert(model->body->getBaseOmega());
  }
  return data;
}

}  // namespace bullet_featherstone
}  // namespace physics
}  // namespace gz
