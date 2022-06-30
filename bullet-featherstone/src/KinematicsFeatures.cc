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
      data.pose.translation() = convert(
        model->body->localPosToWorld(index, btVector3(0, 0, 0)));
      data.pose.linear() = convert(
        model->body->localFrameToWorld(index, btMatrix3x3::getIdentity()));

      // Transform from bullet's inertia frame to Gazebo's link frame
      data.pose = data.pose * linkInfo->inertiaToLinkFrame;

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
    // If the Frame was not a Link then we can assume it's a Model because Free
    // Groups are always Models underneath. If we ever support frame semantics for
    // more than links, models, and free groups, then this implementation needs to
    // change.
    model = this->FrameInterface<ModelInfo>(_id);
  }

  FrameData data;
  data.pose = convert(model->body->getBaseWorldTransform())
    * model->baseInertiaToLinkFrame;
  data.linearVelocity = convert(model->body->getBaseVel());
  data.linearAcceleration = convert(model->body->getBaseOmega());
  return data;
}

}  // namespace bullet_featherstone
}  // namespace physics
}  // namespace gz
