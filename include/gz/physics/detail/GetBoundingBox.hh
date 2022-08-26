/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#ifndef GZ_PHYSICS_DETAIL_GETBOUNDINGBOX_HH_
#define GZ_PHYSICS_DETAIL_GETBOUNDINGBOX_HH_

#include <gz/physics/GetBoundingBox.hh>

namespace ignition
{
  namespace physics
  {
    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto GetLinkBoundingBox::Link<PolicyT, FeaturesT>
    ::GetAxisAlignedBoundingBox(const FrameID &_referenceFrame) const
    -> AlignedBoxType
    {
      AlignedBoxType result;
      std::size_t shapeCount = this->GetShapeCount();
      for (std::size_t i = 0; i < shapeCount; ++i)
      {
        // for each shape in this link, merge its AlignedBox into result
        auto shape = this->GetShape(i);
        result.extend(shape->GetAxisAlignedBoundingBox(_referenceFrame));
      }
      return result;
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto GetModelBoundingBox::Model<PolicyT, FeaturesT>
    ::GetAxisAlignedBoundingBox(const FrameID &_referenceFrame) const
    -> AlignedBoxType
    {
      AlignedBoxType result;
      std::size_t linkCount = this->GetLinkCount();
      for (std::size_t i = 0; i < linkCount; ++i)
      {
        // for each link in this model, merge its AlignedBox into result
        auto link = this->GetLink(i);
        result.extend(link->GetAxisAlignedBoundingBox(_referenceFrame));
      }
      return result;
    }
  }
}

#endif
