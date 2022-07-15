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

#ifndef GZ_PHYSICS_BULLET_FEATHERSTONE_SRC_FREEGROUPFEATURES_HH_
#define GZ_PHYSICS_BULLET_FEATHERSTONE_SRC_FREEGROUPFEATURES_HH_

#include <gz/physics/FreeGroup.hh>

#include "Base.hh"

namespace gz {
namespace physics {
namespace bullet_featherstone {

struct FreeGroupFeatureList : gz::physics::FeatureList<
  FindFreeGroupFeature,
  SetFreeGroupWorldPose
> { };

class FreeGroupFeatures
    : public virtual Base,
      public virtual Implements3d<FreeGroupFeatureList>
{
  // ----- FindFreeGroupFeature -----
  Identity FindFreeGroupForModel(const Identity &_modelID) const override;

  Identity FindFreeGroupForLink(const Identity &_linkID) const override;

  Identity GetFreeGroupRootLink(const Identity &_groupID) const override;

  // ----- SetFreeGroupWorldPose -----
  void SetFreeGroupWorldPose(
      const Identity &_groupID,
      const PoseType &_pose) override;
};

}  // namespace bullet_featherstone
}  // namespace physics
}  // namespace gz

#endif
