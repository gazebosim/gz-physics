/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#ifndef IGNITION_PHYSICS_BULLET_SRC_KINEMATICSFEATURES_HH_
#define IGNITION_PHYSICS_BULLET_SRC_KINEMATICSFEATURES_HH_

#include <ignition/physics/FrameSemantics.hh>
#include <ignition/physics/FreeGroup.hh>

#include "Base.hh"

namespace ignition {
namespace physics {
namespace bullet {

struct KinematicsFeatureList : ignition::physics::FeatureList<
  LinkFrameSemantics
> { };

class KinematicsFeatures :
    public virtual Base,
    public virtual Implements3d<KinematicsFeatureList>
{
  public: FrameData3d FrameDataRelativeToWorld(
              const FrameID &_id) const override;
};

}  // namespace bullet
}  // namespace physics
}  // namespace ignition

#endif
