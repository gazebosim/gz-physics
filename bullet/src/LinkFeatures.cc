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

#include "LinkFeatures.hh"

namespace ignition {
namespace physics {
namespace bullet {

//////////////////////////////////////////
void LinkFeatures::AddLinkExternalForceInWorld(
      const Identity &_id,
      const LinearVectorType &_force,
      const LinearVectorType &/* _position */)
{
    const auto &linkInfo = this->links.at(_id);
    const auto &modelInfo = this->models.at(linkInfo->model);
    const btVector3 appliedForce = btVector3(_force[0], _force[1], _force[2]);

    // todo(anyone) use _position to set where to apply force
    modelInfo->model->addLinkForce(linkInfo->linkIndex, appliedForce);
}

}
}
}
