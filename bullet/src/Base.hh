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

#ifndef IGNITION_PHYSICS_BULLET_BASE_HH_
#define IGNITION_PHYSICS_BULLET_BASE_HH_


#include <Eigen/Geometry>

#include <assert.h>
#include <memory>
#include <string>

#include <ignition/common/Console.hh>
#include <ignition/physics/Implements.hh>

namespace ignition {
namespace physics {
namespace bullet {

class Base : public Implements3d<FeatureList<Feature>>
{
  public: std::size_t entityCount = 0;

  public: inline std::size_t GetNextEntity()
  {
    return entityCount++;
  }

  public: inline Identity InitiateEngine(std::size_t /*_engineID*/) override
  {
    const auto id = this->GetNextEntity();
    assert(id == 0);

    return this->GenerateIdentity(0);
  }
};

}
}
}

#endif
