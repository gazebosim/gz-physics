/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

#ifndef IGNITION_PHYSICS_DARTSIM_BASE_HH_
#define IGNITION_PHYSICS_DARTSIM_BASE_HH_

#include <unordered_map>

#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/Skeleton.hpp>
#include <dart/simulation/World.hpp>

#include <ignition/physics/Implements.hh>

namespace ignition {
namespace physics {
namespace dartsim {

class Base : public Implements3d<FeatureList<Feature>>
{
  public: inline Identity InitiateEngine(std::size_t /*_engineID*/) override
  {
    this->GetNextEntity();
    // dartsim does not have multiple "engines"
    return this->GenerateIdentity(0);
  }

  public: inline std::size_t GetNextEntity()
  {
    return entityCount++;
  }

  public: std::size_t entityCount = 0;

  public: std::unordered_map<std::size_t, dart::simulation::WorldPtr> worlds;
  public: std::unordered_map<std::size_t, dart::dynamics::SkeletonPtr> models;
  public: std::unordered_map<std::size_t, dart::dynamics::BodyNodePtr> links;
  public: std::unordered_map<std::size_t, dart::dynamics::JointPtr> joints;
};

}
}
}

#endif
