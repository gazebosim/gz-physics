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

#ifndef IGNITION_PHYSICS_DARTSIM_SRC_WORLDFEATURES_HH_
#define IGNITION_PHYSICS_DARTSIM_SRC_WORLDFEATURES_HH_

#include <string>

#include <ignition/physics/World.hh>

#include "Base.hh"

namespace ignition {
namespace physics {
namespace dartsim {

struct WorldFeatureList : FeatureList<
  CollisionDetector,
  Gravity,
  Solver
> { };

class WorldFeatures :
    public virtual Base,
    public virtual Implements3d<WorldFeatureList>
{
  // Documentation inherited
  public: void SetWorldCollisionDetector(
      const Identity &_id, const std::string &_collisionDetector) override;

  // Documentation inherited
  public: const std::string &GetWorldCollisionDetector(const Identity &_id)
      const override;

  // Documentation inherited
  public: void SetWorldGravity(
      const Identity &_id, const LinearVectorType &_gravity) override;

  // Documentation inherited
  public: LinearVectorType GetWorldGravity(const Identity &_id) const override;

  // Documentation inherited
  public: void SetWorldSolver(const Identity &_id, const std::string &_solver)
      override;

  // Documentation inherited
  public: const std::string &GetWorldSolver(const Identity &_id) const override;
};

}
}
}

#endif
