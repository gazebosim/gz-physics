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

#ifndef GZ_PHYSICS_BULLET_SRC_WORLDFEATURES_HH_
#define GZ_PHYSICS_BULLET_SRC_WORLDFEATURES_HH_

#include <cstddef>
#include <string>

#include <gz/physics/World.hh>

#include "Base.hh"

namespace gz {
namespace physics {
namespace bullet_featherstone {

struct WorldFeatureList : FeatureList<
  Gravity,
  Solver
> { };

class WorldFeatures :
    public virtual Base,
    public virtual Implements3d<WorldFeatureList>
{
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

  // Documentation inherited
  public: void SetWorldSolverIterations(const Identity &_id, std::size_t)
      override;

  // Documentation inherited
  public: std::size_t GetWorldSolverIterations(const Identity &_id) const
      override;
};

}
}
}

#endif
