/*
 * Copyright (C) 2025 Open Source Robotics Foundation
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

#ifndef GZ_PHYSICS_MUJOCO_BASE_HH_
#define GZ_PHYSICS_MUJOCO_BASE_HH_

#include <mujoco/mjspec.h>
#include <mujoco/mujoco.h>

#include <gz/physics/Implements.hh>
#include <memory>

namespace gz
{
namespace physics
{
namespace mujoco
{

struct LinkInfo {
  mjsBody* body;
};

struct JointInfo {
  mjsJoint* joint;
};

struct ModelInfo {
  mjsBody* body;
  std::vector<std::shared_ptr<LinkInfo>> links{};
  std::vector<std::shared_ptr<JointInfo>> joints{};
};

struct WorldInfo {
  mjsBody* body;
  mjSpec *mjSpecObj;
  mjModel *mjModelObj;
  mjData *mjDataObj;
  std::vector<std::shared_ptr<ModelInfo>> links{};
  std::vector<std::shared_ptr<JointInfo>> joints{};
};

class Base : public Implements3d<FeatureList<Feature>>
{
  public: Identity InitiateEngine(std::size_t /*_engineID*/) override;

  public: std::vector<std::shared_ptr<WorldInfo>> worlds;
};
}  // namespace mujoco
}  // namespace physics
}  // namespace gz

#endif
