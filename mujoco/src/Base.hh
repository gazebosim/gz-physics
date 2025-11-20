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

#include <mujoco/mujoco.h>

#include <cstddef>
#include <gz/physics/Implements.hh>
#include <memory>

namespace gz
{
namespace physics
{
namespace mujoco
{

struct ModelInfo;

struct LinkInfo
{
  mjsBody *body;
  std::weak_ptr<ModelInfo> modelInfo;
};

struct JointInfo
{
  mjsJoint *joint;
};

struct WorldInfo;

struct ModelInfo
{
  mjsBody *body{nullptr};
  mjsBody *parentBody{nullptr};
  std::string name;
  std::vector<std::shared_ptr<LinkInfo>> links{};
  std::vector<std::shared_ptr<JointInfo>> joints{};

  std::shared_ptr<LinkInfo> LinkFromBody(const mjsBody *_body) const {

    auto it = std::find_if(this->links.begin(), this->links.end(),
                           [_body](const std::shared_ptr<LinkInfo> &_linkInfo)
                           { return _body == _linkInfo->body; });
    if (it == this->links.end())
    {
      return nullptr;
    }

    return *it;
  }

};

struct WorldInfo
{
  mjsBody *body{nullptr};
  mjSpec *mjSpecObj;
  mjModel *mjModelObj;
  mjData *mjDataObj;
  std::string name;
  std::vector<std::shared_ptr<ModelInfo>> models{};
  std::vector<std::shared_ptr<JointInfo>> joints{};
};

class Base : public Implements3d<FeatureList<Feature>>
{
  public: Identity InitiateEngine(std::size_t /*_engineID*/) override;

  public: std::vector<std::shared_ptr<WorldInfo>> worlds;

  public: std::string engineName{"mujoco"};

  public: template <typename Info>
  Identity IdentityFromBody(const mjsBody *_body, std::shared_ptr<Info> info) const {
    auto id = static_cast<std::size_t>(mjs_getId(_body->element));
    return this->GenerateIdentity(id, info);
  }
};
}  // namespace mujoco
}  // namespace physics
}  // namespace gz

#endif
