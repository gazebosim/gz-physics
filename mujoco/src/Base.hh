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
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/SemanticVersion.hh>
#include <gz/physics/Implements.hh>
#include <gz/physics/detail/EntityStorage.hh>

namespace gz
{
namespace physics
{
namespace mujoco
{

// Forward declarations
struct LinkInfo;
struct ModelInfo;
struct WorldInfo;

struct ShapeInfo
{
  ShapeInfo(std::size_t _entityId, std::shared_ptr<LinkInfo> _linkInfo)
      : entityId(_entityId), linkInfo(_linkInfo)
  {
  }
  std::size_t entityId;
  mjsGeom *geom{nullptr};
  std::string name;
  std::weak_ptr<LinkInfo> linkInfo;
  std::optional<uint16_t> categoryMask;
};

struct LinkInfo
{
  LinkInfo(std::size_t _entityId, std::shared_ptr<ModelInfo> _modelInfo)
      : entityId(_entityId), modelInfo(_modelInfo)
  {
  }
  std::size_t entityId;
  mjsBody *body{nullptr};
  std::string name;
  std::weak_ptr<ModelInfo> modelInfo;
  WorldInfo *worldInfo{nullptr};
  detail::EntityStorage<std::shared_ptr<ShapeInfo>, const mjsGeom *> shapes{};
};

struct JointInfo
{
  JointInfo(std::size_t _entityId, mjsJoint *_joint,
           std::shared_ptr<ModelInfo> _modelInfo)
      : entityId(_entityId), joint(_joint), modelInfo(_modelInfo)
  {
  }
  std::size_t entityId;
  mjsJoint *joint;
  std::string name;
  std::weak_ptr<ModelInfo> modelInfo;
};


struct ModelInfo
{
  ModelInfo(std::size_t _entityId, WorldInfo *_worldInfo)
      : entityId(_entityId), worldInfo(_worldInfo)
  {
  }
  std::size_t entityId;
  mjsBody *body{nullptr};
  WorldInfo *worldInfo;
  mjsBody *parentBody{nullptr};
  std::string name;
  detail::EntityStorage<std::shared_ptr<LinkInfo>, const mjsBody *> links{};
  std::vector<std::shared_ptr<JointInfo>> joints{};
};

struct FrameInfo
{
  FrameInfo(mjsSite *_site, WorldInfo* _worldInfo)
      : site(_site), worldInfo(_worldInfo)
  {
  }
  mjsSite * site{nullptr};
  WorldInfo* worldInfo;
};

struct WorldInfo
{
  std::size_t entityId;
  mjsBody *body{nullptr};
  mjSpec *mjSpecObj{nullptr};
  mjModel *mjModelObj{nullptr};
  mjData *mjDataObj{nullptr};
  bool specDirty{true};
  std::string name;
  std::vector<std::shared_ptr<JointInfo>> joints{};
  // Key2 is the scoped name of the model, including the world name
  detail::EntityStorage<std::shared_ptr<ModelInfo>, std::string> models;

  // Vector of ShapeInfo, indexed by mujoco geom id
  std::vector<std::shared_ptr<ShapeInfo>> geomIdToShapeInfo{};

  /// \brief body poses from the most recent pose change/update.
  /// The index is the MuJoCo body ID, and the value is the body's pose.
  std::vector<std::optional<gz::math::Pose3d>> prevBodyPoses;
};

class Base
{
  // Note: Entity ID 0 is reserved for the "engine"
  public: std::size_t entityCount = 1;

  public: inline std::size_t GetNextEntity()
  {
    return entityCount++;
  }

  public: std::string JoinNames(const std::string &_parent,
                                const std::string &_name) const
  {
    return _parent + "::" + _name;
  }


  public: detail::EntityStorage<std::shared_ptr<WorldInfo>, std::string> worlds;
  public: std::unordered_map<std::size_t, std::shared_ptr<FrameInfo>> frames{};

  public: const std::string engineName{"mujoco"};
  public: const gz::math::SemanticVersion engineVersion{mj_versionString()};

  public: bool RecompileSpec(WorldInfo &_worldInfo) const;
};
}  // namespace mujoco
}  // namespace physics
}  // namespace gz

#endif
