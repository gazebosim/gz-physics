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

#include <Eigen/Geometry>
#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/SemanticVersion.hh>
#include <gz/math/Vector3.hh>
#include <gz/physics/Implements.hh>
#include <gz/physics/detail/EntityStorage.hh>

namespace gz
{
namespace physics
{
namespace mujoco
{

inline void copyPos(const math::Vector3d &_src,  mjtNum *_dst)
{
  _dst[0] = _src.X();
  _dst[1] = _src.Y();
  _dst[2] = _src.Z();
}

inline void copyQuat(const math::Quaterniond &_src,  mjtNum *_dst)
{
  _dst[0] = _src.W();
  _dst[1] = _src.X();
  _dst[2] = _src.Y();
  _dst[3] = _src.Z();
}

inline Eigen::Vector3d convertPos(const mjtNum *_src)
{
  Eigen::Vector3d dst;
  dst.x() = _src[0];
  dst.y() = _src[1];
  dst.z() = _src[2];
  return dst;
}

inline Eigen::Quaterniond convertQuat(const mjtNum *_src)
{
  Eigen::Quaterniond dst;
  dst.w() = _src[0];
  dst.x() = _src[1];
  dst.y() = _src[2];
  dst.z() = _src[3];
  return dst;
}

inline Eigen::Isometry3d convertPose(const mjtNum *_pos, const mjtNum *_quat)
{
  return Eigen::Translation3d(convertPos(_pos)) * convertQuat(_quat);
}

inline gz::math::Pose3d getBodyWorldPoseFromMjData(mjData *_d, int _bodyId)
{
  return gz::math::Pose3d(_d->xpos[3 * _bodyId],
                          _d->xpos[3 * _bodyId + 1],
                          _d->xpos[3 * _bodyId + 2],
                          _d->xquat[4 * _bodyId],
                          _d->xquat[4 * _bodyId + 1],
                          _d->xquat[4 * _bodyId + 2],
                          _d->xquat[4 * _bodyId + 3]);
}

inline Eigen::Isometry3d getBodyWorldPoseFromMjDataEigen(mjData *_d,
                                                         int _bodyId)
{
  return Eigen::Translation3d(_d->xpos[3 * _bodyId], _d->xpos[3 * _bodyId + 1],
                              _d->xpos[3 * _bodyId + 2]) *
         Eigen::Quaterniond(_d->xquat[4 * _bodyId], _d->xquat[4 * _bodyId + 1],
                            _d->xquat[4 * _bodyId + 2],
                            _d->xquat[4 * _bodyId + 3]);
}

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
  JointInfo(std::size_t _entityId,
           std::shared_ptr<ModelInfo> _modelInfo)
      : entityId(_entityId), modelInfo(_modelInfo)
  {
  }
  std::size_t entityId;
  mjsJoint *joint{nullptr};
  mjsBody *childBody{nullptr};
  // A MuJoCo actuator is used for setting forces and velocity servo commands
  mjsActuator *actuator{nullptr};
  // Index of joint in mjData::qpos
  int nq_index{-1};
  // Index of joint in mjData::qvel and mjData::qacc
  int nv_index{-1};
  std::string name;
  std::weak_ptr<ModelInfo> modelInfo;
  WorldInfo* worldInfo{nullptr};
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
  detail::EntityStorage<std::shared_ptr<JointInfo>, std::string> joints{};
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
