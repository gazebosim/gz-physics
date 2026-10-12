/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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

#include "ShapeFeatures.hh"

#include <mujoco/mujoco.h>
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <utility>
#include <gz/common/Console.hh>
#include <gz/math/eigen3/Conversions.hh>
#include <gz/physics/heightmap/detail/SampleHeightmap.hh>

namespace gz {
namespace physics {
namespace mujoco {

/////////////////////////////////////////////////
AlignedBox3d ShapeFeatures::GetShapeAxisAlignedBoundingBox(
    const Identity &_shapeID) const
{
  const auto *shapeInfo = this->ReferenceInterface<ShapeInfo>(_shapeID);
  if (!shapeInfo || !shapeInfo->geom)
  {
    gzerr << "Error computing shape bounding box. "
          << "Unable to find shape " << _shapeID.id << std::endl;
    return AlignedBox3d();
  }

  auto *worldInfo = shapeInfo->worldInfo;
  if (!worldInfo)
  {
    gzerr << "Error computing shape bounding box. "
          << "Unable to retrieve world for shape "
          << _shapeID.id << std::endl;
    return AlignedBox3d();
  }
  if (!this->RecompileSpec(*worldInfo))
  {
    gzerr << "Error computing shape bounding box. "
          << "Unable to recompile spec for shape "
          << _shapeID.id << std::endl;
    return AlignedBox3d();
  }

  const mjModel *m = worldInfo->mjModelObj;
  if (!m)
  {
    gzerr << "Error computing shape bounding box. "
          << "mjModelObj is null for shape " << _shapeID.id << std::endl;
    return AlignedBox3d();
  }
  int geomId = mjs_getId(shapeInfo->geom->element);
  if (geomId < 0 || geomId >= m->ngeom)
  {
    gzerr << "Error computing shape bounding box. "
          << "Unable to retrieve geom for shape " << _shapeID.id << std::endl;
    return AlignedBox3d();
  }

  if (!m->geom_type || !m->geom_size || !m->geom_dataid)
  {
    gzerr << "Error computing shape bounding box: "
          << "mjModel geom arrays are null." << std::endl;
    return AlignedBox3d();
  }

  int geomType = m->geom_type[geomId];

  // MuJoCo stores the size parameters for every geom in a single flattened
  // array called mjModel::geom_size. It allocates exactly 3 slots in this array
  // for every geom, regardless of its type. We find the starting point for
  // any geom by multiplying its geomId by 3.
  const mjtNum *size = m->geom_size + static_cast<ptrdiff_t>(3) * geomId;

  Vector3d min = Vector3d::Zero();
  Vector3d max = Vector3d::Zero();

  switch (geomType)
  {
    case mjGEOM_BOX:
    {
      max = Vector3d(size[0], size[1], size[2]);
      min = -max;
      break;
    }
    case mjGEOM_SPHERE:
    {
      double radius = size[0];
      max = Vector3d(radius, radius, radius);
      min = -max;
      break;
    }
    case mjGEOM_CYLINDER:
    {
      double radius = size[0];
      double halfLength = size[1];
      max = Vector3d(radius, radius, halfLength);
      min = -max;
      break;
    }
    case mjGEOM_CAPSULE:
    {
      double radius = size[0];
      double halfLength = size[1];
      max = Vector3d(radius, radius, halfLength + radius);
      min = -max;
      break;
    }
    case mjGEOM_ELLIPSOID:
    {
      max = Vector3d(size[0], size[1], size[2]);
      min = -max;
      break;
    }
    case mjGEOM_PLANE:
    {
      max = Vector3d(size[0], size[1], 0);
      min = -max;
      break;
    }
    case mjGEOM_MESH:
    {
      if (shapeInfo->cachedAABB)
      {
        return *shapeInfo->cachedAABB;
      }

      const int meshId = m->geom_dataid[geomId];
      if (meshId < 0 || meshId >= m->nmesh)
      {
        gzerr << "Invalid meshId " << meshId << " for geom "
              << geomId << std::endl;
        break;
      }

      if (!m->mesh_vertadr || !m->mesh_vertnum)
      {
        gzerr << "Error computing mesh bounding box: "
              << "mjModel mesh arrays (mesh_vertadr, mesh_vertnum) are null."
              << std::endl;
        break;
      }

      const int vertAddress = m->mesh_vertadr[meshId];
      const int vertCount = m->mesh_vertnum[meshId];

      if (!m->mesh_vert || !m->mesh_pos || !m->mesh_quat)
      {
        gzerr << "Error computing mesh bounding box: "
              << "mjModel arrays (mesh_vert, mesh_pos, mesh_quat) are null."
              << std::endl;
        break;
      }

      if (vertAddress < 0 || vertAddress + vertCount > m->nmeshvert)
      {
        gzerr << "Error computing mesh bounding box: "
              << "Invalid vertAddress (" << vertAddress << ") or vertCount ("
              << vertCount << ") for total vertices " << m->nmeshvert
              << std::endl;
        break;
      }

      if (vertCount <= 0)
      {
        gzwarn << "Mesh " << meshId << " has zero vertex count." << std::endl;
        break;
      }

      // MuJoCo stores all vertex data for all meshes in a single, large
      // flattened array called mjModel::mesh_vert. There are 3 coordinates
      // per vertex (x, y, z). Multiply vertAddress by 3 to find the start
      // of the mesh's data.
      const float* vertices = m->mesh_vert +
                              static_cast<ptrdiff_t>(3) * vertAddress;
      const mjtNum* meshPos = m->mesh_pos +
                              static_cast<ptrdiff_t>(3) * meshId;
      const mjtNum* meshQuat = m->mesh_quat +
                               static_cast<ptrdiff_t>(4) * meshId;

      Eigen::Quaterniond q = convertQuat(meshQuat);

      min = max = q * Vector3d(static_cast<double>(vertices[0]),
                               static_cast<double>(vertices[1]),
                               static_cast<double>(vertices[2]));

      for (int i = 1; i < vertCount; ++i)
      {
        Vector3d v = q * Vector3d(
            static_cast<double>(vertices[static_cast<ptrdiff_t>(3) * i + 0]),
            static_cast<double>(vertices[static_cast<ptrdiff_t>(3) * i + 1]),
            static_cast<double>(vertices[static_cast<ptrdiff_t>(3) * i + 2]));

        min.x() = std::min(min.x(), v.x());
        min.y() = std::min(min.y(), v.y());
        min.z() = std::min(min.z(), v.z());
        max.x() = std::max(max.x(), v.x());
        max.y() = std::max(max.y(), v.y());
        max.z() = std::max(max.z(), v.z());
      }
      // mesh_vert is in CoM-centered frame; mesh_pos is the CoM offset.
      // Add the offset to get the AABB in the mesh's local frame.
      const Vector3d offset(meshPos[0], meshPos[1], meshPos[2]);
      min += offset;
      max += offset;

      shapeInfo->cachedAABB = AlignedBox3d(min, max);
      return *shapeInfo->cachedAABB;
      break;
    }
    case mjGEOM_HFIELD:
    {
      const int hfieldId = m->geom_dataid[geomId];
      if (hfieldId < 0 || hfieldId >= m->nhfield)
      {
        gzerr << "Invalid hfieldId " << hfieldId << " for geom "
              << geomId << std::endl;
        break;
      }
      // (radius_x, radius_y, elevation_z, base_z). The base box extends
      // below the lowest point, which is at heightmapMinHeight in the shape
      // frame.
      const mjtNum *hsize = m->hfield_size +
                            static_cast<ptrdiff_t>(4) * hfieldId;
      const double z0 = shapeInfo->heightmapMinHeight;
      min = Vector3d(-hsize[0], -hsize[1], z0 - hsize[3]);
      max = Vector3d(hsize[0], hsize[1], z0 + hsize[2]);
      break;
    }
    default:
    {
      gzwarn << "Bounding box for geom type " << geomType
             << " not implemented." << std::endl;
      return AlignedBox3d();
    }
  }

  return AlignedBox3d(min, max);
}

/////////////////////////////////////////////////
Identity ShapeFeatures::CastToHeightmapShape(const Identity &_shapeID) const
{
  const auto *shapeInfo = this->ReferenceInterface<ShapeInfo>(_shapeID);
  if (shapeInfo && shapeInfo->hfield)
  {
    return _shapeID;
  }
  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
LinearVector3d ShapeFeatures::GetHeightmapShapeSize(
    const Identity &_heightmapID) const
{
  const auto *shapeInfo = this->ReferenceInterface<ShapeInfo>(_heightmapID);
  if (!shapeInfo || !shapeInfo->hfield)
  {
    return LinearVector3d::Zero();
  }
  const double *size = shapeInfo->hfield->size;
  return LinearVector3d(2 * size[0], 2 * size[1],
                        shapeInfo->heightmapHeight);
}

/////////////////////////////////////////////////
Identity ShapeFeatures::AttachHeightmapShape(
    const Identity &_linkID,
    const std::string &_name,
    const common::HeightmapData &_heightmapData,
    const Pose3d &_pose,
    const LinearVector3d &_size,
    int _subSampling)
{
  const auto *linkInfoPtr = this->ReferenceInterface<LinkInfo>(_linkID);
  const auto modelInfo =
      linkInfoPtr ? linkInfoPtr->modelInfo.lock() : nullptr;
  if (!modelInfo || !linkInfoPtr->body || !linkInfoPtr->worldInfo)
  {
    gzerr << "Unable to attach heightmap [" << _name << "]: invalid link."
          << std::endl;
    return this->GenerateInvalidId();
  }
  auto linkInfo = modelInfo->links.at(linkInfoPtr->entityId);
  WorldInfo *worldInfo = linkInfo->worldInfo;

  // Leave the world as it was if the heightmap can't be attached:
  // mj_recompile frees the world's model and data when compiling fails, so
  // reject everything that would make it fail before changing the spec.
  // Compile pending changes first: once the spec has been compiled,
  // mjs_findElement only finds compiled elements.
  if (!this->RecompileSpec(*worldInfo) || !worldInfo->mjModelObj)
  {
    gzerr << "Unable to attach heightmap [" << _name << "]: failed to "
          << "recompile the world." << std::endl;
    return this->GenerateInvalidId();
  }

  // MuJoCo only supports height fields on bodies that are welded to the world.
  const int bodyId = mjs_getId(linkInfo->body->element);
  if (bodyId < 0 || worldInfo->mjModelObj->body_weldid[bodyId] != 0)
  {
    gzerr << "Unable to attach heightmap [" << _name << "] to link ["
          << linkInfo->name << "], which is not static. MuJoCo only "
          << "supports heightmaps on static links." << std::endl;
    return this->GenerateInvalidId();
  }

  mjSpec *spec = worldInfo->mjSpecObj;
  const auto *bodyName = mjs_getName(linkInfo->body->element);
  const std::string geomName =
      JoinNames(bodyName ? *bodyName : std::string(), _name);
  const std::string hfieldName = geomName + "::hfield";
  if (mjs_findElement(spec, mjOBJ_GEOM, geomName.c_str()) ||
      mjs_findElement(spec, mjOBJ_HFIELD, hfieldName.c_str()))
  {
    gzerr << "Unable to attach heightmap [" << _name << "]: the name is "
          << "taken." << std::endl;
    return this->GenerateInvalidId();
  }

  // MuJoCo's row 0 is the -y edge.
  auto sampled = heightmap::detail::SampleHeightmap(
      _heightmapData, _size, _subSampling, true);
  if (!sampled)
  {
    gzerr << "Unable to attach heightmap [" << _name << "]: invalid size, "
          << "heightmap data or too many samples." << std::endl;
    return this->GenerateInvalidId();
  }
  const double minHeight = sampled->minHeight;
  const double maxHeight = sampled->maxHeight;

  // MuJoCo rescales the data to [0, 1] when it compiles the model, and size[2]
  // scales it back.
  mjsHField *hfield = mjs_addHField(spec);
  mjs_setName(hfield->element, hfieldName.c_str());
  hfield->nrow = static_cast<int>(sampled->vertSize);
  hfield->ncol = static_cast<int>(sampled->vertSize);
  // MuJoCo spaces the samples size / (vertSize - 1) apart, so the height
  // field spans the full size.
  hfield->size[0] = _size.x() / 2.0;
  hfield->size[1] = _size.y() / 2.0;
  // MuJoCo requires positive sizes: use a placeholder for a flat heightmap,
  // whose normalized heights are all 0, and keep any other range.
  const double heightRange = maxHeight - minHeight;
  hfield->size[2] = heightRange > 0 ? heightRange : 1e-6;
  // Depth of the solid base below the lowest point.
  hfield->size[3] = std::max(hfield->size[2], 0.1);
  *hfield->userdata = std::move(sampled->heights);

  mjsGeom *geom = mjs_addGeom(linkInfo->body, nullptr);
  geom->type = mjGEOM_HFIELD;
  mjs_setString(geom->hfieldname, hfieldName.c_str());
  mjs_setName(geom->element, geomName.c_str());
  geom->contype = std::numeric_limits<uint16_t>::max();
  geom->conaffinity = std::numeric_limits<uint16_t>::max();
  copyPos(Eigen::Vector3d(_pose * Eigen::Vector3d(0, 0, minHeight)), geom->pos);
  copyQuat(Eigen::Quaterniond(_pose.linear()), geom->quat);

  auto shapeInfo = std::make_shared<ShapeInfo>(this->GetNextEntity(), linkInfo);
  shapeInfo->worldInfo = worldInfo;
  shapeInfo->geom = geom;
  shapeInfo->name = _name;
  shapeInfo->hfield = hfield;
  // Like dartsim, report the height range of the sampled terrain, which
  // hfield->size[2] doesn't hold for a flat heightmap.
  shapeInfo->heightmapHeight = heightRange;
  shapeInfo->heightmapMinHeight = minHeight;
  linkInfo->shapes.AddEntity(shapeInfo->entityId, shapeInfo, geom,
                             linkInfo->entityId);
  this->frames[shapeInfo->entityId] =
      std::make_shared<FrameInfo>(linkInfo->body, _pose, worldInfo);

  // Recompile right away, so that GetShape finds the new geom.
  worldInfo->specDirty = true;
  if (!this->RecompileSpec(*worldInfo))
  {
    gzerr << "Unable to attach heightmap [" << _name << "]: failed to "
          << "recompile the world." << std::endl;
    return this->GenerateInvalidId();
  }

  return this->GenerateIdentity(shapeInfo->entityId, shapeInfo);
}

}  // namespace mujoco
}  // namespace physics
}  // namespace gz
