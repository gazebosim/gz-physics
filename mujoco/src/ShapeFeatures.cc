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
#include <gz/math/eigen3/Conversions.hh>

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
    return AlignedBox3d();
  }

  const auto linkInfo = shapeInfo->linkInfo.lock();
  if (!linkInfo)
  {
    return AlignedBox3d();
  }

  auto *worldInfo = linkInfo->worldInfo;
  this->RecompileSpec(*worldInfo);

  const mjModel *m = worldInfo->mjModelObj;
  int geomId = mjs_getId(shapeInfo->geom->element);
  if (geomId < 0 || geomId >= m->ngeom)
  {
    return AlignedBox3d();
  }

  int geomType = m->geom_type[geomId];

  // MuJoCo stores the size parameters for every geom in a single flattened
  // array called mjModel::geom_size. It allocates exactly 3 slots in this array
  // for every geom, regardless of its type. We find the starting point for
  // any geom by multiplying its geomId by 3.
  const mjtNum *size = m->geom_size + 3 * geomId;

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
      if (meshId >= 0 && meshId < m->nmesh)
      {
        const int vertAddress = m->mesh_vertadr[meshId];
        const int vertCount = m->mesh_vertnum[meshId];

        // MuJoCo stores all vertex data for all meshes in a single, large
        // flattened array called mjModel::mesh_vert. There are 3 coordinates
        // per vertex (x, y, z). Multiply vertAddress by 3 to find the start
        // of the mesh's data.
        const float* vertices = m->mesh_vert + 3 * vertAddress;

        if (vertCount > 0)
        {
          min = Vector3d(static_cast<double>(vertices[0]),
                         static_cast<double>(vertices[1]),
                         static_cast<double>(vertices[2]));
          max = min;

          for (int i = 1; i < vertCount; ++i)
          {
            min.x() = std::min(min.x(),
                               static_cast<double>(vertices[3 * i + 0]));
            min.y() = std::min(min.y(),
                               static_cast<double>(vertices[3 * i + 1]));
            min.z() = std::min(min.z(),
                               static_cast<double>(vertices[3 * i + 2]));
            max.x() = std::max(max.x(),
                               static_cast<double>(vertices[3 * i + 0]));
            max.y() = std::max(max.y(),
                               static_cast<double>(vertices[3 * i + 1]));
            max.z() = std::max(max.z(),
                               static_cast<double>(vertices[3 * i + 2]));
          }
          shapeInfo->cachedAABB = AlignedBox3d(min, max);
          return *shapeInfo->cachedAABB;
        }
      }
      break;
    }
    default:
    {
      gzwarn << "Bounding box for geom type " << geomType << " not implemented\n";
      return AlignedBox3d();
    }
  }

  return AlignedBox3d(min, max);
}

}  // namespace mujoco
}  // namespace physics
}  // namespace gz
