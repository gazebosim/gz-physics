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

#include "Base.hh"

#include <gz/common/Console.hh>
#include <gz/physics/Implements.hh>
#include <sdf/Types.hh>

namespace gz
{
namespace physics
{
namespace mujoco
{
namespace {

struct UserDataHeader
{
  uint64_t magic;
  WorldInfo *worldInfo;
};

// Calculate required userdata slots (each slot is a mjtNum)
constexpr int kRequiredUserDataSlots =
    (sizeof(UserDataHeader) + sizeof(mjtNum) - 1) / sizeof(mjtNum);

WorldInfo *GetWorldInfoFromUserData(const mjModel *m, const mjData *d)
{
  if (m && m->nuserdata >= kRequiredUserDataSlots && d && d->userdata)
  {
    UserDataHeader header;
    std::memcpy(&header, d->userdata, sizeof(header));
    if (header.magic == kUserDataMagicNumber)
    {
      return header.worldInfo;
    }
  }
  return nullptr;
}

int ContactFilterCallback(const mjModel *m, mjData *d, int g1, int g2)
{
  int b1 = m->geom_bodyid[g1];
  int b2 = m->geom_bodyid[g2];

  WorldInfo *worldInfo = GetWorldInfoFromUserData(m, d);
  if (worldInfo && !worldInfo->dynamicWeldClusterMap.empty())
  {
    int c1 = worldInfo->dynamicWeldClusterMap[m->body_weldid[b1]];
    int c2 = worldInfo->dynamicWeldClusterMap[m->body_weldid[b2]];

    if (c1 != -1 && c1 == c2)
    {
      return 1;  // Exclude collision
    }
  }

  // Fallback to default contype/conaffinity mask filtering.
  // The global callback replaces the default mechanism, so we must re-implement
  // the bitmask check manually.
  int contype1 = m->geom_contype[g1];
  int conaffinity1 = m->geom_conaffinity[g1];
  int contype2 = m->geom_contype[g2];
  int conaffinity2 = m->geom_conaffinity[g2];

  return !(contype1 & conaffinity2) && !(contype2 & conaffinity1);
}

// Store joint position, velocity, acceleration and force indices. This is an
// optimization that avoids looking up these values in every simulation step.
// Note: qvelAddr is also used for acceleration
void resolveJointIndices(WorldInfo &_worldInfo)
{
  const auto &m = _worldInfo.mjModelObj;
  for (const auto &model : _worldInfo.models.idToObject)
  {
    for (auto &joint : model.second->joints.idToObject)
    {
      auto &jointInfo = joint.second;
      if (!jointInfo->joint)
      {
        // Fixed joint
        // Resolve weld constraint compiled ID
        jointInfo->weldEqIndex = std::nullopt;
        if (jointInfo->weldConstraintSpec)
        {
          int eqId = mjs_getId(jointInfo->weldConstraintSpec->element);
          if (eqId >= 0 && eqId < m->neq)
          {
            jointInfo->weldEqIndex = eqId;
          }
        }
        continue;
      }
      // Reset in case we encounter errors
      jointInfo->nq_index = -1;
      jointInfo->nv_index = -1;
      int jointId = mjs_getId(jointInfo->joint->element);
      if (jointId < 0 || jointId >= m->njnt)
      {
        gzerr << "Error resolving the index of joint [" << jointInfo->name
              << "] in the mjData \n";
        continue;
      }

      int qposAddr = m->jnt_qposadr[jointId];
      if (qposAddr < 0 || qposAddr >= m->nq)
      {
        gzerr << "Error resolving the position index of joint ["
              << jointInfo->name << "] in the mjData \n";
        continue;
      }
      jointInfo->nq_index = qposAddr;

      // The qvel address is confusingly stored in jnt_dofadr, but the comment
      // in the Mujoco documentation states: "jnt_dofadr: start addr in 'qvel'
      // for joint's data".
      int qvelAddr = m->jnt_dofadr[jointId];
      if (qvelAddr < 0 || qvelAddr >= m->nv)
      {
        gzerr << "Error resolving the velocity index of joint ["
              << jointInfo->name << "] in the mjData \n";
        continue;
      }
      jointInfo->nv_index = qvelAddr;

      // Resolve screw constraint compiled ID
      jointInfo->screwEqIndex = std::nullopt;
      if (jointInfo->screwConstraintSpec)
      {
        int eqId = mjs_getId(jointInfo->screwConstraintSpec->element);
        if (eqId >= 0 && eqId < m->neq)
        {
          jointInfo->screwEqIndex = eqId;
        }
      }

      // Resolve mimic constraints compiled IDs
      for (auto &constraint : jointInfo->mimicConstraints)
      {
        int eqId = mjs_getId(constraint.spec->element);
        if (eqId >= 0 && eqId < m->neq)
        {
          constraint.eqId = eqId;
        }
        else
        {
          constraint.eqId = -1;
        }
      }
    }
  }
}
}  // namespace

bool Base::RecompileSpec(WorldInfo &_worldInfo) const
{
  if (!_worldInfo.specDirty)
    return true;

  // Set nuserdata so that the compiler allocates the requested amount of data
  // in mjData::userdata
  _worldInfo.mjSpecObj->nuserdata = kRequiredUserDataSlots;

  int rc = mj_recompile(_worldInfo.mjSpecObj, nullptr, _worldInfo.mjModelObj,
                        _worldInfo.mjDataObj);
  _worldInfo.specDirty = false;

  if (rc != 0) {
    std::cerr << "Error compiling:" << mjs_getError(_worldInfo.mjSpecObj)
              << "\n";
    return false;
  }

  // Inject magic number and pointer to WorldInfo in the userdata slots
  if (_worldInfo.mjModelObj->nuserdata >= kRequiredUserDataSlots)
  {
    UserDataHeader header;
    header.magic = kUserDataMagicNumber;
    header.worldInfo = &_worldInfo;
    std::memcpy(_worldInfo.mjDataObj->userdata, &header,
                sizeof(UserDataHeader));
  }

  mjcb_contactfilter = ContactFilterCallback;

  // Ensure prevBodyPoses is sized correctly for the new model
  _worldInfo.prevBodyPoses.clear();
  _worldInfo.prevBodyPoses.resize(_worldInfo.mjModelObj->nbody);

  // TODO(azeey): Saving the resulting MJCF is useful for debugging, but should
  // be removed once the plugin is finalized
  // mj_saveXML(_worldInfo.mjSpecObj, "/tmp/mujoco_model.xml", nullptr, 0);

  // Build the geomIdToShapeInfo map
  _worldInfo.geomIdToShapeInfo.clear();
  _worldInfo.geomIdToShapeInfo.resize(_worldInfo.mjModelObj->ngeom);
  for (const auto &[modelId, modelInfo] : _worldInfo.models.idToObject)
  {
    for (const auto &[linkId, linkInfo] : modelInfo->links.idToObject)
    {
      for (const auto &[shapeId, shapeInfo] : linkInfo->shapes.idToObject)
      {
        if (shapeInfo->geom)
        {
          int geomId = mjs_getId(shapeInfo->geom->element);
          if (geomId != -1)
          {
            _worldInfo.geomIdToShapeInfo[geomId] = shapeInfo;
          }
        }
      }
    }
  }

  resolveJointIndices(_worldInfo);

  _worldInfo.UpdateWeldExclusions();

  mj_forward(_worldInfo.mjModelObj, _worldInfo.mjDataObj);
  return true;
}

}  // namespace mujoco
}  // namespace physics
}  // namespace gz
