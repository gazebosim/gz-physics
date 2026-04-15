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

#include <gz/physics/Implements.hh>
#include <sdf/Types.hh>

namespace gz
{
namespace physics
{
namespace mujoco
{
namespace {

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
        continue;
      }
      // Reset in case we encounter errors
      jointInfo->nq_index = -1;
      jointInfo->nv_index = -1;
      int jointId = mjs_getId(jointInfo->joint->element);
      if (jointId < 0 || jointId > m->njnt)
      {
        gzerr << "Error resolving the index of joint [" << jointInfo->name
              << "] in the mjData \n";
        continue;
      }

      int qposAddr = m->jnt_qposadr[jointId];
      if (qposAddr < 0 || qposAddr > m->nq)
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
      if (qvelAddr < 0 || qvelAddr > m->nq)
      {
        gzerr << "Error resolving the velocity index of joint ["
              << jointInfo->name << "] in the mjData \n";
        continue;
      }
      jointInfo->nv_index = qvelAddr;
    }
  }

}
}

bool Base::RecompileSpec(WorldInfo &_worldInfo) const
{
  if (!_worldInfo.specDirty)
    return true;

  int rc = mj_recompile(_worldInfo.mjSpecObj, nullptr, _worldInfo.mjModelObj,
                        _worldInfo.mjDataObj);
  _worldInfo.specDirty = false;

  if (rc != 0) {
    std::cerr << "Error compiling:" << mjs_getError(_worldInfo.mjSpecObj)
              << "\n";
    return false;
  }

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

  mj_forward(_worldInfo.mjModelObj, _worldInfo.mjDataObj);
  return true;
}

}  // namespace mujoco
}  // namespace physics
}  // namespace gz
