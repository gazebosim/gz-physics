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
#include <sdf/JointAxis.hh>
#include <limits>
#include <algorithm>

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

      // Resolve associated equality constraint indices
      jointInfo->eqIndices.clear();
      for (int i = 0; i < m->neq; ++i)
      {
        if (m->eq_type[i] == mjEQ_JOINT && m->eq_obj2id[i] == jointId)
        {
          jointInfo->eqIndices.push_back(i);
        }
      }
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

namespace {

/////////////////////////////////////////////////
void convertJointAxis(const ::sdf::JointAxis *_sdfAxis, double *axis)
{
  math::Vector3d resolvedAxis;
  ::sdf::Errors errors = _sdfAxis->ResolveXyz(resolvedAxis);
  if (!errors.empty())
  {
    gzerr << "There was an error in JointAxis::ResolveXyz\n";
    gzerr << errors << std::endl;
    return;
  }
  std::copy(resolvedAxis.Data(), resolvedAxis.Data() + 3, axis);
}

/////////////////////////////////////////////////
double infIfNeg(const double _value)
{
  if (_value < 0.0)
    return std::numeric_limits<double>::infinity();

  return _value;
}

/////////////////////////////////////////////////
double convertScrewThreadPitch(const double _pitch)
{
  return _pitch / (2.0 * mjPI);
}

/////////////////////////////////////////////////
void copyStandardJointAxisProperties(
    mjsJoint * _joint,
    const ::sdf::JointAxis *_sdfAxis)
{
  _joint->damping = _sdfAxis->Damping();
  _joint->frictionloss = _sdfAxis->Friction();
  _joint->springref = _sdfAxis->SpringReference();
  _joint->stiffness = _sdfAxis->SpringStiffness();
  _joint->limited = static_cast<int>(!std::isinf(_sdfAxis->Lower()) &&
                                     !std::isinf(_sdfAxis->Upper()));
  _joint->range[0] = _sdfAxis->Lower();
  _joint->range[1] = _sdfAxis->Upper();

  _joint->actfrclimited =
      static_cast<int>(!std::isinf(infIfNeg(_sdfAxis->Effort())));

  _joint->actfrcrange[0] = -infIfNeg(_sdfAxis->Effort());
  _joint->actfrcrange[1] = infIfNeg(_sdfAxis->Effort());

  if (!std::isinf(_sdfAxis->MaxVelocity()))
  {
    gzwarn << "The MuJoCo physics engine plugin does not support velocity "
             "limits\n";
  }
}

} // namespace

/////////////////////////////////////////////////
math::Pose3d Base::resolveSdfPose(const ::sdf::SemanticPose &_semPose,
                                  const std::string &_resolveTo)
{
  math::Pose3d pose;
  ::sdf::Errors errors = _semPose.Resolve(pose, _resolveTo);
  if (!errors.empty())
  {
    if (!_semPose.RelativeTo().empty())
    {
      gzerr << "There was an error in SemanticPose::Resolve\n";
      for (const auto &err : errors)
      {
        gzerr << err.Message() << std::endl;
      }
      gzerr << "There is no optimal fallback since the relative_to attribute["
             << _semPose.RelativeTo() << "] of the pose is not empty. "
             << "Falling back to using the raw Pose.\n";
    }
    pose = _semPose.RawPose();
  }

  return pose;
}

/////////////////////////////////////////////////
void Base::AddJoint(
    mjSpec *_spec,
    const ::sdf::Joint *sdfJoint,
    mjsBody *child,
    const std::shared_ptr<ModelInfo> &_modelInfo)
{
  auto worldInfo = _modelInfo->worldInfo;
  if (!sdfJoint)
  {
    mjsJoint *joint = mjs_addFreeJoint(child);
    std::string freeJointName = _modelInfo->name + "::freejoint";
    mjs_setName(joint->element, freeJointName.c_str());
  }
  else
  {
    mjsJoint *joint{nullptr};
    mjsJoint *joint2{nullptr};
    mjsActuator *actuator{nullptr};
    if (sdfJoint->Type() == ::sdf::JointType::PRISMATIC)
    {
      joint = mjs_addJoint(child, nullptr);
      joint->type = mjJNT_SLIDE;
      const auto *sdfAxis = sdfJoint->Axis(0);
      convertJointAxis(sdfAxis, joint->axis);
      copyStandardJointAxisProperties(joint, sdfAxis);
    }
    else if (sdfJoint->Type() == ::sdf::JointType::REVOLUTE)
    {
      joint = mjs_addJoint(child, nullptr);
      joint->type = mjJNT_HINGE;
      const auto *sdfAxis = sdfJoint->Axis(0);
      convertJointAxis(sdfAxis, joint->axis);
      copyStandardJointAxisProperties(joint, sdfAxis);
    }
    else if (sdfJoint->Type() == ::sdf::JointType::BALL)
    {
      joint = mjs_addJoint(child, nullptr);
      joint->type = mjJNT_BALL;
      const auto *sdfAxis = sdfJoint->Axis(0);
      if (sdfAxis)
      {
        convertJointAxis(sdfAxis, joint->axis);
        copyStandardJointAxisProperties(joint, sdfAxis);
        if (joint->limited && std::abs(joint->range[0]) > 0.0)
        {
          gzwarn << "MuJoCo requires the lower joint position limit of ball "
                    "joints to be zero.\n";
          joint->range[0] = 0;
        }
      }
    }
    else if (sdfJoint->Type() == ::sdf::JointType::SCREW)
    {
      joint = mjs_addJoint(child, nullptr);
      joint->type = mjJNT_HINGE;
      const auto *sdfAxis1 = sdfJoint->Axis(0);
      if (sdfAxis1)
      {
        convertJointAxis(sdfAxis1, joint->axis);
        copyStandardJointAxisProperties(joint, sdfAxis1);
        joint->limited = false;
      }

      joint2 = mjs_addJoint(child, nullptr);
      joint2->type = mjJNT_SLIDE;
      if (sdfAxis1)
      {
        convertJointAxis(sdfAxis1, joint2->axis);
        joint2->limited = static_cast<int>(!std::isinf(sdfAxis1->Lower()) &&
                                           !std::isinf(sdfAxis1->Upper()));
        if (joint2->limited)
        {
          const double pitch =
              convertScrewThreadPitch(sdfJoint->ScrewThreadPitch());
          joint2->range[0] = sdfAxis1->Lower() * pitch;
          joint2->range[1] = sdfAxis1->Upper() * pitch;
        }
      }
    }
    else if (sdfJoint->Type() == ::sdf::JointType::UNIVERSAL)
    {
      joint = mjs_addJoint(child, nullptr);
      joint->type = mjJNT_HINGE;
      const auto *sdfAxis1 = sdfJoint->Axis(0);
      if (sdfAxis1)
      {
        convertJointAxis(sdfAxis1, joint->axis);
        copyStandardJointAxisProperties(joint, sdfAxis1);
      }

      joint2 = mjs_addJoint(child, nullptr);
      joint2->type = mjJNT_HINGE;
      const auto *sdfAxis2 = sdfJoint->Axis(1);
      if (sdfAxis2)
      {
        convertJointAxis(sdfAxis2, joint2->axis);
        copyStandardJointAxisProperties(joint2, sdfAxis2);
      }
    }
    else if (sdfJoint->Type() != ::sdf::JointType::FIXED)
    {
      gzwarn << "Joint type " << static_cast<int>(sdfJoint->Type())
             << " in joint [" << sdfJoint->Name() << "] not supported\n";
      return;
    }

    auto jointPose = resolveSdfPose(sdfJoint->SemanticPose());
    mjsEquality *eq = nullptr;
    if (joint)
    {
      const std::string mjJointName =
          ::sdf::JoinName(_modelInfo->name, sdfJoint->Name());
      mjs_setName(joint->element, mjJointName.c_str());
      actuator = mjs_addActuator(_spec, nullptr);
      actuator->trntype = mjtTrn::mjTRN_JOINT;
      mjs_setString(actuator->target, mjJointName.c_str());
      copyPos(jointPose.Pos(), joint->pos);

      if (joint2)
      {
        const std::string mjJointName2 = mjJointName + "_axis2";
        mjs_setName(joint2->element, mjJointName2.c_str());
        mjsActuator *actuator2 = mjs_addActuator(_spec, nullptr);
        actuator2->trntype = mjtTrn::mjTRN_JOINT;
        mjs_setString(actuator2->target, mjJointName2.c_str());
        copyPos(jointPose.Pos(), joint2->pos);

        if (sdfJoint->Type() == ::sdf::JointType::SCREW)
        {
          eq = mjs_addEquality(_spec, nullptr);
          mjs_setName(eq->element, (mjJointName + "_eq").c_str());
          eq->type = mjEQ_JOINT;
          eq->active = 1;
          mjs_setString(eq->name1, mjJointName2.c_str());
          mjs_setString(eq->name2, mjJointName.c_str());
          std::fill(std::begin(eq->data), std::end(eq->data), 0.0);
          eq->data[1] =
              convertScrewThreadPitch(sdfJoint->ScrewThreadPitch());
        }
      }
    }
    std::shared_ptr<JointInfo> jointInfo = nullptr;
    if (_modelInfo->joints.HasEntity(sdfJoint->Name()))
    {
      jointInfo = _modelInfo->joints.at(sdfJoint->Name());
    }
    else
    {
      jointInfo = std::make_shared<JointInfo>(
          this->GetNextEntity(), _modelInfo);
      jointInfo->name = sdfJoint->Name();
    }

    jointInfo->joint = joint;
    jointInfo->childBody = child;
    jointInfo->actuator = actuator;
    jointInfo->worldInfo = worldInfo;

    if (sdfJoint->Type() == ::sdf::JointType::BALL)
    {
      jointInfo->worldInfo->ballJointPositionsCache.push_back(std::nullopt);
      jointInfo->ballJointCacheIndex =
          jointInfo->worldInfo->ballJointPositionsCache.size() - 1;
    }

    auto jointSite = mjs_addSite(child, nullptr);
    jointInfo->site = jointSite;
    copyPos(jointPose.Pos(), jointSite->pos);
    copyQuat(jointPose.Rot(), jointSite->quat);
    this->frames[jointInfo->entityId] =
        std::make_shared<FrameInfo>(jointSite, worldInfo);

    if (!_modelInfo->joints.HasEntity(sdfJoint->Name()))
    {
      _modelInfo->joints.AddEntity(jointInfo->entityId, jointInfo,
                                   jointInfo->name, _modelInfo->entityId);
    }
  }
}

}  // namespace mujoco
}  // namespace physics
}  // namespace gz
