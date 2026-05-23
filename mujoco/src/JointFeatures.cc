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

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <string_view>
#include <Eigen/Geometry>
#include <gz/common/Console.hh>
#include <gz/math/Helpers.hh>
#include <mujoco/mujoco.h>
#include <sdf/Types.hh>
#include "Base.hh"
#include "gz/physics/Geometry.hh"

#include "JointFeatures.hh"

namespace gz {
namespace physics {
namespace mujoco {

namespace  {
Eigen::Vector3d *getBallJointPositionImpl(JointInfo *jointInfo)
{
  auto *d = jointInfo->worldInfo->mjDataObj;
  auto &cache = jointInfo->worldInfo->ballJointPositionsCache;

  if (!jointInfo->ballJointCacheIndex)
  {
    gzerr << "Ball joint cache index is not set for ["
          << jointInfo->name << "]\n";
    return nullptr;
  }

  if (*jointInfo->ballJointCacheIndex >= cache.size())
  {
    gzerr << "Ball joint cache index is outside of cache bounds for joint ["
          << jointInfo->name << "]\n";
    return nullptr;
  }
  auto &jointPos = cache[*jointInfo->ballJointCacheIndex];
  if (!jointPos)
  {
    const double *mjQuat = &d->qpos[jointInfo->nq_index];
    Eigen::AngleAxisd angleAxis{
        Eigen::Quaterniond(mjQuat[0], mjQuat[1], mjQuat[2], mjQuat[3])};
    jointPos = angleAxis.axis() * angleAxis.angle();
  }

  return &jointPos.value();
}

double getJointPositionImpl(JointInfo *jointInfo, std::size_t _dof)
{
  auto *d = jointInfo->worldInfo->mjDataObj;
  if (jointInfo->joint->type == mjtJoint::mjJNT_BALL)
  {
    // Ball joints have to be handled differently because the three DOFs exposed
    // in the public API actually represent the components of an Angle Axis
    // representation. We allow setting invididual components (DOFs) here, but
    // convert to quaternion and update mjData right before stepping in
    // SimulationFeatures.

    const Eigen::Vector3d * jointPos = getBallJointPositionImpl(jointInfo);
    if (jointPos)
    {
      return (*jointPos)[_dof];
    }
    return math::NAN_D;
  }
  return d->qpos[jointInfo->nq_index + _dof];
}

void setJointPositionImpl(JointInfo *jointInfo, std::size_t _dof, double _value)
{
  auto *d = jointInfo->worldInfo->mjDataObj;
  if (jointInfo->joint->type == mjtJoint::mjJNT_BALL)
  {
    // Ball joints have to be handled differently because the three DOFs exposed
    // in the public API actually represent the components of an Angle Axis
    // representation. We allow setting invididual components (DOFs) here, but
    // convert to quaternion and update mjData right before stepping in
    // SimulationFeatures.
    Eigen::Vector3d *jointPos = getBallJointPositionImpl(jointInfo);
    if (jointPos)
    {
      (*jointPos)[_dof] = _value;
      const double angle = jointPos->norm();
      if (math::equal(angle, 0.0, 1e-10))
      {
        copyQuat(Eigen::Quaterniond::Identity(), &d->qpos[jointInfo->nq_index]);
      }
      else
      {
        const Eigen::Quaterniond newQuat{
            Eigen::AngleAxisd{angle, *jointPos / angle}};
        copyQuat(newQuat, &d->qpos[jointInfo->nq_index]);
      }
    }
  }
  else
  {
    d->qpos[jointInfo->nq_index + _dof] = _value;
  }
}

void updateScrewJointFollower(
    JointInfo *_jointInfo, double _value, double *_mjDataArray, int _baseIndex)
{
  if (!_jointInfo->screwEqIndex.has_value())
    return;

  auto *m = _jointInfo->worldInfo->mjModelObj;
  const int eqId = *_jointInfo->screwEqIndex;
  const double multiplier = m->eq_data[eqId * mjNEQDATA + 1];

  // Since the primary rotational hinge and secondary translational slide
  // joints are created sequentially on the child body back-to-back in
  // SDFFeatures.cc, and MuJoCo compiles a body's joints contiguously in
  // memory, the follower's state index is guaranteed to be exactly
  // _baseIndex + 1.
  _mjDataArray[_baseIndex + 1] = _value * multiplier;
}

void updateMimicJointFollowers(JointInfo *_jointInfo, std::size_t _dof,
                               double _value, double *_mjDataArray,
                               bool _isQpos)
{
  auto *m = _jointInfo->worldInfo->mjModelObj;
  for (auto &constraint : _jointInfo->mimicConstraints)
  {
    if (constraint.eqId < 0 || constraint.eqId >= m->neq)
      continue;

    // Only update if this constraint targets the DOF/axis currently being set
    if (constraint.leaderDof != _dof)
      continue;

    auto followerInfo = constraint.followerJointInfo.lock();
    if (!followerInfo)
      continue;

    if (followerInfo->nq_index < 0 || followerInfo->nv_index < 0)
      continue;

    int followerAddr = _isQpos
        ? followerInfo->nq_index + constraint.followerDof
        : followerInfo->nv_index + constraint.followerDof;

    const double multiplier = m->eq_data[constraint.eqId * mjNEQDATA + 1];
    if (_isQpos)
    {
      const double offsetTerm = m->eq_data[constraint.eqId * mjNEQDATA + 0];
      _mjDataArray[followerAddr] = multiplier * _value + offsetTerm;
    }
    else
    {
      _mjDataArray[followerAddr] = _value * multiplier;
    }
  }
}

struct MimicConstraintSearchResult
{
  std::shared_ptr<JointInfo> leader{nullptr};
  std::size_t index{0};
  bool found{false};
};

/// \brief Search our C++ tracking structures to find an existing mimic joint
/// constraint relationship matching a given follower joint and active DOF.
/// \param[in] _worldInfo The metadata block of the active simulation world.
/// \param[in] _followerJoint Strong pointer targeting the follower JointInfo.
/// \param[in] _followerDof The degree-of-freedom of the follower axis.
/// \return A structured search result indicating if the constraint was
/// found, along with its leader joint pointer and its vector index.
MimicConstraintSearchResult findMimicConstraint(
    const WorldInfo &_worldInfo,
    const std::shared_ptr<JointInfo> &_followerJoint,
    std::size_t _followerDof)
{
  for (const auto &model : _worldInfo.models.idToObject)
  {
    for (const auto &joint : model.second->joints.idToObject)
    {
      const auto &jInfo = joint.second;
      for (std::size_t i = 0; i < jInfo->mimicConstraints.size(); ++i)
      {
        const auto &c = jInfo->mimicConstraints[i];
        if (c.followerDof == _followerDof &&
            c.followerJointInfo.lock() == _followerJoint)
        {
          return {jInfo, i, true};
        }
      }
    }
  }
  return {nullptr, 0, false};
}
}

/////////////////////////////////////////////////
double JointFeatures::GetJointPosition(
    const Identity &_id, std::size_t _dof) const
{
  auto jointInfo = this->ReferenceInterface<JointInfo>(_id);
  if (!jointInfo->joint)
  {
    gzerr << "Cannot get position for joint [" << jointInfo->name
          << "] because it is a fixed joint.\n";
    return math::NAN_D;
  }

  if (jointInfo->nq_index < 0)
    return math::NAN_D;

  if (!this->ValidateDofParam(_id, _dof))
  {
    return math::NAN_D;
  }
  return getJointPositionImpl(jointInfo, _dof);
}

/////////////////////////////////////////////////
double JointFeatures::GetJointVelocity(
    const Identity &_id, std::size_t _dof) const
{
  auto jointInfo = this->ReferenceInterface<JointInfo>(_id);
  if (!jointInfo->joint)
  {
    gzerr << "Cannot get velocity for joint [" << jointInfo->name
          << "] because it is a fixed joint.\n";
    return math::NAN_D;
  }

  if (jointInfo->nv_index < 0)
    return math::NAN_D;

  if (!this->ValidateDofParam(_id, _dof))
  {
    return math::NAN_D;
  }
  return jointInfo->worldInfo->mjDataObj->qvel[jointInfo->nv_index + _dof];
}

/////////////////////////////////////////////////
double JointFeatures::GetJointAcceleration(
    const Identity &_id, std::size_t _dof) const
{
  auto jointInfo = this->ReferenceInterface<JointInfo>(_id);
  if (!jointInfo->joint)
  {
    gzerr << "Cannot get acceleration for joint [" << jointInfo->name
          << "] because it is a fixed joint.\n";
    return math::NAN_D;
  }

  if (jointInfo->nv_index < 0)
    return math::NAN_D;

  if (!this->ValidateDofParam(_id, _dof))
  {
    return math::NAN_D;
  }
  return jointInfo->worldInfo->mjDataObj->qacc[jointInfo->nv_index + _dof];
}

/////////////////////////////////////////////////
double JointFeatures::GetJointForce(
    const Identity &_id, std::size_t _dof) const
{
  auto jointInfo = this->ReferenceInterface<JointInfo>(_id);

  if (!jointInfo->joint)
  {
    gzerr << "Cannot get force for joint [" << jointInfo->name
          << "] because it is a fixed joint.\n";
    return gz::math::NAN_D;
  }

  if (jointInfo->nv_index < 0)
    return math::NAN_D;

  if (!this->ValidateDofParam(_id, _dof))
  {
    return math::NAN_D;
  }
  return jointInfo->worldInfo->mjDataObj
      ->qfrc_actuator[jointInfo->nv_index + _dof];
}

/////////////////////////////////////////////////
Pose3d JointFeatures::GetJointTransform(const Identity &_id) const
{
  auto jointInfo = this->ReferenceInterface<JointInfo>(_id);
  if (!jointInfo->joint)
  {
    return this->GetJointTransformFromParent(_id) *
           this->GetJointTransformToChild(_id);
  }
  auto m = jointInfo->worldInfo->mjModelObj;
  int childBodyId = mjs_getId(jointInfo->childBody->element);
  if (childBodyId < 0 || childBodyId > m->nbody)
    return {};

  auto *parentBody = mjs_getParent(jointInfo->childBody->element);
  int parentBodyId = mjs_getId(parentBody ->element);

  auto d = jointInfo->worldInfo->mjDataObj;
  Eigen::Isometry3d parentPose =
      getBodyWorldPoseFromMjDataEigen(d, parentBodyId);
  Eigen::Isometry3d childPose = getBodyWorldPoseFromMjDataEigen(d, childBodyId);
  return parentPose.inverse() * childPose;
}

/////////////////////////////////////////////////
void JointFeatures::SetJointPosition(
    const Identity &_id, std::size_t _dof, double _value)
{
  auto jointInfo = this->ReferenceInterface<JointInfo>(_id);

  // Take extra care that the value is finite. A nan might cause the physics
  // engine to fail
  if (!std::isfinite(_value))
  {
    gzerr << "Invalid joint position value [" << _value << "] set on joint ["
           << jointInfo->name << " DOF " << _dof
           << "]. The value will be ignored\n";
    return;
  }
  if (!jointInfo->joint)
  {
    gzerr << "Cannot set position on joint [" << jointInfo->name
          << "] because it is a fixed joint.\n";
    return;
  }

  if (jointInfo->nq_index < 0)
    return;

  if (!this->ValidateDofParam(_id, _dof))
  {
    return;
  }
  setJointPositionImpl(jointInfo, _dof, _value);

  // If this is the primary rotational hinge joint of a screw joint,
  // simultaneously update the coupled secondary slide joint position (`qpos`)
  // by `_value * pitch`. This maintains perfect kinematic consistency and
  // prevents violent solver impulses during state initialization.
  updateScrewJointFollower(
      jointInfo, _value, jointInfo->worldInfo->mjDataObj->qpos,
      jointInfo->nq_index);
  updateMimicJointFollowers(jointInfo, _dof, _value,
                            jointInfo->worldInfo->mjDataObj->qpos, true);

  mj_forward(jointInfo->worldInfo->mjModelObj, jointInfo->worldInfo->mjDataObj);
}

/////////////////////////////////////////////////
void JointFeatures::SetJointVelocity(
    const Identity &_id, std::size_t _dof, double _value)
{
  auto jointInfo = this->ReferenceInterface<JointInfo>(_id);

  // Take extra care that the value is finite. A nan might cause the physics
  // engine to fail
  if (!std::isfinite(_value))
  {
    gzerr << "Invalid joint velocity value [" << _value << "] set on joint ["
           << jointInfo->name << " DOF " << _dof
           << "]. The value will be ignored\n";
    return;
  }
  if (!jointInfo->joint)
  {
    gzerr << "Cannot set velocity on joint [" << jointInfo->name
          << "] because it is a fixed joint.\n";
    return;
  }

  if (jointInfo->nv_index < 0)
    return;

  if (!this->ValidateDofParam(_id, _dof))
  {
    return;
  }
  jointInfo->worldInfo->mjDataObj->qvel[jointInfo->nv_index + _dof] = _value;

  // If this is the primary rotational hinge joint of a screw joint,
  // simultaneously update the coupled secondary slide joint velocity (`qvel`)
  // by `_value * pitch`. This maintains perfect kinematic consistency and
  // prevents violent solver impulses during state initialization.
  updateScrewJointFollower(jointInfo, _value,
                           jointInfo->worldInfo->mjDataObj->qvel,
                           jointInfo->nv_index);
  updateMimicJointFollowers(jointInfo, _dof, _value,
                            jointInfo->worldInfo->mjDataObj->qvel, false);

  mj_forward(jointInfo->worldInfo->mjModelObj, jointInfo->worldInfo->mjDataObj);
}

/////////////////////////////////////////////////
void JointFeatures::SetJointAcceleration(
    const Identity &/* _id */, std::size_t /* _dof */, double /* _value */)
{
  gzerr << "Setting joint acceleration is not supported by the MuJoCo physics "
           "plugin.\n";
}

/////////////////////////////////////////////////
void JointFeatures::SetJointForce(
    const Identity &_id, std::size_t _dof, double _value)
{
  auto jointInfo = this->ReferenceInterface<JointInfo>(_id);

  // Take extra care that the value is finite. A nan might cause the physics
  // engine to fail
  if (!std::isfinite(_value))
  {
    gzerr << "Invalid joint force value [" << _value << "] set on joint ["
           << jointInfo->name << " DOF " << _dof
           << "]. The value will be ignored\n";
    return;
  }
  if (!jointInfo->joint)
  {
    gzerr << "Cannot set force on joint [" << jointInfo->name
          << "] because it is a fixed joint.\n";
    return;
  }

  if (!jointInfo->actuator)
  {
    gzerr << "No actuator set up for this joint\n";
    return;
  }
  int ctrlIndex = mjs_getId(jointInfo->actuator->element);
  if (ctrlIndex < 0)
    return;

  if (!this->ValidateDofParam(_id, _dof))
  {
    return;
  }
  jointInfo->worldInfo->mjDataObj->ctrl[ctrlIndex + _dof] = _value;
  mj_forward(jointInfo->worldInfo->mjModelObj, jointInfo->worldInfo->mjDataObj);
}

/////////////////////////////////////////////////
bool JointFeatures::ValidateDofParam(const Identity &_id,
                                     std::size_t _dof) const
{
  if (_dof >= this->GetJointDegreesOfFreedom(_id))
  {
    auto jointInfo = this->ReferenceInterface<JointInfo>(_id);
    gzerr << "Trying to access an invalid DOF [" << _dof << "] on joint [ "
          << jointInfo->name << "]\n";
    return false;
  }
  return true;
}

/////////////////////////////////////////////////
std::size_t JointFeatures::GetJointDegreesOfFreedom(const Identity &_id) const
{
  auto jointInfo = this->ReferenceInterface<JointInfo>(_id);

  if (!jointInfo->joint)
  {
    return 0;
  }
  auto m = jointInfo->worldInfo->mjModelObj;
  int bodyId = mjs_getId(jointInfo->childBody->element);
  if (bodyId < 0 || bodyId >= m->nbody)
  {
    // The joint is not yet compiled in the active model. Count the DOFs
    // statically from the spec DOM elements!
    std::size_t dofs = 0;
    mjsElement* elem = mjs_firstChild(
        jointInfo->childBody, mjOBJ_JOINT, 0);
    while (elem)
    {
      mjsJoint* j = mjs_asJoint(elem);
      if (j)
      {
        if (j->type == mjJNT_BALL)
        {
          dofs += 3;
        }
        else if (j->type == mjJNT_FREE)
        {
          dofs += 6;
        }
        else
        {
          dofs += 1;
        }
      }
      elem = mjs_nextChild(jointInfo->childBody, elem, 0);
    }
    // For screw joints, the sliding axis is coupled, so subtract 1:
    if (jointInfo->screwConstraintSpec != nullptr)
    {
      dofs -= 1;
    }
    return dofs;
  }

  std::size_t dofs = m->body_dofnum[bodyId];
  if (jointInfo->screwEqIndex.has_value())
  {
    dofs -= 1;
  }
  return dofs;
}

/////////////////////////////////////////////////
Pose3d JointFeatures::GetJointTransformFromParent(const Identity &_id) const
{
  auto jointInfo = this->ReferenceInterface<JointInfo>(_id);
  auto it = this->frames.find(_id);
  if (it != this->frames.end())
  {
    Pose3d childInParent =
        convertPose(jointInfo->childBody->pos, jointInfo->childBody->quat);
    Pose3d jointInChild =
        convertPose(it->second->site->pos, it->second->site->quat);

    return childInParent * jointInChild;
  }
  return {};
}

/////////////////////////////////////////////////
Pose3d JointFeatures::GetJointTransformToChild(const Identity &_id) const
{
  auto it = this->frames.find(_id.id);
  if (it != this->frames.end())
  {
    Pose3d jointInChild =
        convertPose(it->second->site->pos, it->second->site->quat);
    return jointInChild.inverse();
  }
  return {};
}

/////////////////////////////////////////////////
bool JointFeatures::SetJointMimicConstraint(
    const Identity &_id,
    std::size_t _dof,
    const BaseJoint3dPtr &_leaderJoint,
    std::size_t _leaderAxisDof,
    double _multiplier,
    double _offset,
    double _reference)
{
  auto followerJointShared =
      std::static_pointer_cast<JointInfo>(this->Reference(_id));
  auto followerJointInfo = followerJointShared.get();
  if (!followerJointInfo->joint)
  {
    gzerr << "Cannot set mimic constraint on joint [" << followerJointInfo->name
          << "] because it is a fixed joint.\n";
    return false;
  }

  if (followerJointInfo->joint->type == mjJNT_BALL)
  {
    gzerr << "Cannot set mimic constraint on joint ["
          << followerJointInfo->name << "] because it is a ball joint.\n";
    return false;
  }

  if (!this->ValidateDofParam(_id, _dof))
  {
    return false;
  }

  auto leaderJointId = _leaderJoint->FullIdentity();
  auto leaderJointShared =
      std::static_pointer_cast<JointInfo>(this->Reference(leaderJointId));
  auto leaderJointInfo = leaderJointShared.get();
  if (!leaderJointInfo->joint)
  {
    gzerr << "Cannot mimic joint [" << leaderJointInfo->name
          << "] because it is a fixed joint.\n";
    return false;
  }

  if (leaderJointInfo->joint->type == mjJNT_BALL)
  {
    gzerr << "Cannot mimic joint [" << leaderJointInfo->name
          << "] because it is a ball joint.\n";
    return false;
  }

  if (_leaderAxisDof >= this->GetJointDegreesOfFreedom(leaderJointId))
  {
    gzerr << "Trying to access an invalid leader DOF [ " << _leaderAxisDof
          << "] on joint [ " << leaderJointInfo->name << "]\n";
    return false;
  }

  auto followerModel = followerJointInfo->modelInfo.lock();
  auto leaderModel = leaderJointInfo->modelInfo.lock();
  auto worldInfo = followerJointInfo->worldInfo;

  // Search the leader-to-followers tracking vector lists in C++ memory
  // to locate if a mimic constraint already exists for this follower
  // joint axis, preparing it for safe de-duplication/overwriting.
  const auto searchResult =
      findMimicConstraint(*worldInfo, followerJointShared, _dof);

  // Unconditionally delete the old constraint if found
  if (searchResult.found)
  {
    // Delete the spec node from MuJoCo's DOM tree
    mjs_delete(
        worldInfo->mjSpecObj,
        searchResult.leader->mimicConstraints[searchResult.index]
            .spec->element);

    // Erase the record from the old leader's list
    searchResult.leader->mimicConstraints.erase(
        searchResult.leader->mimicConstraints.begin() + searchResult.index);
  }

  const std::string followerName = getJointAxisName(
      ::sdf::JoinName(followerModel->name, followerJointInfo->name), _dof);

  const std::string leaderName = getJointAxisName(
      ::sdf::JoinName(leaderModel->name, leaderJointInfo->name),
      _leaderAxisDof);

  // Recall the linear equation from the definition of the mimic constraint:
  // follower_pos = multiplier * (leader_pos - reference) + offset
  //
  // MuJoCo's official joint equality constraint polynomial is:
  // y - y0 = a0 + a1*(x - x0) + a2*(x - x0)^2 + a3*(x - x0)^3 + a4*(x - x0)^4
  // where:
  // - y is the follower joint position (follower_pos)
  // - y0 is the follower home/resting position (qpos0_follower)
  // - x is the leader joint position (leader_pos)
  // - x0 is the leader home/resting position (qpos0_leader)
  //
  // We only need a linear relationship, so we set a2 = a3 = a4 = 0, leaving:
  // follower_pos - qpos0_follower = a0 + a1 * (leader_pos - qpos0_leader)
  //
  // In our MuJoCo plugin, since all standard 1-DOF joints are created with a
  // zero home configuration (qpos0 = 0), we can substitute qpos0_follower = 0
  // and qpos0_leader = 0, leaving:
  // follower_pos = a0 + a1 * leader_pos
  //
  // Equating the linear mimic equation and the simplified MuJoCo solver
  // equation, we solve for the MuJoCo parameters:
  // 1. a1 (multiplier) = multiplier (saved as eqSpec->data[1])
  // 2. a0 (offsetTerm) = offset - multiplier * reference
  //                      (saved as eqSpec->data[0])
  const double offsetTerm = _offset - _multiplier * _reference;

  mjsEquality* eqSpec = mjs_addEquality(worldInfo->mjSpecObj, nullptr);
  eqSpec->type = mjEQ_JOINT;
  eqSpec->active = 1;
  mjs_setString(eqSpec->name1, followerName.c_str());
  mjs_setString(eqSpec->name2, leaderName.c_str());

  std::fill(std::begin(eqSpec->data), std::end(eqSpec->data), 0.0);
  eqSpec->data[0] = offsetTerm;
  eqSpec->data[1] = _multiplier;

  // Set solver reference parameters (time constant and damping ratio):
  // - solref[0] = 0.001 (1 ms) defines a stiff, highly responsive constraint
  //   to minimize kinematic tracking error.
  // - solref[1] = 1.0 (critical damping) prevents spring-like coordinate
  //   oscillations during constraint corrections.
  eqSpec->solref[0] = 0.001;
  eqSpec->solref[1] = 1.0;

  MimicConstraintInfo constraint;
  constraint.spec = eqSpec;
  constraint.eqId = -1;
  constraint.leaderDof = _leaderAxisDof;
  constraint.followerJointInfo = followerJointShared;
  constraint.followerDof = _dof;

  leaderJointInfo->mimicConstraints.push_back(constraint);

  worldInfo->specDirty = true;
  this->RecompileSpec(*worldInfo);
  return true;
}
void JointFeatures::SetJointTransformFromParent(const Identity &_id,
                                                const Pose3d &_pose)
{
  auto jointInfo = this->ReferenceInterface<JointInfo>(_id);
  if (!jointInfo)
  {
    gzerr << "Failed to find JointInfo when setting transform from parent.\n";
    return;
  }

  if (jointInfo->weldConstraintSpec)
  {
    auto weldSpec = jointInfo->weldConstraintSpec;

    // Note: The memory layout of `weldSpec->data` (mjNEQDATA=11 elements) for
    // mjEQ_WELD is:
    // data[0..2]: anchor (3 elements)
    // data[3..9]: relpose (7 elements)
    // data[10]: torquescale (1 element)

    // We choose the anchor point to be the origin of the child link.
    // Therefore, the anchor in the child frame (data[3..5]) is (0, 0, 0).
    std::fill(weldSpec->data + 3, weldSpec->data + 6, 0.0);

    // The anchor in the parent frame (data[0..2]) is the origin of the child
    // link expressed in the parent frame, which is _pose.translation()
    copyPos(_pose.translation(), weldSpec->data);

    // The relative orientation (data[6..9]) is the parent orientation in the
    // child frame, which is exactly _pose.inverse().rotation()
    copyQuat(Eigen::Quaterniond(_pose.inverse().rotation()),
             weldSpec->data + 6);

    auto m = jointInfo->worldInfo->mjModelObj;
    if (m)
    {
      int eqId = jointInfo->weldEqIndex.value_or(mjs_getId(weldSpec->element));
      if (eqId >= 0 && eqId < m->neq)
      {
        jointInfo->weldEqIndex = eqId;
        double *eqData = m->eq_data + eqId * mjNEQDATA;
        std::copy(weldSpec->data, weldSpec->data + mjNEQDATA, eqData);
      }
    }
  }
  else
  {
    auto it = this->frames.find(_id);
    if (it != this->frames.end())
    {
      const Pose3d jointInChild =
          convertPose(it->second->site->pos, it->second->site->quat);
      const Pose3d childInParent = _pose * jointInChild.inverse();

      copyPos(childInParent.translation(), jointInfo->childBody->pos);
      copyQuat(Eigen::Quaterniond(childInParent.linear()),
               jointInfo->childBody->quat);

      jointInfo->worldInfo->specDirty = true;
    }
    else
    {
      gzerr << "Failed to find FrameInfo when setting transform from parent.\n";
    }
  }
}

/////////////////////////////////////////////////
void JointFeatures::DetachJoint(const Identity &_jointId)
{
  auto jointInfo = this->ReferenceInterface<JointInfo>(_jointId);
  if (!jointInfo)
  {
    gzerr << "Failed to find JointInfo when detaching joint.\n";
    return;
  }

  auto worldInfo = jointInfo->worldInfo;
  auto modelInfo = jointInfo->modelInfo.lock();
  if (!worldInfo || !modelInfo)
  {
    gzerr << "Invalid worldInfo or modelInfo when detaching joint.\n";
    return;
  }

  if (jointInfo->weldConstraintSpec)
  {
    jointInfo->weldConstraintSpec->active = 0;
    if (jointInfo->weldEqIndex && worldInfo->mjDataObj)
    {
      worldInfo->mjDataObj->eq_active[jointInfo->weldEqIndex.value()] = 0;
    }
    jointInfo->weldConstraintSpec = nullptr;

    if (this->frames.find(jointInfo->entityId) != this->frames.end())
    {
      this->frames.erase(jointInfo->entityId);
    }

    modelInfo->joints.RemoveEntity(jointInfo->name);
  }
}

/////////////////////////////////////////////////
Identity JointFeatures::CastToFixedJoint(const Identity &_jointID) const
{
  auto jointInfo = this->ReferenceInterface<JointInfo>(_jointID);
  if (jointInfo && jointInfo->weldConstraintSpec)
  {
    return _jointID;
  }
  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
// Performance Optimization for AttachFixedJoint & DetachJoint:
// Instead of creating and deleting weld constraints via the mjSpec API (which
// triggers an expensive full model recompilation `mj_compile` each time a
// joint is attached or detached), we implement constraint caching/reuse.
//
// In DetachJoint, we simply deactivate the constraint (`active = 0`).
// In AttachFixedJoint, we first use `mjs_findElement` to efficiently look up
// an existing inactive weld constraint by its name. If found, we reuse it by
// setting `active = 1` in both the spec and live mjData, updating bodies if
// necessary.
// This avoids recompilation entirely for repetitive attach/detach operations.
Identity JointFeatures::AttachFixedJoint(
    const Identity &_childID,
    const BaseLink3dPtr &_parent,
    const std::string &_name)
{
  auto childLinkInfo = this->ReferenceInterface<LinkInfo>(_childID);
  if (!childLinkInfo)
  {
    gzerr << "Failed to find LinkInfo for child entity when attaching fixed "
          << "joint.\n";
    return this->GenerateInvalidId();
  }

  LinkInfo* parentLinkInfo = nullptr;
  if (_parent)
  {
    parentLinkInfo =
      this->ReferenceInterface<LinkInfo>(_parent->FullIdentity());
    if (!parentLinkInfo)
    {
      gzerr << "Failed to find LinkInfo for parent entity when attaching "
            << "fixed joint.\n";
      return this->GenerateInvalidId();
    }
  }

  auto worldInfo = childLinkInfo->worldInfo;
  auto modelInfo = childLinkInfo->modelInfo.lock();
  if (!worldInfo || !modelInfo)
  {
    gzerr << "Invalid worldInfo or modelInfo when attaching fixed joint.\n";
    return this->GenerateInvalidId();
  }
  if (modelInfo->joints.HasEntity(_name))
  {
    gzerr << "There's already a joint with the same name [ " << _name
          << " ].\n";
    return this->GenerateInvalidId();
  }

  const char* childBodyName
      = mjs_getString(mjs_getName(childLinkInfo->body->element));
  const char* parentBodyName
      = parentLinkInfo
            ? mjs_getString(mjs_getName(parentLinkInfo->body->element))
            : "";

  const std::string mjJointName = ::sdf::JoinName(modelInfo->name, _name);

  mjsEquality *eqSpec = mjs_asEquality(mjs_findElement(
      worldInfo->mjSpecObj, mjOBJ_EQUALITY, mjJointName.c_str()));

  bool requireRecompile = false;
  if (eqSpec)
  {
    if (eqSpec->type != mjEQ_WELD)
    {
      gzerr
          << "Found a matching equality constraint, but with incorrect type.\n";
      return this->GenerateInvalidId();
    }
    // If the bodies changed, update the constraint with the new bodies and
    // force a recompile.
    // We assume the previous fixed joint with the same name has been detached
    // (i.e. removed) since the HasEntity check above would have failed
    // otherwise.
    const char *n1 = mjs_getString(eqSpec->name1);
    const char *n2 = mjs_getString(eqSpec->name2);
    if (!n1 || !n2 || std::string_view(childBodyName) != n1 ||
        std::string_view(parentBodyName) != n2)
    {
      mjs_setString(eqSpec->name1, childBodyName);
      mjs_setString(eqSpec->name2, parentBodyName);
      requireRecompile = true;
    }
  }
  if (!eqSpec)
  {
    eqSpec = mjs_addEquality(worldInfo->mjSpecObj, nullptr);
    if (!eqSpec)
    {
      gzerr << "Failed to add equality constraint element to the spec.\n";
      return this->GenerateInvalidId();
    }
    mjs_setName(eqSpec->element, mjJointName.c_str());
    eqSpec->type = mjEQ_WELD;
    eqSpec->objtype = mjOBJ_BODY;
    mjs_setString(eqSpec->name1, childBodyName);
    mjs_setString(eqSpec->name2, parentBodyName);

    // Stiffen the constraint solver parameters to make the weld constraint
    // virtually rigid
    eqSpec->solref[0] = 0.0001;  // 100 microseconds time constant
    eqSpec->solimp[0] = 0.999;   // highly rigid compliance bounds
    eqSpec->solimp[1] = 0.9999;

    // Fill data with zeros so MuJoCo computes initial relative pose at
    // compilation. Note that the memory layout for mjEQ_WELD is:
    // data[0..2] = anchor, data[3..9] = relpose, data[10] = torquescale.
    std::fill(std::begin(eqSpec->data), std::end(eqSpec->data), 0.0);
    // Default value for torquescale
    eqSpec->data[10] = 1.0;

    requireRecompile = true;
  }

  auto jointInfo =
    std::make_shared<JointInfo>(this->GetNextEntity(), modelInfo);
  jointInfo->name = _name;
  jointInfo->childBody = childLinkInfo->body;
  jointInfo->worldInfo = worldInfo;
  jointInfo->weldConstraintSpec = eqSpec;

  eqSpec->active = 1;

  if (!requireRecompile && worldInfo->mjModelObj)
  {
    int eqId = mjs_getId(eqSpec->element);
    if (eqId >= 0 && eqId < worldInfo->mjModelObj->neq)
    {
      jointInfo->weldEqIndex = eqId;
      worldInfo->mjDataObj->eq_active[eqId] = 1;
    }
  }

  auto childFrameIt = this->frames.find(childLinkInfo->entityId);
  if (childFrameIt != this->frames.end())
  {
    this->frames[jointInfo->entityId] = childFrameIt->second;
  }

  modelInfo->joints.AddEntity(
    jointInfo->entityId, jointInfo, jointInfo->name, modelInfo->entityId);

  if (requireRecompile)
  {
    worldInfo->specDirty = true;
  }

  return this->GenerateIdentity(jointInfo->entityId, jointInfo);
}

}  // namespace mujoco
}  // namespace physics
}  // namespace gz
