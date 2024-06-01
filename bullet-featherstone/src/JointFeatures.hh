/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#ifndef GZ_PHYSICS_BULLET_FEATHERSTONE_SRC_JOINTFEATURES_HH_
#define GZ_PHYSICS_BULLET_FEATHERSTONE_SRC_JOINTFEATURES_HH_

#include <string>

#include <gz/physics/FixedJoint.hh>
#include <gz/physics/Joint.hh>
#include <gz/physics/PrismaticJoint.hh>
#include <gz/physics/RevoluteJoint.hh>

#include "Base.hh"

namespace gz {
namespace physics {
namespace bullet_featherstone {

struct JointFeatureList : FeatureList<
  GetBasicJointState,
  SetBasicJointState,
  GetBasicJointProperties,

  SetJointVelocityCommandFeature,

  SetJointTransformFromParentFeature,
  AttachFixedJointFeature,
  DetachJointFeature,

  GetRevoluteJointProperties,
  GetPrismaticJointProperties,

  FixedJointCast,

  GetJointTransmittedWrench,

  SetMimicConstraintFeature
> { };

class JointFeatures :
    public virtual Base,
    public virtual Implements3d<JointFeatureList>
{
  // ----- Get Basic Joint State -----
  public: double GetJointPosition(
      const Identity &_id, const std::size_t _dof) const override;

  public: double GetJointVelocity(
      const Identity &_id, const std::size_t _dof) const override;

  public: double GetJointAcceleration(
      const Identity &_id, const std::size_t _dof) const override;

  public: double GetJointForce(
      const Identity &_id, const std::size_t _dof) const override;

  public: Pose3d GetJointTransform(const Identity &_id) const override;

  // ----- Set Basic Joint State -----
  public: void SetJointPosition(
      const Identity &_id, const std::size_t _dof,
      const double _value) override;

  public: void SetJointVelocity(
      const Identity &_id, const std::size_t _dof,
      const double _value) override;

  public: void SetJointAcceleration(
      const Identity &_id, const std::size_t _dof,
      const double _value) override;

  public: void SetJointForce(
      const Identity &_id, const std::size_t _dof,
      const double _value) override;

  // ----- Get Basic Joint Properties -----
  public: std::size_t GetJointDegreesOfFreedom(
      const Identity &_id) const override;

  public: Pose3d GetJointTransformFromParent(
      const Identity &_id) const override;

  public: Pose3d GetJointTransformToChild(
      const Identity &_id) const override;

  // ----- Fixed Joint -----
  public: Identity CastToFixedJoint(
      const Identity &_jointID) const override;

  // ----- Revolute Joint -----
  public: Identity CastToRevoluteJoint(
      const Identity &_jointID) const override;

  public: AngularVector3d GetRevoluteJointAxis(
      const Identity &_jointID) const override;

  // ----- Prismatic Joint -----
  public: Identity CastToPrismaticJoint(
      const Identity &_jointID) const override;

  public: Eigen::Vector3d GetPrismaticJointAxis(
      const Identity &_jointID) const override;

  public: Identity CastToJointType(
      const Identity &_jointID,
      btMultibodyLink::eFeatherstoneJointType type) const;

  // ----- Joint Commands -----
  public: void SetJointVelocityCommand(
    const Identity &_id, const std::size_t _dof,
    const double _value) override;

  // ----- AttachFixedJointFeature -----
  public: Identity AttachFixedJoint(
      const Identity &_childID,
      const BaseLink3dPtr &_parent,
      const std::string &_name) override;

  // ----- Detach Joint -----
  public: void DetachJoint(const Identity &_jointId) override;

  // ----- Set Basic Joint Properties -----
  public: void SetJointTransformFromParent(
      const Identity &_id, const Pose3d &_pose) override;

  // ----- Transmitted wrench -----
  public: Wrench3d GetJointTransmittedWrenchInJointFrame(
      const Identity &_id) const override;

  // ----- Mimic joint constraint -----
  public: bool SetJointMimicConstraint(
      const Identity &_id,
      std::size_t _dof,
      const BaseJoint3dPtr &_leaderJoint,
      std::size_t _leaderAxisDof,
      double _multiplier,
      double _offset,
      double _reference) override;
};
}  // namespace bullet_featherstone
}  // namespace physics
}  // namespace gz

#endif
