/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#ifndef IGNITION_PHYSICS_BULLET_SRC_JOINTFEATURES_HH_
#define IGNITION_PHYSICS_BULLET_SRC_JOINTFEATURES_HH_

#include <string>

#include <ignition/physics/Joint.hh>
#include <ignition/physics/FixedJoint.hh>
#include <ignition/physics/PrismaticJoint.hh>
#include <ignition/physics/RevoluteJoint.hh>

#include "Base.hh"

namespace ignition {
namespace physics {
namespace bullet {

struct JointFeatureList : FeatureList<
  GetBasicJointState,
  SetBasicJointState,
  GetBasicJointProperties,

  GetRevoluteJointProperties,
  FixedJointCast,
  SetJointVelocityCommandFeature
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

  // ----- Joint Commands -----
  public: void SetJointVelocityCommand(
      const Identity &_id, const std::size_t _dof,
      const double _value) override;
};

}  // namespace bullet
}  // namespace physics
}  // namespace ignition

#endif
