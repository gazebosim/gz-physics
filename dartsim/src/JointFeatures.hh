/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

#ifndef IGNITION_PHYSICS_DARTSIM_SRC_JOINTFEATURES_HH_
#define IGNITION_PHYSICS_DARTSIM_SRC_JOINTFEATURES_HH_

#include <string>

#include <ignition/physics/Joint.hh>
#include <ignition/physics/FixedJoint.hh>
#include <ignition/physics/FreeJoint.hh>
#include <ignition/physics/PrismaticJoint.hh>
#include <ignition/physics/RevoluteJoint.hh>

#include "Base.hh"

namespace ignition {
namespace physics {
namespace dartsim {

using JointFeatureList = FeatureList<
  GetBasicJointState,
  SetBasicJointState,
  GetBasicJointProperties,
  SetJointTransformFromParentFeature,
  SetJointTransformToChildFeature,

  SetFreeJointRelativeTransformFeature,

  AttachFixedJointFeature,

  SetRevoluteJointProperties,
  GetRevoluteJointProperties,
  AttachRevoluteJointFeature,

  SetPrismaticJointProperties,
  GetPrismaticJointProperties,
  AttachPrismaticJointFeature
>;

class JointFeatures :
    public virtual Base,
    public virtual Implements3d<JointFeatureList>
{
  // ----- Get Basic Joint State -----
  public: double GetJointPosition(
      const std::size_t _id, const std::size_t _dof) const override;

  public: double GetJointVelocity(
      const std::size_t _id, const std::size_t _dof) const override;

  public: double GetJointAcceleration(
      const std::size_t _id, const std::size_t _dof) const override;

  public: double GetJointForce(
      const std::size_t _id, const std::size_t _dof) const override;

  public: Pose3d GetJointTransform(const std::size_t _id) const override;


  // ----- Set Basic Joint State -----
  public: void SetJointPosition(
      const std::size_t _id, const std::size_t _dof,
      const double _value) override;

  public: void SetJointVelocity(
      const std::size_t _id, const std::size_t _dof,
      const double _value) override;

  public: void SetJointAcceleration(
      const std::size_t _id, const std::size_t _dof,
      const double _value) override;

  public: void SetJointForce(
      const std::size_t _id, const std::size_t _dof,
      const double _value) override;


  // ----- Get Basic Joint Properties -----
  public: std::size_t GetJointDegreesOfFreedom(
      const std::size_t _id) const override;

  public: Pose3d GetJointTransformFromParent(
      const std::size_t _id) const override;

  public: Pose3d GetJointTransformToChild(
      const std::size_t _id) const override;


  // ----- Set Basic Joint Properties -----
  public: void SetJointTransformFromParent(
      const std::size_t _id, const Pose3d &_pose) override;

  public: void SetJointTransformToChild(
      const std::size_t _id, const Pose3d &_pose) override;


  // ----- Fixed Joint -----
  public: Identity CastToFixedJoint(
      const std::size_t _jointID) const override;

  public: Identity AttachFixedJoint(
      std::size_t _childID,
      const BaseLink3dPtr &_parent,
      const std::string &_name) override;


  // ----- Free Joint -----
  public: Identity CastToFreeJoint(
      const std::size_t _jointID) const override;

  public: void SetFreeJointRelativeTransform(
      const std::size_t _jointID, const Pose3d &_pose) override;


  // ----- Revolute Joint -----
  public: Identity CastToRevoluteJoint(
      const std::size_t _jointID) const override;

  public: AngularVector3d GetRevoluteJointAxis(
      const std::size_t _jointID) const override;

  public: void SetRevoluteJointAxis(
      const std::size_t _jointID, const AngularVector3d &_axis) override;

  public: Identity AttachRevoluteJoint(
      const std::size_t _childID,
      const BaseLink3dPtr &_parent,
      const std::string &_name,
      const AngularVector3d &_axis) override;


  // ----- Prismatic Joint -----
  public: Identity CastToPrismaticJoint(
      const std::size_t _jointID) const override;

  public: LinearVector3d GetPrismaticJointAxis(
      const std::size_t _jointID) const override;

  public: void SetPrismaticJointAxis(
      const std::size_t _jointID, const LinearVector3d &_axis) override;

  public: Identity AttachPrismaticJoint(
      const std::size_t _childID,
      const BaseLink3dPtr &_parent,
      const std::string &_name,
      const LinearVector3d &_axis) override;
};

}
}
}

#endif
