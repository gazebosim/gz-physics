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

#include <ignition/physics/Joint.hh>

#include "Base.hh"

namespace ignition {
namespace physics {
namespace dartsim {

using JointFeatureList = FeatureList<
  GetBasicJointState,
  SetBasicJointState,
  GetBasicJointProperties,
  SetJointTransformFromParentFeature,
  SetJointTransformToChildFeature
>;

class JointFeatures :
    public virtual Base,
    public virtual Implements3d<JointFeatureList>
{
  public: double GetJointPosition(
      const std::size_t _id, const std::size_t _dof) const override;

  public: double GetJointVelocity(
      const std::size_t _id, const std::size_t _dof) const override;

  public: double GetJointAcceleration(
      const std::size_t _id, const std::size_t _dof) const override;

  public: double GetJointForce(
      const std::size_t _id, const std::size_t _dof) const override;

  public: Pose3d GetJointTransform(const std::size_t _id) const override;

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

  public: std::size_t GetJointDegreesOfFreedom(
      const std::size_t _id) const override;

  public: Pose3d GetJointTransformFromParent(
      const std::size_t _id) const override;

  public: Pose3d GetJointTransformToChild(
      const std::size_t _id) const override;

  public: void SetJointTransformFromParent(
      const std::size_t _id, const Pose3d &_pose) override;

  public: void SetJointTransformToChild(
      const std::size_t _id, const Pose3d &_pose) override;
};

}
}
}

#endif
