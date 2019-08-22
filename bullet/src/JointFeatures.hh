/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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
#include <ignition/physics/FreeJoint.hh>
#include <ignition/physics/PrismaticJoint.hh>
#include <ignition/physics/RevoluteJoint.hh>

#include "Base.hh"

namespace ignition {
namespace physics {
namespace bullet {

using JointFeatureList = FeatureList<
  GetBasicJointState,
  SetBasicJointState,
  GetBasicJointProperties
>;

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
      const Identity &/* _id */, const std::size_t /* _dof */) const override
      { return 0; };

  public: double GetJointForce(
      const Identity &_id, const std::size_t _dof) const override;

  public: Pose3d GetJointTransform(const Identity &/* _id */) const override
      { return Pose3d(); };


  // ----- Set Basic Joint State -----
  public: void SetJointPosition(
      const Identity &/* _id */, const std::size_t /* _dof */,
      const double /* _value */) override
      { };

  public: void SetJointVelocity(
      const Identity &_id, const std::size_t _dof,
      const double _value) override;

  public: void SetJointAcceleration(
      const Identity &/* _id */, const std::size_t /* _dof */,
      const double /* _value */) override
      { };

  public: void SetJointForce(
      const Identity &_id, const std::size_t _dof,
      const double _value) override;


  // ----- Get Basic Joint Properties -----
  public: std::size_t GetJointDegreesOfFreedom(
      const Identity &_id) const override;

  public: Pose3d GetJointTransformFromParent(
      const Identity &/* _id */) const override
      { return Pose3d(); };

  public: Pose3d GetJointTransformToChild(
      const Identity &/* _id */) const override
      { return Pose3d(); };
};

}
}
}

#endif
