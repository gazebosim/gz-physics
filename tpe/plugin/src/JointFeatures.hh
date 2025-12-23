/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#ifndef GZ_PHYSICS_TPE_PLUGIN_SRC_JOINTFEATURES_HH_
#define GZ_PHYSICS_TPE_PLUGIN_SRC_JOINTFEATURES_HH_

#include <string>
#include <gz/physics/FixedJoint.hh>
#include <gz/physics/GetEntities.hh>
#include <gz/physics/Joint.hh>
#include "Base.hh"

namespace gz {
namespace physics {
namespace tpeplugin {

struct JointFeatureList : FeatureList<
  GetJointFromModel, GetBasicJointProperties, GetBasicJointState,
  SetBasicJointState, SetJointTransformFromParentFeature,
  DetachJointFeature, AttachFixedJointFeature
> { };

class JointFeatures :
    public virtual Base,
    public virtual Implements3d<JointFeatureList>
{
  public: std::size_t GetJointCount(const Identity &_modelID) const override;
  public: Identity GetJoint(const Identity &_modelID, std::size_t _jointIndex) const override;
  public: Identity GetJoint(const Identity &_modelID, const std::string &_jointName) const override;
  public: const std::string &GetJointName(const Identity &_jointID) const override;
  public: std::size_t GetJointIndex(const Identity &_jointID) const override;
  public: Identity GetModelOfJoint(const Identity &_jointID) const override;

  public: std::size_t GetJointDegreesOfFreedom(const Identity &_id) const override;
  public: Pose3d GetJointTransformFromParent(const Identity &_id) const override;
  public: Pose3d GetJointTransformToChild(const Identity &_id) const override;

  public: double GetJointPosition(const Identity &_id, std::size_t _dof) const override;
  public: double GetJointVelocity(const Identity &_id, std::size_t _dof) const override;
  public: double GetJointAcceleration(const Identity &_id, std::size_t _dof) const override;
  public: double GetJointForce(const Identity &_id, std::size_t _dof) const override;
  public: Pose3d GetJointTransform(const Identity &_id) const override;

  public: void SetJointPosition(const Identity &_id, std::size_t _dof, double _value) override;
  public: void SetJointVelocity(const Identity &_id, std::size_t _dof, double _value) override;
  public: void SetJointAcceleration(const Identity &_id, std::size_t _dof, double _value) override;
  public: void SetJointForce(const Identity &_id, std::size_t _dof, double _value) override;

  public: void SetJointTransformFromParent(const Identity &_id, const Pose3d &_pose) override;
  public: void DetachJoint(const Identity &_jointId) override;
  public: Identity CastToFixedJoint(const Identity &_jointID) const override;
  public: Identity AttachFixedJoint(const Identity &_childID, const BaseLink3dPtr &_parent, const std::string &_name) override;
};

}  // namespace tpeplugin
}  // namespace physics
}  // namespace gz

#endif
