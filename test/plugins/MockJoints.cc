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

#include <unordered_map>
#include <vector>

#include <ignition/physics/Implements.hh>

#include "../MockJoints.hh"

using namespace ignition::physics;

namespace mock
{
  template <typename PolicyT>
  class JointPlugin : public Implements<PolicyT, MockJointList>
  {
    using Scalar = typename PolicyT::Scalar;

    using RevoluteAxis =
        typename FromPolicy<PolicyT>::template Use<AngularVector>;

    using Pose =
        typename FromPolicy<PolicyT>::template Use<Pose>;

    using FrameData =
        typename FromPolicy<PolicyT>::template Use<FrameData>;

    using LinearVector =
        typename FromPolicy<PolicyT>::template Use<LinearVector>;

    using VectorX = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;

    struct JointState
    {
      VectorX position;
      VectorX velocity;
      VectorX acceleration;
      VectorX force;
    };

    using IdToJointState = std::unordered_map<std::size_t, JointState>;

    IdToJointState idToJointState;

    struct JointProperties
    {
      Pose parentLinkToJoint;
      Pose jointToChildLink;

      FrameID parentJointID;
    };

    using IdToJointProperties =
        std::unordered_map<std::size_t, JointProperties>;

    IdToJointProperties idToJointProperties;

    struct RevoluteJointProperties
    {
      RevoluteAxis axis;
    };

    using IdToRevoluteJointProperties =
        std::unordered_map<std::size_t, RevoluteJointProperties>;

    IdToRevoluteJointProperties idToRevoluteJointProperties;

    std::vector<std::size_t> jointIdToEntity;

    FrameID MakeRevoluteJoint(
        const JointProperties &_jp,
        const RevoluteJointProperties &_rp)
    {
      const std::size_t entityID = NextId();
      jointIdToEntity.push_back(entityID);

      idToJointProperties.insert(std::make_pair(entityID, _jp));
      idToRevoluteJointProperties.insert(std::make_pair(entityID, _rp));

      VectorX zero = VectorX::Constant(1, 0.0);
      JointState js = {zero, zero, zero, zero};
      idToJointState.insert(std::make_pair(entityID, js));

      return this->GenerateFrameID(this->GenerateIdentity(entityID));
    }

    ignition::physics::Identity InitiateEngine(std::size_t) override
    {
      NextId(); // Pass on the zero ID, which is reserved for the engine

      Pose pose = Pose::Identity();
      pose.translation() = LinearVector::UnitY();

      RevoluteAxis axis = RevoluteAxis::Zero();
      axis[0] = 1.0;

      FrameID parent = FrameID::World();

      parent = this->MakeRevoluteJoint({pose, pose, parent}, {axis});
      parent = this->MakeRevoluteJoint({pose, pose, parent}, {axis});
      parent = this->MakeRevoluteJoint({pose, pose, parent}, {axis});

      return this->GenerateIdentity(0);
    }

    Identity GetJointByIndex(const std::size_t _index) const override
    {
      if (this->jointIdToEntity.size() <= _index)
        return this->GenerateInvalidId();

      return this->GenerateIdentity(this->jointIdToEntity[_index]);
    }

    Identity CastToRevoluteJoint(const std::size_t _id) override
    {
      if (idToRevoluteJointProperties.count(_id) > 0)
        return this->GenerateIdentity(_id);

      return this->GenerateInvalidId();
    }

    Identity CastToRevoluteJoint(const std::size_t _id) const override
    {
      if (idToRevoluteJointProperties.count(_id) > 0)
        return this->GenerateIdentity(_id);

      return this->GenerateInvalidId();
    }

    void SetRevoluteJointAxis(
        std::size_t _id, const RevoluteAxis &_axis) override
    {
      idToRevoluteJointProperties.at(_id).axis = _axis;
    }

    RevoluteAxis GetRevoluteJointAxis(std::size_t _id) const override
    {
      return idToRevoluteJointProperties.at(_id).axis;
    }

    FrameData FrameDataRelativeToWorld(const FrameID &_id) const override
    {
      if (_id.IsWorld())
        return FrameData();

      const auto &p = idToJointProperties.at(_id.ID());
      const auto &axis = idToRevoluteJointProperties.at(_id.ID()).axis;

      const Pose R{Rotate(idToJointState.at(_id.ID()).position[0], axis)};

      Pose parentJointToParentLink = Pose::Identity();
      if (!p.parentJointID.IsWorld())
      {
        parentJointToParentLink =
            idToJointProperties.at(p.parentJointID.ID()).jointToChildLink;
      }

      FrameData data;
      data.pose = FrameDataRelativeToWorld(p.parentJointID).pose
          * parentJointToParentLink
          * p.parentLinkToJoint
          * R;

      return data;
    }

    Scalar GetJointPosition(
        const std::size_t _id, const std::size_t _dof) const override
    {
      return this->idToJointState.at(_id).position[_dof];
    }

    Scalar GetJointVelocity(
        const std::size_t _id, const std::size_t _dof) const override
    {
      return this->idToJointState.at(_id).velocity[_dof];
    }

    Scalar GetJointAcceleration(
        const std::size_t _id, const std::size_t _dof) const override
    {
      return this->idToJointState.at(_id).acceleration[_dof];
    }

    Scalar GetJointForce(
        const std::size_t _id, const std::size_t _dof) const override
    {
      return this->idToJointState.at(_id).force[_dof];
    }

    void SetJointPosition(
        const std::size_t _id, const std::size_t _dof,
        const Scalar _value) override
    {
      this->idToJointState.at(_id).position[_dof] = _value;
    }

    void SetJointVelocity(
        const std::size_t _id, const std::size_t _dof,
        const Scalar _value) override
    {
      this->idToJointState.at(_id).velocity[_dof] = _value;
    }

    void SetJointAcceleration(
        const std::size_t _id, const std::size_t _dof,
        const Scalar _value) override
    {
      this->idToJointState.at(_id).acceleration[_dof] = _value;
    }

    void SetJointForce(
        const std::size_t _id, const std::size_t _dof,
        const Scalar _value) override
    {
      this->idToJointState.at(_id).force[_dof] = _value;
    }

    Pose GetJointTransformFromParent(std::size_t _id) const override
    {
      return this->idToJointProperties.at(_id).parentLinkToJoint;
    }

    Pose GetJointTransformToChild(std::size_t _id) const override
    {
      return this->idToJointProperties.at(_id).jointToChildLink;
    }

    void SetJointTransformFromParent(
        const std::size_t _id, const Pose &_pose) override
    {
      this->idToJointProperties.at(_id).parentLinkToJoint = _pose;
    }

    void SetJointTransformToChild(
        const std::size_t _id, const Pose &_pose) override
    {
      this->idToJointProperties.at(_id).jointToChildLink = _pose;
    }

    std::size_t NextId()
    {
      return highestId++;
    }

    std::size_t highestId = 0;

  };

  class JointPlugin3d
      : public JointPlugin<ignition::physics::FeaturePolicy3d> { };

  IGN_PHYSICS_ADD_PLUGIN(
      JointPlugin3d,
      ignition::physics::FeaturePolicy3d,
      MockJointList)

  class JointPlugin2d
      : public JointPlugin<ignition::physics::FeaturePolicy2d> { };

  IGN_PHYSICS_ADD_PLUGIN(
      JointPlugin2d,
      ignition::physics::FeaturePolicy2d,
      MockJointList)

  class JointPlugin3f
      : public JointPlugin<ignition::physics::FeaturePolicy3f> { };

  IGN_PHYSICS_ADD_PLUGIN(
      JointPlugin3f,
      ignition::physics::FeaturePolicy3f,
      MockJointList)

  class JointPlugin2f
      : public JointPlugin<ignition::physics::FeaturePolicy2f> { };

  IGN_PHYSICS_ADD_PLUGIN(
      JointPlugin2f,
      ignition::physics::FeaturePolicy2f,
      MockJointList)
}
