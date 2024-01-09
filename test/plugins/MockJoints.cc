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

#include <gz/physics/Register.hh>

#include "mock/MockJoints.hh"

namespace mock
{
  template <typename PolicyT>
  class JointPlugin
      : public gz::physics::Implements<PolicyT, MockJointList>
  {
    using Scalar = typename PolicyT::Scalar;

    using RevoluteAxis =
        typename gz::physics::FromPolicy<PolicyT>
        ::template Use<gz::physics::AngularVector>;

    using PrismaticAxis =
        typename gz::physics::FromPolicy<PolicyT>
        ::template Use<gz::physics::LinearVector>;

    using Pose =
        typename gz::physics::FromPolicy<PolicyT>
        ::template Use<gz::physics::Pose>;

    using FrameData =
        typename gz::physics::FromPolicy<PolicyT>
        ::template Use<gz::physics::FrameData>;

    using LinearVector =
        typename gz::physics::FromPolicy<PolicyT>
        ::template Use<gz::physics::LinearVector>;

    using VectorX = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;

    using FrameID = gz::physics::FrameID;
    using Identity = gz::physics::Identity;

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

    gz::physics::Identity InitiateEngine(std::size_t) override
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

    Identity CastToRevoluteJoint(const Identity & _id) const override
    {
      if (idToRevoluteJointProperties.count(_id) > 0)
        return this->GenerateIdentity(_id);

      return this->GenerateInvalidId();
    }

    void SetRevoluteJointAxis(
        const Identity &_id, const RevoluteAxis &_axis) override
    {
      idToRevoluteJointProperties.at(_id).axis = _axis;
    }

    RevoluteAxis GetRevoluteJointAxis(const Identity &_id) const override
    {
      return idToRevoluteJointProperties.at(_id).axis;
    }

    Identity CastToPrismaticJoint(const Identity &/*_id*/) const override
    {
      // TODO(MXG): Implement this
      return this->GenerateInvalidId();
    }

    void SetPrismaticJointAxis(
        const Identity &/*_id*/, const PrismaticAxis &/*_axis*/) override
    {
      // TODO(MXG): Implement this
    }

    PrismaticAxis GetPrismaticJointAxis(const Identity &/*_id*/) const override
    {
      // TODO(MXG): Implement this
      return PrismaticAxis::UnitZ();
    }

    Identity CastToFreeJoint(const Identity &/*_id*/) const override
    {
      // TODO(MXG): Implement this
      return this->GenerateInvalidId();
    }

    void SetFreeJointRelativeTransform(
        const Identity &/*_id*/, const Pose &/*_pose*/) override
    {
      // TODO(MXG): Implement this
    }

    FrameData FrameDataRelativeToWorld(const FrameID &_id) const override
    {
      if (_id.IsWorld())
        return FrameData();

      const auto &p = idToJointProperties.at(_id.ID());
      const auto &axis = idToRevoluteJointProperties.at(_id.ID()).axis;

      const Pose R{gz::physics::Rotate(
              idToJointState.at(_id.ID()).position[0], axis)};

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
        const Identity &_id, std::size_t _dof) const override
    {
      return this->idToJointState.at(_id).position[_dof];
    }

    Scalar GetJointVelocity(
        const Identity &_id, std::size_t _dof) const override
    {
      return this->idToJointState.at(_id).velocity[_dof];
    }

    Scalar GetJointAcceleration(
        const Identity &_id, std::size_t _dof) const override
    {
      return this->idToJointState.at(_id).acceleration[_dof];
    }

    Scalar GetJointForce(
        const Identity & _id, std::size_t _dof) const override
    {
      return this->idToJointState.at(_id).force[_dof];
    }

    Pose GetJointTransform(const Identity & _id) const override
    {
      const auto &p = this->idToJointProperties.at(_id);
      const auto &axis = this->idToRevoluteJointProperties.at(_id).axis;

      const Pose R{gz::physics::Rotate(
              this->idToJointState.at(_id).position[0], axis)};

      return p.parentLinkToJoint * R * p.jointToChildLink;
    }

    void SetJointPosition(
        const Identity &_id, std::size_t _dof,
        const Scalar _value) override
    {
      this->idToJointState.at(_id).position[_dof] = _value;
    }

    void SetJointVelocity(
        const Identity &_id, std::size_t _dof,
        const Scalar _value) override
    {
      this->idToJointState.at(_id).velocity[_dof] = _value;
    }

    void SetJointAcceleration(
        const Identity &_id, std::size_t _dof,
        const Scalar _value) override
    {
      this->idToJointState.at(_id).acceleration[_dof] = _value;
    }

    void SetJointForce(
        const Identity &_id, std::size_t _dof,
        const Scalar _value) override
    {
      this->idToJointState.at(_id).force[_dof] = _value;
    }

    std::size_t GetJointDegreesOfFreedom(const Identity &/*_id*/) const override
    {
      // So far we only support revolute joints, which have 1 DOF
      return 1u;
    }

    Pose GetJointTransformFromParent(const Identity &_id) const override
    {
      return this->idToJointProperties.at(_id).parentLinkToJoint;
    }

    Pose GetJointTransformToChild(const Identity &_id) const override
    {
      return this->idToJointProperties.at(_id).jointToChildLink;
    }

    void SetJointTransformFromParent(
        const Identity &_id, const Pose &_pose) override
    {
      this->idToJointProperties.at(_id).parentLinkToJoint = _pose;
    }

    void SetJointTransformToChild(
        const Identity &_id, const Pose &_pose) override
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
      : public JointPlugin<gz::physics::FeaturePolicy3d> { };

  GZ_PHYSICS_ADD_PLUGIN(
      JointPlugin3d,
      gz::physics::FeaturePolicy3d,
      MockJointList)

  class JointPlugin2d
      : public JointPlugin<gz::physics::FeaturePolicy2d> { };

  GZ_PHYSICS_ADD_PLUGIN(
      JointPlugin2d,
      gz::physics::FeaturePolicy2d,
      MockJointList)

  class JointPlugin3f
      : public JointPlugin<gz::physics::FeaturePolicy3f> { };

  GZ_PHYSICS_ADD_PLUGIN(
      JointPlugin3f,
      gz::physics::FeaturePolicy3f,
      MockJointList)

  class JointPlugin2f
      : public JointPlugin<gz::physics::FeaturePolicy2f> { };

  GZ_PHYSICS_ADD_PLUGIN(
      JointPlugin2f,
      gz::physics::FeaturePolicy2f,
      MockJointList)
}
