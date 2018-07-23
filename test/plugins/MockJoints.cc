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
    using RevoluteAxis =
        typename FromPolicy<PolicyT>::template Use<AngularVector>;

    using Pose =
        typename FromPolicy<PolicyT>::template Use<Pose>;

    using FrameData =
        typename FromPolicy<PolicyT>::template Use<FrameData>;

    struct JointState
    {
      double position;
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

    ignition::physics::Identity InitiateEngine(std::size_t) override
    {
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

      const Pose R{Rotate(idToJointState.at(_id.ID()).position, axis)};

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
}
