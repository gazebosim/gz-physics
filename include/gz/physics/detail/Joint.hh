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

#ifndef GZ_PHYSICS_DETAIL_JOINT_HH_
#define GZ_PHYSICS_DETAIL_JOINT_HH_

#include <gz/physics/Joint.hh>
#include "gz/physics/detail/FrameSemantics.hh"

#include <string>

namespace gz
{
  namespace physics
  {
    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto GetBasicJointState::Joint<PolicyT, FeaturesT>::GetPosition(
        const std::size_t _dof) const -> Scalar
    {
      return this->template Interface<GetBasicJointState>()->GetJointPosition(
            this->identity, _dof);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto GetBasicJointState::Joint<PolicyT, FeaturesT>::GetVelocity(
        const std::size_t _dof) const -> Scalar
    {
      return this->template Interface<GetBasicJointState>()->GetJointVelocity(
            this->identity, _dof);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto GetBasicJointState::Joint<PolicyT, FeaturesT>::GetAcceleration(
        const std::size_t _dof) const -> Scalar
    {
      return this->template Interface<GetBasicJointState>()
          ->GetJointAcceleration(this->identity, _dof);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto GetBasicJointState::Joint<PolicyT, FeaturesT>::GetForce(
        const std::size_t _dof) const -> Scalar
    {
      return this->template Interface<GetBasicJointState>()
          ->GetJointForce(this->identity, _dof);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto GetBasicJointState::Joint<PolicyT, FeaturesT>::GetTransform() const
        -> Pose
    {
      return this->template Interface<GetBasicJointState>()
          ->GetJointTransform(this->identity);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    void SetBasicJointState::Joint<PolicyT, FeaturesT>::SetPosition(
        const std::size_t _dof, const Scalar _value)
    {
      this->template Interface<SetBasicJointState>()
          ->SetJointPosition(this->identity, _dof, _value);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    void SetBasicJointState::Joint<PolicyT, FeaturesT>::SetVelocity(
        const std::size_t _dof, const Scalar _value)
    {
      this->template Interface<SetBasicJointState>()
          ->SetJointVelocity(this->identity, _dof, _value);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    void SetBasicJointState::Joint<PolicyT, FeaturesT>::SetAcceleration(
        const std::size_t _dof, const Scalar _value)
    {
      this->template Interface<SetBasicJointState>()
          ->SetJointAcceleration(this->identity, _dof, _value);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    void SetBasicJointState::Joint<PolicyT, FeaturesT>::SetForce(
        const std::size_t _dof, const Scalar _value)
    {
      this->template Interface<SetBasicJointState>()
          ->SetJointForce(this->identity, _dof, _value);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    std::size_t GetBasicJointProperties::Joint<PolicyT, FeaturesT>::
    GetDegreesOfFreedom() const
    {
      return this->template Interface<GetBasicJointProperties>()
          ->GetJointDegreesOfFreedom(this->identity);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto GetBasicJointProperties::Joint<PolicyT, FeaturesT>::
    GetTransformFromParent() const -> Pose
    {
      return this->template Interface<GetBasicJointProperties>()
          ->GetJointTransformFromParent(this->identity);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto GetBasicJointProperties::Joint<PolicyT, FeaturesT>::
    GetTransformToChild() const -> Pose
    {
      return this->template Interface<GetBasicJointProperties>()
          ->GetJointTransformToChild(this->identity);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    void SetJointTransformFromParentFeature::Joint<PolicyT, FeaturesT>::
    SetTransformFromParent(const Pose &_pose)
    {
      this->template Interface<SetJointTransformFromParentFeature>()
        ->SetJointTransformFromParent(this->identity, _pose);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    void SetJointTransformToChildFeature::Joint<PolicyT, FeaturesT>::
    SetTransformToChild(const Pose &_pose)
    {
      this->template Interface<SetJointTransformToChildFeature>()
        ->SetJointTransformToChild(this->identity, _pose);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    void SetJointVelocityCommandFeature::Joint<PolicyT, FeaturesT>::
    SetVelocityCommand(const std::size_t _dof, const Scalar _value)
    {
      this->template Interface<SetJointVelocityCommandFeature>()
          ->SetJointVelocityCommand(this->identity, _dof, _value);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    void SetJointPositionLimitsFeature::Joint<PolicyT, FeaturesT>::
    SetMinPosition(const std::size_t _dof, const Scalar _value)
    {
      this->template Interface<SetJointPositionLimitsFeature>()
        ->SetJointMinPosition(this->identity, _dof, _value);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    void SetJointPositionLimitsFeature::Joint<PolicyT, FeaturesT>::
    SetMaxPosition(const std::size_t _dof, const Scalar _value)
    {
      this->template Interface<SetJointPositionLimitsFeature>()
        ->SetJointMaxPosition(this->identity, _dof, _value);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    void SetJointVelocityLimitsFeature::Joint<PolicyT, FeaturesT>::
    SetMinVelocity(const std::size_t _dof, const Scalar _value)
    {
      this->template Interface<SetJointVelocityLimitsFeature>()
        ->SetJointMinVelocity(this->identity, _dof, _value);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    void SetJointVelocityLimitsFeature::Joint<PolicyT, FeaturesT>::
    SetMaxVelocity(const std::size_t _dof, const Scalar _value)
    {
      this->template Interface<SetJointVelocityLimitsFeature>()
        ->SetJointMaxVelocity(this->identity, _dof, _value);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    void SetJointEffortLimitsFeature::Joint<PolicyT, FeaturesT>::
    SetMinEffort(const std::size_t _dof, const Scalar _value)
    {
      this->template Interface<SetJointEffortLimitsFeature>()
        ->SetJointMinEffort(this->identity, _dof, _value);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    void SetJointEffortLimitsFeature::Joint<PolicyT, FeaturesT>::
    SetMaxEffort(const std::size_t _dof, const Scalar _value)
    {
      this->template Interface<SetJointEffortLimitsFeature>()
        ->SetJointMaxEffort(this->identity, _dof, _value);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    void SetJointFrictionFeature::Joint<PolicyT, FeaturesT>::
    SetFriction(const std::size_t _dof, const Scalar _value)
    {
      this->template Interface<SetJointFrictionFeature>()
        ->SetJointFriction(this->identity, _dof, _value);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    void SetJointDampingCoefficientFeature::Joint<PolicyT, FeaturesT>::
    SetDampingCoefficient(const std::size_t _dof, const Scalar _value)
    {
      this->template Interface<SetJointDampingCoefficientFeature>()
        ->SetJointDampingCoefficient(this->identity, _dof, _value);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    void SetJointSpringStiffnessFeature::Joint<PolicyT, FeaturesT>::
    SetSpringStiffness(const std::size_t _dof, const Scalar _value)
    {
      this->template Interface<SetJointSpringStiffnessFeature>()
        ->SetJointSpringStiffness(this->identity, _dof, _value);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    void SetJointSpringReferenceFeature::Joint<PolicyT, FeaturesT>::
    SetSpringReference(const std::size_t _dof, const Scalar _value)
    {
      this->template Interface<SetJointSpringReferenceFeature>()
        ->SetJointSpringReference(this->identity, _dof, _value);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    bool SetMimicConstraintFeature::Joint<PolicyT, FeaturesT>::
    SetMimicConstraint(
      std::size_t _dof,
      const BaseJointPtr<PolicyT> &_leaderJoint,
      std::size_t _leaderAxisDof,
      Scalar _multiplier,
      Scalar _offset,
      Scalar _reference)
    {
      return this->template Interface<SetMimicConstraintFeature>()
        ->SetJointMimicConstraint(this->identity, _dof, _leaderJoint,
          _leaderAxisDof, _multiplier, _offset, _reference);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    void DetachJointFeature::Joint<PolicyT, FeaturesT>::Detach()
    {
      this->template Interface<DetachJointFeature>()
          ->DetachJoint(this->identity);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto GetJointTransmittedWrench::Joint<PolicyT, FeaturesT>::
    GetTransmittedWrench() const -> Wrench
    {
      return this->template Interface<GetJointTransmittedWrench>()
          ->GetJointTransmittedWrenchInJointFrame(this->identity);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto GetJointTransmittedWrench::Joint<PolicyT, FeaturesT>::
    GetTransmittedWrench(const FrameID &_relativeTo,
                         const FrameID &_inCoordinatesOf) const -> Wrench
    {
      using RelativeWrench =
          physics::RelativeWrench<typename PolicyT::Scalar, PolicyT::Dim>;

      return detail::Resolve(
          *this->template Interface<FrameSemantics>(),
          RelativeWrench(this->GetFrameID(), this->GetTransmittedWrench()),
          _relativeTo, _inCoordinatesOf);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    void SetFixedJointWeldChildToParentFeature::Joint<PolicyT, FeaturesT>::
    SetWeldChildToParent(bool _weldChildToParent)
    {
      this->template Interface<SetFixedJointWeldChildToParentFeature>()
        ->SetFixedJointWeldChildToParent(this->identity, _weldChildToParent);
    }
  }
}

#endif
