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

#ifndef IGNITION_PHYSICS_JOINT_HH_
#define IGNITION_PHYSICS_JOINT_HH_

#include <ignition/physics/FeatureList.hh>
#include <ignition/physics/Geometry.hh>

namespace ignition
{
  namespace physics
  {
    /////////////////////////////////////////////////
    /// \brief This feature retrieves the generalized joint states such as
    /// position, velocity, acceleration of the joint, applied force to the
    /// joint and the transformation matrix from the joint's parent link to this
    /// joint's child link based on its current position.
    class IGNITION_PHYSICS_VISIBLE GetBasicJointState : public virtual Feature
    {
      /// \brief The Joint API for getting basic joint state
      public: template <typename PolicyT, typename FeaturesT>
      class Joint : public virtual Feature::Joint<PolicyT, FeaturesT>
      {
        public: using Scalar = typename PolicyT::Scalar;
        public: using Pose = typename FromPolicy<PolicyT>::template Use<Pose>;

        /// \brief Get the generalized position of a specific generalized
        /// coordinate within this joint.
        /// \param[in] _dof
        ///   The desired generalized coordinate within this joint. Values start
        ///   from 0 and stop before Joint::GetDegreesOfFreedom().
        /// \return the generalized position of _dof
        public: Scalar GetPosition(const std::size_t _dof) const;

        /// \brief Get the generalized velocity of a specific generalized
        /// coordinate within this joint.
        /// \param[in] _dof
        ///   The desired generalized coordinate within this joint. Values start
        ///   from 0 and stop before Joint::GetDegreesOfFreedom().
        /// \return the generalized velocity of _dof
        public: Scalar GetVelocity(const std::size_t _dof) const;

        /// \brief Get the generalized acceleration of a specific generalized
        /// coordinate within this joint.
        /// \param[in] _dof
        ///   The desired generalized coordinate within this joint. Values start
        ///   from 0 and stop before Joint::GetDegreesOfFreedom().
        /// \return the generalized acceleration of _dof
        public: Scalar GetAcceleration(const std::size_t _dof) const;

        /// \brief Get the generalized force of a specific generalized
        /// coordinate within this joint.
        /// \param[in] _dof
        ///   The desired generalized coordinate within this joint. Values start
        ///   from 0 and stop before Joint::GetDegreesOfFreedom().
        /// \return the generalized force of _dof
        public: Scalar GetForce(const std::size_t _dof) const;

        /// \brief Get the transformation from the joint's parent to the joint's
        /// child produced by the current generalized positions and joint
        /// properties.
        /// \return Transformation matrix of this Joint
        public: Pose GetTransform() const;
      };

      /// \private The implementation API for getting basic joint state
      public: template <typename PolicyT>
      class Implementation : public virtual Feature::Implementation<PolicyT>
      {
        public: using Scalar = typename PolicyT::Scalar;
        public: using Pose = typename FromPolicy<PolicyT>::template Use<Pose>;

        // see Joint::GetPosition above
        public: virtual Scalar GetJointPosition(
            const Identity &_id, std::size_t _dof) const = 0;

        // see Joint::GetVelocity above
        public: virtual Scalar GetJointVelocity(
            const Identity &_id, std::size_t _dof) const = 0;

        // see Joint::GetAcceleration above
        public: virtual Scalar GetJointAcceleration(
            const Identity &_id, std::size_t _dof) const = 0;

        // see Joint::GetForce above
        public: virtual Scalar GetJointForce(
            const Identity &_id, std::size_t _dof) const = 0;

        // see Joint::GetTransform above
        public: virtual Pose GetJointTransform(const Identity &_id) const = 0;
      };
    };

    /////////////////////////////////////////////////
    /// \brief This feature sets the generalized joint states such as
    /// position, velocity, acceleration of the joint and the applied force to
    /// the joint.
    class IGNITION_PHYSICS_VISIBLE SetBasicJointState : public virtual Feature
    {
      /// \brief The Joint API for setting basic joint state
      public: template <typename PolicyT, typename FeaturesT>
      class Joint : public virtual Feature::Joint<PolicyT, FeaturesT>
      {
        public: using Scalar = typename PolicyT::Scalar;

        /// \brief Set the generalized position of a specific generalized
        /// coordinate within this joint.
        /// \param[in] _dof
        ///   The desired generalized coordinate within this joint. Values start
        ///   from 0 and stop before Joint::GetDegreesOfFreedom().
        /// \param[in] _value
        ///   The desired generalized position. Units depend on the underlying
        ///   joint type.
        public: void SetPosition(const std::size_t _dof, const Scalar _value);

        /// \brief Set the generalized velocity of a specific generalized
        /// coordinate within this joint.
        /// \param[in] _dof
        ///   The desired generalized coordinate within this joint. Values start
        ///   from 0 and stop before Joint::GetDegreesOfFreedom().
        /// \param[in] _value
        ///   The desired generalized velocity. Units depend on the underlying
        ///   joint type.
        public: void SetVelocity(const std::size_t _dof, const Scalar _value);

        /// \brief Set the generalized acceleration of a specific generalized
        /// coordinate within this joint.
        /// \param[in] _dof
        ///   The desired generalized coordinate within this joint. Values start
        ///   from 0 and stop before Joint::GetDegreesOfFreedom().
        /// \param[in] _value
        ///   The desired generalized acceleration. Units depend on the
        ///   underlying joint type.
        public: void SetAcceleration(
            const std::size_t _dof, const Scalar _value);

        /// \brief Set the generalized force of a specific generalized
        /// coordinate within this joint.
        /// \param[in] _dof
        ///   The desired generalized coordinate within this joint. Values start
        ///   from 0 and stop before Joint::GetDegreesOfFreedom().
        /// \param[in] _value
        ///   The desired generalized force. Units depend on the underlying
        ///   joint type.
        public: void SetForce(const std::size_t _dof, const Scalar _value);
      };

      /// \private The implementation API for setting basic joint state
      public: template <typename PolicyT>
      class Implementation : public virtual Feature::Implementation<PolicyT>
      {
        public: using Scalar = typename PolicyT::Scalar;

        // see Joint::SetPosition above
        public: virtual void SetJointPosition(
            const Identity &_id, std::size_t _dof, Scalar _value) = 0;

        // See Joint::SetVelocity above
        public: virtual void SetJointVelocity(
            const Identity &_id, std::size_t _dof, Scalar _value) = 0;

        // See Joint::SetAcceleration above
        public: virtual void SetJointAcceleration(
            const Identity &_id, std::size_t _dof, Scalar _value) = 0;

        // See Joint::SetForce above
        public: virtual void SetJointForce(
            const Identity &_id, std::size_t _dof, Scalar _value) = 0;
      };
    };

    /////////////////////////////////////////////////
    /// \brief This feature retrieves the generalized joint properties such as
    /// Degrees Of Freedom (DoF), the transformation matrix from the joint's
    /// parent link to this joint and the transformation matrix from this joint
    /// to its child link.
    class IGNITION_PHYSICS_VISIBLE GetBasicJointProperties
        : public virtual Feature
    {
      /// \brief The Joint API for getting basic joint properties
      public: template <typename PolicyT, typename FeaturesT>
      class Joint : public virtual Feature::Joint<PolicyT, FeaturesT>
      {
        public: using Pose = typename FromPolicy<PolicyT>::template Use<Pose>;

        /// \brief Get the number of degrees of freedom (a.k.a. number of
        /// generalized coordinates) within this joint.
        /// \return Number of degrees of freedom
        public: std::size_t GetDegreesOfFreedom() const;

        /// \brief Get the transformation matrix from the parent link (or world)
        /// to the base of the joint transform. The final result of
        /// Joint::GetTransform() will be
        ///
        /// \code
        /// GetTransformFromParent() * T(q) * GetTransformToChild()
        /// \endcode
        ///
        /// where `T(q)` is the transform produced by the current generalized
        /// positions of the joint.
        ///
        /// \return Transform from parent
        public: Pose GetTransformFromParent() const;

        /// \brief Get the transformation matrix from the tip of the joint
        /// transform to the child link. The final result of
        /// Joint::GetTransform() will be
        ///
        /// \code
        /// GetTransformFromParent() * T(q) * GetTransformToChild()
        /// \endcode
        ///
        /// where `T(q)` is the transform produced by the current generalized
        /// positions of the joint.
        ///
        /// \return Transform to child
        public: Pose GetTransformToChild() const;
      };

      /// \private The implementation API for getting basic joint properties
      public: template <typename PolicyT>
      class Implementation : public virtual Feature::Implementation<PolicyT>
      {
        public: using Pose = typename FromPolicy<PolicyT>::template Use<Pose>;

        public: virtual std::size_t GetJointDegreesOfFreedom(
            const Identity &_id) const = 0;

        public: virtual Pose GetJointTransformFromParent(
            const Identity &_id) const = 0;

        public: virtual Pose GetJointTransformToChild(
            const Identity &_id) const = 0;
      };
    };

    /////////////////////////////////////////////////
    class IGNITION_PHYSICS_VISIBLE SetJointTransformFromParentFeature
        : public virtual Feature
    {
      /// \brief The Joint API for setting the transform from the joint's parent
      public: template <typename PolicyT, typename FeaturesT>
      class Joint : public virtual Feature::Joint<PolicyT, FeaturesT>
      {
        public: using Pose = typename FromPolicy<PolicyT>::template Use<Pose>;

        /// \brief Set the transformation matrix from the parent link (or world)
        /// to the base of the joint transform. The final result of
        /// Joint::GetTransform() will be
        ///
        /// \code
        /// GetTransformFromParent() * T(q) * GetTransformToChild()
        /// \endcode
        ///
        /// where `T(q)` is the transform produced by the current generalized
        /// positions of the joint.
        ///
        /// \param[in] _pose
        ///   The desired transformation matrix
        public: void SetTransformFromParent(const Pose &_pose);
      };

      /// \private The implementation API for setting the transform from the
      /// parent
      public: template <typename PolicyT>
      class Implementation : public virtual Feature::Implementation<PolicyT>
      {
        public: using Pose = typename FromPolicy<PolicyT>::template Use<Pose>;

        // see Joint::SetTransformFromParent above
        public: virtual void SetJointTransformFromParent(
            const Identity &_id, const Pose &_pose) = 0;
      };
    };

    /////////////////////////////////////////////////
    class IGNITION_PHYSICS_VISIBLE SetJointTransformToChildFeature
        : public virtual Feature
    {
      /// \brief The Joint API for setting the transform to the joint's child
      public: template <typename PolicyT, typename FeaturesT>
      class Joint : public virtual Feature::Joint<PolicyT, FeaturesT>
      {
        public: using Pose = typename FromPolicy<PolicyT>::template Use<Pose>;

        /// \brief Set the transformation matrix from the tip of the joint
        /// transform to the child link. The final result of
        /// Joint::GetTransform() will be
        ///
        /// \code
        /// GetTransformFromParent() * T(q) * GetTransformToChild()
        /// \endcode
        ///
        /// where `T(q)` is the transform produced by the current generalized
        /// positions of the joint.
        ///
        /// \param[in] _pose
        ///   The desired transformation matrix
        public: void SetTransformToChild(const Pose &_pose);
      };

      /// \private The implementation API for setting the transform to the child
      public: template <typename PolicyT>
      class Implementation : public virtual Feature::Implementation<PolicyT>
      {
        public: using Pose = typename FromPolicy<PolicyT>::template Use<Pose>;

        // see Joint::SetTransformToChild above
        public: virtual void SetJointTransformToChild(
            const Identity &_id, const Pose &_pose) = 0;
      };
    };

    /////////////////////////////////////////////////
    /// \brief This feature sets the commanded value of generalized velocity of
    /// this Joint.
    class IGNITION_PHYSICS_VISIBLE SetJointVelocityCommandFeature
        : public virtual Feature
    {
      /// \brief The Joint API for setting velocity commands (target velocity).
      /// This is different from SetVelocity in that this does not modify the
      /// internal state of the joint. Instead, the physics engine is expected
      /// to compute the necessary joint torque for the commanded velocity and
      /// apply it in the next simulation step.
      public: template <typename PolicyT, typename FeaturesT>
      class Joint : public virtual Feature::Joint<PolicyT, FeaturesT>
      {
        public: using Scalar = typename PolicyT::Scalar;

        /// \brief Set the commanded value of generalized velocity of a specific
        /// generalized coordinate within this joint.
        /// \param[in] _dof
        ///   The desired generalized coordinate within this joint. Values start
        ///   from 0 and stop before Joint::GetDegreesOfFreedom().
        /// \param[in] _value
        ///   The desired generalized velocity. Units depend on the underlying
        ///   joint type.
        public: void SetVelocityCommand(
            const std::size_t _dof, const Scalar _value);
      };

      /// \private The implementation API for setting joint velocity commands
      public: template <typename PolicyT>
      class Implementation : public virtual Feature::Implementation<PolicyT>
      {
        public: using Scalar = typename PolicyT::Scalar;

        // See Joint::SetVelocityCommand above
        public: virtual void SetJointVelocityCommand(
            const Identity &_id, std::size_t _dof, Scalar _value) = 0;
      };
    };

    /////////////////////////////////////////////////
    /// \brief This feature sets the min and max generalized position of this
    /// Joint.
    class IGNITION_PHYSICS_VISIBLE SetJointPositionLimitsFeature
        : public virtual Feature
    {
      /// \brief The Joint API for setting position limits of a joint.
      public: template <typename PolicyT, typename FeaturesT>
      class Joint : public virtual Feature::Joint<PolicyT, FeaturesT>
      {
        public: using Scalar = typename PolicyT::Scalar;

        /// \brief Set the minimum allowed value of the generalized coordinate
        /// within this joint.
        /// \param[in] _dof
        ///   The desired generalized coordinate within this joint. Values start
        ///   from 0 and stop before Joint::GetDegreesOfFreedom().
        /// \param[in] _value
        ///   The minimum allowed value of the generalized coordinate. Units
        ///   depend on the underlying joint type.
        public: void SetMinPosition(
            const std::size_t _dof, const Scalar _value);

        /// \brief Set the maximum allowed value of the generalized coordinate
        /// within this joint.
        /// \param[in] _dof
        ///   The desired generalized coordinate within this joint. Values start
        ///   from 0 and stop before Joint::GetDegreesOfFreedom().
        /// \param[in] _value
        ///   The maximum allowed value of the generalized coordinate. Units
        ///   depend on the underlying joint type.
        public: void SetMaxPosition(
            const std::size_t _dof, const Scalar _value);
      };

      /// \private The implementation API for setting position limit commands
      public: template <typename PolicyT>
      class Implementation : public virtual Feature::Implementation<PolicyT>
      {
        public: using Scalar = typename PolicyT::Scalar;

        // See Joint::SetMinPositionCommand above
        public: virtual void SetJointMinPosition(
            const Identity &_id, std::size_t _dof, Scalar _value) = 0;

        // See Joint::SetMaxPositionCommand above
        public: virtual void SetJointMaxPosition(
            const Identity &_id, std::size_t _dof, Scalar _value) = 0;
      };
    };

    /////////////////////////////////////////////////
    /// \brief This feature sets the min and max value of generalized velocity
    /// of this Joint.
    class IGNITION_PHYSICS_VISIBLE SetJointVelocityLimitsFeature
        : public virtual Feature
    {
      /// \brief The Joint API for setting velocity limits of a joint. These
      /// limits apply to joints commanded via velocity or positional commands.
      public: template <typename PolicyT, typename FeaturesT>
      class Joint : public virtual Feature::Joint<PolicyT, FeaturesT>
      {
        public: using Scalar = typename PolicyT::Scalar;

        /// \brief Set the minimum value of generalized velocity of a specific
        /// generalized coordinate within this joint.
        /// \param[in] _dof
        ///   The desired generalized coordinate within this joint. Values start
        ///   from 0 and stop before Joint::GetDegreesOfFreedom().
        /// \param[in] _value
        ///   The minimum generalized velocity. Units depend on the underlying
        ///   joint type.
        public: void SetMinVelocity(
            const std::size_t _dof, const Scalar _value);

        /// \brief Set the maximum value of generalized velocity of a specific
        /// generalized coordinate within this joint.
        /// \param[in] _dof
        ///   The desired generalized coordinate within this joint. Values start
        ///   from 0 and stop before Joint::GetDegreesOfFreedom().
        /// \param[in] _value
        ///   The maximum generalized velocity. Units depend on the underlying
        ///   joint type.
        public: void SetMaxVelocity(
            const std::size_t _dof, const Scalar _value);
      };

      /// \private The implementation API for setting velocity limit commands
      public: template <typename PolicyT>
      class Implementation : public virtual Feature::Implementation<PolicyT>
      {
        public: using Scalar = typename PolicyT::Scalar;

        // See Joint::SetMinVelocityCommand above
        public: virtual void SetJointMinVelocity(
            const Identity &_id, std::size_t _dof, Scalar _value) = 0;

        // See Joint::SetMaxVelocityCommand above
        public: virtual void SetJointMaxVelocity(
            const Identity &_id, std::size_t _dof, Scalar _value) = 0;
      };
    };

    /////////////////////////////////////////////////
    /// \brief This feature sets the min and max value of effort of this Joint.
    class IGNITION_PHYSICS_VISIBLE SetJointEffortLimitsFeature
        : public virtual Feature
    {
      /// \brief The Joint API for setting effort limits of a joint. These
      /// limits are applied to joints controlled via positional, velocity or
      /// effort commands.
      public: template <typename PolicyT, typename FeaturesT>
      class Joint : public virtual Feature::Joint<PolicyT, FeaturesT>
      {
        public: using Scalar = typename PolicyT::Scalar;

        /// \brief Set the minimum value of effort of a specific generalized
        /// coordinate within this joint.
        /// \param[in] _dof
        ///   The desired generalized coordinate within this joint. Values start
        ///   from 0 and stop before Joint::GetDegreesOfFreedom().
        /// \param[in] _value
        ///   The minimum effort. Units depend on the underlying joint type.
        public: void SetMinEffort(const std::size_t _dof, const Scalar _value);

        /// \brief Set the maximum value of effort of a specific generalized
        /// coordinate within this joint.
        /// \param[in] _dof
        ///   The desired generalized coordinate within this joint. Values start
        ///   from 0 and stop before Joint::GetDegreesOfFreedom().
        /// \param[in] _value
        ///   The maximum effort. Units depend on the underlying joint type.
        public: void SetMaxEffort(const std::size_t _dof, const Scalar _value);
      };

      /// \private The implementation API for setting effort limit commands
      public: template <typename PolicyT>
      class Implementation : public virtual Feature::Implementation<PolicyT>
      {
        public: using Scalar = typename PolicyT::Scalar;

        // See Joint::SetMinEffortCommand above
        public: virtual void SetJointMinEffort(
            const Identity &_id, std::size_t _dof, Scalar _value) = 0;

        // See Joint::SetMaxEffortCommand above
        public: virtual void SetJointMaxEffort(
            const Identity &_id, std::size_t _dof, Scalar _value) = 0;
      };
    };

    class IGNITION_PHYSICS_VISIBLE DetachJointFeature
        : public virtual Feature
    {
      public: template <typename PolicyT, typename FeaturesT>
      class Joint : public virtual Feature::Joint<PolicyT, FeaturesT>
      {
        /// \brief Detach this joint.
        public: void Detach();
      };

      public: template <typename PolicyT>
      class Implementation : public virtual Feature::Implementation<PolicyT>
      {
        public: virtual void DetachJoint(const Identity &_jointID) = 0;
      };
    };
  }
}

#include <ignition/physics/detail/Joint.hh>

#endif
