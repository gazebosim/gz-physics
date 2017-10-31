/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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

#ifndef IGNITION_PHYSICS_FRAMEDQUANTITY_HH_
#define IGNITION_PHYSICS_FRAMEDQUANTITY_HH_

#include <ignition/physics/FrameID.hh>
#include <ignition/physics/FrameData.hh>

#include <ignition/math/Vector2.hh>
#include <ignition/math/Matrix3.hh>

namespace ignition
{
  namespace physics
  {
    /// \brief The FramedQuantity class is a wrapper for the native ign-math
    /// classes. The purpose of this wrapper is to endow the native classes with
    /// frame semantics, so that they can express the frame of reference of
    /// their values.
    ///
    /// Note that the raw value of the quantity being held can only be retrieved
    /// through the member function RelativeToParent(). We choose to contain the
    /// quantity like this instead of inheriting it to avoid a situation where
    /// a user might add two framed quantities like
    ///
    /// u = v + w
    ///
    /// where v and w belong to different frames. That would be an ill-formed
    /// expression, since values that are being expressed in different reference
    /// frames are not directly compatible. Use the Resolve(~) or Reframe(~)
    /// function in the FrameSemantics class of your physics engine plugin to
    /// transform a FramedQuantity into a different reference frame.
    template <typename Q, std::size_t Dim, typename CoordinateSpace>
    class FramedQuantity
    {
      /// \brief This constructor will specify the parent frame and then forward
      /// the remaining arguments to the constructor of the underlying quantity.
      public: template <typename... Args>
      FramedQuantity(const FrameID &_parentID, Args&&... _args);

      /// \brief Implicit conversion constructor.
      public: FramedQuantity(const Q &_rawValue);

      /// \brief Get the value of this FramedQuantity relative to its parent
      /// frame. To get the value of this FramedQuantity with respect to the
      /// world frame, use the FrameSemantics::Resolve(~) function of your
      /// physics engine like so:
      ///
      /// Q quantity = engine->GetInterface<FrameSemantics>()->Resolve(fq)
      ///
      /// where Q is your quantity type, and fq is your FramedQuantity instance.
      ///
      /// To get the value of this FramedQuantity with respect to an arbitrary
      /// reference frame, again use the Resolve function:
      ///
      /// Q quantity = engine->GetInterface<FrameSemantics>()->Resolve(fq, F);
      ///
      /// where F is the FrameID of the desired reference frame.
      public: Q &RelativeToParent();

      /// \brief const-qualified version of RelativeToParent.
      public: const Q &RelativeToParent() const;

      /// \brief Get the ID of this FramedQuantity's parent frame.
      ///
      /// To change the parent frame of this FramedQuantity, use the Reframe(~)
      /// function of your physics engine's FrameSemantics interface like this:
      ///
      /// fq = engine->GetInterface<FrameSemantics>()->Reframe(fq, A);
      ///
      /// where A is the FrameID of the new frame. The Reframe function will
      /// keep the values of your FramedQuantity consistent (with respect to the
      /// World Frame) as it reassigns the parent frame.
      ///
      /// Alternatively, to change the parent frame of this FramedQuantity while
      /// making its values relative to the new frame equal to what its values
      /// were relative to the old frame, you can use the
      /// MoveToNewParentFrame(~) function below.
      public: const FrameID &ParentFrame() const;

      /// \brief This function will change the parent frame of your
      /// FramedQuantity.
      public: void MoveToNewParentFrame(const FrameID &_newParentFrame);

      /// \brief The underlying type of the quantity that is being expressed.
      public: using Quantity = Q;

      /// \brief The mathematical space which defines how this quantity is
      /// transformed between reference frames.
      public: using Space = CoordinateSpace;

      /// \brief The number of physical (spatial) dimensions that this quantity
      /// exists in. Currently we only support Dim==3, but in the future we may
      /// support Dim==2, so this field is for forward compatibility.
      public: static constexpr std::size_t Dimension = Dim;

      /// \brief This variable specifies the parent frame that this
      /// FramedQuantity belongs to.
      private: FrameID parentFrame;

      /// \brief The raw quantity, expressed in terms of the parent frame.
      private: Q value;
    };

    namespace detail
    {
      /////////////////////////////////////////////////
      // Forward delcarations of CoordinateSpaces
      template <std::size_t, typename> struct SESpace;
      template <std::size_t, typename, typename> struct SOSpace;
      template <std::size_t, typename> struct EuclideanSpace;
      template <std::size_t, typename> struct LinearVelocitySpace;
      template <std::size_t, typename> struct AngularVelocitySpace;
      template <std::size_t, typename> struct LinearAccelerationSpace;
      template <std::size_t, typename> struct AngularAccelerationSpace;
      template <std::size_t, typename> struct VectorSpace;
      template <std::size_t, typename> struct FrameSpace;
      // TODO: We can add more spaces to support other types like Moments of
      // Inertia, Jacobians, Spatial Velocities/Accelerations, Wrench+Point
      // pairs, and so on.
      //
      // Users can also define Spaces for their own types (see the header
      // ignition/physics/detail/FrameSemantics.hpp for example implementations)
      // and use them seamlessly in the Frame Semantics infrastructure.
    }

    /////////////////////////////////////////////////
    template <typename Scalar>
    using FramedPose3 = FramedQuantity<
        math::Pose3<Scalar>, 3, detail::SESpace<3, Scalar>>;
    using FramedPose3d = FramedPose3<double>;
    using FramedPose3f = FramedPose3<float>;

    /////////////////////////////////////////////////
    template <typename Scalar>
    using FramedRotationMatrix3 = FramedQuantity<
        math::Matrix3<Scalar>, 3,
        detail::SOSpace<3, Scalar, math::Matrix3<Scalar>>>;
    using FramedRotationMatrix3d = FramedRotationMatrix3<double>;
    using FramedRotationMatrix3f = FramedRotationMatrix3<float>;

    /////////////////////////////////////////////////
    template <typename Scalar>
    using FramedQuaternion = FramedQuantity<
        math::Quaternion<Scalar>, 3,
        detail::SOSpace<3, Scalar, math::Quaternion<Scalar>>>;
    using FramedQuaterniond = FramedQuaternion<double>;
    using FramedQuaternionf = FramedQuaternion<float>;

    /////////////////////////////////////////////////
    template <typename Scalar>
    using FramedPosition3 = FramedQuantity<
        math::Vector3<Scalar>, 3, detail::EuclideanSpace<3, Scalar>>;
    using FramedPosition3d = FramedPosition3<double>;
    using FramedPosition3f = FramedPosition3<float>;

    /////////////////////////////////////////////////
    /// Note: A FramedLinearVelocity can be thought of as the instantaneous
    /// velocity of a point which is incidental to the origin of the parent
    /// frame. The function RelativeToParent() expresses the point's
    /// instantaneous velocity relative to the current instantaneous velocity of
    /// parent frame.
    ///
    /// To find the instantenous velocity of a framed point which is not located
    /// at the origin of its parent frame, use RelativeFrameData.
    template <typename Scalar>
    using FramedLinearVelocity3 = FramedQuantity<
        math::Vector3<Scalar>, 3, detail::LinearVelocitySpace<3, Scalar>>;
    using FramedLinearVelocity3d = FramedLinearVelocity3<double>;
    using FramedLinearVelocity3f = FramedLinearVelocity3<float>;

    /////////////////////////////////////////////////
    /// Note: A FramedAngularVelocity can be thought of as the instantaneous
    /// angular velocity of a body expressed in terms of its parent frame.
    /// Unlike FramedLinearVelocity, FramedLinearAcceleration, and
    /// FramedAngularAcceleration, this is no different than computing the
    /// relative angular velocity of a frame which is a child of the parent
    /// frame, because it does not depend on relative position or any other
    /// factors.
    template <typename Scalar>
    using FramedAngularVelocity3 = FramedQuantity<
        math::Vector3<Scalar>, 3, detail::AngularVelocitySpace<3, Scalar>>;
    using FramedAngularVelocity3d = FramedAngularVelocity3<double>;
    using FramedAngularVelocity3f = FramedAngularVelocity3<float>;

    /////////////////////////////////////////////////
    /// Note: A FramedLinearAcceleration can be thought of as the instantaneous
    /// linear acceleration of a point which is incidental to the origin of its
    /// parent frame and has no velocity relative to its parent frame. The
    /// function RelativeToParent() expresses the point's instantaneous linear
    /// acceleration relative to the current instantaneous linear acceleration
    /// of the parent frame. Since the hypothetical point is located at the
    /// origin with zero instantaneous velocity, this will not be subject to
    /// centrifugal or Coriolis effects.
    ///
    /// To find the instantaneous linear acceleration of a framed point which is
    /// not located at the origin of its parent frame and/or has non-zero
    /// velocity (and which therefore experiences centrifugal and Coriolis
    /// effects), use a RelativeFrameData object.
    template <typename Scalar>
    using FramedLinearAcceleration3 = FramedQuantity<
        math::Vector3<Scalar>, 3, detail::LinearAccelerationSpace<3, Scalar>>;
    using FramedLinearAcceleration3d = FramedLinearAcceleration3<double>;
    using FramedLinaerAcceleration3f = FramedLinearAcceleration3<float>;

    /////////////////////////////////////////////////
    /// Note: A FramedAngularAcceleration can be thought of as the instantaneous
    /// angular acceleration of a body which is incidental to the origin of its
    /// parent frame and has no velocity relative to its parent frame. The
    /// function RelativeToParent() expresses the body's instantaneous angular
    /// acceleration relative to the current instantaneous angular acceleration
    /// of the parent frame. Since the hypothetical body has zero instantaneous
    /// velocity, this will not be subject to Coriolis effects.
    ///
    /// To find the instantaneous angular acceleration of a framed body which
    /// has non-zero velocity (and therefore experiences Coriolis effects), use
    /// a RelativeFrameData object.
    template <typename Scalar>
    using FramedAngularAcceleration3 = FramedQuantity<
        math::Vector3<Scalar>, 3, detail::AngularAccelerationSpace<3, Scalar>>;
    using FramedAngularAcceleration3d = FramedAngularAcceleration3<double>;
    using FramedAngularAcceleration3f = FramedAngularAcceleration3<float>;

    /////////////////////////////////////////////////
    template <typename Scalar>
    using FramedForce3 = FramedQuantity<
        math::Vector3<Scalar>, 3, detail::VectorSpace<3, Scalar>>;
    using FramedForce3d = FramedForce3<double>;
    using FramedForce3f = FramedForce3<float>;

    /////////////////////////////////////////////////
    template <typename Scalar>
    using FramedTorque3 = FramedQuantity<
        math::Vector3<Scalar>, 3, detail::VectorSpace<3, Scalar>>;
    using FramedTorque3d = FramedTorque3<double>;
    using FramedTorque3f = FramedTorque3<float>;

    /////////////////////////////////////////////////
    template <typename Scalar>
    using RelativeFrameData3 = FramedQuantity<
        FrameData3<Scalar>, 3, detail::FrameSpace<3, Scalar>>;
    using RelativeFrameData3d = RelativeFrameData3<double>;
    using RelativeFrameData3f = RelativeFrameData3<float>;
  }
}

#include <ignition/physics/detail/FramedQuantity.hh>

#endif
