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

#ifndef IGNITION_PHYSICS_FRAMESEMANTICS_HH_
#define IGNITION_PHYSICS_FRAMESEMANTICS_HH_

#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector2.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Matrix3.hh>

namespace ignition
{
  namespace physics
  {
    /////////////////////////////////////////////////
    /// \brief Container for specifying Frame IDs. We do not want to use a generic
    /// integer type for this, because it may lead to bugs where a plain integer
    /// is mistaken for a FrameID. This also allows the compiler to always
    /// perform argument deduction successfully.
    struct FrameID
    {
      public: inline explicit FrameID(const std::size_t _id)
        : id(_id)
      {
        // Do nothing
      }

      public: const std::size_t id;
    };

    /////////////////////////////////////////////////
    /// \brief The ID of the World Frame is always uniquely 0.
    const FrameID WorldFrameID = FrameID(0);

    /// \brief The FrameData3d struct fully describes the kinematic state of a
    /// Frame with 3 dimensions and double precision.
    ///
    /// The frame of reference for this data is dependent on the context in
    /// which it is used. For FrameData which explicitly expresses its frame of
    /// reference, see RelativeFrameData3.
    template <typename Scalar>
    struct FrameData3
    {
      /// \brief Constructor. This will initialize the transform with identity
      /// and all velocity and acceleration vectors to zero.
      public: FrameData3();

      /// \brief The current SE3 transformation of the frame.
      public: math::Pose3<Scalar> transform;

      /// \brief The current linear velocity of the frame.
      public: math::Vector3<Scalar> linearVelocity;

      /// \brief The current angular velocity of the frame.
      public: math::Vector3<Scalar> angularVelocity;

      /// \brief The current linear acceleration of the frame.
      public: math::Vector3<Scalar> linearAcceleration;

      /// \brief The current angular acceleration of the frame.
      public: math::Vector3<Scalar> angularAcceleration;
    
      /// \brief Set the transform to identity and all velocity and acceleration
      /// vectors to zero.
      public: void SetToZero();
    };
    using FrameData3d = FrameData3<double>;
    using FrameData3f = FrameData3<float>;

    /////////////////////////////////////////////////
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
    template <typename Q, std::size_t Dim, typename ConfigSpace>
    class FramedQuantity
    {
      /// \brief This constructor will specify the parent frame and then forward
      /// the remaining arguments to the constructor of the underlying quantity.
      public: template <typename... Args>
      FramedQuantity(const FrameID &_parentID, Args&&... _args);

      /// \brief This constructor will forward all of the arguments to the
      /// constructor of the underlying quantity, and then set the parent frame
      /// to be the World Frame.
      public: template <typename... Args>
      FramedQuantity(Args&&... _args);

      /// \brief Implicit conversion constructor.
      public: FramedQuantity(const Q &_rawValue);

      /// \brief Get the value of this FramedQuantity relative to its parent
      /// frame.
      public: Q &RelativeToParent();

      /// \brief const-qualified version of RelativeToParent.
      public: const Q &RelativeToParent() const;

      /// \brief Get the ID of this FramedQuantity's parent frame.
      ///
      /// To change the parent frame of this FramedQuantity, use the Reframe(~)
      /// function of your physics engine like this:
      ///
      /// fq = Reframe(fq, A);
      ///
      /// where A is the FrameID of the new frame. The Reframe function will
      /// keep the value of your FramedQuantity consistent (with respect to the
      /// World Frame).
      ///
      /// Alternatively, to change the parent frame of this FramedQuantity while
      /// making its value relative to the new frame equal to what its value was
      /// relative to the old frame, you can simply use the assignment operator
      /// to overwrite this FramedQuantity:
      ///
      /// fq = MyFramedQuantity(A, fq.RelativeToParent());
      ///
      /// where MyFramedQuantity is your FramedQuantity's type (e.g.
      /// FramedPose3d, FramedPosition3d, FramedForce3f, etc...).
      public: const FrameID &ParentFrame() const;

      /// \brief The underlying raw type of the quantity that is being expressed.
      public: using Quantity = Q;

      /// \brief The mathematical space which defines how this quantity can be
      /// transformed between reference frames.
      public: using Space = ConfigSpace;

      public: static constexpr std::size_t Dimension = Dim;

      /// \brief The raw quantity, relative to the parent frame.
      private: Q value;

      /// \brief This variable specifies the parent frame that this
      /// FramedQuantity belongs to.
      private: FrameID parentFrame;
    };

    /////////////////////////////////////////////////
    /// \brief FrameSemantics is an Interface that can be provided by
    /// ignition-physics engines to provide users with easy ways to express
    /// kinematic quantities in terms of frames and compute their values in
    /// terms of arbitrary frames of reference.
    class FrameSemantics
    {
      /// \brief Resolve is able to take a FramedQuantity (FQ) and compute its
      /// values in terms of other reference frames. The argument `relativeTo`
      /// indicates a frame that the quantity should be compared against (e.g.
      /// the velocity of Frame A relative to Frame B where both A and B may be
      /// moving). The argument `inCoordinatesOf` indicates the coordinate frame
      /// that the values should be expressed in (this is usually just a change
      /// in rotation).
      public: template <typename FQ>
      typename FQ::Quantity Resolve(
        const FQ &_quantity,
        const FrameID _relativeTo,
        const FrameID _inCoordinatesOf);

      /// \brief This overload causes the World Frame to be used as the default
      /// frame when relativeTo is not specified. It also causes the frame
      /// specified for relativeTo to be used as the frame for inCoordinatesOf.
      ///
      /// In other words:
      ///
      /// Resolve(v) // Get the value of v in terms of the World Frame
      /// Resolve(v, A) // Get the value of v relative to frame A and in coordinates of frame A
      /// Resolve(v, A, B) // Get the value of v relative to frame A, but in the coordinates of frame B
      public: template <typename FQ>
      typename FQ::Quantity Resolve(
        const FQ &_quantity,
        const FrameID _relativeTo = WorldFrameID);

      /// \brief Create a new FramedQuantity which expresses the input quantity
      /// in terms of a new parent frame. Note that the returned FramedQuantity
      /// will behave as though it has a constant value within its new parent
      /// frame.
      public: template <typename FQ>
      FQ Reframe(const FQ &_quantity, const FrameID _withRespectTo = WorldFrameID);

      /// \brief Get the current 3D transformation of the specified frame with
      /// respect to the WorldFrame.
      public: virtual FrameData3d FrameDataRelativeToWorld(
          const FrameID &_id) const = 0;
    };

    namespace detail
    {
      /////////////////////////////////////////////////
      // Forward delcarations of ConfigSpaces
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
        FrameData3, 3, detail::FrameSpace<3, Scalar>>;
    using RelativeFrameData3d = RelativeFrameData3<double>;
    using RelativeFrameData3f = RelativeFrameData3<float>;
  }
}

#endif // IGNITION_PHYSICS_FRAMESEMANTICS_HH_
