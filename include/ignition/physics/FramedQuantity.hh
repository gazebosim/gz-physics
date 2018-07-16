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

namespace ignition
{
  namespace physics
  {
    /// \brief The FramedQuantity class is a wrapper for classes that represent
    /// mathematical quantities (e.g. points, vectors, matrices, transforms).
    /// The  purpose of this wrapper is to endow raw mathematical quantities
    /// with frame semantics, so that they can express the frame of reference of
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
    /// function in the FrameSemantics interface of your physics engine plugin
    /// to transform a FramedQuantity into a different reference frame.
    template <typename Q, std::size_t Dim, typename CoordinateSpace>
    class FramedQuantity
    {
      /// \brief This constructor will specify the parent frame and then forward
      /// the remaining arguments to the constructor of the underlying quantity.
      public: template <typename... Args>
      FramedQuantity(const FrameID &_parentID, Args&&... _args);

      /// \brief Implicit conversion constructor.
      // cppcheck-suppress noExplicitConstructor
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
      /// exists in. Only values of 2 or 3 are supported.
      public: enum { Dimension = Dim };

      /// \brief This variable specifies the parent frame that this
      /// FramedQuantity belongs to.
      private: FrameID parentFrame;

      /// \brief The raw quantity, expressed in terms of the parent frame.
      private: Q value;
    };

    template <typename Q, std::size_t Dim, typename CoordinateSpace>
    std::ostream& operator <<(
        std::ostream& stream,
        const FramedQuantity<Q, Dim, CoordinateSpace> &_fq)
    {
      stream << "Parent Frame ID: " << _fq.ParentFrame().ID()
             << "\nRelative To Parent:\n" << _fq.RelativeToParent();

      return stream;
    }

    namespace detail
    {
      /////////////////////////////////////////////////
      // Forward delcarations of CoordinateSpaces
      template <typename, std::size_t> struct SESpace;
      template <typename, std::size_t, typename> struct SOSpace;
      template <typename, std::size_t> struct EuclideanSpace;
      template <typename, std::size_t> struct LinearVelocitySpace;
      template <typename, std::size_t> struct AngularVelocitySpace;
      template <typename, std::size_t> struct LinearAccelerationSpace;
      template <typename, std::size_t> struct AngularAccelerationSpace;
      template <typename, std::size_t> struct VectorSpace;
      template <typename, std::size_t> struct FrameSpace;
      // TODO(MXG): We can add more spaces to support other types like Moments
      // of Inertia, Jacobians, Spatial Velocities/Accelerations, Wrench+Point
      // pairs, and so on.
      //
      // Users can also define Spaces for their own types (see the header
      // ignition/physics/detail/FrameSemantics.hpp for example implementations)
      // and use them seamlessly in the Frame Semantics infrastructure.
    }

    /////////////////////////////////////////////////
    template <typename Scalar, std::size_t Dim>
    using FramedPose = FramedQuantity<
        Pose<Scalar, Dim>, Dim, detail::SESpace<Scalar, Dim>>;
    IGN_PHYSICS_MAKE_ALL_TYPE_COMBOS(FramedPose)

    /////////////////////////////////////////////////
    template <typename Scalar, std::size_t Dim>
    using FramedRotationMatrix = FramedQuantity<
        Eigen::Matrix<Scalar, Dim, Dim>, Dim,
        detail::SOSpace<Scalar, Dim, Eigen::Matrix<Scalar, Dim, Dim>>>;
    IGN_PHYSICS_MAKE_ALL_TYPE_COMBOS(FramedRotationMatrix)

    /////////////////////////////////////////////////
    template <typename Scalar>
    using FramedQuaternion = FramedQuantity<
        Eigen::Quaternion<Scalar>, 3, detail::SOSpace<Scalar, 3,
        Eigen::Quaternion<Scalar>>>;
    // Note: Eigen only supports quaternions for 3 dimensional space, so we do
    // not have a dimensionality template argument.
    using FramedQuaterniond = FramedQuaternion<double>;
    using FramedQuaternionf = FramedQuaternion<float>;

    /////////////////////////////////////////////////
    template <typename Scalar, std::size_t Dim>
    using FramedPosition = FramedQuantity<
        LinearVector<Scalar, Dim>, Dim, detail::EuclideanSpace<Scalar, Dim>>;
    IGN_PHYSICS_MAKE_ALL_TYPE_COMBOS(FramedPosition)

    /////////////////////////////////////////////////
    template <typename Scalar, std::size_t Dim>
    using FramedForce = FramedQuantity<
        LinearVector<Scalar, Dim>, Dim, detail::VectorSpace<Scalar, Dim>>;
    IGN_PHYSICS_MAKE_ALL_TYPE_COMBOS(FramedForce)

    /////////////////////////////////////////////////
    template <typename Scalar, std::size_t Dim>
    using FramedTorque = FramedQuantity<
        AngularVector<Scalar, Dim>, Dim,
        detail::VectorSpace<Scalar, (Dim*(Dim-1))/2>>;
    IGN_PHYSICS_MAKE_ALL_TYPE_COMBOS(FramedTorque)

    /////////////////////////////////////////////////
    template <typename Scalar, std::size_t Dim>
    using RelativeFrameData = FramedQuantity<
        FrameData<Scalar, Dim>, Dim, detail::FrameSpace<Scalar, Dim>>;
    IGN_PHYSICS_MAKE_ALL_TYPE_COMBOS(RelativeFrameData)
  }
}

#include <ignition/physics/detail/FramedQuantity.hh>

#endif
