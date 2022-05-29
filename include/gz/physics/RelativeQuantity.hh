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

#ifndef GZ_PHYSICS_RELATIVEQUANTITY_HH_
#define GZ_PHYSICS_RELATIVEQUANTITY_HH_

#include <gz/physics/FrameID.hh>
#include <gz/physics/FrameData.hh>

namespace gz
{
  namespace physics
  {
    /// \brief The RelativeQuantity class is a wrapper for classes that
    /// represent mathematical quantities
    /// (e.g. points, vectors, matrices, transforms).
    /// The  purpose of this wrapper is to endow raw mathematical quantities
    /// with frame semantics, so that they can express the frame of reference of
    /// their values.
    ///
    /// Note that the raw value of the quantity being held can only be retrieved
    /// through the member function RelativeToParent(). We choose to contain the
    /// quantity like this instead of inheriting it to avoid a situation where
    /// a user might add two relative quantities like
    ///
    /// u = v + w
    ///
    /// where v and w belong to different frames. That would be an ill-formed
    /// expression, since values that are being expressed in different reference
    /// frames are not directly compatible. Use the Resolve(~) or Reframe(~)
    /// function in the FrameSemantics interface of your physics engine plugin
    /// to transform a RelativeQuantity into a different reference frame.
    template <typename Q, std::size_t Dim, typename CoordinateSpace>
    class RelativeQuantity
    {
      /// \brief This constructor will specify the parent frame and then forward
      /// the remaining arguments to the constructor of the underlying quantity.
      public: template <typename... Args>
      explicit RelativeQuantity(const FrameID &_parentID, Args&&... _args);

      /// \brief Implicit conversion constructor.
      public: RelativeQuantity(const Q &_rawValue);

      /// \brief Get the value of this RelativeQuantity relative to its parent
      /// frame. To get the value of this RelativeQuantity with respect to the
      /// world frame, use the FrameSemantics::Resolve(~) function of your
      /// physics engine like so:
      ///
      /// Q quantity = engine->GetInterface<FrameSemantics>()->Resolve(fq)
      ///
      /// where Q is your quantity type, and fq is your RelativeQuantity
      /// instance.
      ///
      /// To get the value of this RelativeQuantity with respect to an arbitrary
      /// reference frame, again use the Resolve function:
      ///
      /// Q quantity = engine->GetInterface<FrameSemantics>()->Resolve(fq, F);
      ///
      /// where F is the FrameID of the desired reference frame.
      public: Q &RelativeToParent();

      /// \brief const-qualified version of RelativeToParent.
      public: const Q &RelativeToParent() const;

      /// \brief Get the ID of this RelativeQuantity's parent frame.
      ///
      /// To change the parent frame of this RelativeQuantity, use the
      /// Reframe(~) function of your physics engine's FrameSemantics interface
      /// like this:
      ///
      /// fq = engine->GetInterface<FrameSemantics>()->Reframe(fq, A);
      ///
      /// where A is the FrameID of the new frame. The Reframe function will
      /// keep the values of your RelativeQuantity consistent (with respect to
      /// the World Frame) as it reassigns the parent frame.
      ///
      /// Alternatively, to change the parent frame of this RelativeQuantity
      /// while making its values relative to the new frame equal to what its
      /// values were relative to the old frame, you can use the
      /// MoveToNewParentFrame(~) function below.
      public: const FrameID &ParentFrame() const;

      /// \brief This function will change the parent frame of your
      /// RelativeQuantity.
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
      /// RelativeQuantity belongs to.
      private: FrameID parentFrame;

      /// \brief The raw quantity, expressed in terms of the parent frame.
      private: Q value;
    };

    template <typename Q, std::size_t Dim, typename CoordinateSpace>
    std::ostream& operator <<(
        std::ostream& stream,
        const RelativeQuantity<Q, Dim, CoordinateSpace> &_fq)
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
      template <typename, std::size_t> struct AABBSpace;
      template <typename, std::size_t> struct WrenchSpace;
      // TODO(MXG): We can add more spaces to support other types like Moments
      // of Inertia, Jacobians, Spatial Velocities/Accelerations, Wrench+Point
      // pairs, and so on.
      //
      // Users can also define Spaces for their own types (see the header
      // gz/physics/detail/FrameSemantics.hpp for example implementations)
      // and use them seamlessly in the Frame Semantics infrastructure.
    }

    /////////////////////////////////////////////////
    template <typename Scalar, std::size_t Dim>
    using RelativePose = RelativeQuantity<
        Pose<Scalar, Dim>, Dim, detail::SESpace<Scalar, Dim>>;
    IGN_PHYSICS_MAKE_ALL_TYPE_COMBOS(RelativePose)

    /////////////////////////////////////////////////
    template <typename Scalar, std::size_t Dim>
    using RelativeRotationMatrix = RelativeQuantity<
        Eigen::Matrix<Scalar, Dim, Dim>, Dim,
        detail::SOSpace<Scalar, Dim, Eigen::Matrix<Scalar, Dim, Dim>>>;
    IGN_PHYSICS_MAKE_ALL_TYPE_COMBOS(RelativeRotationMatrix)

    /////////////////////////////////////////////////
    template <typename Scalar>
    using RelativeQuaternion = RelativeQuantity<
        Eigen::Quaternion<Scalar>, 3, detail::SOSpace<Scalar, 3,
        Eigen::Quaternion<Scalar>>>;
    // Note: Eigen only supports quaternions for 3 dimensional space, so we do
    // not have a dimensionality template argument.
    using RelativeQuaterniond = RelativeQuaternion<double>;
    using RelativeQuaternionf = RelativeQuaternion<float>;

    /////////////////////////////////////////////////
    template <typename Scalar, std::size_t Dim>
    using RelativePosition = RelativeQuantity<
        LinearVector<Scalar, Dim>, Dim, detail::EuclideanSpace<Scalar, Dim>>;
    IGN_PHYSICS_MAKE_ALL_TYPE_COMBOS(RelativePosition)

    /////////////////////////////////////////////////
    template <typename Scalar, std::size_t Dim>
    using RelativeForce = RelativeQuantity<
        LinearVector<Scalar, Dim>, Dim, detail::VectorSpace<Scalar, Dim>>;
    IGN_PHYSICS_MAKE_ALL_TYPE_COMBOS(RelativeForce)

    /////////////////////////////////////////////////
    template <typename Scalar, std::size_t Dim>
    using RelativeTorque = RelativeQuantity<
        AngularVector<Scalar, Dim>, Dim,
        detail::VectorSpace<Scalar, (Dim*(Dim-1))/2>>;
    IGN_PHYSICS_MAKE_ALL_TYPE_COMBOS(RelativeTorque)

    /////////////////////////////////////////////////
    template <typename Scalar, std::size_t Dim>
    using RelativeAlignedBox = RelativeQuantity<
        AlignedBox<Scalar, Dim>, Dim, detail::AABBSpace<Scalar, Dim>>;
    IGN_PHYSICS_MAKE_ALL_TYPE_COMBOS(RelativeAlignedBox)

    /////////////////////////////////////////////////
    template <typename Scalar, std::size_t Dim>
    using RelativeFrameData = RelativeQuantity<
        FrameData<Scalar, Dim>, Dim, detail::FrameSpace<Scalar, Dim>>;
    IGN_PHYSICS_MAKE_ALL_TYPE_COMBOS(RelativeFrameData)

    /////////////////////////////////////////////////
    template <typename Scalar, std::size_t Dim>
    using RelativeWrench = RelativeQuantity<
        Wrench<Scalar, Dim>, Dim, detail::WrenchSpace<Scalar, Dim>>;
    IGN_PHYSICS_MAKE_ALL_TYPE_COMBOS(RelativeWrench)
  }
}

#include <gz/physics/detail/RelativeQuantity.hh>

#endif
