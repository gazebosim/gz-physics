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

#ifndef IGNITION_PHYSICS_DETAIL_FRAMEDQUANTITY_HH_
#define IGNITION_PHYSICS_DETAIL_FRAMEDQUANTITY_HH_

#include <utility>

#include <ignition/physics/FramedQuantity.hh>

namespace ignition
{
  namespace physics
  {
    /////////////////////////////////////////////////
    template <typename Q, std::size_t Dim, typename CoordinateSpace>
    template <typename... Args>
    FramedQuantity<Q, Dim, CoordinateSpace>::FramedQuantity(
        const FrameID &_parentID, Args&&... _args)
      : parentFrame(_parentID),
        value(std::forward<Args>(_args)...)
    {
      // Do nothing
    }

    /////////////////////////////////////////////////
    template <typename Q, std::size_t Dim, typename CoordinateSpace>
    FramedQuantity<Q, Dim, CoordinateSpace>::FramedQuantity(const Q &_rawValue)
      : parentFrame(FrameID::World()),
        value(_rawValue)
    {
      // Do nothing
    }

    /////////////////////////////////////////////////
    template <typename Q, std::size_t Dim, typename CoordinateSpace>
    Q &FramedQuantity<Q, Dim, CoordinateSpace>::RelativeToParent()
    {
      return value;
    }

    /////////////////////////////////////////////////
    template <typename Q, std::size_t Dim, typename CoordinateSpace>
    const Q &FramedQuantity<Q, Dim, CoordinateSpace>::RelativeToParent() const
    {
      return value;
    }

    /////////////////////////////////////////////////
    template <typename Q, std::size_t Dim, typename CoordinateSpace>
    const FrameID &FramedQuantity<Q, Dim, CoordinateSpace>::ParentFrame() const
    {
      return parentFrame;
    }

    namespace detail
    {
      /////////////////////////////////////////////////
      /// \brief This macro provides some typedefs which are used (or will be
      /// used) by Resolve and Reframe. Each different CoordinateSpace type
      /// should include this macro in its class definition.
      ///
      /// _Dim should be 2 or 3 depending on whether the quantity is intended
      /// for 2D or 3D simulation (currently we only support 3D).
      ///
      /// _Quantity should be the type of quantity that the coordinate space is
      /// intended for.
      ///
      /// _Scalar should be the 1D primitive type which determines the numerical
      /// precision (i.e. double or float).
      #define IGNITION_PHYSICS_DEFINE_COORDINATE_SPACE(...) \
        public: using Quantity = __VA_ARGS__; \
        public: using Scalar = _Scalar; \
        public: using FrameDataType = FrameData<Scalar, _Dim>; \
        public: using PoseType = Pose<Scalar, _Dim>; \
        public: using RotationType = Eigen::Matrix<Scalar, _Dim, _Dim>; \
        public: enum { Dimension = _Dim };


      /////////////////////////////////////////////////
      /// \brief SESpace is used by SE(2) and SE(3) [Special Euclidean Group]
      /// constructs, e.g. homogeneous transformation matrices.
      template <typename _Scalar, std::size_t _Dim>
      struct SESpace
      {
        IGNITION_PHYSICS_DEFINE_COORDINATE_SPACE(Pose<_Scalar, _Dim>)

        /// \brief Resolve the pose to the world frame
        public: static Quantity ResolveToWorldFrame(
          const Quantity &_pose,
          const FrameDataType &_parentFrame)
        {
          return _parentFrame.pose * _pose;
        }

        /// \brief Resolve the pose to the target frame
        public: static Quantity ResolveToTargetFrame(
          const Quantity &_pose,
          const FrameDataType &_parentFrame,
          const FrameDataType &_targetFrame)
        {
          return _targetFrame.pose.Inverse()
                 * _parentFrame.pose
                 * _pose;
        }

        /// \brief Resolve the coordinates of the pose to the world frame
        public: static Quantity ResolveToWorldCoordinates(
          const Quantity &_pose,
          const RotationType &_currentCoordinates)
        {
          // Premultiplying by a homogeneous transform with zero translation is
          // the same as changing the coordinates of a pose.
          return PoseType(_currentCoordinates) * _pose;
        }

        /// \brief Resolve the coordinates of the pose to the target frame
        public: static Quantity ResolveToTargetCoordinates(
          const Quantity &_pose,
          const RotationType &_currentCoordinates,
          const RotationType &_targetCoordinates)
        {
          return PoseType(_targetCoordinates.transpose())
                 * PoseType(_currentCoordinates) * _pose;
        }
      };

      /////////////////////////////////////////////////
      /// \brief SOSpace is used by SO(2|3) [Special Orthogonal Group 2 and 3]
      /// constructs like rotation matrices or quaternions.
      template <typename _Scalar, std::size_t _Dim, typename _Quantity>
      struct SOSpace
      {
        IGNITION_PHYSICS_DEFINE_COORDINATE_SPACE(_Quantity)

        /// \brief Resolve the rotation to the world frame
        public: static Quantity ResolveToWorldFrame(
          const Quantity &_rotation,
          const FrameDataType &_parentFrame)
        {
          return ResolveToWorldCoordinates(
                _rotation, _parentFrame.pose.linear());
        }

        /// \brief Resolve the rotation to the target frame
        public: static Quantity ResolveToTargetFrame(
          const Quantity &_rotation,
          const FrameDataType &_parentFrame,
          const FrameDataType &_targetFrame)
        {
          return ResolveToTargetCoordinates(
                _rotation,
                _parentFrame.pose.linear(),
                _targetFrame.pose.linear());
        }

        /// \brief Resolve the coordinates of the rotation to the world frame
        public: static Quantity ResolveToWorldCoordinates(
          const Quantity &_rotation,
          const RotationType &_currentCoordinates)
        {
          return Quantity(_currentCoordinates * _rotation);
        }

        /// \brief Resolve the coordinates of the rotation to the target frame
        public: static Quantity ResolveToTargetCoordinates(
          const Quantity &_rotation,
          const RotationType &_currentCoordinates,
          const RotationType &_targetCoordinates)
        {
          return Quantity(_targetCoordinates.transpose()
                          * _currentCoordinates
                          * _rotation);
        }
      };

      /////////////////////////////////////////////////
      /// \brief EuclideanSpace is used by points (i.e. positions or
      /// translations).
      template <typename _Scalar, std::size_t _Dim>
      struct EuclideanSpace : public VectorSpace<_Scalar, _Dim>
      {
        IGNITION_PHYSICS_DEFINE_COORDINATE_SPACE(Vector<_Scalar, _Dim>)

        /// \brief Resolve the point to the world frame
        public: static Quantity ResolveToWorldFrame(
          const Quantity &_point,
          const FrameDataType &_parentFrame)
        {
          return _parentFrame.pose * _point;
        }

        /// \brief Resolve to the target frame
        public: static Quantity ResolveToTargetFrame(
          const Quantity &_point,
          const FrameDataType &_parentFrame,
          const FrameDataType &_targetFrame)
        {
          return _targetFrame.pose.inverse() * _parentFrame.pose * _point;
        }

        // Note: We inherit ResolveToWorldCoordinates and
        // ResolveToTargetCoordinates from VectorSpace since they are the same
        // as what we need for EuclideanSpace. However, the EuclideanSpace
        // implementations of ResolveToWorldFrame and ResolveToTargetFrame are
        // different than VectorSpace's, so we shadow those functions.
      };

      /////////////////////////////////////////////////
      /// \brief VectorSpace is used by classical "free vectors", like Force and
      /// Torque, which are not dependent on any of the properties of a frame,
      /// besides the orientation.
      template <typename _Scalar, std::size_t _Dim>
      struct VectorSpace
      {
        IGNITION_PHYSICS_DEFINE_COORDINATE_SPACE(Vector<_Scalar, _Dim>)

        /// \brief Resolve the vector to the world frame
        public: static Quantity ResolveToWorldFrame(
          const Quantity &_vec,
          const FrameDataType &_parentFrame)
        {
          return ResolveToWorldCoordinates(_vec, _parentFrame.pose.linear());
        }

        /// \brief Resolve the vector to the target frame
        public: static Quantity ResolveToTargetFrame(
          const Quantity &_vec,
          const FrameDataType &_parentFrame,
          const FrameDataType &_targetFrame)
        {
          return ResolveToTargetCoordinates(
                _vec,
                _parentFrame.pose.linear(),
                _targetFrame.pose.linear());
        }

        /// \brief Resolve the coordinates of the vector to the world frame.
        public: static Quantity ResolveToWorldCoordinates(
          const Quantity &_vec,
          const RotationType &_currentCoordinates)
        {
          return _currentCoordinates * _vec;
        }

        /// \brief Resolve the coordinates of the vector to the target frame.
        public: static Quantity ResolveToTargetCoordinates(
          const Quantity &_vec,
          const RotationType &_currentCoordinates,
          const RotationType &_targetCoordinates)
        {
          return _targetCoordinates.transpose() * _currentCoordinates * _vec;
        }

        /// \brief Resolve the coordinates of the vector from the world
        /// coordinates to the target coordinates. This is a helper function for
        /// some of the classes that use VectorSpace. It is not used directly by
        /// Resolve(~) or Reframe(~).
        public: static Quantity ResolveFromWorldToTargetCoordinates(
          const Quantity &_vec,
          const RotationType &_targetCoordinates)
        {
          return _targetCoordinates.transpose() * _vec;
        }
      };

      /////////////////////////////////////////////////
      /// \brief This is a partial template specialization for 1D "vector"
      /// types. Examples include angular velocity, angular acceleration, and
      /// torque in 2D simulations.
      ///
      /// These operations are no-ops, because the orientation of an "angular"
      /// vector cannot be changed in a 2D simulation; it is always orthogonal
      /// to the xy-plane.
      template <typename _Scalar>
      struct VectorSpace<_Scalar, 1>
      {
        // Dev note: Normally _Dim is provided as a template argument, but this
        // is a partially specialized template. The macro assumes the existence
        // of both "_Scalar" and "_Dim", so we define this enum as a workaround.
        private: enum { _Dim = 2 };
        IGNITION_PHYSICS_DEFINE_COORDINATE_SPACE(Vector<_Scalar, 1>)

        /// \brief Resolve the vector to the world frame
        public: static Quantity ResolveToWorldFrame(
          const Quantity &_vec, const FrameDataType &)
        {
          return _vec;
        }

        /// \brief Resolve the vector to the target frame
        public: static Quantity ResolveToTargetFrame(
          const Quantity &_vec, const FrameDataType &, const FrameDataType &)
        {
          return _vec;
        }

        /// \brief Resolve the coordinates of the vector to the world frame.
        public: static Quantity ResolveToWorldCoordinates(
          const Quantity &_vec, const RotationType &)
        {
          return _vec;
        }

        /// \brief Resolve the coordinates of the vector to the target frame.
        public: static Quantity ResolveToTargetCoordinates(
          const Quantity &_vec, const RotationType &, const RotationType &)
        {
          return _vec;
        }

        /// \brief Resolve the coordinates of the vector from the world
        /// coordinates to the target coordinates. This is a helper function for
        /// some of the classes that use VectorSpace. It is not used directly by
        /// Resolve(~) or Reframe(~).
        public: static Quantity ResolveFromWorldToTargetCoordinates(
          const Quantity &_vec, const RotationType &)
        {
          return _vec;
        }
      };

      /////////////////////////////////////////////////
      /// \brief The Operator class allows us to generalize certain math
      /// operations depending on whether we are working in 2D space or 3D
      /// space. Currently, we only need this for the cross product.
      template <typename _Scalar, std::size_t _Dim>
      struct Operator
      {
        public: using LinearVectorType = LinearVector<_Scalar, _Dim>;
        public: using AngularVectorType = AngularVector<_Scalar, _Dim>;

        /// \brief A cross-product operator for 3D simulation.
        ///
        /// Since AngularVectorType and LinearVectorType are equivalent in 3D
        /// simulation, this is suitable for cross products between:
        /// linear x linear
        /// angular x angular
        /// angular x linear
        /// linear x angular
        public: static LinearVectorType Cross(
          const AngularVectorType &_v1,
          const LinearVectorType &_v2)
        {
          return _v1.cross(_v2);
        }
      };

      /////////////////////////////////////////////////
      /// \brief This is a partial template specialization for Operator which
      /// allows us to customize operations for 2D space.
      template <typename _Scalar>
      struct Operator<_Scalar, 2>
      {
        public: using LinearVectorType = LinearVector<_Scalar, 2>;
        public: using AngularVectorType = AngularVector<_Scalar, 2>;

        /// \brief A 2D cross product operator for linear x linear vectors.
        /// Note that this takes two 2D vectors and produces a 1D "vector".
        ///
        /// An example is torque = radius x force
        public: static AngularVectorType Cross(
          const LinearVectorType &_u,
          const LinearVectorType &_v)
        {
          return AngularVectorType(_u[0]*_v[1] - _u[1]*_v[0]);
        }

        /// \brief A 2D cross product operator for angular x angular vectors.
        /// Technically the AngularVectorType is a scalar, so calling this a
        /// cross product may be a misnomer. However, defining this operator
        /// allows us to generalize some mathematical expressions between 2D
        /// and 3D.
        ///
        /// This always produces zero, because the "vectors" u and v can be
        /// imagined as vectors which only have a z-component and no x or y
        /// component. Cross-producting two such vectors will produce a zero
        /// vector.
        public: constexpr static AngularVectorType Cross(
          const AngularVectorType &,
          const AngularVectorType &)
        {
          return AngularVectorType(0.0);
        }

        /// \brief A 2D cross product operator for angular x linear vectors.
        /// Technically the AngularVectorType is a scalar, so calling this a
        /// cross product may be a misnomer. However, defining this operator
        /// allows us to generalize some mathematical expressions between 2D
        /// and 3D.
        ///
        /// An example is tangential_velocity = angular_velocity x radius
        public: static LinearVectorType Cross(
          const AngularVectorType &_u,
          const LinearVectorType &_v)
        {
          return LinearVectorType(-_u[0]*_v[1], _u[0]*_v[0]);
        }

        /// \brief A 2D cross product operator for linear x angular vectors.
        /// Technically the AngularVectorType is a scalar, so calling this a
        /// cross product may be a misnomer. However, defining this operator
        /// allows us to generalize some mathematical expressions between 2D
        /// and 3D.
        ///
        /// Note that this will produce the opposite result as the
        /// angular x linear version of the cross product operator. This is to
        /// maintain consistency for the cross product, because we need to
        /// follow the rule a x b == -b x a.
        public: static LinearVectorType Cross(
          const LinearVectorType &_v,
          const AngularVectorType &_u)
        {
          return LinearVectorType(_u[0]*_v[1], -_u[0]*_v[0]);
        }
      };

      /////////////////////////////////////////////////
      /// \brief FrameSpace is used to compute RelativeFrameData. This space is
      /// able to account for Coriolis and centrifugal effects in non-inertial
      /// reference frames.
      template <typename _Scalar, std::size_t _Dim>
      struct FrameSpace
      {
        IGNITION_PHYSICS_DEFINE_COORDINATE_SPACE(FrameData<_Scalar, _Dim>)

        using LinearVectorType = LinearVector<_Scalar, _Dim>;
        using LinearVectorSpaceType = VectorSpace<_Scalar, _Dim>;
        using AngularVectorType = AngularVector<_Scalar, _Dim>;
        using AngularVectorSpaceType = VectorSpace<_Scalar, (_Dim*(_Dim-1))/2>;
        using Op = Operator<_Scalar, _Dim>;

        /// \brief Express the data of the frame with respect to the world
        /// frame.
        public: static Quantity ResolveToWorldFrame(
          const Quantity &_relativeFrameData,
          const FrameDataType &_parentFrame)
        {
          Quantity resultFrameData;

          // The transform of the input frame, with respect to the world frame.
          resultFrameData.pose = _parentFrame.pose * _relativeFrameData.pose;

          // The orientation of the parent frame with respect to the world
          // frame.
          const RotationType &R_parent = _parentFrame.pose.linear();

          // The position of our input frame, relative to its parent frame,
          // expressed in world coordinates.
          const LinearVectorType &p_rel =
              LinearVectorSpaceType::ResolveToWorldCoordinates(
                _relativeFrameData.pose.translation(), R_parent);

          // The linear velocity of our input frame, relative to its parent
          // frame, expressed in world coordinates.
          const LinearVectorType &v_rel =
              LinearVectorSpaceType::ResolveToWorldCoordinates(
                _relativeFrameData.linearVelocity, R_parent);

          // The linear velocity of the parent frame, relative to the world
          // frame, expressed in world coordinates.
          const LinearVectorType &v_parent = _parentFrame.linearVelocity;

          // The angular velocity of the parent frame, relative to the world
          // frame, expressed in world coordinates.
          const AngularVectorType &w_parent = _parentFrame.angularVelocity;

          // The velocity of our input frame, relative to the world frame,
          // expressed in world coordinates.
          const LinearVectorType &v =
              v_parent + v_rel + Op::Cross(w_parent, p_rel);

          // Copy the linear velocity into our result data.
          resultFrameData.linearVelocity = v;

          // The linear acceleration of our input frame, relative to its parent
          // frame, expressed in world coordinates.
          const LinearVectorType &a_rel =
              LinearVectorSpaceType::ResolveToWorldCoordinates(
                _relativeFrameData.linearAcceleration, R_parent);

          // The linear acceleration of the parent frame, relative to the world
          // frame, expressed in world coordinates.
          const LinearVectorType &a_parent = _parentFrame.linearAcceleration;

          // The angular acceleration of the parent frame, relative to the world
          // frame, expressed in world coordinates.
          const AngularVectorType &alpha_parent =
              _parentFrame.angularAcceleration;

          // The linear acceleration of the input frame, relative to the world
          // frame, expressed in world coordinates.
          const LinearVectorType &a =
              a_parent + a_rel
              + Op::Cross(alpha_parent, p_rel)
              + 2*Op::Cross(w_parent, v_rel)
              + Op::Cross(w_parent, Op::Cross(w_parent, p_rel));

          // Copy the linear acceleration into our result data.
          resultFrameData.linearAcceleration = a;

          // The angular velocity of the input frame, relative to its parent
          // frame, expressed in world coordinates.
          const AngularVectorType &w_rel =
              AngularVectorSpaceType::ResolveToWorldCoordinates(
                _relativeFrameData.angularVelocity, R_parent);

          // The angular velocity of the input frame, relative to the world
          // frame, expressed in world coordinates.
          const AngularVectorType &w = w_parent + w_rel;

          // Copy the angular velocity into our result data.
          resultFrameData.angularVelocity = w;

          // The angular acceleration of the input frame, relative to the world
          // frame, expressed in world coordinates.
          const AngularVectorType &alpha_rel =
              AngularVectorSpaceType::ResolveToWorldCoordinates(
                _relativeFrameData.angularAcceleration, R_parent);

          // The angular acceleration of the input frame, relative to the world
          // frame, expressed in world coordinates.
          const AngularVectorType &alpha =
              alpha_parent + alpha_rel + Op::Cross(w_parent, w_rel);

          // Copy the angular acceleration into our result data.
          resultFrameData.angularAcceleration = alpha;

          return resultFrameData;
        }

        /// \brief Express the data of the frame with respect to the target
        /// frame.
        public: static Quantity ResolveToTargetFrame(
          const Quantity &_relativeFrameData,
          const FrameDataType &_parentFrame,
          const FrameDataType &_targetFrame)
        {
          Quantity resultFrameData;

          // First we compute the input frame data with respect to the world
          // frame
          const Quantity &frameDataWrtWorld =
              ResolveToWorldFrame(_relativeFrameData, _parentFrame);

          // The pose of the input frame, with respect to the target frame.
          resultFrameData.pose =
              _targetFrame.pose.inverse() * frameDataWrtWorld.pose;

          // The position of the input frame, relative to the target frame, in
          // coordinates of the world frame.
          const LinearVectorType &p_rel =
              frameDataWrtWorld.pose.translation()
              - _targetFrame.pose.translation();

          // The orientation of the target frame, with respect to the world
          // frame.
          const RotationType &R_target = _targetFrame.pose.linear();

          // The linear velocity of the input frame, with respect to the world
          // frame.
          const LinearVectorType &v = frameDataWrtWorld.linearVelocity;

          // The linear velocity of the target frame, with respect to the world
          // frame.
          const LinearVectorType &v_target = _targetFrame.linearVelocity;

          // The angular velocity of the target frame, with respect to the world
          // frame.
          const AngularVectorType &w_target = _targetFrame.angularVelocity;

          // The linear velocity of the input frame, relative to the target
          // frame, in coordinates of the world frame.
          const LinearVectorType &v_rel =
              v - v_target - Op::Cross(w_target, p_rel);

          // Convert the linear velocity into the target coordinates, and then
          // copy it to the result data.
          resultFrameData.linearVelocity =
              LinearVectorSpaceType::ResolveFromWorldToTargetCoordinates(
                v_rel, _targetFrame.pose.linear());

          // The linear acceleration of the input frame, with respect to the
          // world frame.
          const LinearVectorType &a = frameDataWrtWorld.linearAcceleration;

          // The linear acceleration of the target frame, with respect to the
          // world frame.
          const LinearVectorType &a_target = _targetFrame.linearAcceleration;

          // The angular acceleration of the target frame, with respect to the
          // world frame.
          const AngularVectorType &alpha_target =
              _targetFrame.angularAcceleration;

          // The angular acceleration of the input frame, relative to the target
          // frame, in coordinates of the world frame.
          const LinearVectorType &a_rel =
              a - a_target
              - Op::Cross(alpha_target, p_rel)
              - 2*Op::Cross(w_target, v_rel)
              - Op::Cross(w_target, Op::Cross(w_target, p_rel));

          // Convert the linear acceleration into the target coordinates, and
          // then copy it to the result data.
          resultFrameData.linearAcceleration =
              LinearVectorSpaceType::ResolveFromWorldToTargetCoordinates(
                a_rel, _targetFrame.pose.linear());

          // The angular velocity of the input frame, with respect to the world
          // frame.
          const AngularVectorType &w = frameDataWrtWorld.angularVelocity;

          // The angular velocity of the input frame, relative to the target
          // frame, in coordinates of the world frame.
          const AngularVectorType &w_rel = w - w_target;

          // Convert the angular velocity into the target coordinates, and then
          // copy it to the result data.
          resultFrameData.angularVelocity =
              AngularVectorSpaceType::ResolveFromWorldToTargetCoordinates(
                w_rel, _targetFrame.pose.linear());

          // The angular acceleration of the input frame, with respect to the
          // world frame.
          const AngularVectorType &alpha =
              frameDataWrtWorld.angularAcceleration;

          // The angular acceleration of the input frame, relative to the target
          // frame, in coordinates of the world frame.
          const AngularVectorType &alpha_rel =
              alpha - alpha_target - Op::Cross(w_target, w_rel);

          // Convert the angular acceleration into the target coordinates, and
          // then copy it to the result data.
          resultFrameData.angularAcceleration =
              AngularVectorSpaceType::ResolveFromWorldToTargetCoordinates(
                alpha_rel, R_target);

          return resultFrameData;
        }

        /// \brief Resolve the coordinates of the specified property to the
        /// world coordinates. Note that this does not work for the transform
        /// property; it is only meant for linear/angular velocity/acceleration.
        public: template <typename PropertyType,
                          typename PropertySpace,
                          PropertyType Quantity::*property>
        static void ResolvePropertyToWorldCoordinates(
          Quantity &_output, const FrameDataType &_input,
          const RotationType &_currentCoordinates)
        {
          _output.*property =
              PropertySpace::ResolveToWorldCoordinates(
                _input.*property, _currentCoordinates);
        }

        /// \brief Resolve the coordinates of the specified property to the
        /// target coordinates. Note that this does not work for the transform
        /// property; it is only meant for linear/angular velocity/acceleration.
        public: template <typename PropertyType,
                          typename PropertySpace,
                          PropertyType Quantity::*property>
        static void ResolvePropertyToTargetCoordinates(
          Quantity &_output, const FrameDataType &_input,
          const RotationType &_currentCoordinates,
          const RotationType &_targetCoordinates)
        {
          _output.*property =
              PropertySpace::ResolveToTargetCoordinates(
                _input.*property, _currentCoordinates, _targetCoordinates);
        }

        /// \brief Resolve the coordinates of the frame to the world frame.
        public: static Quantity ResolveToWorldCoordinates(
          const Quantity &_inputFrameData,
          const RotationType &_currentCoordinates)
        {
          Quantity resultFrameData;

          resultFrameData.pose =
              SESpace<_Scalar, _Dim>::ResolveToWorldCoordinates(
                _inputFrameData.pose, _currentCoordinates);

          ResolvePropertyToWorldCoordinates<
              LinearVectorType, LinearVectorSpaceType,
              &Quantity::linearVelocity>(
                resultFrameData, _inputFrameData, _currentCoordinates);

          ResolvePropertyToWorldCoordinates<
              LinearVectorType, LinearVectorSpaceType,
              &Quantity::linearAcceleration>(
                resultFrameData, _inputFrameData, _currentCoordinates);

          ResolvePropertyToWorldCoordinates<
              AngularVectorType, AngularVectorSpaceType,
              &Quantity::angularVelocity>(
                resultFrameData, _inputFrameData, _currentCoordinates);

          ResolvePropertyToWorldCoordinates<
              AngularVectorType, AngularVectorSpaceType,
              &Quantity::angularAcceleration>(
                resultFrameData, _inputFrameData, _currentCoordinates);

          return resultFrameData;
        }

        /// \brief Resolve the coordinates of the frame to the target frame.
        public: static Quantity ResolveToTargetCoordinates(
            const Quantity &_inputFrameData,
            const RotationType &_currentCoordinates,
            const RotationType &_targetCoordinates)
        {
          Quantity resultFrameData;

          resultFrameData.pose =
              SESpace<_Scalar, _Dim>::ResolveToTargetCoordinates(
                _inputFrameData.pose, _currentCoordinates,
                _targetCoordinates);

          ResolvePropertyToTargetCoordinates<
              LinearVectorType, LinearVectorSpaceType,
              &Quantity::linearVelocity>(
                resultFrameData, _inputFrameData,
                _currentCoordinates, _targetCoordinates);

          ResolvePropertyToTargetCoordinates<
              LinearVectorType, LinearVectorSpaceType,
              &Quantity::linearAcceleration>(
                resultFrameData, _inputFrameData,
                _currentCoordinates, _targetCoordinates);

          ResolvePropertyToTargetCoordinates<
              AngularVectorType, AngularVectorSpaceType,
              &Quantity::angularVelocity>(
                resultFrameData, _inputFrameData,
                _currentCoordinates, _targetCoordinates);

          ResolvePropertyToTargetCoordinates<
              AngularVectorType, AngularVectorSpaceType,
              &Quantity::angularAcceleration>(
                resultFrameData, _inputFrameData,
                _currentCoordinates, _targetCoordinates);

          return resultFrameData;
        }
      };
    }
  }
}

#endif
