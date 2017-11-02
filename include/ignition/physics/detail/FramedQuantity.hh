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
      // Note that all the different CoordinateSpace types have a std::size_t
      // template parameter which is currently going unused. That parameter is a
      // placeholder for forwards compatibility so that we can easily transition
      // into supporting 2D simulation, if we decide to pursue that later.


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
      /// \brief SESpace is used by SE(2) and SE(3) constructs, e.g. homogeneous
      /// transformation matrices.
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
      /// \brief SOSpace is used by SO(3) constructs (and perhaps in the future
      /// SO(2) constructs as well) like rotation matrices or quaternions.
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
                _rotation, _parentFrame.pose.Rot());
        }

        /// \brief Resolve the rotation to the target frame
        public: static Quantity ResolveToTargetFrame(
          const Quantity &_rotation,
          const FrameDataType &_parentFrame,
          const FrameDataType &_targetFrame)
        {
          return ResolveToTargetCoordinates(
                _rotation,
                _parentFrame.pose.Rot(),
                _targetFrame.pose.Rot());
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
      /// \brief This is an implementation for the Linear/Angular
      /// Velocity/Acceleration Spaces. Those spaces only differ by which
      /// kinematic property is being summed, so we use a pointer to a class
      /// member as a template parameter to choose which property we sum.
      template <typename _Scalar, std::size_t _Dim,
                Vector<_Scalar, _Dim> FrameData<_Scalar, _Dim>::*property>
      struct KinematicDerivativeSpace : public VectorSpace<_Scalar, _Dim>
      {
        IGNITION_PHYSICS_DEFINE_COORDINATE_SPACE(Vector<_Scalar, _Dim>)

        /// \brief Resolve the property to the world frame
        public: static Quantity ResolveToWorldFrame(
          const Quantity &_vec,
          const FrameDataType &_parentFrame)
        {
          // Get the property that we care about from the parent frame. This
          // may be its (linear or angular) (velocity or acceleration).
          const Quantity &parentProperty = _parentFrame.*property;
          const Quantity vecInWorldCoordinates =
              ResolveToWorldCoordinates(_vec, _parentFrame.pose.Rot());

          return parentProperty + vecInWorldCoordinates;
        }

        /// \brief Resolve the property to the target frame
        public: static Quantity ResolveToTargetFrame(
          const Quantity &_vec,
          const FrameDataType &_parentFrame,
          const FrameDataType &_targetFrame)
        {
          // First we find all the relevant vectors in world coordinates. The
          // FrameData that gets passed to us is already expressed in world
          // coordinates, so we don't need to do anything with those.
          const Quantity &parentProperty = _parentFrame.*property;
          const Quantity &targetProperty = _targetFrame.*property;

          // The vector quantity that gets passed to us is in coordinates of the
          // parent frame, we so we must convert it from the parent coordinates
          // into world coordinates.
          const Quantity vecInWorldCoordinates =
              ResolveToWorldCoordinates(_vec, _parentFrame.pose.Rot());

          // Now that all our vectors are in world coordinates, we can safely
          // sum/difference them. Note that the result of this sum will also
          // be in world coordinates.
          const Quantity resultInWorldCoordinates =
              parentProperty + vecInWorldCoordinates - targetProperty;

          // The caller expects the result that we give back to be in the target
          // coordinates, so we must rotate it from the world coordinates into
          // the target coordinates before returning.
          return ResolveFromWorldToTargetCoordinates(
                resultInWorldCoordinates,
                _targetFrame.pose.Rot());
        }

        // Note: We inherit ResolveToWorldCoordinates and
        // ResolveToTargetCoordinates from VectorSpace since they are the same
        // as what we need for these kinematic derivatives.
      };

      /////////////////////////////////////////////////
      /// \brief LinearVelocitySpace is used by linear velocity vectors.
      template <typename _Scalar, std::size_t _Dim>
      struct LinearVelocitySpace
          : public KinematicDerivativeSpace<_Scalar, _Dim,
              &FrameData<_Scalar, _Dim>::linearVelocity> { };

      /////////////////////////////////////////////////
      /// \brief AngularVelocitySpace is used by angular velocity vectors.
      template <typename _Scalar, std::size_t _Dim>
      struct AngularVelocitySpace
          : public KinematicDerivativeSpace<_Scalar, _Dim,
              &FrameData<_Scalar, _Dim>::angularVelocity> { };

      /////////////////////////////////////////////////
      /// \brief LinearAccelerationSpace is used by linear acceleration vectors.
      template <typename _Scalar, std::size_t _Dim>
      struct LinearAccelerationSpace
          : public KinematicDerivativeSpace<_Scalar, _Dim,
              &FrameData<_Scalar, _Dim>::linearAcceleration> { };

      /////////////////////////////////////////////////
      /// \brief AngularAccelerationSpace is used by angular acceleration
      /// vectors.
      template <typename _Scalar, std::size_t _Dim>
      struct AngularAccelerationSpace
          : public KinematicDerivativeSpace<_Scalar, _Dim,
              &FrameData<_Scalar, _Dim>::angularAcceleration> { };

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
          return ResolveToWorldCoordinates(_parentFrame.pose.Rot(), _vec);
        }

        /// \brief Resolve the vector to the target frame
        public: static Quantity ResolveToTargetFrame(
          const Quantity &_vec,
          const FrameDataType &_parentFrame,
          const FrameDataType &_targetFrame)
        {
          return ResolveToTargetCoordinates(
                _vec,
                _parentFrame.pose.Rot(),
                _targetFrame.pose.Rot());
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
      /// \brief FrameSpace is used to compute RelativeFrameData. This space is
      /// able to account for Coriolis and centrifugal effects in non-inertial
      /// reference frames.
      template <typename _Scalar, std::size_t _Dim>
      struct FrameSpace
      {
        IGNITION_PHYSICS_DEFINE_COORDINATE_SPACE(FrameData<_Scalar, _Dim>)

        using VectorType = Vector<_Scalar, _Dim>;
        using VectorSpaceType = VectorSpace<_Scalar, _Dim>;

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
          const VectorType &p_rel =
              VectorSpaceType::ResolveToWorldCoordinates(
                _relativeFrameData.pose.translation(), R_parent);

          // The linear velocity of our input frame, relative to its parent
          // frame, expressed in world coordinates.
          const VectorType &v_rel =
              VectorSpaceType::ResolveToWorldCoordinates(
                _relativeFrameData.linearVelocity, R_parent);

          // The linear velocity of the parent frame, relative to the world
          // frame, expressed in world coordinates.
          const VectorType &v_parent = _parentFrame.linearVelocity;

          // The angular velocity of the parent frame, relative to the world
          // frame, expressed in world coordinates.
          const VectorType &w_parent = _parentFrame.angularVelocity;

          // The velocity of our input frame, relative to the world frame,
          // expressed in world coordinates.
          const VectorType &v = v_parent + v_rel + w_parent.cross(p_rel);

          // Copy the linear velocity into our result data.
          resultFrameData.linearVelocity = v;

          // The linear acceleration of our input frame, relative to its parent
          // frame, expressed in world coordinates.
          const VectorType &a_rel =
              VectorSpaceType::ResolveToWorldCoordinates(
                _relativeFrameData.angularAcceleration, R_parent);

          // The linear acceleration of the parent frame, relative to the world
          // frame, expressed in world coordinates.
          const VectorType &a_parent = _parentFrame.linearAcceleration;

          // The angular acceleration of the parent frame, relative to the world
          // frame, expressed in world coordinates.
          const VectorType &alpha_parent = _parentFrame.angularAcceleration;

          // The linear acceleration of the input frame, relative to the world
          // frame, expressed in world coordinates.
          const VectorType &a =
              a_parent + a_rel
              + alpha_parent.cross(p_rel)
              + 2*w_parent.cross(v_rel)
              + w_parent.cross(w_parent.cross(p_rel));

          // Copy the linear acceleration into our result data.
          resultFrameData.linearAcceleration = a;

          // The angular velocity of the input frame, relative to its parent
          // frame, expressed in world coordinates.
          const VectorType &w_rel =
              VectorSpaceType::ResolveToWorldCoordinates(
                _relativeFrameData.angularVelocity, R_parent);

          // The angular velocity of the input frame, relative to the world
          // frame, expressed in world coordinates.
          const VectorType &w = w_parent + w_rel;

          // Copy the angular velocity into our result data.
          resultFrameData.angularVelocity = w;

          // The angular acceleration of the input frame, relative to the world
          // frame, expressed in world coordinates.
          const VectorType &alpha_rel =
              VectorSpaceType::ResolveToWorldCoordinates(
                _relativeFrameData.angularAcceleration, R_parent);

          // The angular acceleration of the input frame, relative to the world
          // frame, expressed in world coordinates.
          const VectorType &alpha =
              alpha_parent + alpha_rel + w_parent.cross(w_rel);

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

          // The position of the input frame, relative to the target frame, in
          // coordinates of the world frame.
          const VectorType &p_rel =
              frameDataWrtWorld.pose.translation()
              - _targetFrame.pose.translation();

          // The orientation of the target frame, with respect to the world
          // frame.
          const RotationType &R_target = _targetFrame.pose.linear();

          // The linear velocity of the input frame, with respect to the world
          // frame.
          const VectorType &v = frameDataWrtWorld.linearVelocity;

          // The linear velocity of the target frame, with respect to the world
          // frame.
          const VectorType &v_target = _targetFrame.linearVelocity;

          // The angular velocity of the target frame, with respect to the world
          // frame.
          const VectorType &w_target = _targetFrame.angularVelocity;

          // The linear velocity of the input frame, relative to the target
          // frame, in coordinates of the world frame.
          const VectorType &v_rel = v - v_target - w_target.cross(p_rel);

          // Convert the linear velocity into the target coordinates, and then
          // copy it to the result data.
          resultFrameData.linearVelocity =
              VectorSpaceType::ResolveFromWorldToTargetCoordinates(
                v_rel, _targetFrame.pose.linear());

          // The linear acceleration of the input frame, with respect to the
          // world frame.
          const VectorType &a = frameDataWrtWorld.linearAcceleration;

          // The linear acceleration of the target frame, with respect to the
          // world frame.
          const VectorType &a_target = _targetFrame.linearAcceleration;

          // The angular acceleration of the target frame, with respect to the
          // world frame.
          const VectorType &alpha_target = _targetFrame.angularAcceleration;

          // The angular acceleration of the input frame, relative to the target
          // frame, in coordinates of the world frame.
          const VectorType &a_rel =
              a - a_target
              - alpha_target.cross(p_rel)
              - 2*w_target.cross(v_rel)
              - w_target.cross(w_target.cross(p_rel));

          // Convert the linear acceleration into the target coordinates, and
          // then copy it to the result data.
          resultFrameData.linearAcceleration =
              VectorSpaceType::ResolveFromWorldToTargetCoordinates(
                a_rel, _targetFrame.pose.linear());

          // The angular velocity of the input frame, with respect to the world
          // frame.
          const VectorType &w = frameDataWrtWorld.angularVelocity;

          // The angular velocity of the input frame, relative to the target
          // frame, in coordinates of the world frame.
          const VectorType &w_rel = w - w_target;

          // Convert the angular velocity into the target coordinates, and then
          // copy it to the result data.
          resultFrameData.angularVelocity =
              VectorSpaceType::ResolveFromWorldToTargetCoordinates(
                w_rel, _targetFrame.pose.linear());

          // The angular acceleration of the input frame, with respect to the
          // world frame.
          const VectorType &alpha = frameDataWrtWorld.angularAcceleration;

          // The angular acceleration of the input frame, relative to the target
          // frame, in coordinates of the world frame.
          const VectorType &alpha_rel =
              alpha - alpha_target - w_target.cross(w_rel);

          // Convert the angular acceleration into the target coordinates, and
          // then copy it to the result data.
          resultFrameData.angularAcceleration =
              VectorSpaceType::ResolveFromWorldToTargetCoordinates(
                alpha_rel, R_target);

          return resultFrameData;
        }

        /// \brief Resolve the coordinates of the specified property to the
        /// world coordinates. Note that this does not work for the transform
        /// property; it is only meant for linear/angular velocity/acceleration.
        public: template <VectorType Quantity::*property>
        static void ResolvePropertyToWorldCoordinates(
          Quantity &_output, const FrameDataType &_input,
          const RotationType &_currentCoordinates)
        {
          _output.*property =
              VectorSpaceType::ResolveToWorldCoordinates(
                _input.*property, _currentCoordinates);
        }

        /// \brief Resolve the coordinates of the specified property to the
        /// target coordinates. Note that this does not work for the transform
        /// property; it is only meant for linear/angular velocity/acceleration.
        public: template <VectorType Quantity::*property>
        static void ResolvePropertyToTargetCoordinates(
          Quantity &_output, const FrameDataType &_input,
          const RotationType &_currentCoordinates,
          const RotationType &_targetCoordinates)
        {
          _output.*property =
              VectorSpaceType::ResolveToTargetCoordinates(
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

          ResolvePropertyToWorldCoordinates<&Quantity::linearVelocity>(
                resultFrameData, _inputFrameData, _currentCoordinates);

          ResolvePropertyToWorldCoordinates<&Quantity::linearAcceleration>(
                resultFrameData, _inputFrameData, _currentCoordinates);

          ResolvePropertyToWorldCoordinates<&Quantity::angularVelocity>(
                resultFrameData, _inputFrameData, _currentCoordinates);

          ResolvePropertyToWorldCoordinates<&Quantity::angularAcceleration>(
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

          ResolvePropertyToTargetCoordinates<&Quantity::linearVelocity>(
                resultFrameData, _inputFrameData,
                _currentCoordinates, _targetCoordinates);

          ResolvePropertyToTargetCoordinates<&Quantity::linearAcceleration>(
                resultFrameData, _inputFrameData,
                _currentCoordinates, _targetCoordinates);

          ResolvePropertyToTargetCoordinates<&Quantity::angularVelocity>(
                resultFrameData, _inputFrameData,
                _currentCoordinates, _targetCoordinates);

          ResolvePropertyToTargetCoordinates<&Quantity::angularAcceleration>(
                resultFrameData, _inputFrameData,
                _currentCoordinates, _targetCoordinates);

          return resultFrameData;
        }
      };
    }
  }
}

#endif
