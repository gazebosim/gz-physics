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

#ifndef IGNITION_PHYSICS_DETAIL_FRAMESEMANTICS_HH_
#define IGNITION_PHYSICS_DETAIL_FRAMESEMANTICS_HH_

#include <ignition/physics/FrameSemantics.hh>

namespace ignition
{
  namespace physics
  {
    /////////////////////////////////////////////////
    template <typename Scalar>
    FrameData3<Scalar>::FrameData3()
    {
      /// Technically we do not need to call this function in the constructor
      /// since the ignition::math types initialize their values to zero (or
      /// identity for Pose3) automatically, but I am putting this here for
      /// safety, just in case that behavior ever changes.
      this->SetToZero();
    }

    /////////////////////////////////////////////////
    template <typename Scalar>
    void FrameData3<Scalar>::SetToZero()
    {
      this->transform.Set(math::Vector3<Scalar>(0.0, 0.0, 0.0),
                          math::Quaternion<Scalar>(1.0, 0.0, 0.0, 0.0));
      this->linearVelocity.Set();
      this->angularVelocity.Set();
      this->linearAcceleration.Set();
      this->angularAcceleration.Set();
    }

    /////////////////////////////////////////////////
    template <typename Q, std::size_t Dim, typename ConfigSpace>
    template <typename... Args>
    FramedQuantity<Q, Dim, ConfigSpace>::FramedQuantity(
        const FrameID &_parentID, Args&&... _args)
      : parentFrame(_parentID),
        value(std::forward<Args>(_args)...)
    {
      // Do nothing
    }

    /////////////////////////////////////////////////
    template <typename Q, std::size_t Dim, typename ConfigSpace>
    template <typename... Args>
    FramedQuantity<Q, Dim, ConfigSpace>::FramedQuantity(Args&&... _args)
      : parentFrame(WorldFrameID),
        value(std::forward<Args>(_args)...)
    {
      // Do nothing
    }

    /////////////////////////////////////////////////
    template <typename Q, std::size_t Dim, typename ConfigSpace>
    FramedQuantity<Q, Dim, ConfigSpace>::FramedQuantity(const Q &_rawValue)
      : parentFrame(WorldFrameID),
        value(_rawValue)
    {
      // Do nothing
    }

    /////////////////////////////////////////////////
    template <typename Q, std::size_t Dim, typename ConfigSpace>
    Q &FramedQuantity<Q, Dim, ConfigSpace>::RelativeToParent()
    {
      return value;
    }

    /////////////////////////////////////////////////
    template <typename Q, std::size_t Dim, typename ConfigSpace>
    const Q &FramedQuantity<Q, Dim, ConfigSpace>::RelativeToParent() const
    {
      return value;
    }

    /////////////////////////////////////////////////
    template <typename Q, std::size_t Dim, typename ConfigSpace>
    const FrameID &FramedQuantity<Q, Dim, ConfigSpace>::ParentFrame() const
    {
      return parentFrame;
    }

    /////////////////////////////////////////////////
    // This namespace-scope definition is required by the ISO standard until
    // C++17. Once we migrate to C++17, this definition will be deprecated and
    // should be removed.
    template <typename Q, std::size_t Dim, typename ConfigSpace>
    constexpr std::size_t FramedQuantity::Dimension = Dim;

    /////////////////////////////////////////////////
    template <typename FQ>
    typename FQ::Quantity FrameSemantics::Resolve(
        const FQ &_quantity,
        const FrameID _relativeTo,
        const FrameID _inCoordinatesOf)
    {
      using Quantity = typename FQ::Quantity;
      using Space = typename FQ::Space;
      using FrameDataType = typename Space::FrameDataType;
      using RotationType = typename Space::RotationType;
      using Scalar = typename Space::Scalar;

      const FrameID parentFrame = _quantity.ParentFrame();

      Quantity q;
      RotationType currentCoordinates;

      if (_quantity.ParentFrame().id == _relativeTo.id)
      {
        // The quantity is already expressed relative to the _relativeTo frame

        if (_relativeTo.id == _inCoordinatesOf.id)
        {
          // The quantity is already expressed in coordinates of the
          // _inCoordinatesOf frame
          return _quantity.RelativeToParent();
        }

        q = _quantity.RelativeToParent();
        currentCoordinates = this->FrameDataRelativeToWorld(
              _relativeTo).transform.Rot();
      }
      else
      {
        if (WorldFrameID.id == _relativeTo.id)
        {
          // Resolving quantities to the world frame requires fewer operations
          // than resolving to an arbitrary frame, so we use a special function
          // for that.
          q = Space::ResolveToWorldFrame(
                _quantity.RelativeToParent(),
                this->FrameDataRelativeToWorld(parentFrame));

          // The World Frame has all zero fields
          currentCoordinates = RotationType(1.0, 0.0, 0.0, 0.0);
        }
        else
        {
          const FrameDataType relativeToData =
              this->FrameDataRelativeToWorld(_relativeTo);

          q = Space::ResolveToTargetFrame(
                _quantity.RelativeToParent(),
                this->FrameDataRelativeToWorld(parentFrame),
                relativeToData);

          currentCoordinates = relativeToData.transform.Rot();
        }
      }

      if (_relativeTo.id != _inCoordinatesOf.id)
      {
        if (WorldFrameID.id == _inCoordinatesOf.id)
        {
          // Resolving quantities to the world coordinates requires fewer
          // operations than resolving to an arbitrary frame.
          return Space::ResolveToWorldCoordinates(q, currentCoordinates);
        }
        else
        {
          const RotationType inCoordinatesOfRotation =
              this->FrameDataRelativeToWorld(
                _inCoordinatesOf).transform.Rot();

          return Space::ResolveToTargetCoordinates(
                q, currentCoordinates, inCoordinatesOfRotation);
        }
      }

      return q;
    }

    /////////////////////////////////////////////////
    template <typename FQ>
    typename FQ::Quantity FrameSemantics::Resolve(
        const FQ &_quantity, const FrameID _relativeTo)
    {
      return this->Resolve(_quantity, _relativeTo, _relativeTo);
    }

    /////////////////////////////////////////////////
    template <typename FQ>
    FQ FrameSemantics::Reframe(
        const FQ &_quantity, const FrameID _withRespectTo)
    {
      return FQ(_withRespectTo,
                this->Resolve(_quantity, _withRespectTo, _withRespectTo));
    }

    namespace detail
    {
      // Note that all the different ConfigSpace types have a std::size_t
      // template parameter which is currently going unused. That parameter is a
      // placeholder for forwards compatibility so that we can easily transition
      // into supporting 2D simulation, if we decide to pursue that later.


      /////////////////////////////////////////////////
      /// \brief SpaceTypes defines some typedefs which are used (or will be
      /// used) by Resolve and Reframe. Each different ConfigSpace type should
      /// inherit this to ensure that it has these types defined.
      template <std::size_t Dim, typename S, typename Q>
      struct SpaceTypes
      {
        public: using Quantity = Q;
        public: using Scalar = S;
        public: using FrameDataType = FrameData3<Scalar>;
        public: using RotationType = math::Quaternion<Scalar>;
        public: static constexpr std::size_t Dimension = Dim;
      };

      /////////////////////////////////////////////////
      // This namespace-scope definition is required by the ISO standard until
      // C++17. Once we migrate to C++17, this definition will be deprecated and
      // should be removed.
      template <typename Q, std::size_t Dim, typename S>
      constexpr std::size_t SpaceTypes<Q, Dim, S>::Dimension = Dim;

      /////////////////////////////////////////////////
      /// \brief SESpace is used by SE(3) constructs (and perhaps in the future
      /// SE(2) constructs as well) like homogeneous transformation matrices
      /// (i.e. Pose3).
      template <std::size_t Dim, typename S>
      struct SESpace : public SpaceTypes<Dim, S, math::Pose3<S>>
      {
        /// \brief Resolve the pose to the world frame
        public: static Quantity ResolveToWorldFrame(
          const Quantity &_pose,
          const FrameDataType &_parentFrame)
        {
          return _parentFrame.transform * _pose;
        }

        /// \brief Resolve the pose to the target frame
        public: static Quantity ResolveToTargetFrame(
          const Quantity &_pose,
          const FrameDataType &_parentFrame,
          const FrameDataType &_targetFrame)
        {
          return _targetFrame.transform.Inverse()
                 * _parentFrame.transform
                 * _pose;
        }

        /// \brief Resolve the coordinates of the pose to the world frame
        public: static Quantity ResolveToWorldCoordinates(
          const Quantity &_pose,
          const RotationType &_currentCoordinates)
        {
          const math::Pose3<Scalar> coordinates(
                math::Vector3<Scalar>(0.0, 0.0, 0.0),
                _currentCoordinates);

          return coordinates * _pose;
        }

        /// \brief Resolve the coordinates of the pose to the target frame
        public: static Quantity ResolveToTargetCoordinates(
          const Quantity &_pose,
          const RotationType &_currentCoordinates,
          const RotationType &_targetCoordinates)
        {
          // Premultiplying by a homogeneous transform with zero translation is
          // the same as changing the coordinates of a pose.

          const math::Pose3<Scalar> current(
                math::Vector3<Scalar>(0.0, 0.0, 0.0),
                _currentCoordinates);

          const math::Pose3<Scalar> target(
                math::Vector3<Scalar>(0.0, 0.0, 0.0),
                _targetCoordinates);

          return target.Inverse() * current * _pose;
        }
      };

      /////////////////////////////////////////////////
      /// \brief SOSpace is used by SO(3) constructs (and perhaps in the future
      /// SO(2) constructs as well) like rotation matrices or quaternions.
      template <std::size_t Dim, typename S, typename Q>
      struct SOSpace : public SpaceTypes<Dim, S, Q>
      {
        /// \brief Resolve the rotation to the world frame
        public: static Quantity ResolveToWorldFrame(
          const Quantity &_rotation,
          const FrameDataType &_parentFrame)
        {
          return ResolveToWorldCoordinates(
                _rotation, _parentFrame.transform.Rot());
        }

        /// \brief Resolve the rotation to the target frame
        public: static Quantity ResolveToTargetFrame(
          const Quantity &_rotation,
          const FrameDataType &_parentFrame,
          const FrameDataType &_targetFrame)
        {
          return ResolveToTargetCoordinates(
                _rotation,
                _parentFrame.transform.Rot(),
                _targetFrame.transform.Rot());
        }

        /// \brief Resolve the coordinates of the rotation to the world frame
        public: static Quantity ResolveToWorldCoordinates(
          const Quantity &_rotation,
          const RotationType &_currentCoordinates)
        {
          // This ensures that the rotation is being expressed as a quaternion,
          // no matter whether it was provided as a quaternion or a matrix
          const math::Quaternion<Scalar> R{_rotation};

          return Quantity(_currentCoordinates * R);
        }

        /// \brief Resolve the coordinates of the rotation to the target frame
        public: static Quantity ResolveToTargetCoordinates(
          const Quantity &_rotation,
          const RotationType &_currentCoordinates,
          const RotationType &_targetCoordinates)
        {
          const math::Quaternion<Scalar> R{_rotation};

          return Quantity(_currentCoordinates
                          * _targetCoordinates.Inverse()
                          * R);
        }
      };

      /////////////////////////////////////////////////
      /// \brief EuclideanSpace is used by points (i.e. positions or 
      /// translations).
      template <std::size_t Dim, typename S>
      struct EuclideanSpace : public VectorSpace<Dim, S>
      {
        /// \brief Resolve the point to the world frame
        public: static Quantity ResolveToWorldFrame(
          const Quantity &_point,
          const FrameDataType &_parentFrame)
        {
          return math::Matrix3<Scalar>(_parentFrame.transform.Rot()) * _point
                  + _parentFrame.transform.Pos();
        }

        /// \brief Resolve to the target frame
        public: static Quantity ResolveToTargetFrame(
          const Quantity &_point,
          const FrameDataType &_parentFrame,
          const FrameDataType &_targetFrame)
        {
          const math::Pose3<Scalar> T =
              _targetFrame.transform.Inverse()
              * _parentFrame.transform;

          return math::Matrix3<Scalar>(T.Rot()) * _point + T.Pos();
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
      template <std::size_t Dim, typename S,
                math::Vector3<S> FrameData3<S>::*property>
      struct KinematicDerivativeSpace : public VectorSpace<Dim, S>
      {
        /// \brief Resolve the property to the world frame
        public: static Quantity ResolveToWorldFrame(
          const Quantity &_vec,
          const FrameDataType &_parentFrame)
        {
          // Get the property that we care about from the parent frame. This
          // may be its (linear or angular) (velocity or acceleration).
          const Quantity &parentProperty = _parentFrame.*property;
          const Quantity vecInWorldCoordinates =
              ResolveToWorldCoordinates(_vec, _parentFrame.transform.Rot());

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
              ResolveToWorldCoordinates(_vec, _parentFrame.transform.Rot());

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
                _targetFrame.transform.Rot());
        }

        // Note: We inherit ResolveToWorldCoordinates and
        // ResolveToTargetCoordinates from VectorSpace since they are the same
        // as what we need for these kinematic derivatives.
      };

      /////////////////////////////////////////////////
      /// \brief LinearVelocitySpace is used by linear velocity vectors.
      template <std::size_t Dim, typename S>
      struct LinearVelocitySpace
          : public KinematicDerivativeSpace<Dim, S,
              &FrameData3<S>::linearVelocity> { };

      /////////////////////////////////////////////////
      /// \brief AngularVelocitySpace is used by angular velocity vectors.
      template <std::size_t Dim, typename S>
      struct AngularVelocitySpace
          : public KinematicDerivativeSpace<Dim, S,
              &FrameData3<S>::angularVelocity> { };

      /////////////////////////////////////////////////
      /// \brief LinearAccelerationSpace is used by linear acceleration vectors.
      template <std::size_t Dim, typename S>
      struct LinearAccelerationSpace
          : public KinematicDerivativeSpace<Dim, S,
              &FrameData3<S>::linearAcceleration> { };

      /////////////////////////////////////////////////
      /// \brief AngularAccelerationSpace is used by angular acceleration
      /// vectors.
      template <std::size_t Dim, typename S>
      struct AngularAccelerationSpace
          : public KinematicDerivativeSpace<Dim, S,
              &FrameData3<S>::angularAcceleration> { };

      /////////////////////////////////////////////////
      /// \brief VectorSpace is used by classical "free vectors", like Force and
      /// Torque, which are not dependent on any of the properties of a frame,
      /// besides the orientation.
      template <std::size_t Dim, typename S>
      struct VectorSpace : public SpaceTypes<Dim, S, math::Vector3<S>>
      {
        /// \brief Resolve the vector to the world frame
        public: static Quantity ResolveToWorldFrame(
          const Quantity &_vec,
          const FrameDataType &_parentFrame)
        {
          return ResolveToWorldCoordinates(_parentFrame.transform.Rot(), _vec);
        }

        /// \brief Resolve the vector to the target frame
        public: static Quantity ResolveToTargetFrame(
          const Quantity &_vec,
          const FrameDataType &_parentFrame,
          const FrameDataType &_targetFrame)
        {
          return ResolveToTargetCoordinates(
                _vec,
                _parentFrame.transform.Rot(),
                _targetFrame.transform.Rot());
        }

        /// \brief Resolve the coordinates of the vector to the world frame.
        public: static Quantity ResolveToWorldCoordinates(
          const Quantity &_vec,
          const RotationType &_currentCoordinates)
        {
          return math::Matrix3<Scalar>(_currentCoordinates) * _vec;
        }

        /// \brief Resolve the coordinates of the vector to the target frame.
        public: static Quantity ResolveToTargetCoordinates(
          const Quantity &_vec,
          const RotationType &_currentCoordinates,
          const RotationType &_targetCoordinates)
        {
          const math::Matrix3<Scalar> R_current{_currentCoordinates};
          const math::Matrix3<Scalar> R_target{_targetCoordinates};

          return R_target.Transposed() * (R_current * _vec);
        }

        /// \brief Resolve the coordinates of the vector from the world
        /// coordinates to the target coordinates. This is a helper function for
        /// some of the classes that use VectorSpace. It is not used directly by
        /// Resolve(~) or Reframe(~).
        public: static Quantity ResolveFromWorldToTargetCoordinates(
          const Quantity &_vec,
          const RotationType &_targetCoordinates)
        {
          const math::Matrix3<Scalar> R_target{_targetCoordinates};
          return R_target.Transposed() * _vec;
        }
      };

      /////////////////////////////////////////////////
      /// \brief FrameSpace is used to compute RelativeFrameData. This space is
      /// able to account for Coriolis and centrifugal effects in non-inertial
      /// reference frames.
      template <std::size_t Dim, typename S>
      struct FrameSpace : public SpaceTypes<Dim, S, FrameData3<S>>
      {
        using VectorType = math::Vector3<S>;
        using VectorSpaceType = VectorSpace<Dim, S>;

        /// \brief Express the data of the frame with respect to the world
        /// frame.
        public: static Quantity ResolveToWorldFrame(
          const Quantity &_relativeFrameData,
          const FrameDataType &_parentFrame)
        {
          const RotationType &Rot = _parentFrame.transform.Rot();

          Quantity resultFrameData;

          // The transform of the input frame, with respect to the world frame.
          resultFrameData.transform =
              _parentFrame.transform * _relativeFrameData.transform;

          // The position of our input frame, relative to its parent frame,
          // expressed in world coordinates.
          const VectorType &p_rel =
              VectorSpaceType::ResolveToWorldCoordinates(
                _relativeFrameData.transform.Pos(), Rot);

          // The linear velocity of our input frame, relative to its parent
          // frame, expressed in world coordinates.
          const VectorType &v_rel =
              VectorSpaceType::ResolveToWorldCoordinates(
                _relativeFrameData.linearVelocity, Rot);

          // The linear velocity of the parent frame, relative to the world
          // frame, expressed in world coordinates.
          const VectorType &v_parent = _parentFrame.linearVelocity;

          // The angular velocity of the parent frame, relative to the world
          // frame, expressed in world coordinates.
          const VectorType &w_parent = _parentFrame.angularVelocity;

          // The velocity of our input frame, relative to the world frame,
          // expressed in world coordinates.
          const VectorType &v = v_parent + v_rel + w_parent.Cross(p_rel);

          // Copy the linear velocity into our result data.
          resultFrameData.linearVelocity = v;

          // The linear acceleration of our input frame, relative to its parent
          // frame, expressed in world coordinates.
          const VectorType &a_rel =
              VectorSpaceType::ResolveToWorldCoordinates(
                _relativeFrameData.transform.Pos(), Rot);

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
              + alpha_parent.Cross(p_rel)
              + 2*w_parent.Cross(v_rel)
              + w_parent.Cross(w_parent.Cross(p_rel));

          // Copy the linear acceleration into our result data.
          resultFrameData.linearAcceleration = a;

          // The angular velocity of the input frame, relative to its parent
          // frame, expressed in world coordinates.
          const VectorType &w_rel =
              VectorSpaceType::ResolveToWorldCoordinates(
                _relativeFrameData.angularVelocity, Rot);

          // The angular velocity of the input frame, relative to the world
          // frame, expressed in world coordinates.
          const VectorType &w = w_parent + w_rel;

          // Copy the angular velocity into our result data.
          resultFrameData.angularVelocity = w;

          // The angular acceleration of the input frame, relative to the world
          // frame, expressed in world coordinates.
          const VectorType &alpha_rel =
              VectorSpaceType::ResolveToWorldCoordinates(
                _relativeFrameData.angularAcceleration, Rot);

          // The angular acceleration of the input frame, relative to the world
          // frame, expressed in world coordinates.
          const VectorType &alpha =
              alpha_parent + alpha_rel + w_parent.Cross(w_rel);

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
              frameDataWrtWorld.transform.Pos() - _targetFrame.transform.Pos();

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
          const VectorType &v_rel = v - v_target - w_target.Cross(p_rel);

          // Convert the linear velocity into the target coordinates, and then
          // copy it to the result data.
          resultFrameData.linearVelocity =
              VectorSpaceType::ResolveFromWorldToTargetCoordinates(
                v_rel, _targetFrame.transform.Rot());

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
              - alpha_target.Cross(p_rel)
              - 2*w_target.Cross(v_rel)
              - w_target.Cross(w_target.Cross(p_rel));

          // Convert the linear acceleration into the target coordinates, and
          // then copy it to the result data.
          resultFrameData.linearAcceleration =
              VectorSpaceType::ResolveFromWorldToTargetCoordinates(
                a_rel, _targetFrame.transform.Rot());

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
                w_rel, _targetFrame.transform.Rot());

          // The angular acceleration of the input frame, with respect to the
          // world frame.
          const VectorType &alpha = frameDataWrtWorld.angularAcceleration;

          // The angular acceleration of the input frame, relative to the target
          // frame, in coordinates of the world frame.
          const VectorType &alpha_rel =
              alpha - alpha_target - w_target.Cross(w_rel);

          // Convert the angular acceleration into the target coordinates, and
          // then copy it to the result data.
          resultFrameData.angularAcceleration =
              VectorSpaceType::ResolveFromWorldToTargetCoordinates(
                alpha_rel, _targetFrame.transform.Rot());

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

          resultFrameData.transform =
              SESpace<Dim, S>::ResolveToWorldCoordinates(
                _inputFrameData.transform, _currentCoordinates);

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

          resultFrameData.transform =
              SESpace<Dim, S>::ResolveToTargetCoordinates(
                _inputFrameData.transform, _currentCoordinates,
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

#endif // IGNITION_PHYSICS_DETAIL_FRAMESEMANTICS_HH_
