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

#ifndef IGNITION_PHYSICS_FRAMEDATA_HH_
#define IGNITION_PHYSICS_FRAMEDATA_HH_

#include <ignition/physics/Geometry.hh>

namespace ignition
{
  namespace physics
  {

    /// \brief The FrameData struct fully describes the kinematic state of a
    /// Frame with "Dim" dimensions and "Scalar" precision. Dim is allowed to be
    /// 2 or 3, and Scalar is allowed to be double or float. We provide the
    /// following fully qualified types:
    ///
    /// FrameData2d  -- 2 dimensional frame with double precision
    /// FrameData2f  -- 2 dimensional frame with float precision
    /// FrameData3d  -- 3 dimensional frame with double precision
    /// FrameData3f  -- 3 dimensional frame with float precision
    ///
    /// The frame of reference for this data is dependent on the context in
    /// which it is used. For FrameData which explicitly expresses its frame of
    /// reference, see RelativeFrameData3.
    template <typename Scalar, std::size_t Dim>
    struct FrameData
    {
      using Pose = ignition::physics::Pose<Scalar, Dim>;
      using LinearVector = ignition::physics::LinearVector<Scalar, Dim>;
      using AngularVector = ignition::physics::AngularVector<Scalar, Dim>;

      /// \brief Constructor. This will initialize the transform with identity
      /// and all velocity and acceleration vectors to zero.
      public: FrameData();

      /// \brief The current SE3 transformation of the frame.
      public: Pose pose;

      /// \brief The current linear velocity of the frame.
      public: LinearVector linearVelocity;

      /// \brief The current angular velocity of the frame.
      public: AngularVector angularVelocity;

      /// \brief The current linear acceleration of the frame.
      public: LinearVector linearAcceleration;

      /// \brief The current angular acceleration of the frame.
      public: AngularVector angularAcceleration;

      /// \brief Set the transform to identity and all velocity and acceleration
      /// vectors to zero.
      public: void SetToZero();
    };
    IGN_PHYSICS_MAKE_ALL_TYPE_COMBOS(FrameData)

    template <typename Scalar, std::size_t Dim>
    std::ostream& operator <<(std::ostream& stream,
                              const FrameData<Scalar, Dim> &_frame)
    {
      stream
        << "Pose:\n" << _frame.pose.matrix()
        << "\nLinear Velocity:      " << _frame.linearVelocity.transpose()
        << "\nAngular Velocity:     " << _frame.angularVelocity.transpose()
        << "\nLinear Acceleration:  " << _frame.linearAcceleration.transpose()
        << "\nAngular Acceleration: " << _frame.angularAcceleration.transpose();

      return stream;
    }
  }
}

#include <ignition/physics/detail/FrameData.hh>

#endif
