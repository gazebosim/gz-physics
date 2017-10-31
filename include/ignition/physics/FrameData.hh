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

#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

namespace ignition
{
  namespace physics
  {
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
  }
}

#include <ignition/physics/detail/FrameData.hh>

#endif
