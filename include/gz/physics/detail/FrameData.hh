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

#ifndef GZ_PHYSICS_DETAIL_FRAMEDATA_HH_
#define GZ_PHYSICS_DETAIL_FRAMEDATA_HH_

#include <ignition/physics/FrameData.hh>

namespace gz
{
  namespace physics
  {
    /////////////////////////////////////////////////
    template <typename Scalar, std::size_t Dim>
    FrameData<Scalar, Dim>::FrameData()
    {
      /// Technically we do not need to call this function in the constructor
      /// since the gz::math types initialize their values to zero (or
      /// identity for Pose3) automatically, but I am putting this here for
      /// safety, just in case that behavior ever changes.
      this->SetToZero();
    }

    /////////////////////////////////////////////////
    template <typename Scalar, std::size_t Dim>
    void FrameData<Scalar, Dim>::SetToZero()
    {
      this->pose = Pose::Identity();
      this->linearVelocity = LinearVector::Zero();
      this->angularVelocity = AngularVector::Zero();
      this->linearAcceleration = LinearVector::Zero();
      this->angularAcceleration = AngularVector::Zero();
    }
  }
}

#endif
