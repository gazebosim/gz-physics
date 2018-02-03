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

#ifndef IGNITION_PHYSICS_ODE_MATHCONVERSIONS_HH_
#define IGNITION_PHYSICS_ODE_MATHCONVERSIONS_HH_

#include <ignition/math.hh>
#include <ode/ode.h>

namespace ignition
{
  namespace physics
  {
    namespace ode
    {
      // ---------------- Converting to dReal ----------------------
      inline dReal* convert(const ignition::math::Vector3d &v)
      {
        dReal *vec = new dReal[3];
        vec[0] = v.X();
        vec[1] = v.Y();
        vec[2] = v.Z();
        return vec;
      }

      inline dReal* convert(const ignition::math::Quaterniond &q)
      {
        dReal *quaternion = new dReal[4];
        quaternion[0] = q.W();
        quaternion[1] = q.X();
        quaternion[2] = q.Y();
        quaternion[3] = q.Z();
        return quaternion;
      }

      // ---------------- Converting to ignition -------------------
      // TODO(ying) Change to a generic way to convert vector and quaternion
      inline ignition::math::Vector3d convertVec(const dReal *v)
      {
        ignition::math::Vector3d vec(v[0], v[1], v[2]);
        return vec;
      }

      inline ignition::math::Quaterniond convertQuat(const dReal *q)
      {
        ignition::math::Quaterniond quat;
        quat.W() = q[0];
        quat.X() = q[1];
        quat.Y() = q[2];
        quat.Z() = q[3];
        return quat;
      }
    }
  }
}

#endif
