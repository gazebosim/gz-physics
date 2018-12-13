/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

#ifndef IGNITION_PHYSICS_BULLET_MATHCONVERSIONS_HH_
#define IGNITION_PHYSICS_BULLET_MATHCONVERSIONS_HH_

#include <ignition/math.hh>
#include "BulletIncludes.hh"

namespace mock
{
    namespace bullet
    {
      // vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
      // ---------------- Converting to Bullet ---------------------
      inline btVector3 convert(const ignition::math::Vector3d &v)
      {
        return btVector3(v[0], v[1], v[2]);
      }

      inline btMatrix3x3 convert(const ignition::math::Matrix3d &m)
      {
        btMatrix3x3 matrix;
        for (std::size_t i=0; i < 3; ++i)
          for (std::size_t j=0; j < 3; ++j)
            matrix[i][j] = m(i, j);

        return matrix;
      }

      inline btQuaternion convert(const ignition::math::Quaterniond &q)
      {
        return btQuaternion(q.X(), q.Y(), q.Z(), q.W());
      }

      inline btTransform convert(const ignition::math::Pose3d &pose)
      {
        btTransform tf;
        tf.setOrigin(btVector3(convert(pose.Pos())));
        tf.setRotation(btQuaternion(convert(pose.Rot())));

        return tf;
      }

      // vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
      // ---------------- Converting to ignition -------------------
      inline ignition::math::Vector3d convert(const btVector3 &v)
      {
        ignition::math::Vector3d vec;
        vec.X() = v[0];
        vec.Y() = v[1];
        vec.Z() = v[2];

        return vec;
      }

      inline ignition::math::Matrix3d convert(const btMatrix3x3 &m)
      {
        ignition::math::Matrix3d matrix;
        for (std::size_t i=0; i < 3; ++i)
          for (std::size_t j=0; j < 3; ++j)
            matrix(i, j) = m[i][j];

        return matrix;
      }

      inline ignition::math::Quaterniond convert(const btQuaternion &q)
      {
        ignition::math::Quaterniond quat;
        quat.W() = q.w();
        quat.X() = q.x();
        quat.Y() = q.y();
        quat.Z() = q.z();

        return quat;
      }

      inline ignition::math::Pose3d convert(const btTransform &tf)
      {
        ignition::math::Pose3d pose;
        pose.Pos() = convert(btVector3(tf.getOrigin()));
        pose.Rot() = convert(btQuaternion(tf.getRotation()));

        return pose;
      }
    }
}

#endif
