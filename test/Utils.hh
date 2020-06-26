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

#ifndef IGNITION_PHYSICS_TEST_UTILS_HH_
#define IGNITION_PHYSICS_TEST_UTILS_HH_

#include <ignition/math/Helpers.hh>
#include <ignition/math/Rand.hh>
#include <ignition/physics/FrameData.hh>

#include "test/PhysicsPluginsList.hh"

namespace ignition
{
  namespace physics
  {
    namespace test
    {
      /////////////////////////////////////////////////
      template <typename VectorType>
      VectorType RandomVector(const double range)
      {
        VectorType v;
        for (std::size_t i = 0; i < VectorType::RowsAtCompileTime; ++i)
          v[i] = math::Rand::DblUniform(-range, range);

        return v;
      }

      /////////////////////////////////////////////////
      template <typename Scalar, std::size_t Dim>
      struct Rotation
      {
        /// \brief Randomize the orientation of a 3D pose
        static void Randomize(ignition::physics::Pose<Scalar, Dim> &_pose)
        {
          for (std::size_t i = 0; i < 3; ++i)
          {
            Vector<Scalar, Dim> axis = Vector<Scalar, Dim>::Zero();
            axis[i] = 1.0;
            _pose.rotate(Eigen::AngleAxis<Scalar>(
              static_cast<Scalar>(math::Rand::DblUniform(0, 2*IGN_PI)), axis));
          }
        }

        static bool Equal(
            const Eigen::Matrix<Scalar, Dim, Dim> &_R1,
            const Eigen::Matrix<Scalar, Dim, Dim> &_R2,
            const double _tolerance)
        {
          Eigen::AngleAxis<Scalar> R(_R1.transpose() * _R2);
          if (std::abs(R.angle()) > _tolerance)
          {
            std::cout << "Difference in angle: " << R.angle() << std::endl;
            return false;
          }

          return true;
        }

        static AngularVector<Scalar, Dim> Apply(
            const Eigen::Matrix<Scalar, Dim, Dim> &_R,
            const AngularVector<Scalar, Dim> &_input)
        {
          // In 3D simulation, this is a normal multiplication
          return _R * _input;
        }
      };

      /////////////////////////////////////////////////
      template <typename Scalar>
      struct Rotation<Scalar, 2>
      {
        /// \brief Randomize the orientation of a 2D pose
        static void Randomize(ignition::physics::Pose<Scalar, 2> &_pose)
        {
          _pose.rotate(Eigen::Rotation2D<Scalar>(
                         math::Rand::DblUniform(0, 2*IGN_PI)));
        }

        static bool Equal(
            const Eigen::Matrix<Scalar, 2, 2> &_R1,
            const Eigen::Matrix<Scalar, 2, 2> &_R2,
            const double _tolerance)
        {
          // Choose the largest of either 1.0 or the size of the larger angle.
          const double scale =
              std::max(
                static_cast<Scalar>(1.0),
                std::max(
                  Eigen::Rotation2D<Scalar>(_R1).angle(),
                  Eigen::Rotation2D<Scalar>(_R2).angle()));

          const Eigen::Rotation2D<Scalar> R(_R1.transpose() * _R2);
          if (std::abs(R.angle()/scale) > _tolerance)
          {
            std::cout << "Scaled difference in angle: "
                      << R.angle()/scale << " | Difference: " << R.angle()
                      << " | Scale: " << scale
                      << " | (Tolerance: " << _tolerance << ")" << std::endl;
            return false;
          }

          return true;
        }

        static AngularVector<Scalar, 2> Apply(
            const Eigen::Matrix<Scalar, 2, 2> &,
            const AngularVector<Scalar, 2> &_input)
        {
          // Angular vectors cannot be rotated in 2D simulations, so we just
          // pass back the value that was given.
          return _input;
        }
      };

      /////////////////////////////////////////////////
      template <typename Scalar, std::size_t Dim>
      FrameData<Scalar, Dim> RandomFrameData()
      {
        using LinearVector = LinearVector<Scalar, Dim>;
        using AngularVector = AngularVector<Scalar, Dim>;

        FrameData<Scalar, Dim> data;
        data.pose.translation() = RandomVector<LinearVector>(100.0);
        Rotation<Scalar, Dim>::Randomize(data.pose);
        data.linearVelocity = RandomVector<LinearVector>(10.0);
        data.angularVelocity = RandomVector<AngularVector>(10.0);
        data.linearAcceleration = RandomVector<LinearVector>(1.0);
        data.angularAcceleration = RandomVector<AngularVector>(1.0);

        return data;
      }

      /////////////////////////////////////////////////
      template <typename Scalar, int Dim>
      bool Equal(const Vector<Scalar, Dim> &_vec1,
                 const Vector<Scalar, Dim> &_vec2,
                 const double _tolerance,
                 const std::string &_label = "vectors")
      {
        // Choose the largest of either 1.0 or the length of the longer vector.
        const double scale = std::max(static_cast<Scalar>(1.0),
                                      std::max(_vec1.norm(), _vec2.norm()));
        const double diff = (_vec1 - _vec2).norm();
        if (diff/scale <= _tolerance)
          return true;

        std::cout << "Scaled difference in " << _label << ": " << diff/scale
                  << " | Difference: " << diff << " | Scale: " << scale
                  << " | (Tolerance: " << _tolerance << ")" << std::endl;

        return false;
      }

      /////////////////////////////////////////////////
      template <typename Scalar, int Dim>
      bool Equal(const AlignedBox<Scalar, Dim> &_box1,
                 const AlignedBox<Scalar, Dim> &_box2,
                 const double _tolerance)
      {
        bool min = Equal(_box1.min(), _box2.min(), _tolerance, "box minimums");
        bool max = Equal(_box1.max(), _box2.max(), _tolerance, "box maximums");
        return min && max;
      }

      /////////////////////////////////////////////////
      template <typename Scalar, int Dim>
      bool Equal(const Pose<Scalar, Dim> &_T1,
                 const Pose<Scalar, Dim> &_T2,
                 const double _tolerance)
      {
        bool result = true;
        result &= Equal(Vector<Scalar, Dim>(_T1.translation()),
                    Vector<Scalar, Dim>(_T2.translation()),
                    _tolerance, "position");

        result &= Rotation<Scalar, Dim>::Equal(
                    _T1.linear(), _T2.linear(), _tolerance);

        return result;
      }

      /////////////////////////////////////////////////
      template <typename Scalar, std::size_t Dim>
      bool Equal(const FrameData<Scalar, Dim> &_data1,
                 const FrameData<Scalar, Dim> &_data2,
                 const double _tolerance)
      {
        bool result = true;
        result &= Equal(_data1.pose, _data2.pose, _tolerance);

        result &= Equal(_data1.linearVelocity, _data2.linearVelocity,
                    _tolerance, "linear velocity");

        result &= Equal(_data1.angularVelocity, _data2.angularVelocity,
                    _tolerance, "angular velocity");

        result &= Equal(_data1.linearAcceleration, _data2.linearAcceleration,
                    _tolerance, "linear acceleration");

        result &= Equal(_data1.angularAcceleration,
                    _data2.angularAcceleration,
                    _tolerance, "angular acceleration");

        return result;
      }
    }
  }
}


#endif
