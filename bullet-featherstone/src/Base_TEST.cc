/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include "Base.hh"

using gz::physics::bullet_featherstone::GzMultiBody;

namespace
{
/// \brief A floating base with one revolute link about the z axis. The pivot
/// is 1 m from the base center of mass along x, and the center of mass of the
/// link is 0.5 m further along x. The base is rotated by 90 degrees about z,
/// so that the link offsets point along the world y axis.
struct Fixture
{
  Fixture() : body(1, 1.0, btVector3(1, 1, 1), false, false)
  {
    body.setupRevolute(0, 1.0, btVector3(1, 1, 1), -1,
        btQuaternion::getIdentity(), btVector3(0, 0, 1), btVector3(1, 0, 0),
        btVector3(0.5, 0, 0), true);
    body.finalizeMultiDof();
    body.SetBaseWorldTransform(
        btTransform(btQuaternion(btVector3(0, 0, 1), SIMD_HALF_PI)));
    this->Refresh();
  }

  /// \brief Update the cached transforms, which the velocity of the links
  /// is expressed with.
  void Refresh()
  {
    btAlignedObjectArray<btQuaternion> rot;
    btAlignedObjectArray<btVector3> origin;
    this->body.forwardKinematics(rot, origin);
  }

  /// \brief Velocity state: base linear velocity, base angular velocity about
  /// z, joint velocity.
  void SetVelocities(const Eigen::Vector3d &_baseVel, double _baseOmegaZ,
      double _jointVel)
  {
    this->body.setBaseVel(
        btVector3(static_cast<btScalar>(_baseVel.x()),
                  static_cast<btScalar>(_baseVel.y()),
                  static_cast<btScalar>(_baseVel.z())));
    this->body.setBaseOmega(
        btVector3(0, 0, static_cast<btScalar>(_baseOmegaZ)));
    this->body.setJointVel(0, static_cast<btScalar>(_jointVel));
  }

  GzMultiBody body;
};
}  // namespace

/////////////////////////////////////////////////
TEST(GzMultiBody, AccelerationsAreZeroBeforeAStep)
{
  Fixture f;
  f.SetVelocities({1, 2, 3}, 1, 2);
  for (int i = -1; i < 1; ++i)
  {
    EXPECT_TRUE(f.body.GetLinkComLinearAcceleration(i).isZero());
    EXPECT_TRUE(f.body.GetLinkAngularAcceleration(i).isZero());
  }
  // Out of range
  EXPECT_TRUE(f.body.GetLinkComLinearAcceleration(5).isZero());
  EXPECT_TRUE(f.body.GetLinkAngularAcceleration(-3).isZero());
}

/////////////////////////////////////////////////
TEST(GzMultiBody, NonPositiveStepKeepsAccelerations)
{
  Fixture f;
  f.SetVelocities({0, 0, 0}, 0, 0);
  f.body.RecordVelocitiesBeforeStep();
  f.SetVelocities({0, 0, 1}, 0, 0);
  f.body.UpdateAccelerations(0.0);
  EXPECT_TRUE(f.body.GetLinkComLinearAcceleration(-1).isZero());

  f.body.UpdateAccelerations(1.0);
  EXPECT_NEAR(1.0, f.body.GetLinkComLinearAcceleration(-1).z(), 1e-5);

  // A step of zero length leaves the last accelerations
  f.body.UpdateAccelerations(0.0);
  EXPECT_NEAR(1.0, f.body.GetLinkComLinearAcceleration(-1).z(), 1e-5);
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
