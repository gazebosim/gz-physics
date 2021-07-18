/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#include "Collision.hh"
#include "Shape.hh"
#include "Link.hh"

using namespace ignition;
using namespace physics;
using namespace tpelib;
/////////////////////////////////////////////////
TEST(Collision, BasicAPI)
{
  Collision collision;
  collision.SetId(1234u);
  EXPECT_EQ(1234u, collision.GetId());

  collision.SetName("collision_1");
  EXPECT_EQ("collision_1", collision.GetName());

  collision.SetPose(math::Pose3d(1, 2, 3, 0.1, 0.2, 0.3));
  EXPECT_EQ(math::Pose3d(1, 2, 3, 0.1, 0.2, 0.3), collision.GetPose());

  Link link;
  auto linkPose = math::Pose3d(1, 2, 3, 0.1, 0.2, 0.3);
  link.SetPose(linkPose);
  Entity &collisionEnt = link.AddCollision();
  ASSERT_NE(nullptr, collisionEnt.GetParent());

  collisionEnt.SetPose(math::Pose3d(0, 0.2, 0.5, 0, 1, 0));
  EXPECT_EQ(math::Pose3d(1.05416, 2.17281, 3.50715, 0.265579, 1.18879,
      0.527304), collisionEnt.GetWorldPose());

  EXPECT_EQ(0xFF, collision.GetCollideBitmask());
  collision.SetCollideBitmask(0x03);
  EXPECT_EQ(0x03, collision.GetCollideBitmask());

  Collision collision2;
  EXPECT_NE(collision.GetId(), collision2.GetId());
}

/////////////////////////////////////////////////
TEST(Collision, BoxShape)
{
  Collision collision;
  BoxShape boxShape;
  boxShape.SetSize(ignition::math::Vector3d(0.5, 0.5, 0.5));
  collision.SetShape(boxShape);
  auto result = collision.GetShape();
  ASSERT_NE(nullptr, result);
  EXPECT_EQ(ignition::math::Vector3d(0.25, 0.25, 0.25),
      result->GetBoundingBox().Max());
}

/////////////////////////////////////////////////
TEST(Collision, CapsuleShape)
{
  Collision collision;
  CapsuleShape CapsuleShape;
  CapsuleShape.SetRadius(2.0);
  CapsuleShape.SetLength(3.0);
  collision.SetShape(CapsuleShape);
  auto result = collision.GetShape();
  ASSERT_NE(nullptr, result);
  EXPECT_EQ(ignition::math::Vector3d(2.0, 2.0, 3.5),
      result->GetBoundingBox().Max());
}

/////////////////////////////////////////////////
TEST(Collision, CylinderShape)
{
  Collision collision;
  CylinderShape cylinderShape;
  cylinderShape.SetRadius(2.0);
  cylinderShape.SetLength(3.0);
  collision.SetShape(cylinderShape);
  auto result = collision.GetShape();
  ASSERT_NE(nullptr, result);
  EXPECT_EQ(ignition::math::Vector3d(2.0, 2.0, 1.5),
      result->GetBoundingBox().Max());
}

/////////////////////////////////////////////////
TEST(Collision, EllipsoidShape)
{
  Collision collision;
  EllipsoidShape EllipsoidShape;
  EllipsoidShape.SetRadii({1.0, 2.0, 1.3});
  collision.SetShape(EllipsoidShape);
  auto result = collision.GetShape();
  ASSERT_NE(nullptr, result);
  EXPECT_EQ(ignition::math::Vector3d(1.0, 2.0, 1.3),
      result->GetBoundingBox().Max());}

/////////////////////////////////////////////////
TEST(Collision, SphereShape)
{
  Collision collision;
  SphereShape sphereShape;
  sphereShape.SetRadius(2.0);
  collision.SetShape(sphereShape);
  auto result = collision.GetShape();
  ASSERT_NE(nullptr, result);
  EXPECT_EQ(ignition::math::Vector3d(2.0, 2.0, 2.0),
      result->GetBoundingBox().Max());
}

/////////////////////////////////////////////////
TEST(Collision, MeshShape)
{
  Collision collision;
  MeshShape meshShape;
  collision.SetShape(meshShape);
  auto result = collision.GetShape();
  ASSERT_NE(nullptr, result);
}
