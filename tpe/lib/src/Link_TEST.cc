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
#include "Link.hh"
#include "Model.hh"

using namespace ignition;
using namespace physics;
using namespace tpelib;

/////////////////////////////////////////////////
TEST(Link, BasicAPI)
{
  Link link;
  link.SetId(1234u);
  EXPECT_EQ(1234u, link.GetId());

  link.SetName("link_1");
  EXPECT_EQ("link_1", link.GetName());

  math::Pose3d link1Pose(1, 2, 3, 0.1, 0.2, 0.3);
  link.SetPose(link1Pose);
  EXPECT_EQ(link1Pose, link.GetPose());

  Model model;
  auto modelPose = math::Pose3d(10, 0, 2, 1, 0, 0);
  model.SetPose(modelPose);
  Entity &linkEnt = model.AddLink();
  ASSERT_NE(nullptr, linkEnt.GetParent());

  math::Pose3d linkEntPose(0, 0.2, 0.5, 0, 1, 0);
  linkEnt.SetPose(linkEntPose);
  EXPECT_EQ(math::Pose3d(10, -0.312675, 2.43845, 1.23686, 0.471978, 0.918989),
            linkEnt.GetWorldPose());

  math::Vector3d linVel(0, 0.1, 0);
  link.SetLinearVelocity(linVel);
  EXPECT_EQ(linVel, link.GetLinearVelocity());

  math::Vector3d angVel(0.2, 0, 1);
  link.SetAngularVelocity(angVel);
  EXPECT_EQ(angVel, link.GetAngularVelocity());

  double timeStep = 0.1;
  math::Pose3d expectedPose(
    link1Pose.Pos() + linVel * timeStep,
    link1Pose.Rot().Integrate(angVel, timeStep));
  link.UpdatePose(timeStep);
  EXPECT_EQ(expectedPose, link.GetPose());

  Link link2;
  EXPECT_NE(link.GetId(), link2.GetId());
}

/////////////////////////////////////////////////
TEST(Link, Collision)
{
  Link link;
  EXPECT_EQ(0u, link.GetChildCount());

  // add a child
  Entity &collisionEnt = link.AddCollision();
  collisionEnt.SetName("collision_1");
  collisionEnt.SetPose(math::Pose3d(2, 3, 4, 0, 0, 1));
  EXPECT_EQ(1u, link.GetChildCount());

  std::size_t collisionId = collisionEnt.GetId();
  Entity ent = link.GetChildById(collisionId);
  EXPECT_EQ(collisionId, ent.GetId());
  EXPECT_EQ("collision_1", ent.GetName());
  EXPECT_EQ(math::Pose3d(2, 3, 4, 0, 0, 1), ent.GetPose());

  // test casting to link
  Collision *collision = static_cast<Collision *>(&collisionEnt);
  EXPECT_NE(nullptr, collision);
  EXPECT_EQ(collisionEnt.GetId(), collision->GetId());

  // add another child
  Entity &collisionEnt2 = link.AddCollision();
  EXPECT_EQ(2u, link.GetChildCount());

  Collision *collision2 = static_cast<Collision *>(&collisionEnt2);
  EXPECT_NE(nullptr, collision2);
  EXPECT_EQ(collisionEnt2.GetId(), collision2->GetId());

  // test remove child by id
  link.RemoveChildById(collisionId);
  EXPECT_EQ(1u, link.GetChildCount());

  Entity nullEnt = link.GetChildById(collisionId);
  EXPECT_EQ(Entity::kNullEntity.GetId(), nullEnt.GetId());
}

