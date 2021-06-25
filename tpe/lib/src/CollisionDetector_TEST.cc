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
#include <ignition/math/AxisAlignedBox.hh>

#include "Collision.hh"
#include "CollisionDetector.hh"
#include "Model.hh"
#include "Link.hh"
#include "Shape.hh"

using namespace ignition;
using namespace physics;
using namespace tpelib;

/////////////////////////////////////////////////
TEST(CollisionDetector, GetIntersectionPoints)
{
  CollisionDetector cd;

  // get intersection points between two invalid boxes
  math::AxisAlignedBox box1;
  math::AxisAlignedBox box2;
  std::vector<math::Vector3d> points;
  EXPECT_FALSE(cd.GetIntersectionPoints(box1, box2, points));
  EXPECT_TRUE(points.empty());

  // get intersection points between two valid boxes
  // there should be 8 intersection points representing the region of
  // intersection
  box1 = math::AxisAlignedBox(
      math::Vector3d(-2, -2, -2), math::Vector3d(2, 2, 2));
  box2 = math::AxisAlignedBox(
      math::Vector3d(-1, -1, -1), math::Vector3d(3, 3, 3));
  EXPECT_TRUE(cd.GetIntersectionPoints(box1, box2, points));
  EXPECT_EQ(8u, points.size());

  std::vector<math::Vector3d> expectedPoints;
  expectedPoints.push_back(math::Vector3d(-1, -1, -1));
  expectedPoints.push_back(math::Vector3d(-1, -1, 2));
  expectedPoints.push_back(math::Vector3d(-1, 2, -1));
  expectedPoints.push_back(math::Vector3d(2, -1, -1));
  expectedPoints.push_back(math::Vector3d(2, 2, 2));
  expectedPoints.push_back(math::Vector3d(2, 2, -1));
  expectedPoints.push_back(math::Vector3d(2, -1, 2));
  expectedPoints.push_back(math::Vector3d(-1, 2, 2));

  // check against expected points and remove if found
  for (const auto &p : points)
  {
    expectedPoints.erase(
      std::remove(expectedPoints.begin(), expectedPoints.end(), p),
      expectedPoints.end());
  }
  // all contact points should match expected points
  EXPECT_TRUE(expectedPoints.empty());

  // Test single contact point
  points.clear();
  EXPECT_TRUE(cd.GetIntersectionPoints(box1, box2, points, true));
  EXPECT_EQ(1u, points.size());
  EXPECT_EQ(math::Vector3d(0.5, 0.5, 0.5), points[0]);
}

/////////////////////////////////////////////////
TEST(CollisionDetector, CheckCollisions)
{
  // set up entities for testing collision detection

  // model A
  std::shared_ptr<Model> modelA(new Model);
  Entity &linkAEnt = modelA->AddLink();
  Link *linkA = static_cast<Link *>(&linkAEnt);
  Entity &collisionAEnt = linkA->AddCollision();
  Collision *collisionA = static_cast<Collision *>(&collisionAEnt);
  BoxShape boxShapeA;
  boxShapeA.SetSize(ignition::math::Vector3d(4, 4, 4));
  collisionA->SetShape(boxShapeA);

  // model B
  std::shared_ptr<Model> modelB(new Model);
  Entity &linkBEnt = modelB->AddLink();
  Link *linkB = static_cast<Link *>(&linkBEnt);
  Entity &collisionBEnt = linkB->AddCollision();
  Collision *collisionB = static_cast<Collision *>(&collisionBEnt);
  SphereShape sphereShapeB;
  sphereShapeB.SetRadius(5);
  collisionB->SetShape(sphereShapeB);

  // model C
  std::shared_ptr<Model> modelC(new Model);
  Entity &linkCEnt = modelC->AddLink();
  Link *linkC = static_cast<Link *>(&linkCEnt);
  Entity &collisionCEnt = linkC->AddCollision();
  Collision *collisionC = static_cast<Collision *>(&collisionCEnt);
  CylinderShape cylinderShapeC;
  cylinderShapeC.SetRadius(2);
  cylinderShapeC.SetLength(4);
  collisionC->SetShape(cylinderShapeC);

  // model D
  std::shared_ptr<Model> modelD(new Model);
  Entity &linkDEnt = modelD->AddLink();
  Link *linkD = static_cast<Link *>(&linkDEnt);
  Entity &collisionDEnt = linkD->AddCollision();
  Collision *collisionD = static_cast<Collision *>(&collisionDEnt);
  CapsuleShape capsuleShapeD;
  capsuleShapeD.SetRadius(0.2);
  capsuleShapeD.SetLength(0.6);
  collisionD->SetShape(capsuleShapeD);

  // model E
  std::shared_ptr<Model> modelE(new Model);
  Entity &linkEEnt = modelE->AddLink();
  Link *linkE = static_cast<Link *>(&linkEEnt);
  Entity &collisionEEnt = linkE->AddCollision();
  Collision *collisionE = static_cast<Collision *>(&collisionEEnt);
  EllipsoidShape ellipsoidShapeE;
  ellipsoidShapeE.SetRadii({2, 2, 0.5});
  collisionE->SetShape(ellipsoidShapeE);

  // check collisions
  CollisionDetector cd;
  std::map<std::size_t, std::shared_ptr<Entity>> entities;
  // verify no contacts if models are far apart
  modelA->SetPose(math::Pose3d(100, 0, 0, 0, 0, 0));
  modelB->SetPose(math::Pose3d(0, 0, 0, 0, 0, 0));
  modelC->SetPose(math::Pose3d(-100, 0, 0, 0, 0, 0));
  modelD->SetPose(math::Pose3d(-200, 0, 0, 0, 0, 0));
  modelE->SetPose(math::Pose3d(200, 0, 0, 0, 0, 0));
  entities[modelA->GetId()] = modelA;
  entities[modelB->GetId()] = modelB;
  entities[modelC->GetId()] = modelC;
  entities[modelD->GetId()] = modelD;
  entities[modelE->GetId()] = modelE;

  std::vector<Contact> contacts = cd.CheckCollisions(entities);
  EXPECT_TRUE(contacts.empty());

  // collision between model A and B but not model C, D and E
  modelA->SetPose(math::Pose3d(2, 2, 2, 0, 0, 0));
  modelB->SetPose(math::Pose3d(0, 0, 0, 0, 0, 0));
  modelC->SetPose(math::Pose3d(100, 0, 0, 0, 0, 0));
  contacts = cd.CheckCollisions(entities);
  EXPECT_EQ(8u, contacts.size());

  for (const auto &c : contacts)
  {
    EXPECT_TRUE(c.entity1 == modelA->GetId() || c.entity2 == modelA->GetId());
    EXPECT_TRUE(c.entity1 == modelB->GetId() || c.entity2 == modelB->GetId());
    EXPECT_TRUE(c.entity1 != modelC->GetId() && c.entity2 != modelC->GetId());
    EXPECT_TRUE(c.entity1 != modelD->GetId() && c.entity2 != modelD->GetId());
    EXPECT_TRUE(c.entity1 != modelE->GetId() && c.entity2 != modelE->GetId());
    EXPECT_NE(c.entity1, c.entity2);
  }
  // check single contact point
  contacts = cd.CheckCollisions(entities, true);
  EXPECT_EQ(1u, contacts.size());
  EXPECT_TRUE((contacts[0].entity1 == modelA->GetId()) ||
      (contacts[0].entity2 == modelA->GetId()));
  EXPECT_TRUE((contacts[0].entity1 == modelB->GetId()) ||
      (contacts[0].entity2 == modelB->GetId()));
  EXPECT_TRUE((contacts[0].entity1 != modelC->GetId()) &&
      (contacts[0].entity2 != modelC->GetId()));
  EXPECT_TRUE((contacts[0].entity1 != modelD->GetId()) &&
      (contacts[0].entity2 != modelD->GetId()));
  EXPECT_TRUE((contacts[0].entity1 != modelE->GetId()) &&
      (contacts[0].entity2 != modelE->GetId()));

  // collision between model A and C but not model B, D and E
  modelA->SetPose(math::Pose3d(2, 1, 2, 0, 0, 0));
  modelB->SetPose(math::Pose3d(100, 0, 0, 0, 0, 0));
  modelC->SetPose(math::Pose3d(2, 3, 4, 0, 0, 0));
  contacts = cd.CheckCollisions(entities);
  EXPECT_EQ(8u, contacts.size());

  for (const auto &c : contacts)
  {
    EXPECT_TRUE(c.entity1 == modelA->GetId() || c.entity2 == modelA->GetId());
    EXPECT_TRUE(c.entity1 != modelB->GetId() && c.entity2 != modelB->GetId());
    EXPECT_TRUE(c.entity1 == modelC->GetId() || c.entity2 == modelC->GetId());
    EXPECT_TRUE(c.entity1 != modelD->GetId() && c.entity2 != modelD->GetId());
    EXPECT_TRUE(c.entity1 != modelE->GetId() && c.entity2 != modelE->GetId());
    EXPECT_NE(c.entity1, c.entity2);
  }
  // check single contact point
  contacts = cd.CheckCollisions(entities, true);
  EXPECT_EQ(1u, contacts.size());
  EXPECT_TRUE((contacts[0].entity1 == modelA->GetId()) ||
      (contacts[0].entity2 == modelA->GetId()));
  EXPECT_TRUE((contacts[0].entity1 != modelB->GetId()) &&
      (contacts[0].entity2 != modelB->GetId()));
  EXPECT_TRUE((contacts[0].entity1 == modelC->GetId()) ||
      (contacts[0].entity2 == modelC->GetId()));
  EXPECT_TRUE((contacts[0].entity1 != modelD->GetId()) &&
      (contacts[0].entity2 != modelD->GetId()));
  EXPECT_TRUE((contacts[0].entity1 != modelE->GetId()) &&
      (contacts[0].entity2 != modelE->GetId()));

  // collision between model A and B, and B and C, but not A and C
  modelA->SetPose(math::Pose3d(-2, -2, -2, 0, 0, 0));
  modelB->SetPose(math::Pose3d(0, 0, 0, 0, 0, 0));
  modelC->SetPose(math::Pose3d(3, 3, 3, 0, 0, 0));
  contacts = cd.CheckCollisions(entities);
  EXPECT_EQ(16u, contacts.size());

  unsigned int contactAB = 0u;
  unsigned int contactBC = 0u;
  for (const auto &c : contacts)
  {
    if ((c.entity1 == modelA->GetId() || c.entity2 == modelA->GetId()) &&
        (c.entity1 == modelB->GetId() || c.entity2 == modelB->GetId()))
      contactAB++;
    else if ((c.entity1 == modelB->GetId() || c.entity2 == modelB->GetId()) &&
        (c.entity1 == modelC->GetId() || c.entity2 == modelC->GetId()))
      contactBC++;
    else if ((c.entity1 == modelA->GetId() || c.entity2 == modelA->GetId()) &&
        (c.entity1 == modelC->GetId() || c.entity2 == modelC->GetId()))
      FAIL() << "There should be no contacts between model A and C";
    else if ((c.entity1 == modelA->GetId() || c.entity2 == modelA->GetId()) &&
        (c.entity1 == modelD->GetId() || c.entity2 == modelD->GetId()))
      FAIL() << "There should be no contacts between model A and D";
    else if ((c.entity1 == modelA->GetId() || c.entity2 == modelA->GetId()) &&
        (c.entity1 == modelE->GetId() || c.entity2 == modelE->GetId()))
      FAIL() << "There should be no contacts between model A and E";
    else if ((c.entity1 == modelB->GetId() || c.entity2 == modelB->GetId()) &&
        (c.entity1 == modelD->GetId() || c.entity2 == modelD->GetId()))
      FAIL() << "There should be no contacts between model B and D";
    else if ((c.entity1 == modelB->GetId() || c.entity2 == modelB->GetId()) &&
        (c.entity1 == modelE->GetId() || c.entity2 == modelE->GetId()))
      FAIL() << "There should be no contacts between model B and E";
    else if ((c.entity1 == modelC->GetId() || c.entity2 == modelC->GetId()) &&
        (c.entity1 == modelD->GetId() || c.entity2 == modelD->GetId()))
      FAIL() << "There should be no contacts between model C and D";
    else if ((c.entity1 == modelC->GetId() || c.entity2 == modelC->GetId()) &&
        (c.entity1 == modelE->GetId() || c.entity2 == modelE->GetId()))
      FAIL() << "There should be no contacts between model C and E";
    else if ((c.entity1 == modelD->GetId() || c.entity2 == modelD->GetId()) &&
        (c.entity1 == modelE->GetId() || c.entity2 == modelE->GetId()))
      FAIL() << "There should be no contacts between model D and E";
  }
  EXPECT_EQ(8u, contactAB);
  EXPECT_EQ(8u, contactBC);

  // check single contact point
  contacts = cd.CheckCollisions(entities, true);
  EXPECT_EQ(2u, contacts.size());

  // collision between model A and B, B and C, and A and C
  modelA->SetPose(math::Pose3d(-1, -1, -1, 0, 0, 0));
  modelB->SetPose(math::Pose3d(0, 0, 0, 0, 0, 0));
  modelC->SetPose(math::Pose3d(1, 1, 1, 0, 0, 0));
  contacts = cd.CheckCollisions(entities);
  EXPECT_EQ(24u, contacts.size());

  contactAB = 0u;
  contactBC = 0u;
  unsigned int contactAC = 0u;
  for (const auto &c : contacts)
  {
    if ((c.entity1 == modelA->GetId() || c.entity2 == modelA->GetId()) &&
        (c.entity1 == modelB->GetId() || c.entity2 == modelB->GetId()))
      contactAB++;
    else if ((c.entity1 == modelB->GetId() || c.entity2 == modelB->GetId()) &&
        (c.entity1 == modelC->GetId() || c.entity2 == modelC->GetId()))
      contactBC++;
    else if ((c.entity1 == modelA->GetId() || c.entity2 == modelA->GetId()) &&
        (c.entity1 == modelC->GetId() || c.entity2 == modelC->GetId()))
      contactAC++;
  }
  EXPECT_EQ(8u, contactAB);
  EXPECT_EQ(8u, contactBC);
  EXPECT_EQ(8u, contactAC);

  // check single contact point
  contacts = cd.CheckCollisions(entities, true);
  EXPECT_EQ(3u, contacts.size());

  // remove entity and check again
  entities.erase(modelC->GetId());
  contacts = cd.CheckCollisions(entities, false);
  EXPECT_EQ(8u, contacts.size());
  contacts = cd.CheckCollisions(entities, true);
  EXPECT_EQ(1u, contacts.size());

  entities[modelC->GetId()] = modelC;
  // collision between model E and B but not model A, C, D
  modelA->SetPose(math::Pose3d(100, 0, 0, 0, 0, 0));
  modelB->SetPose(math::Pose3d(0, 0, 0, 0, 0, 0));
  modelC->SetPose(math::Pose3d(-100, 0, 0, 0, 0, 0));
  modelD->SetPose(math::Pose3d(-200, 0, 0, 0, 0, 0));
  modelE->SetPose(math::Pose3d(5, 0, 0, 0, 0, 0));
  contacts = cd.CheckCollisions(entities);
  EXPECT_EQ(8u, contacts.size());

  for (const auto &c : contacts)
  {
    EXPECT_TRUE(c.entity1 == modelE->GetId() || c.entity2 == modelE->GetId());
    EXPECT_TRUE(c.entity1 == modelB->GetId() || c.entity2 == modelB->GetId());
    EXPECT_TRUE(c.entity1 != modelA->GetId() && c.entity2 != modelA->GetId());
    EXPECT_TRUE(c.entity1 != modelC->GetId() && c.entity2 != modelC->GetId());
    EXPECT_TRUE(c.entity1 != modelD->GetId() && c.entity2 != modelD->GetId());
    EXPECT_NE(c.entity1, c.entity2);
  }
  // check single contact point
  contacts = cd.CheckCollisions(entities, true);
  EXPECT_EQ(1u, contacts.size());
  EXPECT_TRUE((contacts[0].entity1 != modelA->GetId()) &&
      (contacts[0].entity2 != modelA->GetId()));
  EXPECT_TRUE((contacts[0].entity1 == modelB->GetId()) ||
      (contacts[0].entity2 == modelB->GetId()));
  EXPECT_TRUE((contacts[0].entity1 != modelC->GetId()) &&
      (contacts[0].entity2 != modelC->GetId()));
  EXPECT_TRUE((contacts[0].entity1 != modelD->GetId()) &&
      (contacts[0].entity2 != modelD->GetId()));
  EXPECT_TRUE((contacts[0].entity1 == modelE->GetId()) ||
      (contacts[0].entity2 == modelE->GetId()));

  // collision between model D and B but not model A, C, E
  modelA->SetPose(math::Pose3d(100, 0, 0, 0, 0, 0));
  modelB->SetPose(math::Pose3d(0, 0, 0, 0, 0, 0));
  modelC->SetPose(math::Pose3d(-100, 0, 0, 0, 0, 0));
  modelD->SetPose(math::Pose3d(5, 0, 0, 0, 0, 0));
  modelE->SetPose(math::Pose3d(200, 0, 0, 0, 0, 0));
  contacts = cd.CheckCollisions(entities);
  EXPECT_EQ(8u, contacts.size());

  for (const auto &c : contacts)
  {
    EXPECT_TRUE(c.entity1 != modelA->GetId() && c.entity2 != modelA->GetId());
    EXPECT_TRUE(c.entity1 == modelB->GetId() || c.entity2 == modelB->GetId());
    EXPECT_TRUE(c.entity1 != modelC->GetId() && c.entity2 != modelC->GetId());
    EXPECT_TRUE(c.entity1 == modelD->GetId() || c.entity2 == modelD->GetId());
    EXPECT_TRUE(c.entity1 != modelE->GetId() && c.entity2 != modelE->GetId());
    EXPECT_NE(c.entity1, c.entity2);
  }
  // check single contact point
  contacts = cd.CheckCollisions(entities, true);
  EXPECT_EQ(1u, contacts.size());
  EXPECT_TRUE((contacts[0].entity1 != modelA->GetId()) &&
      (contacts[0].entity2 != modelA->GetId()));
  EXPECT_TRUE((contacts[0].entity1 == modelB->GetId()) ||
      (contacts[0].entity2 == modelB->GetId()));
  EXPECT_TRUE((contacts[0].entity1 != modelC->GetId()) &&
      (contacts[0].entity2 != modelC->GetId()));
  EXPECT_TRUE((contacts[0].entity1 == modelD->GetId()) ||
      (contacts[0].entity2 == modelD->GetId()));
  EXPECT_TRUE((contacts[0].entity1 != modelE->GetId()) &&
      (contacts[0].entity2 != modelE->GetId()));
}

/////////////////////////////////////////////////
TEST(CollisionDetector, CheckStaticCollisionFiltering)
{
  // set up entities for testing collision filtering between static objects

  // model A
  std::shared_ptr<Model> modelA(new Model);
  modelA->SetStatic(true);
  Entity &linkAEnt = modelA->AddLink();
  Link *linkA = static_cast<Link *>(&linkAEnt);
  Entity &collisionAEnt = linkA->AddCollision();
  Collision *collisionA = static_cast<Collision *>(&collisionAEnt);
  BoxShape boxShapeA;
  boxShapeA.SetSize(ignition::math::Vector3d(4, 4, 4));
  collisionA->SetShape(boxShapeA);

  // model B
  std::shared_ptr<Model> modelB(new Model);
  modelB->SetStatic(true);
  Entity &linkBEnt = modelB->AddLink();
  Link *linkB = static_cast<Link *>(&linkBEnt);
  Entity &collisionBEnt = linkB->AddCollision();
  Collision *collisionB = static_cast<Collision *>(&collisionBEnt);
  BoxShape boxShapeB;
  boxShapeB.SetSize(ignition::math::Vector3d(4, 4, 4));
  collisionB->SetShape(boxShapeB);

  // check collisions
  CollisionDetector cd;
  std::map<std::size_t, std::shared_ptr<Entity>> entities;
  // verify that no contacts are reported if the models are static
  modelA->SetPose(math::Pose3d(1, 1, 1, 0, 0, 0));
  modelB->SetPose(math::Pose3d(0, 0, 0, 0, 0, 0));
  entities[modelA->GetId()] = modelA;
  entities[modelB->GetId()] = modelB;

  std::vector<Contact> contacts = cd.CheckCollisions(entities);
  EXPECT_TRUE(contacts.empty());
}
