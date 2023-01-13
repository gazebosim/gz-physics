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
#include "Shape.hh"

using namespace gz;
using namespace physics;
using namespace tpelib;

/////////////////////////////////////////////////
TEST(Model, BasicAPI)
{
  Model model;
  model.SetId(1234u);
  EXPECT_EQ(1234u, model.GetId());

  model.SetName("model_1");
  EXPECT_EQ("model_1", model.GetName());
  EXPECT_EQ("model_1", model.GetNameRef());

  model.SetPose(math::Pose3d(1, 2, 3, 0.1, 0.2, 0.3));
  EXPECT_EQ(math::Pose3d(1, 2, 3, 0.1, 0.2, 0.3), model.GetPose());
  EXPECT_EQ(math::Pose3d(1, 2, 3, 0.1, 0.2, 0.3), model.GetWorldPose());

  model.SetLinearVelocity(math::Vector3d(.5, .5, .5));
  EXPECT_EQ(math::Vector3d(.5, .5, .5), model.GetLinearVelocity());

  model.SetAngularVelocity(math::Vector3d(1.0, 1.0, 1.0));
  EXPECT_EQ(math::Vector3d(1.0, 1.0, 1.0), model.GetAngularVelocity());

  model.UpdatePose(0.1);
  EXPECT_EQ(model.GetPose(), model.GetWorldPose());

  EXPECT_FALSE(model.GetStatic());
  model.SetStatic(true);
  EXPECT_TRUE(model.GetStatic());

  Model model2;
  EXPECT_NE(model.GetId(), model2.GetId());

  // test UpdatePose
  math::Pose3d originalPose = math::Pose3d::Zero;
  double timeStep = 0.1;
  model2.SetPose(originalPose);
  model2.SetLinearVelocity(math::Vector3d(0.1, 0.1, 0.1));
  model2.SetAngularVelocity(math::Vector3d(1.0, 0, 0));
  math::Pose3d expectedPose(
    originalPose.Pos() + math::Vector3d(0.1, 0.1, 0.1) * timeStep,
    originalPose.Rot().Integrate(math::Vector3d(1.0, 0, 0), timeStep));
  model2.UpdatePose(timeStep);
  EXPECT_EQ(expectedPose, model2.GetPose());
}

/////////////////////////////////////////////////
TEST(Model, Link)
{
  Model model;
  EXPECT_EQ(0u, model.GetChildCount());

  // add a child
  Entity &linkEnt = model.AddLink();
  linkEnt.SetName("link_1");
  linkEnt.SetPose(math::Pose3d(2, 3, 4, 0, 0, 1));
  EXPECT_EQ(1u, model.GetChildCount());

  std::size_t linkId = linkEnt.GetId();
  Entity ent = model.GetChildById(linkId);
  EXPECT_EQ(linkId, ent.GetId());
  EXPECT_EQ("link_1", ent.GetName());
  EXPECT_EQ(math::Pose3d(2, 3, 4, 0, 0, 1), ent.GetPose());

  Entity entByName = model.GetChildByName("link_1");
  EXPECT_EQ("link_1", entByName.GetName());

  Entity entByIdx = model.GetChildByIndex(0u);
  EXPECT_EQ("link_1", entByIdx.GetName());

  // test casting to link
  Link *link = static_cast<Link *>(&linkEnt);
  EXPECT_NE(nullptr, link);
  EXPECT_EQ(linkEnt.GetId(), link->GetId());

  // add another child
  Entity &linkEnt2 = model.AddLink();
  EXPECT_EQ(2u, model.GetChildCount());

  Entity ent2ByIdx = model.GetChildByIndex(1u);
  EXPECT_EQ(linkEnt2.GetId(), ent2ByIdx.GetId());

  Link *link2 = static_cast<Link *>(&linkEnt2);
  EXPECT_NE(nullptr, link2);
  EXPECT_EQ(linkEnt2.GetId(), link2->GetId());

  // test canonical link
  model.SetCanonicalLink(link->GetId());
  EXPECT_NE(Entity::kNullEntity.GetId(), model.GetCanonicalLink().GetId());
  EXPECT_EQ(link->GetId(), model.GetCanonicalLink().GetId());

  // test remove child by id
  model.RemoveChildById(linkId);
  EXPECT_EQ(1u, model.GetChildCount());

  Entity nullEnt = model.GetChildById(linkId);
  EXPECT_EQ(Entity::kNullEntity.GetId(), nullEnt.GetId());
}

/////////////////////////////////////////////////
TEST(Model, BoundingBox)
{
  Model model;
  EXPECT_EQ(0u, model.GetChildCount());
  EXPECT_EQ(math::AxisAlignedBox(), model.GetBoundingBox());

  // add link with sphere collision shape
  Entity &linkEnt = model.AddLink();
  linkEnt.SetPose(math::Pose3d(0, 0, 1, 0, 0, 0));
  EXPECT_EQ(math::AxisAlignedBox(), linkEnt.GetBoundingBox());
  EXPECT_EQ(math::AxisAlignedBox(), model.GetBoundingBox());

  Link *link = static_cast<Link *>(&linkEnt);
  Entity &collisionEnt = link->AddCollision();
  Collision *collision = static_cast<Collision *>(&collisionEnt);
  SphereShape sphereShape;
  sphereShape.SetRadius(2.0);
  collision->SetShape(sphereShape);

  math::AxisAlignedBox expectedBoxLinkFrame(
      math::Vector3d(-2, -2, -2), math::Vector3d(2, 2, 2));
  EXPECT_EQ(expectedBoxLinkFrame, linkEnt.GetBoundingBox());

  math::AxisAlignedBox expectedBoxModelFrame(
      math::Vector3d(-2, -2, -1), math::Vector3d(2, 2, 3));
  EXPECT_EQ(expectedBoxModelFrame, model.GetBoundingBox());

  // add another link with box collision shape
  Entity &linkEnt2 = model.AddLink();
  linkEnt2.SetPose(math::Pose3d(0, 1, 0, 0, 0, 0));
  EXPECT_EQ(math::AxisAlignedBox(), linkEnt2.GetBoundingBox());
  EXPECT_EQ(expectedBoxModelFrame, model.GetBoundingBox());

  Link *link2 = static_cast<Link *>(&linkEnt2);
  Entity &collisionEnt2 = link2->AddCollision();
  Collision *collision2 = static_cast<Collision *>(&collisionEnt2);
  BoxShape boxShape;
  boxShape.SetSize(math::Vector3d(3, 4, 5));
  collision2->SetShape(boxShape);

  expectedBoxLinkFrame = math::AxisAlignedBox(
      math::Vector3d(-1.5, -2, -2.5), math::Vector3d(1.5, 2, 2.5));
  EXPECT_EQ(expectedBoxLinkFrame, linkEnt2.GetBoundingBox());

  expectedBoxModelFrame = math::AxisAlignedBox(
      math::Vector3d(-2, -2, -2.5), math::Vector3d(2, 3, 3));
  EXPECT_EQ(expectedBoxModelFrame, model.GetBoundingBox());

  // add nested model with 1 link that has a cylinder collision shape
  Entity &nestedModelEnt = model.AddModel();
  nestedModelEnt.SetPose(math::Pose3d(1, 0, 0, 0, 0, 0));
  EXPECT_EQ(math::AxisAlignedBox(), nestedModelEnt.GetBoundingBox());
  EXPECT_EQ(expectedBoxModelFrame, model.GetBoundingBox());

  Model *nestedModel = static_cast<Model *>(&nestedModelEnt);
  Entity &nestedLinkEnt = nestedModel->AddLink();
  nestedLinkEnt.SetPose(math::Pose3d(1, 0, 0, 0, 0, 0));
  EXPECT_EQ(math::AxisAlignedBox(), nestedLinkEnt.GetBoundingBox());
  EXPECT_EQ(math::AxisAlignedBox(), nestedModelEnt.GetBoundingBox());
  EXPECT_EQ(expectedBoxModelFrame, model.GetBoundingBox());

  Link *nestedLink = static_cast<Link *>(&nestedLinkEnt);
  Entity &nestedCollisionEnt = nestedLink->AddCollision();
  Collision *nestedCollision = static_cast<Collision *>(&nestedCollisionEnt);
  CylinderShape cylinderShape;
  cylinderShape.SetRadius(2.0);
  cylinderShape.SetLength(2.0);
  nestedCollision->SetShape(cylinderShape);

  math::AxisAlignedBox expectedBoxNestedLinkFrame(
      math::Vector3d(-2, -2, -1), math::Vector3d(2, 2, 1));
  EXPECT_EQ(expectedBoxNestedLinkFrame, nestedLinkEnt.GetBoundingBox());

  math::AxisAlignedBox expectedBoxNestedModelFrame(
      math::Vector3d(-1, -2, -1), math::Vector3d(3, 2, 1));
  EXPECT_EQ(expectedBoxNestedModelFrame, nestedModelEnt.GetBoundingBox());

  expectedBoxModelFrame = math::AxisAlignedBox(
      math::Vector3d(-2, -2, -2.5), math::Vector3d(4, 3, 3));
  EXPECT_EQ(expectedBoxModelFrame, model.GetBoundingBox());
}

/////////////////////////////////////////////////
TEST(Model, CollideBitmask)
{
  Model model;
  EXPECT_EQ(0x00, model.GetCollideBitmask());

  // add link and verify bitmask is still empty
  Entity &linkEnt = model.AddLink();
  EXPECT_EQ(0x00, linkEnt.GetCollideBitmask());
  EXPECT_EQ(0x00, model.GetCollideBitmask());

  // add a collision and verify the model has the same collision bitmask
  Link *link = static_cast<Link *>(&linkEnt);
  Entity &collisionEnt = link->AddCollision();
  Collision *collision = static_cast<Collision *>(&collisionEnt);
  collision->SetCollideBitmask(0x01);
  EXPECT_EQ(0x01, collision->GetCollideBitmask());
  EXPECT_EQ(0x01, linkEnt.GetCollideBitmask());
  EXPECT_EQ(0x01, model.GetCollideBitmask());

  // add another collision and verify bitmasks are bitwise OR'd.
  Entity &collisionEnt2 = link->AddCollision();
  Collision *collision2 = static_cast<Collision *>(&collisionEnt2);
  collision2->SetCollideBitmask(0x04);
  EXPECT_EQ(0x04, collision2->GetCollideBitmask());
  EXPECT_EQ(0x05, linkEnt.GetCollideBitmask());
  EXPECT_EQ(0x05, model.GetCollideBitmask());

  // add another link with collision and verify
  Entity &linkEnt2 = model.AddLink();
  Link *link2 = static_cast<Link *>(&linkEnt2);
  Entity &collisionEnt3 = link2->AddCollision();
  Collision *collision3 = static_cast<Collision *>(&collisionEnt3);
  collision3->SetCollideBitmask(0x09);
  EXPECT_EQ(0x09, collision3->GetCollideBitmask());
  EXPECT_EQ(0x09, linkEnt2.GetCollideBitmask());
  EXPECT_EQ(0x0D, model.GetCollideBitmask());

  // add nested model and verify bitmask is empty
  Entity &nestedModelEnt = model.AddModel();
  EXPECT_EQ(0x00, nestedModelEnt.GetCollideBitmask());
  EXPECT_EQ(0x0D, model.GetCollideBitmask());

  // add nested link and verify bitmask is still empty
  Model *nestedModel = static_cast<Model *>(&nestedModelEnt);
  Entity &nestedLinkEnt = nestedModel->AddLink();
  EXPECT_EQ(0x00, nestedLinkEnt.GetCollideBitmask());
  EXPECT_EQ(0x00, nestedModelEnt.GetCollideBitmask());
  EXPECT_EQ(0x0D, model.GetCollideBitmask());

  // add a nested collision and verify the nested model has the same bitmask
  // as the nested collision bitmask, and the top level model's bitmask now
  // includes the nested collision's bitmask
  Link *nestedLink = static_cast<Link *>(&nestedLinkEnt);
  Entity &nestedCollisionEnt = nestedLink->AddCollision();
  Collision *nestedCollision = static_cast<Collision *>(&nestedCollisionEnt);
  nestedCollision->SetCollideBitmask(0x02);
  EXPECT_EQ(0x02, nestedCollision->GetCollideBitmask());
  EXPECT_EQ(0x02, nestedLinkEnt.GetCollideBitmask());
  EXPECT_EQ(0x02, nestedModelEnt.GetCollideBitmask());
  EXPECT_EQ(0x0F, model.GetCollideBitmask());
}

/////////////////////////////////////////////////
TEST(Model, NestedModel)
{
  Model model;
  EXPECT_EQ(0u, model.GetChildCount());

  // add a child
  Entity &nestedModelEnt = model.AddModel();
  nestedModelEnt.SetName("model_1");
  nestedModelEnt.SetPose(math::Pose3d(2, 3, 4, 0, 0, 1));
  EXPECT_EQ(1u, model.GetChildCount());

  std::size_t modelId = nestedModelEnt.GetId();
  Entity ent = model.GetChildById(modelId);
  EXPECT_EQ(modelId, ent.GetId());
  EXPECT_EQ("model_1", ent.GetName());
  EXPECT_EQ(math::Pose3d(2, 3, 4, 0, 0, 1), ent.GetPose());

  Entity entByName = model.GetChildByName("model_1");
  EXPECT_EQ("model_1", entByName.GetName());

  Entity entByIdx = model.GetChildByIndex(0u);
  EXPECT_EQ("model_1", entByIdx.GetName());

  // test casting to model
  Model *nestedModel = static_cast<Model *>(&nestedModelEnt);
  EXPECT_NE(nullptr, nestedModel);
  EXPECT_EQ(nestedModelEnt.GetId(), nestedModel->GetId());

  // add another child
  Entity &nestedModelEnt2 = model.AddModel();
  EXPECT_EQ(2u, model.GetChildCount());

  Entity ent2ByIdx = model.GetChildByIndex(1u);
  EXPECT_EQ(nestedModelEnt2.GetId(), ent2ByIdx.GetId());

  Model *nestedModel2 = static_cast<Model *>(&nestedModelEnt2);
  EXPECT_NE(nullptr, nestedModel2);
  EXPECT_EQ(nestedModelEnt2.GetId(), nestedModel2->GetId());

  // test remove child by id
  model.RemoveChildById(modelId);
  EXPECT_EQ(1u, model.GetChildCount());

  Entity nullEnt = model.GetChildById(modelId);
  EXPECT_EQ(Entity::kNullEntity.GetId(), nullEnt.GetId());

  // test canonical link within nested model
  Model m0;
  m0.SetName("m0");

  // add nested models m1 and m2
  Entity &nestedModelEntm1 = m0.AddModel();
  nestedModelEntm1.SetName("m1");
  Model *m1 = static_cast<Model *>(&nestedModelEntm1);
  Entity &nestedModelEntm2 = m0.AddModel();
  nestedModelEntm2.SetName("m2");
  Model *m2 = static_cast<Model *>(&nestedModelEntm2);

  // add links to nested models
  Entity &linkEnt1 = m1->AddLink();
  linkEnt1.SetName("x");
  EXPECT_EQ(1u, m1->GetChildCount());
  Entity &linkEnt2 = m2->AddLink();
  linkEnt2.SetName("y");
  EXPECT_EQ(1u, m2->GetChildCount());

  // set link y to be the canonical link of m0
  m0.SetCanonicalLink(linkEnt2.GetId());
  EXPECT_EQ(linkEnt2.GetId(), m0.GetCanonicalLink().GetId());

  // Set link y to be default canonical link of m2
  m2->SetCanonicalLink();
  EXPECT_EQ(linkEnt2.GetId(), m2->GetCanonicalLink().GetId());
}
