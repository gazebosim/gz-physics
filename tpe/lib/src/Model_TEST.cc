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

#include "ignition/physics/tpe/Link.hh"
#include "ignition/physics/tpe/Model.hh"

using namespace ignition;
using namespace physics;
using namespace tpe;

/////////////////////////////////////////////////
TEST(Model, BasicAPI)
{
  Model model;
  model.SetId(1234u);
  EXPECT_EQ(1234u, model.GetId());

  model.SetName("model_1");
  EXPECT_EQ("model_1", model.GetName());

  model.SetPose(math::Pose3d(1, 2, 3, 0.1, 0.2, 0.3));
  EXPECT_EQ(math::Pose3d(1, 2, 3, 0.1, 0.2, 0.3), model.GetPose());

  Model model2;
  EXPECT_NE(model.GetId(), model2.GetId());
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

  uint64_t linkId = linkEnt.GetId();
  Entity ent = model.GetChildById(linkId);
  EXPECT_EQ(linkId, ent.GetId());
  EXPECT_EQ("link_1", ent.GetName());
  EXPECT_EQ(math::Pose3d(2, 3, 4, 0, 0, 1), ent.GetPose());

  Entity entByName = model.GetChildByName("link_1");
  EXPECT_EQ("link_1", entByName.GetName());

  // test casting to link
  Link *link = static_cast<Link *>(&linkEnt);
  EXPECT_NE(nullptr, link);
  EXPECT_EQ(linkEnt.GetId(), link->GetId());

  // add another child
  Entity &linkEnt2 = model.AddLink();
  EXPECT_EQ(2u, model.GetChildCount());

  Link *link2 = static_cast<Link *>(&linkEnt2);
  EXPECT_NE(nullptr, link2);
  EXPECT_EQ(linkEnt2.GetId(), link2->GetId());

  // test remove child by id
  model.RemoveChildById(linkId);
  EXPECT_EQ(1u, model.GetChildCount());

  Entity nullEnt = model.GetChildById(linkId);
  EXPECT_EQ(Entity::kNullEntity.GetId(), nullEnt.GetId());
}

