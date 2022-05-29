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

#include "AABBTree.hh"

using namespace gz;
using namespace physics;
using namespace tpelib;

/////////////////////////////////////////////////
TEST(AABBTree, AABBTree)
{
  AABBTree tree;
  EXPECT_EQ(0u, tree.NodeCount());

  // add node
  math::AxisAlignedBox a(-math::Vector3d::One, math::Vector3d::One);
  std::size_t aId = 1u;
  tree.AddNode(aId, a);
  EXPECT_EQ(1u, tree.NodeCount());
  EXPECT_TRUE(tree.HasNode(aId));

  math::AxisAlignedBox b(math::Vector3d(-3, -3, -3),
     math::Vector3d(-2, -2, -2));
  std::size_t bId = 2u;
  tree.AddNode(bId, b);
  EXPECT_EQ(2u, tree.NodeCount());
  EXPECT_TRUE(tree.HasNode(bId));

  // c overlaps with a and b
  math::AxisAlignedBox c(math::Vector3d(-2.5, -2.5, -2.5),
      math::Vector3d(0.5, 0.5, 0.5));
  std::size_t cId = 3u;
  tree.AddNode(cId, c);
  EXPECT_EQ(3u, tree.NodeCount());
  EXPECT_TRUE(tree.HasNode(cId));

  // d overlaps with a only
  math::AxisAlignedBox d(math::Vector3d(0.55, 0.55, 0.55),
      math::Vector3d(0.75, 0.75, 0.75));
  std::size_t dId = 4u;
  tree.AddNode(dId, d);
  EXPECT_EQ(4u, tree.NodeCount());
  EXPECT_TRUE(tree.HasNode(dId));

  // e does not overlap with any node
  math::AxisAlignedBox e(math::Vector3d(2.55, 2.55, 2.55),
      math::Vector3d(3.75, 3.75, 3.75));
  std::size_t eId = 5u;
  tree.AddNode(eId, e);
  EXPECT_EQ(5u, tree.NodeCount());
  EXPECT_TRUE(tree.HasNode(eId));

  // check collisions
  std::set<std::size_t> result = tree.Collisions(aId);
  EXPECT_EQ(2u, result.size());
  EXPECT_EQ(1u, result.count(cId));
  EXPECT_EQ(1u, result.count(dId));

  result.clear();
  result = tree.Collisions(bId);
  EXPECT_EQ(1u, result.size());
  EXPECT_EQ(1u, result.count(cId));

  result.clear();
  result = tree.Collisions(cId);
  EXPECT_EQ(2u, result.size());
  EXPECT_EQ(1u, result.count(aId));
  EXPECT_EQ(1u, result.count(bId));

  result.clear();
  result = tree.Collisions(dId);
  EXPECT_EQ(1u, result.size());
  EXPECT_EQ(1u, result.count(aId));

  result.clear();
  result = tree.Collisions(eId);
  EXPECT_EQ(0u, result.size());

  // remove non-existent node - this should fail
  EXPECT_FALSE(tree.RemoveNode(555u));

  // remove node e
  EXPECT_TRUE(tree.RemoveNode(eId));
  EXPECT_EQ(4u, tree.NodeCount());
  EXPECT_FALSE(tree.HasNode(eId));

  // remove node b
  EXPECT_TRUE(tree.RemoveNode(bId));
  EXPECT_EQ(3u, tree.NodeCount());
  EXPECT_FALSE(tree.HasNode(bId));

  // verify collisions again
  // collision check for b and d should fail and return 0 collisions
  result.clear();
  result = tree.Collisions(aId);
  EXPECT_EQ(2u, result.size());
  EXPECT_EQ(1u, result.count(cId));
  EXPECT_EQ(1u, result.count(dId));

  result.clear();
  result = tree.Collisions(bId);
  EXPECT_EQ(0u, result.size());

  result.clear();
  result = tree.Collisions(cId);
  EXPECT_EQ(1u, result.size());
  EXPECT_EQ(1u, result.count(aId));

  result.clear();
  result = tree.Collisions(dId);
  EXPECT_EQ(1u, result.size());
  EXPECT_EQ(1u, result.count(aId));

  result.clear();
  result = tree.Collisions(eId);
  EXPECT_EQ(0u, result.size());

  // update node c so it no longer overlaps with a or b or any other nodes
  EXPECT_TRUE(tree.UpdateNode(cId, math::AxisAlignedBox(
    math::Vector3d(-40, -40, -40),
    math::Vector3d(-10, -10, -10))));
  EXPECT_EQ(3u, tree.NodeCount());
  EXPECT_TRUE(tree.HasNode(cId));

  // verify collisions again
  // c should have 0 collisions
  result.clear();
  result = tree.Collisions(aId);
  EXPECT_EQ(1u, result.size());
  EXPECT_EQ(1u, result.count(dId));

  result.clear();
  result = tree.Collisions(bId);
  EXPECT_EQ(0u, result.size());

  result.clear();
  result = tree.Collisions(cId);
  EXPECT_EQ(0u, result.size());

  result.clear();
  result = tree.Collisions(dId);
  EXPECT_EQ(1u, result.size());
  EXPECT_EQ(1u, result.count(aId));

  result.clear();
  result = tree.Collisions(eId);
  EXPECT_EQ(0u, result.size());
}
