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

#include <ignition/common/Mesh.hh>
#include <ignition/common/SubMesh.hh>

#include "Shape.hh"

using namespace ignition;
using namespace physics;
using namespace tpe;
using namespace lib;

/////////////////////////////////////////////////
TEST(Shape, BoxShape)
{
  BoxShape shape;
  EXPECT_EQ(ShapeType::BOX, shape.GetType());
  math::AxisAlignedBox empty = shape.GetBoundingBox();
  EXPECT_EQ(math::Vector3d::Zero, empty.Center());
  EXPECT_EQ(math::Vector3d::Zero, empty.Size());

  math::Vector3d size(1.2, 3.6, 5.8);
  shape.SetSize(size);
  math::AxisAlignedBox bbox = shape.GetBoundingBox();
  EXPECT_EQ(math::Vector3d::Zero, bbox.Center());
  EXPECT_EQ(size, bbox.Size());
  EXPECT_EQ(math::Vector3d(-0.6, -1.8, -2.9), bbox.Min());
  EXPECT_EQ(math::Vector3d(0.6, 1.8, 2.9), bbox.Max());
}

/////////////////////////////////////////////////
TEST(Shape, CylinderShape)
{
  CylinderShape shape;
  EXPECT_EQ(ShapeType::CYLINDER, shape.GetType());
  math::AxisAlignedBox empty = shape.GetBoundingBox();
  EXPECT_EQ(math::Vector3d::Zero, empty.Center());
  EXPECT_EQ(math::Vector3d::Zero, empty.Size());

  double radius = 0.6;
  double length = 2.8;
  shape.SetRadius(radius);
  shape.SetLength(length);
  math::AxisAlignedBox bbox = shape.GetBoundingBox();
  EXPECT_EQ(math::Vector3d::Zero, bbox.Center());
  EXPECT_EQ(math::Vector3d(1.2, 1.2, 2.8), bbox.Size());
  EXPECT_EQ(math::Vector3d(-0.6, -0.6, -1.4), bbox.Min());
  EXPECT_EQ(math::Vector3d(0.6, 0.6, 1.4), bbox.Max());
}

/////////////////////////////////////////////////
TEST(Shape, SphereShape)
{
  SphereShape shape;
  EXPECT_EQ(ShapeType::SPHERE, shape.GetType());
  math::AxisAlignedBox empty = shape.GetBoundingBox();
  EXPECT_EQ(math::Vector3d::Zero, empty.Center());
  EXPECT_EQ(math::Vector3d::Zero, empty.Size());

  double radius = 30.2;
  shape.SetRadius(radius);
  math::AxisAlignedBox bbox = shape.GetBoundingBox();
  EXPECT_EQ(math::Vector3d::Zero, bbox.Center());
  EXPECT_EQ(math::Vector3d(60.4, 60.4, 60.4), bbox.Size());
  EXPECT_EQ(math::Vector3d(-30.2, -30.2, -30.2), bbox.Min());
  EXPECT_EQ(math::Vector3d(30.2, 30.2, 30.2), bbox.Max());
}

/////////////////////////////////////////////////
TEST(Shape, MeshShape)
{
  MeshShape shape;
  EXPECT_EQ(ShapeType::MESH, shape.GetType());

  // create mesh
  common::Mesh mesh;
  common::SubMesh submesh;
  math::Vector3d v0(0, 0, 0);
  math::Vector3d v1(0, 1, 0);
  math::Vector3d v2(0, 1, 1);
  submesh.AddVertex(v0);
  submesh.AddVertex(v1);
  submesh.AddVertex(v2);
  mesh.AddSubMesh(submesh);

  shape.SetMesh(mesh);
  math::Vector3d scale = math::Vector3d(1.0, 1.0, 1.0);
  shape.SetScale(scale);
  math::AxisAlignedBox bbox = shape.GetBoundingBox();
  EXPECT_EQ(math::Vector3d(0, 0.5, 0.5), bbox.Center());
  EXPECT_EQ(math::Vector3d(0, 1.0, 1.0), bbox.Size());
  EXPECT_EQ(v0, bbox.Min());
  EXPECT_EQ(v2, bbox.Max());
}
