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

#include <memory>
#include <string>

#include <dart/config.hpp>

#include <gz/common/Mesh.hh>
#include <gz/common/SubMesh.hh>

#include "CustomMeshShape.hh"

using namespace gz;
using namespace physics;
using namespace dartsim;

namespace
{
/////////////////////////////////////////////////
/// \brief Add a single triangle submesh with the given vertices.
void AddTriangleSubMesh(common::Mesh &_mesh, const std::string &_name,
    const math::Vector3d &_v0, const math::Vector3d &_v1,
    const math::Vector3d &_v2)
{
  common::SubMesh subMesh;
  subMesh.SetName(_name);
  subMesh.SetPrimitiveType(common::SubMesh::TRIANGLES);
  for (const auto &v : {_v0, _v1, _v2})
  {
    subMesh.AddVertex(v);
    subMesh.AddNormal(0, 0, 1);
  }
  subMesh.AddIndex(0);
  subMesh.AddIndex(1);
  subMesh.AddIndex(2);
  _mesh.AddSubMesh(subMesh);
}

/////////////////////////////////////////////////
/// \brief Build a mesh with two separated triangle submeshes and one LINES
/// submesh (which carries no collision geometry).
std::unique_ptr<common::Mesh> MakeTwoTriangleMesh()
{
  auto mesh = std::make_unique<common::Mesh>();
  mesh->SetName("two_triangles");

  // Triangle in the z=0 plane near the origin
  AddTriangleSubMesh(*mesh, "tri_a",
      {0, 0, 0}, {1, 0, 0}, {0, 1, 0});

  // Line submesh: accepted by the loader but produces no collision geometry.
  // Its vertices are kept inside the triangles' bounding box since, on
  // DART < 6.20, line vertices are still stored in the aiScene and thus
  // contribute to the bounding box.
  common::SubMesh lines;
  lines.SetName("lines");
  lines.SetPrimitiveType(common::SubMesh::LINES);
  lines.AddVertex(0, 0, 0.5);
  lines.AddNormal(0, 0, 1);
  lines.AddVertex(0, 0, 1.5);
  lines.AddNormal(0, 0, 1);
  lines.AddIndex(0);
  lines.AddIndex(1);
  mesh->AddSubMesh(lines);

  // Second triangle, offset in +z so the bounding box spans both submeshes
  AddTriangleSubMesh(*mesh, "tri_b",
      {0, 0, 2}, {-1, 0, 2}, {0, -1, 2});

  return mesh;
}
}  // namespace

/////////////////////////////////////////////////
TEST(CustomMeshShape_TEST, MultipleSubMeshes)
{
  auto mesh = MakeTwoTriangleMesh();
  ASSERT_EQ(3u, mesh->SubMeshCount());

  auto shape = std::make_shared<CustomMeshShape>(
      *mesh, Eigen::Vector3d::Ones());

  // The bounding box must cover both triangle submeshes.
  const auto &bbox = shape->getBoundingBox();
  const double tol = 1e-6;
  EXPECT_NEAR(-1.0, bbox.getMin().x(), tol);
  EXPECT_NEAR(-1.0, bbox.getMin().y(), tol);
  EXPECT_NEAR(0.0, bbox.getMin().z(), tol);
  EXPECT_NEAR(1.0, bbox.getMax().x(), tol);
  EXPECT_NEAR(1.0, bbox.getMax().y(), tol);
  EXPECT_NEAR(2.0, bbox.getMax().z(), tol);

#if DART_VERSION_AT_LEAST(6, 20, 0)
  // On DART >= 6.20 the shape is built directly as a TriMesh. Check that the
  // submeshes were merged into a single vertex buffer with offset indices and
  // that the line submesh was dropped.
  auto triMesh = shape->getTriMesh();
  ASSERT_NE(nullptr, triMesh);
  ASSERT_EQ(6u, triMesh->getVertices().size());
  ASSERT_EQ(2u, triMesh->getTriangles().size());
  EXPECT_TRUE(triMesh->hasVertexNormals());
  EXPECT_EQ(6u, triMesh->getVertexNormals().size());

  // First submesh vertices and triangle
  EXPECT_TRUE(triMesh->getVertices()[0].isApprox(Eigen::Vector3d(0, 0, 0)));
  EXPECT_TRUE(triMesh->getVertices()[1].isApprox(Eigen::Vector3d(1, 0, 0)));
  EXPECT_TRUE(triMesh->getVertices()[2].isApprox(Eigen::Vector3d(0, 1, 0)));
  EXPECT_EQ(0u, triMesh->getTriangles()[0][0]);
  EXPECT_EQ(1u, triMesh->getTriangles()[0][1]);
  EXPECT_EQ(2u, triMesh->getTriangles()[0][2]);

  // Second submesh vertices and triangle, offset by the first submesh's
  // vertex count
  EXPECT_TRUE(triMesh->getVertices()[3].isApprox(Eigen::Vector3d(0, 0, 2)));
  EXPECT_TRUE(triMesh->getVertices()[4].isApprox(Eigen::Vector3d(-1, 0, 2)));
  EXPECT_TRUE(triMesh->getVertices()[5].isApprox(Eigen::Vector3d(0, -1, 2)));
  EXPECT_EQ(3u, triMesh->getTriangles()[1][0]);
  EXPECT_EQ(4u, triMesh->getTriangles()[1][1]);
  EXPECT_EQ(5u, triMesh->getTriangles()[1][2]);
#endif
}

/////////////////////////////////////////////////
TEST(CustomMeshShape_TEST, Scale)
{
  auto mesh = MakeTwoTriangleMesh();

  const Eigen::Vector3d scale(2.0, 0.5, 3.0);
  auto shape = std::make_shared<CustomMeshShape>(*mesh, scale);
  EXPECT_TRUE(shape->getScale().isApprox(scale));

  const auto &bbox = shape->getBoundingBox();
  const double tol = 1e-6;
  EXPECT_NEAR(-2.0, bbox.getMin().x(), tol);
  EXPECT_NEAR(-0.5, bbox.getMin().y(), tol);
  EXPECT_NEAR(0.0, bbox.getMin().z(), tol);
  EXPECT_NEAR(2.0, bbox.getMax().x(), tol);
  EXPECT_NEAR(0.5, bbox.getMax().y(), tol);
  EXPECT_NEAR(6.0, bbox.getMax().z(), tol);
}

/////////////////////////////////////////////////
TEST(CustomMeshShape_TEST, InvalidSubMeshesAreSkipped)
{
  common::Mesh mesh;
  mesh.SetName("invalid");

  // Valid triangle
  AddTriangleSubMesh(mesh, "tri",
      {0, 0, 0}, {1, 0, 0}, {0, 1, 0});

  // Normal count does not match vertex count: must be ignored
  common::SubMesh noNormals;
  noNormals.SetName("no_normals");
  noNormals.SetPrimitiveType(common::SubMesh::TRIANGLES);
  noNormals.AddVertex(0, 0, 10);
  noNormals.AddVertex(1, 0, 10);
  noNormals.AddVertex(0, 1, 10);
  noNormals.AddIndex(0);
  noNormals.AddIndex(1);
  noNormals.AddIndex(2);
  mesh.AddSubMesh(noNormals);

  // Unsupported primitive type: must be ignored
  common::SubMesh triStrip;
  triStrip.SetName("tristrip");
  triStrip.SetPrimitiveType(common::SubMesh::TRISTRIPS);
  for (int i = 0; i < 4; ++i)
  {
    triStrip.AddVertex(i, 0, -10);
    triStrip.AddNormal(0, 0, 1);
    triStrip.AddIndex(i);
  }
  mesh.AddSubMesh(triStrip);

  auto shape = std::make_shared<CustomMeshShape>(
      mesh, Eigen::Vector3d::Ones());

  // Only the valid triangle contributes to the bounding box.
  const auto &bbox = shape->getBoundingBox();
  const double tol = 1e-6;
  EXPECT_NEAR(0.0, bbox.getMin().z(), tol);
  EXPECT_NEAR(0.0, bbox.getMax().z(), tol);
  EXPECT_NEAR(1.0, bbox.getMax().x(), tol);
  EXPECT_NEAR(1.0, bbox.getMax().y(), tol);

#if DART_VERSION_AT_LEAST(6, 20, 0)
  auto triMesh = shape->getTriMesh();
  ASSERT_NE(nullptr, triMesh);
  EXPECT_EQ(3u, triMesh->getVertices().size());
  EXPECT_EQ(1u, triMesh->getTriangles().size());
#endif
}
