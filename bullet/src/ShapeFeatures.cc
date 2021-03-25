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

#include "ShapeFeatures.hh"
#include <BulletCollision/Gimpact/btGImpactShape.h>

#include <memory>

namespace ignition {
namespace physics {
namespace bullet {

/////////////////////////////////////////////////
Identity ShapeFeatures::AttachMeshShape(
    const Identity &_linkID,
    const std::string &_name,
    const ignition::common::Mesh &_mesh,
    const Pose3d &_pose,
    const LinearVector3d &_scale)
{
  double *vertices = nullptr;
  int *indices = nullptr;
  _mesh.FillArrays(&vertices, &indices);

  unsigned int numVertices = _mesh.VertexCount();
  unsigned int numIndices = _mesh.IndexCount();

  const auto mTriMesh = std::make_shared<btTriangleMesh>();

  for (unsigned int j = 0;  j < numVertices; ++j)
  {
    // Scale the vertex data
    vertices[j*3+0] = vertices[j*3+0] * _scale[0];
    vertices[j*3+1] = vertices[j*3+1] * _scale[1];
    vertices[j*3+2] = vertices[j*3+2] * _scale[2];
  }

  // Create the Bullet trimesh
  for (unsigned int j = 0; j < numIndices; j += 3)
  {
    btVector3 bv0(vertices[indices[j]*3+0],
                  vertices[indices[j]*3+1],
                  vertices[indices[j]*3+2]);

    btVector3 bv1(vertices[indices[j+1]*3+0],
                  vertices[indices[j+1]*3+1],
                  vertices[indices[j+1]*3+2]);

    btVector3 bv2(vertices[indices[j+2]*3+0],
                  vertices[indices[j+2]*3+1],
                  vertices[indices[j+2]*3+2]);

    mTriMesh.get()->addTriangle(bv0, bv1, bv2);
  }

  auto gimpactMeshShape =
    std::make_shared<btGImpactMeshShape>(mTriMesh.get());
  gimpactMeshShape.get()->updateBound();

  // TODO(lobotuerk) Save collision if needed
  // collision->shape = gimpactMeshShape;

  delete [] vertices;
  delete [] indices;

  const auto &linkInfo = this->links.at(_linkID);
  const auto &modelID = linkInfo->model;
  const auto &body = linkInfo->link.get();

  auto poseWithInertia =
    linkInfo->inertialPose.Inverse() * ignition::math::eigen3::convert(_pose);
  const auto poseIsometry = ignition::math::eigen3::convert(poseWithInertia);
  const auto poseTranslation = poseIsometry.translation();
  const auto poseLinear = poseIsometry.linear();
  btTransform baseTransform;
  baseTransform.setOrigin(convertVec(poseTranslation));
  baseTransform.setBasis(convertMat(poseLinear));

  /* TO-DO(Lobotuerk): figure out if this line is needed */
  // gimpactMeshShape->setMargin(btScalar(0.001));

  dynamic_cast<btCompoundShape *>(
    body->getCollisionShape())->addChildShape(
    baseTransform, gimpactMeshShape.get());

  auto identity = this->AddCollision(
    _linkID, {_name, gimpactMeshShape, _linkID, modelID,
    ignition::math::eigen3::convert(_pose), true});
  return identity;

}

/////////////////////////////////////////////////
Identity ShapeFeatures::CastToMeshShape(
    const Identity &_shapeID) const
{
  if (this->collisions.at(_shapeID)->isMesh)
  {
    return _shapeID;
  }
  return this->GenerateInvalidId();
}

}  // namespace bullet
}  // namespace physics
}  // namespace ignition
