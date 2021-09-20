/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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
#include <BulletDynamics/Featherstone/btMultiBodyLinkCollider.h>
#include <LinearMath/btVector3.h>

#include <ignition/math/Pose3.hh>
#include <memory>
#include <utility>

#include "Base.hh"

namespace ignition {
namespace physics {
namespace bullet {

/////////////////////////////////////////////////
Identity ShapeFeatures::AttachMeshShape(const Identity& _linkID,
                                        const std::string& _name,
                                        const ignition::common::Mesh& _mesh,
                                        const Pose3d& _pose,
                                        const LinearVector3d& _scale) {
  // double* vertices = nullptr;
  // int* indices = nullptr;
  // _mesh.FillArrays(&vertices, &indices);

  // unsigned int numVertices = _mesh.VertexCount();
  // unsigned int numIndices = _mesh.IndexCount();

  // // TODO(joxoby): unique ptr
  // auto mTriMesh = new btTriangleMesh();

  // for (unsigned int j = 0; j < numVertices; ++j) {
  //   // Scale the vertex data
  //   vertices[j * 3 + 0] = vertices[j * 3 + 0] * _scale[0];
  //   vertices[j * 3 + 1] = vertices[j * 3 + 1] * _scale[1];
  //   vertices[j * 3 + 2] = vertices[j * 3 + 2] * _scale[2];
  // }

  // // Create the Bullet trimesh
  // for (unsigned int j = 0; j < numIndices; j += 3) {
  //   btVector3 bv0(vertices[indices[j] * 3 + 0], vertices[indices[j] * 3 + 1],
  //                 vertices[indices[j] * 3 + 2]);

  //   btVector3 bv1(vertices[indices[j + 1] * 3 + 0],
  //                 vertices[indices[j + 1] * 3 + 1],
  //                 vertices[indices[j + 1] * 3 + 2]);

  //   btVector3 bv2(vertices[indices[j + 2] * 3 + 0],
  //                 vertices[indices[j + 2] * 3 + 1],
  //                 vertices[indices[j + 2] * 3 + 2]);

  //   mTriMesh->addTriangle(bv0, bv1, bv2);
  // }

  // // TODO(lobotuerk) Save collision if needed
  // // collision->shape = gimpactMeshShape;

  // delete[] vertices;
  // delete[] indices;

  // auto link = std::get<Link*>(this->entities.at(_linkID));
  // auto rootModel = link->rootModel;
  // auto linkIndex = rootModel->vertexIdToLinkIndex.at(link->vertexId);

  // /* TO-DO(Lobotuerk): figure out if this line is needed */
  // // gimpactMeshShape->setMargin(btScalar(0.001));

  // auto gImpactMeshShape = std::make_unique<btGImpactMeshShape>(mTriMesh);
  // gImpactMeshShape->updateBound();

  // // TODO(joxoby): store this a unique ptr
  // auto btCollision =
  //     new btMultiBodyLinkCollider(rootModel->multibody.get(), linkIndex);
  // btCollision->setCollisionShape(gImpactMeshShape.get());

  // // Collison pose
  // // TODO(joxoby): Use the collision pose from the SDF
  // auto localPose = rootModel->vertexIdToLinkPoseFromPivot.at(link->vertexId);
  // auto shapePose = localPose;
  // auto localPos = convertVec(ignition::math::eigen3::convert(shapePose.Pos()));
  // auto localRot = convertQuat(shapePose.Rot());
  // auto pos = rootModel->multibody->localPosToWorld(linkIndex, localPos);
  // auto mat = rootModel->multibody->localFrameToWorld(linkIndex, btMatrix3x3(localRot));
  // btTransform tr(mat, pos);
  // btCollision->setWorldTransform(tr);

  // if (linkIndex == -1) {
  //   rootModel->multibody->setBaseCollider(btCollision);
  // } else {
  //   rootModel->multibody->getLink(linkIndex).m_collider = btCollision;
  // }
  // // TODO(joxoby): More than one collision shape
  // // TODO(joxoby): 2, 1 + 2 ??
  // rootModel->world->addCollisionObject(btCollision, 2, 1 + 2);

  return GenerateInvalidId();
}

/////////////////////////////////////////////////
Identity ShapeFeatures::CastToMeshShape(const Identity& _shapeID) const {
  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
Identity ShapeFeatures::AttachBoxShape(const Identity& _linkID,
                                       const std::string& _name,
                                       const LinearVector3d& _size,
                                       const Pose3d& _pose) {
  // btVector3 halfExtents(_size(0) / 2.0, _size(1) / 2.0, _size(2) / 2.0);

  // auto box = std::make_unique<btBoxShape>(halfExtents);

  // auto link = std::get<Link*>(this->entities.at(_linkID));
  // auto body = link->body.get();

  // auto poseWithInertia =
  //     link->sdfInertialPose.Inverse() * ignition::math::eigen3::convert(_pose);
  // auto poseIsometry = ignition::math::eigen3::convert(poseWithInertia);
  // auto poseTranslation = poseIsometry.translation();
  // auto poseLinear = poseIsometry.linear();
  // btTransform baseTransform;
  // baseTransform.setOrigin(convertVec(poseTranslation));
  // baseTransform.setBasis(convertMat(poseLinear));

  // bool isMesh = false;
  // auto collision = std::make_unique<Collision>(
  //     _name, body, isMesh, std::move(box), nullptr, baseTransform);

  // auto identity = this->AddCollision(_linkID, std::move(collision));

  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
Identity ShapeFeatures::CastToBoxShape(const Identity& _shapeID) const {
  auto collision = std::get<Collision*>(this->entities.at(_shapeID));

  // auto shape = collision->shape.get();

  // if (dynamic_cast<btBoxShape*>(shape)) {
  //   return this->GenerateIdentity(_shapeID, this->Reference(_shapeID));
  // }

  return this->GenerateInvalidId();
}

}  // namespace bullet
}  // namespace physics
}  // namespace ignition
