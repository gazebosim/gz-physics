#include "ShapeFeatures.hh"
#include <BulletCollision/Gimpact/btGImpactShape.h>
#include <BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h>

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

  btTriangleMesh *mTriMesh = new btTriangleMesh();

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

    mTriMesh->addTriangle(bv0, bv1, bv2);
  }

  btGImpactMeshShape *gimpactMeshShape =
    new btGImpactMeshShape(mTriMesh);
  gimpactMeshShape->updateBound();

  // TODO(lobotuerk) Save collision if needed
  // collision->shape = gimpactMeshShape;

  delete [] vertices;
  delete [] indices;

  const auto &linkInfo = this->links.at(_linkID);
  const auto &modelID = linkInfo->model;
  const auto &modelInfo = this->models.at(modelID);
  const auto &worldInfo = this->worlds.at(modelInfo->world);
  const auto &world = worldInfo->world;
  delete link->getCollisionShape();
  gimpactMeshShape->setMargin(btScalar(0.001));
  link->setCollisionShape(gimpactMeshShape);

  const auto poseIsometry = _pose;
  const auto poseTranslation = poseIsometry.translation();
  const auto poseLinear = poseIsometry.linear();
  btTransform baseTransform;
  baseTransform.setOrigin(convertVec(poseTranslation));
  baseTransform.setBasis(convertMat(poseLinear));

  // gimpactMeshShape->setMargin(btScalar(0.001));

  dynamic_cast<btCompoundShape *>(body->getCollisionShape())->addChildShape(baseTransform, gimpactMeshShape);
  btGImpactCollisionAlgorithm::registerAlgorithm(worldInfo->dispatcher);

  auto identity = this->AddCollision({_name, gimpactMeshShape, _linkID,
                             modelID, ignition::math::eigen3::convert(_pose)});
  return identity;

}

/////////////////////////////////////////////////
Identity ShapeFeatures::CastToMeshShape(
    const Identity &_shapeID) const
{
  ignwarn << "Dummy implementation CastToMeshShape" << std::endl;
  (void) _shapeID;
  return this->GenerateInvalidId();
}

}
}
}
