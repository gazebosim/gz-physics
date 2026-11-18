/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

#include "CustomMeshShape.hh"

#include <memory>
#include <string>

#ifdef DART_USES_TRIMESH
#include <dart/math/TriMesh.hpp>
#endif

#include <gz/common/Console.hh>
#include <gz/common/SubMesh.hh>
#include <gz/common/Profiler.hh>

namespace gz {
namespace physics {
namespace dartsim {

namespace {
/////////////////////////////////////////////////
unsigned int CheckNumVerticesPerFaces(
    const common::SubMesh &_inputSubmesh,
    const unsigned int _submeshIndex,
    const std::string &_path)
{
  using namespace common;

  const SubMesh::PrimitiveType type = _inputSubmesh.SubMeshPrimitiveType();

  const auto printWarning = [&](const std::string type_str)
  {
    gzwarn << "[dartsim::CustomMeshShape] The dartsim plugin does not support "
            << type_str << " meshes, requested by submesh [" << _submeshIndex
           << ":" << _inputSubmesh.Name() << "] in the input mesh [" << _path
           << "]. This submesh will be ignored.\n";

    return 0u;
  };

  if (SubMesh::POINTS == type)
    return 1u;

  if (SubMesh::LINES == type)
    return 2u;

  if (SubMesh::LINESTRIPS == type)
    return printWarning("linestrip");

  if (SubMesh::TRIANGLES == type)
    return 3u;

  if (SubMesh::TRIFANS == type)
    return printWarning("trifan");

  if (SubMesh::TRISTRIPS == type)
    return printWarning("tristrip");

  gzwarn << "[dartsim::CustomMeshShape] One of the submeshes ["
          << _submeshIndex << ":" << _inputSubmesh.Name() << "] in the input "
          << "mesh [" << _path << "] has an unknown primitive type value ["
          << type << "]. This submesh will be ignored.\n";

  return 0;
}

#ifdef DART_USES_TRIMESH
/////////////////////////////////////////////////
/// \brief Convert a common::Mesh into a DART TriMesh. Only triangle submeshes
/// contribute geometry; point and line submeshes carry no collision geometry
/// and are skipped silently, while unsupported primitive types are reported.
std::shared_ptr<dart::math::TriMesh<double>> MakeTriMesh(
    const common::Mesh &_input)
{
  GZ_PROFILE("CustomMeshShape::MakeTriMesh");
  using TriMesh = dart::math::TriMesh<double>;
  auto triMesh = std::make_shared<TriMesh>();

  const unsigned int numSubMeshes = _input.SubMeshCount();
  for (unsigned int i = 0; i < numSubMeshes; ++i)
  {
    const common::SubMeshPtr &inputSubmesh =
        _input.SubMeshByIndex(i).lock();

    if (!inputSubmesh)
    {
      gzerr << "[dartsim::CustomMeshShape] One of the submeshes [" << i
             << "] in the input mesh [" << _input.Path() << "] has expired!\n";
      continue;
    }

    const unsigned int numVertices = inputSubmesh->VertexCount();
    if (inputSubmesh->NormalCount() != numVertices)
    {
      gzerr << "[dartsim::CustomMeshShape] One of the submeshes [" << i << ":"
             << inputSubmesh->Name() << "] in the input mesh [" << _input.Path()
             << "] does not have a normal count ["
             << inputSubmesh->NormalCount() << "] that matches its vertex "
             << "count [" << numVertices << "]. This submesh will be "
             << "ignored!\n";
      continue;
    }

    const unsigned int numVerticesPerFace =
        CheckNumVerticesPerFaces(*inputSubmesh, i, _input.Path());
    // Points and lines are accepted by the legacy Assimp path but they do not
    // produce any collision geometry, so there is nothing to add to a TriMesh.
    if (3u != numVerticesPerFace)
      continue;

    const unsigned int numFaces = inputSubmesh->IndexCount() / 3u;

    // Validate indices before touching the output mesh so that a broken
    // submesh can be skipped as a whole.
    bool invalidIndex = false;
    for (unsigned int j = 0; j < numFaces * 3u; ++j)
    {
      const int vertexIndex = inputSubmesh->Index(j);
      if (vertexIndex < 0 ||
          static_cast<unsigned int>(vertexIndex) >= numVertices)
      {
        gzwarn << "[dartsim::CustomMeshShape] The submesh [" << i << ":"
                << inputSubmesh->Name() << "] of mesh [" << _input.Path()
                << "] has an invalid vertex index [" << vertexIndex
                << "] at primitive index [" << j << "]. Its vertex count is ["
                << numVertices << "]. This submesh will be ignored.\n";
        invalidIndex = true;
        break;
      }
    }
    if (invalidIndex)
      continue;

    // Submeshes share one vertex buffer in the TriMesh, so offset indices.
    const auto vertexOffset =
        static_cast<TriMesh::Index>(triMesh->getVertices().size());

    triMesh->reserveVertices(vertexOffset + numVertices);
    triMesh->reserveVertexNormals(vertexOffset + numVertices);
    for (unsigned int j = 0; j < numVertices; ++j)
    {
      const math::Vector3d &v = inputSubmesh->Vertex(j);
      triMesh->addVertex(v.X(), v.Y(), v.Z());

      const math::Vector3d &n = inputSubmesh->Normal(j);
      triMesh->addVertexNormal(n.X(), n.Y(), n.Z());
    }

    triMesh->reserveTriangles(triMesh->getTriangles().size() + numFaces);
    const auto index = [&](unsigned int _k) -> TriMesh::Index
    {
      return vertexOffset +
          static_cast<TriMesh::Index>(inputSubmesh->Index(_k));
    };
    for (unsigned int j = 0; j < numFaces; ++j)
    {
      triMesh->addTriangle(index(3 * j), index(3 * j + 1), index(3 * j + 2));
    }
  }

  return triMesh;
}
#else
/////////////////////////////////////////////////
unsigned int GetPrimitiveType(
    const common::SubMesh &_inputSubmesh)
{
  using namespace common;

  const SubMesh::PrimitiveType type = _inputSubmesh.SubMeshPrimitiveType();

  if (SubMesh::POINTS == type)
    return aiPrimitiveType_POINT;

  if (SubMesh::LINES == type)
    return aiPrimitiveType_LINE;

  if (SubMesh::TRIANGLES == type)
    return aiPrimitiveType_TRIANGLE;

  return 0;
}
#endif
}

#ifdef DART_USES_TRIMESH
/////////////////////////////////////////////////
CustomMeshShape::CustomMeshShape(
    const common::Mesh &_input,
    const Eigen::Vector3d &_scale)
  : dart::dynamics::MeshShape(_scale, MakeTriMesh(_input))
{
}
#else
/////////////////////////////////////////////////
CustomMeshShape::CustomMeshShape(
    const common::Mesh &_input,
    const Eigen::Vector3d &_scale)
  : dart::dynamics::MeshShape(_scale, nullptr)
{
  GZ_PROFILE("CustomMeshShape::CustomMeshShape");
  // Create the root
  aiNode* node = new aiNode;

  // Allocate space for pointer arrays
  const unsigned int numSubMeshes = _input.SubMeshCount();
  node->mNumMeshes = numSubMeshes;
  node->mMeshes = new unsigned int[numSubMeshes];
  for (unsigned int i = 0; i < numSubMeshes; ++i)
    node->mMeshes[i] = i;

  aiScene *scene = new aiScene;
  scene->mNumMeshes = numSubMeshes;
  scene->mMeshes = new aiMesh*[numSubMeshes];
  for (unsigned int i = 0; i < numSubMeshes; ++i)
    scene->mMeshes[i] = nullptr;
  scene->mRootNode = node;

  // NOTE(MXG): We are ignoring materials and colors from the mesh, because
  // gazebo will only use dartsim for physics, not for rendering.
  scene->mMaterials = nullptr;

  // Number of submeshes that were successfully converted. Skipped submeshes
  // must not leave null entries in scene->mMeshes since DART iterates over
  // all of them when computing the bounding box and volume.
  unsigned int numValidMeshes = 0;

  // Fill in submesh contents
  for (unsigned int i = 0; i < numSubMeshes; ++i)
  {
    const common::SubMeshPtr &inputSubmesh =
        _input.SubMeshByIndex(i).lock();

    if (!inputSubmesh)
    {
      gzerr << "[dartsim::CustomMeshShape] One of the submeshes [" << i
             << "] in the input mesh [" << _input.Path() << "] has expired!\n";
      continue;
    }

    std::unique_ptr<aiMesh> mesh = std::make_unique<aiMesh>();
    mesh->mMaterialIndex = static_cast<unsigned int>(-1);

    const unsigned int numVertices = inputSubmesh->VertexCount();
    if (inputSubmesh->NormalCount() != numVertices)
    {
      gzerr << "[dartsim::CustomMeshShape] One of the submeshes [" << i << ":"
             << inputSubmesh->Name() << "] in the input mesh [" << _input.Path()
             << "] does not have a normal count ["
             << inputSubmesh->NormalCount() << "] that matches its vertex "
             << "count [" << numVertices << "]. This submesh will be "
             << "ignored!\n";
      continue;
    }

    mesh->mNumVertices = numVertices;
    mesh->mVertices = new aiVector3D[numVertices];
    mesh->mNormals = new aiVector3D[numVertices];

    const unsigned int numVerticesPerFace =
        CheckNumVerticesPerFaces(*inputSubmesh, i, _input.Path());
    if (0 == numVerticesPerFace)
      continue;

    mesh->mPrimitiveTypes = GetPrimitiveType(*inputSubmesh);
    if (0 == mesh->mPrimitiveTypes)
      continue;

    const unsigned int numFaces = static_cast<unsigned int>(
          static_cast<double>(inputSubmesh->IndexCount())
          /static_cast<double>(numVerticesPerFace));

    // Fill in the contents of each submesh
    mesh->mNumFaces = numFaces;
    mesh->mFaces = new aiFace[numFaces];
    unsigned int currentPrimitiveIndex = 0;
    bool primitiveIndexOverflow = false;
    for (unsigned int j = 0; j < numFaces; ++j)
    {
      mesh->mFaces[j].mNumIndices = numVerticesPerFace;
      mesh->mFaces[j].mIndices = new unsigned int[numVerticesPerFace];

      for (unsigned int k = 0; k < numVerticesPerFace; ++k)
      {
        int vertexIndex = inputSubmesh->Index(currentPrimitiveIndex);
        if (vertexIndex == -1)
        {
          gzwarn << "[dartsim::CustomMeshShape] The submesh [" << i << ":"
                  << inputSubmesh->Name() << "] of mesh [" << _input.Path()
                  << "] overflowed at primitive index ["
                  << currentPrimitiveIndex << "]. Its expected number of "
                  << "primitive indices is [" << _input.IndexCount()
                  << "]. This submesh will be ignored.\n";
          primitiveIndexOverflow = true;
          break;
        }

        mesh->mFaces[j].mIndices[k] = static_cast<unsigned int>(vertexIndex);
        ++currentPrimitiveIndex;
      }

      if (primitiveIndexOverflow)
        break;
    }

    if (primitiveIndexOverflow)
      continue;

    for (unsigned int j = 0; j < numVertices; ++j)
    {
      const math::Vector3d &v = inputSubmesh->Vertex(j);
      for (unsigned int k = 0; k < 3; ++k)
        mesh->mVertices[j][k] = static_cast<ai_real>(v[k]);

      const math::Vector3d &n = inputSubmesh->Normal(j);
      for (unsigned int k = 0; k < 3; ++k)
        mesh->mNormals[j][k] = static_cast<ai_real>(n[k]);
    }

    scene->mMeshes[numValidMeshes++] = mesh.release();
  }

  scene->mNumMeshes = numValidMeshes;
  node->mNumMeshes = numValidMeshes;

  this->mMesh = scene;
  this->mIsBoundingBoxDirty = true;
  this->mIsVolumeDirty = true;
}
#endif

}
}
}
