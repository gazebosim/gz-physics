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

#include <gz/common/Console.hh>
#include <gz/common/SubMesh.hh>

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
}

/////////////////////////////////////////////////
CustomMeshShape::CustomMeshShape(
    const common::Mesh &_input,
    const Eigen::Vector3d &_scale)
  : dart::dynamics::MeshShape(_scale, nullptr)
{
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
  scene->mRootNode = node;

  // NOTE(MXG): We are ignoring materials and colors from the mesh, because
  // gazebo will only use dartsim for physics, not for rendering.
  scene->mMaterials = nullptr;

  // Fill in submesh contents
  for (unsigned int i = 0; i < numSubMeshes; ++i)
  {
    const common::SubMeshPtr &inputSubmesh =
        _input.SubMeshByIndex(i).lock();

    scene->mMeshes[i] = nullptr;

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

    scene->mMeshes[i] = mesh.release();
  }

  this->mMesh = scene;
  this->mIsBoundingBoxDirty = true;
  this->mIsVolumeDirty = true;
}

}
}
}
