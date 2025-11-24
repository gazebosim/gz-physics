/*
 * Copyright (C) 2025 Open Source Robotics Foundation
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

#include "SDFFeatures.hh"

#include <mujoco/mjmodel.h>
#include <mujoco/mjspec.h>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <gz/common/Console.hh>
#include <gz/common/Mesh.hh>
#include <gz/common/MeshManager.hh>
#include <gz/common/SubMesh.hh>
#include <gz/physics/Entity.hh>
#include <iostream>
#include <iterator>
#include <memory>
#include <sdf/Box.hh>
#include <sdf/Capsule.hh>
#include <sdf/Collision.hh>
#include <sdf/Cone.hh>
#include <sdf/Cylinder.hh>
#include <sdf/Ellipsoid.hh>
#include <sdf/Geometry.hh>
#include <sdf/Joint.hh>
#include <sdf/JointAxis.hh>
#include <sdf/Link.hh>
#include <sdf/Mesh.hh>
#include <sdf/Model.hh>
#include <sdf/Physics.hh>
#include <sdf/Plane.hh>
#include <sdf/Sphere.hh>
#include <sdf/Surface.hh>
#include <string>

#include "Base.hh"

namespace gz
{
namespace physics
{
namespace mujoco
{

/////////////////////////////////////////////////
Identity SDFFeatures::ConstructSdfModel(const Identity &_parentID,
                                        const ::sdf::Model &_sdfModel)
{
  return this->ConstructSdfModelImpl(_parentID, _sdfModel);
}

/////////////////////////////////////////////////
struct ModelKinematicStructure
{
  std::vector<const ::sdf::Link *> links;
  std::vector<const ::sdf::Link *> parents;
  std::vector<std::vector<std::size_t>> children;
  // For index i, parentInJoint[i]->parent = links[i], unless the link is
  // "world" or that link is not referenced by any joint.
  // TODO(azeey) This might not be needed at all.
  std::vector<const ::sdf::Joint *> parentInJoint;
  // For index i, childJInoint[i]->parent = links[i], unless that link is not
  // referenced by any joint as a child.
  std::vector<const ::sdf::Joint *> childInJoint;

  std::optional<std::size_t> FindLinkByName(const std::string &_name)
  {
    auto it = std::find_if(links.begin(), links.end(),
                           [&_name](const ::sdf::Link *_link)
                           { return _link->Name() == _name; });
    if (it == links.end())
    {
      return {};
    }
    return std::distance(links.begin(), it);
  }

  bool AddMesh(mjSpec *_spec, mjsBody *_body, const ::sdf::Mesh *_meshSdf)
  {
    auto &meshManager = *gz::common::MeshManager::Instance();
    auto *mesh = meshManager.Load(_meshSdf->Uri());
    if (nullptr == mesh)
    {
      gzwarn << "Failed to load mesh from [" << _meshSdf->Uri() << "]."
             << std::endl;
      return false;
    }

    auto geom = mjs_addGeom(_body, nullptr);
    geom->type = mjGEOM_MESH;
    auto meshName = mesh->Name();
    mjs_setString(geom->meshname, meshName.c_str());

    auto *muMesh = mjs_addMesh(_spec, nullptr);
    mjs_setName(muMesh->element, meshName.c_str());

    std::copy(_meshSdf->Scale().Data(), _meshSdf->Scale().Data() + 3,
              muMesh->scale);
    double *verts{nullptr};
    int *indices{nullptr};

    mesh->FillArrays(&verts, &indices);
    auto nverts = mesh->VertexCount();
    muMesh->uservert->assign(3 * nverts, 0.0);
    for (int i = 0; i < nverts / 3; ++i)
    {
      std::cout << verts[3 * i] << " " << verts[3 * i + 1] << " "
                << verts[3 * i + 2] << std::endl;
    }
    std::copy(verts, verts + 3 * nverts, muMesh->uservert->begin());

    mjs_setInt(muMesh->userface, indices, mesh->IndexCount());

    delete[] verts;
    delete[] indices;
    return true;
  }

  void AddToSpec(Base &_base, const ::sdf::Model &_sdfModel, mjSpec *_spec,
                 std::size_t _index,
                 const std::shared_ptr<ModelInfo> &_modelInfo)
  {
    mjsBody *parent;
    auto *world = mjs_findBody(_spec, "world");
    const auto *parentLink = this->parents[_index];
    // Find parent
    if (!this->parents[_index])
    {
      parent = world;
    }
    else
    {
      parent = mjs_findChild(world, parentLink->Name().c_str());
    }

    if (!parent)
    {
      // TODO (azeey) Better error message
      gzerr << "Error finding parent\n";
      return;
    }

    auto worldInfo = _modelInfo->worldInfo.lock();
    if (!worldInfo)
    {
      // TODO (azeey) Better error message
      gzerr << "Error getting worldInfo\n";
      return;
    }

    const auto *link = this->links[_index];
    auto child = mjs_addBody(parent, nullptr);
    const std::string body_name =
        ::sdf::JoinName(_modelInfo->name, link->Name());
    mjs_setName(child->element, body_name.c_str());
    auto linkInfo =
        std::make_shared<LinkInfo>(_base.GetNextEntity(), _modelInfo);
    linkInfo->body = child;
    linkInfo->name = link->Name();
    _modelInfo->links.push_back(linkInfo);
    // TODO(azeey) This will end up assigning the first root level link as the
    // body associated with the model. We should probably consider using the
    // canonical link here instead.
    if (!_modelInfo->body)
    {
      _modelInfo->body = child;
      // TODO(azeey): Resolve link poses
      const auto &pose = _sdfModel.RawPose() * link->RawPose();
      gzdbg << "--- Pose: " << pose << "\n";
      child->pos[0] = pose.X();
      child->pos[1] = pose.Y();
      child->pos[2] = pose.Z();

      child->quat[0] = pose.Rot().W();
      child->quat[1] = pose.Rot().X();
      child->quat[2] = pose.Rot().Y();
      child->quat[3] = pose.Rot().Z();
    }

    child->explicitinertial = true;
    const auto &massM = link->Inertial().MassMatrix();
    const math::Matrix3d &moi = link->Inertial().Moi();
    const math::Vector3d &com = link->Inertial().Pose().Pos();
    child->mass = massM.Mass();
    child->fullinertia[0] = moi(0, 0);
    child->fullinertia[1] = moi(1, 1);
    child->fullinertia[2] = moi(2, 2);
    child->fullinertia[3] = moi(0, 1);
    child->fullinertia[4] = moi(0, 2);
    child->fullinertia[5] = moi(1, 2);
    auto inertialPose = link->Inertial().Pose();
    child->ipos[0] = inertialPose.X();
    child->ipos[1] = inertialPose.Y();
    child->ipos[2] = inertialPose.Z();

    child->iquat[0] = inertialPose.Rot().W();
    child->iquat[1] = inertialPose.Rot().X();
    child->iquat[2] = inertialPose.Rot().Y();
    child->iquat[3] = inertialPose.Rot().Z();

    // TODO(azeey): Apply link poses
    if (_modelInfo->body != child)
    {
      const auto &pose = link->RawPose();
      child->pos[0] = pose.X();
      child->pos[1] = pose.Y();
      child->pos[2] = pose.Z();

      child->quat[0] = pose.Rot().W();
      child->quat[1] = pose.Rot().X();
      child->quat[2] = pose.Rot().Y();
      child->quat[3] = pose.Rot().Z();
    }
    // TODO(azeey) Apply pose of inertia frame.

    // Parse collisions
    std::size_t meshCounter = 0;
    for (std::size_t i = 0; i < link->CollisionCount(); ++i)
    {
      const auto *collision = link->CollisionByIndex(i);
      auto *shape = collision->Geom();
      mjsGeom *geom = nullptr;
      // TODO(azeey): Apply pose of collision

      switch (shape->Type())
      {
        case ::sdf::GeometryType::BOX:
        {
          geom = mjs_addGeom(child, nullptr);
          geom->type = mjGEOM_BOX;
          for (int j = 0; j < 3; ++j)
          {
            geom->size[j] = shape->BoxShape()->Size()[j] / 2.0;
          }
          break;
        }
        case ::sdf::GeometryType::CONE:
        {
          geom = mjs_addGeom(child, nullptr);
          geom->type = mjGEOM_MESH;
          const std::string meshName =
              collision->Name() + "_cone_" + std::to_string(meshCounter++);
          auto *muMesh = mjs_addMesh(_spec, nullptr);
          mjs_setName(muMesh->element, meshName.c_str());
          mjs_setString(geom->meshname, meshName.c_str());
          muMesh->scale[0] = shape->ConeShape()->Radius();
          muMesh->scale[1] = shape->ConeShape()->Radius();
          muMesh->scale[2] = shape->ConeShape()->Length() / 2.0;
          // 36 is the number of segments in the DART plugin implementation
          double params[3] = {36, 0};
          mjs_makeMesh(muMesh, mjMESH_BUILTIN_CONE, params, 2);
          break;
        }
        case ::sdf::GeometryType::CYLINDER:
        {
          geom = mjs_addGeom(child, nullptr);
          geom->type = mjGEOM_CYLINDER;
          geom->size[0] = shape->CylinderShape()->Radius();
          geom->size[1] = shape->CylinderShape()->Length() / 2.0;
          break;
        }
        case ::sdf::GeometryType::PLANE:
        {
          geom = mjs_addGeom(child, nullptr);
          // Set mass to 0 to mark the body as static
          geom->type = mjGEOM_PLANE;
          for (int j = 0; j < 2; ++j)
          {
            geom->size[j] = shape->PlaneShape()->Size()[j] / 2.0;
          }
          geom->size[2] = 1.0;
          break;
        }
        case ::sdf::GeometryType::SPHERE:
        {
          geom = mjs_addGeom(child, nullptr);
          geom->type = mjGEOM_SPHERE;
          geom->size[0] = shape->SphereShape()->Radius();
          break;
        }
        case ::sdf::GeometryType::CAPSULE:
        {
          geom = mjs_addGeom(child, nullptr);
          geom->type = mjGEOM_CAPSULE;
          geom->size[0] = shape->CapsuleShape()->Radius();
          geom->size[1] = shape->CapsuleShape()->Length() / 2.0;
          break;
        }
        case ::sdf::GeometryType::ELLIPSOID:
        {
          geom = mjs_addGeom(child, nullptr);
          geom->type = mjGEOM_ELLIPSOID;
          for (int j = 0; j < 3; ++j)
          {
            geom->size[j] = shape->EllipsoidShape()->Radii()[j];
          }
          break;
        }
        case ::sdf::GeometryType::MESH:
        {
          this->AddMesh(worldInfo->mjSpecObj, child, shape->MeshShape());
          break;
        }
        case ::sdf::GeometryType::HEIGHTMAP:
        case ::sdf::GeometryType::POLYLINE:
        default:
          gzwarn << "Shape type " << static_cast<int>(shape->Type())
                 << " not supported\n";
          continue;
      }
      if (geom)
      {
        geom->contype = 1;
        geom->conaffinity = 1;
        mjs_setName(geom->element,
                    ::sdf::JoinName(body_name, collision->Name()).c_str());
        auto shapeInfo =
            std::make_shared<ShapeInfo>(_base.GetNextEntity(), linkInfo);
        shapeInfo->geom = geom;
        shapeInfo->name = collision->Name();
        linkInfo->shapes.push_back(shapeInfo);
      }
    }
    if (!this->childInJoint[_index] && !_sdfModel.Static())
    {
      // No joint has this link as a child, so we add a freejoint.
      mjs_addFreeJoint(child);
    }

    // Recursively add children
    for (std::size_t i = 0; i < this->children[_index].size(); ++i)
    {
      this->AddToSpec(_base, _sdfModel, _spec, this->children[_index][i],
                      _modelInfo);
    }
  }
};

/////////////////////////////////////////////////
Identity SDFFeatures::ConstructSdfModelImpl(Identity _parentID,
                                            const ::sdf::Model &_sdfModel)
{
  auto start = std::chrono::high_resolution_clock::now();
  auto *worldInfo = this->ReferenceInterface<WorldInfo>(_parentID);
  if (!worldInfo)
  {
    gzerr << "Parent of model is not a world\n";
    return this->GenerateInvalidId();
  }

  auto *spec = worldInfo->mjSpecObj;
  worldInfo->specDirety = true;
  ModelKinematicStructure kinTree;
  kinTree.links.reserve(_sdfModel.LinkCount());
  for (std::size_t i = 0; i < _sdfModel.LinkCount(); ++i)
  {
    kinTree.links.push_back(_sdfModel.LinkByIndex(i));
  }
  kinTree.parents.resize(_sdfModel.LinkCount(), nullptr);
  kinTree.parentInJoint.resize(_sdfModel.LinkCount(), nullptr);
  kinTree.childInJoint.resize(_sdfModel.LinkCount(), nullptr);
  kinTree.children.resize(_sdfModel.LinkCount(), {});

  // Now go through the joints and update parent and children
  for (std::size_t i = 0; i < _sdfModel.JointCount(); ++i)
  {
    const auto *joint = _sdfModel.JointByIndex(i);
    std::string parentLinkName;
    // TODO(azeey) Handle errors
    joint->ResolveParentLink(parentLinkName);
    if (parentLinkName == "world")
    {
      continue;
    }
    auto parentIndex = kinTree.FindLinkByName(parentLinkName);
    if (!parentIndex)
    {
      gzerr << "Error finding link " << parentLinkName << "\n";
      return this->GenerateInvalidId();
    }

    std::string childLinkName;
    // TODO(azeey) Handle errors
    joint->ResolveChildLink(childLinkName);
    auto childIndex = kinTree.FindLinkByName(childLinkName);
    if (!childIndex)
    {
      gzerr << "Error finding link " << childLinkName << "\n";
      return this->GenerateInvalidId();
    }

    kinTree.parents[*childIndex] = kinTree.links[*parentIndex];
    kinTree.children[*parentIndex].push_back(*childIndex);
    kinTree.parentInJoint[*parentIndex] = joint;
    kinTree.childInJoint[*childIndex] = joint;
  }

  auto modelInfo = std::make_shared<ModelInfo>(
      this->GetNextEntity(),
      std::reinterpret_pointer_cast<WorldInfo>(this->Reference(_parentID)));
  modelInfo->entityId = this->GetNextEntity();
  modelInfo->name = _sdfModel.Name();
  // TODO(azeey) Change this when we support nested models.
  modelInfo->parentBody = mjs_findBody(spec, "world");
  worldInfo->models.push_back(modelInfo);
  for (std::size_t i = 0; i < kinTree.parents.size(); ++i)
  {
    if (!kinTree.parents[i])
    {
      kinTree.AddToSpec(*this, _sdfModel, spec, i, modelInfo);
    }
  }
  if (!modelInfo->body)
  {
    gzerr << "There was no body associated with the model\n";
    return this->GenerateInvalidId();
  }

  auto end = std::chrono::high_resolution_clock::now();
  std::cout << "Model: " << _sdfModel.Name() << " constructed in "
            << std::chrono::duration<double>(end - start).count() << "\n";
  return this->GenerateIdentity(modelInfo->entityId, modelInfo);
}

/////////////////////////////////////////////////
Identity SDFFeatures::ConstructSdfWorld(const Identity &_engine,
                                        const ::sdf::World &_sdfWorld)
{
  const Identity worldID = this->ConstructEmptyWorld(_engine, _sdfWorld.Name());

  for (std::size_t i = 0; i < _sdfWorld.ModelCount(); ++i)
  {
    const ::sdf::Model *model = _sdfWorld.ModelByIndex(i);

    if (!model)
      continue;
    this->ConstructSdfModel(worldID, *model);
  }

  return worldID;
}

}  // namespace mujoco
}  // namespace physics
}  // namespace gz
