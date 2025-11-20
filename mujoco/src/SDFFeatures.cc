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
#include <cstddef>
#include <gz/common/Console.hh>
#include <gz/physics/Entity.hh>
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

#include "Base.hh"
#include "gz/physics/detail/Identity.hh"

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

  void AddToSpec(mjSpec *_spec, std::size_t _index, ModelInfo &_modelInfo)
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

    const auto *link = this->links[_index];
    auto child = mjs_addBody(parent, nullptr);
    mjs_setName(child->element, ::sdf::JoinName(_modelInfo.name, link->Name()).c_str());
    auto linkInfo = std::make_shared<LinkInfo>();
    linkInfo->body = child;
    _modelInfo.links.push_back(linkInfo);
    // TODO(azeey) This will end up assigning the first root level link as the
    // body associated with the model. We should probably consider using the
    // canonical link here instead.
    if (!_modelInfo.body) {
      _modelInfo.body = child;
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
    // TODO(azeey) Apply pose of inertia frame.

    // Parse collisions
    for (std::size_t i = 0; i < link->CollisionCount(); ++i)
    {
      const auto *collision = link->CollisionByIndex(i);
      auto *shape = collision->Geom();
      mjsGeom *geom = nullptr;

      switch (shape->Type())
      {
        case ::sdf::GeometryType::BOX:
          geom = mjs_addGeom(child, nullptr);
          geom->type = mjGEOM_BOX;
          for (int j = 0; j < 3; ++j)
          {
            geom->size[j] = shape->BoxShape()->Size()[j] / 2.0;
          }
          break;
        case ::sdf::GeometryType::CYLINDER:
          geom = mjs_addGeom(child, nullptr);
          geom->type = mjGEOM_CYLINDER;
          geom->size[0] = shape->CylinderShape()->Radius();
          geom->size[1] = shape->CylinderShape()->Length() / 2.0;
          break;
        case ::sdf::GeometryType::PLANE:
          geom = mjs_addGeom(child, nullptr);
          geom->type = mjGEOM_PLANE;
          for (int j = 0; j < 2; ++j)
          {
            geom->size[j] = shape->PlaneShape()->Size()[j] / 2.0;
          }
          geom->size[2] = 1.0;
          break;
        case ::sdf::GeometryType::SPHERE:
          geom = mjs_addGeom(child, nullptr);
          geom->type = mjGEOM_SPHERE;
          geom->size[0] = shape->SphereShape()->Radius();
          break;
        case ::sdf::GeometryType::CAPSULE:
          geom = mjs_addGeom(child, nullptr);
          geom->type = mjGEOM_CAPSULE;
          geom->size[0] = shape->CapsuleShape()->Radius();
          geom->size[1] = shape->CapsuleShape()->Length() / 2.0;
          break;
        case ::sdf::GeometryType::ELLIPSOID:
          geom = mjs_addGeom(child, nullptr);
          geom->type = mjGEOM_ELLIPSOID;
          for (int j = 0; j < 3; ++j)
          {
            geom->size[j] = shape->EllipsoidShape()->Radii()[j];
          }
          break;
        case ::sdf::GeometryType::HEIGHTMAP:
        case ::sdf::GeometryType::MESH:
        case ::sdf::GeometryType::POLYLINE:
        case ::sdf::GeometryType::CONE:
        default:
          gzwarn << "Shape type " << static_cast<int>(shape->Type())
                 << " not supported\n";
          continue;
      }
      if (geom)
      {
        geom->contype = 1;
        geom->conaffinity = 1;
        mjs_setName(geom->element, collision->Name().c_str());
      }
    }
    if (!this->childInJoint[_index])
    {
      // No joint has this link as a child, so we add a freejoint.
      mjs_addFreeJoint(child);
    }

    // Recursively add children
    for (std::size_t i = 0; i < this->children[_index].size(); ++i)
    {
      this->AddToSpec(_spec, this->children[_index][i], _modelInfo);
    }
  }
};

/////////////////////////////////////////////////
Identity SDFFeatures::ConstructSdfModelImpl(Identity _parentID,
                                            const ::sdf::Model &_sdfModel)
{
  auto *worldInfo = this->ReferenceInterface<WorldInfo>(_parentID);
  if (!worldInfo)
  {
    gzerr << "Parent of model is not a world\n";
    return this->GenerateInvalidId();
  }

  auto *spec = worldInfo->mjSpecObj;
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

  auto modelInfo = std::make_shared<ModelInfo>();
  modelInfo->name = _sdfModel.Name();
  // TODO(azeey) Change this when we support nested models.
  modelInfo->parentBody = mjs_findBody(spec, "world");
  worldInfo->models.push_back(modelInfo);
  for (std::size_t i = 0; i < kinTree.parents.size(); ++i)
  {
    if (!kinTree.parents[i])
    {
      kinTree.AddToSpec(spec, i, *modelInfo);
    }
  }
  int rc =
      mj_recompile(spec, nullptr, worldInfo->mjModelObj, worldInfo->mjDataObj);
  if (rc != 0)
  {
    gzerr << "Error compiling:" << mjs_getError(spec) << "\n";
  }

  mj_saveXML(spec, "/tmp/mujoco_model.xml", nullptr, 0);
  if (!modelInfo->body) {
    gzerr << "There was no body associated with the model\n";
    return this->GenerateInvalidId();
  }

  auto modelID = static_cast<std::size_t>(mjs_getId(modelInfo->body->element));
  return this->GenerateIdentity(modelID, modelInfo);
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
