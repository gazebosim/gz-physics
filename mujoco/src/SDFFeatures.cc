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
#include <mujoco/mujoco.h>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <iostream>
#include <iterator>
#include <limits>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/common/Mesh.hh>
#include <gz/common/MeshManager.hh>
#include <gz/common/SubMesh.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/physics/Entity.hh>
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

namespace {


/////////////////////////////////////////////////
/// \brief Resolve the pose of an SDF DOM object with respect to its relative_to
/// frame. If that fails, return the raw pose
math::Pose3d resolveSdfPose(const ::sdf::SemanticPose &_semPose,
                            const std::string &_resolveTo = "")
{
  math::Pose3d pose;
  ::sdf::Errors errors = _semPose.Resolve(pose, _resolveTo);
  if (!errors.empty())
  {
    if (!_semPose.RelativeTo().empty())
    {
      gzerr << "There was an error in SemanticPose::Resolve\n";
      for (const auto &err : errors)
      {
        gzerr << err.Message() << std::endl;
      }
      gzerr << "There is no optimal fallback since the relative_to attribute["
             << _semPose.RelativeTo() << "] of the pose is not empty. "
             << "Falling back to using the raw Pose.\n";
    }
    pose = _semPose.RawPose();
  }

  return pose;
}
/////////////////////////////////////////////////
void convertJointAxis(const ::sdf::JointAxis *_sdfAxis, double *axis)
{
  math::Vector3d resolvedAxis;
  ::sdf::Errors errors = _sdfAxis->ResolveXyz(resolvedAxis);
  if (!errors.empty())
  {
    gzerr << "There was an error in JointAxis::ResolveXyz\n";
    gzerr << errors << std::endl;
    return;
  }
  std::copy(resolvedAxis.Data(), resolvedAxis.Data() + 3, axis);
}

/////////////////////////////////////////////////
double infIfNeg(const double _value)
{
  if (_value < 0.0)
    return std::numeric_limits<double>::infinity();

  return _value;
}

/////////////////////////////////////////////////
void copyStandardJointAxisProperties(
    mjsJoint * _joint,
    const ::sdf::JointAxis *_sdfAxis)
{
  _joint->damping = _sdfAxis->Damping();
  _joint->frictionloss = _sdfAxis->Friction();
  _joint->springref = _sdfAxis->SpringReference();
  _joint->stiffness = _sdfAxis->SpringStiffness();
  _joint->limited = static_cast<int>(!std::isinf(infIfNeg(_sdfAxis->Lower())) &&
                                     !std::isinf(infIfNeg(_sdfAxis->Upper())));
  if (_joint->limited)
  {
    _joint->range[0] = infIfNeg(_sdfAxis->Lower());
    _joint->range[1] = infIfNeg(_sdfAxis->Upper());
  }

  _joint->actfrclimited =
      static_cast<int>(!std::isinf(infIfNeg(_sdfAxis->Effort())));
  if (_joint->actfrclimited)
  {
    _joint->actfrcrange[0] = -infIfNeg(_sdfAxis->Effort());
    _joint->actfrcrange[1] = infIfNeg(_sdfAxis->Effort());
  }

  // TODO(azeey) Investigate whether velocity limits can be supported
  if (!std::isinf(_sdfAxis->MaxVelocity()))
  {
    gzerr << "The MuJoCo physics engine plugin does not support velocity "
             "limits\n";
  }
}
/////////////////////////////////////////////////
struct ModelKinematicStructure
{
  std::string name;

  std::vector<const ::sdf::Link *> links;
  // For index i, parents[i]  is the parent link of link[i]
  std::vector<const ::sdf::Link *> parents;
  // For index i, children[i] contains the list of children of link[i]
  std::vector<std::vector<std::size_t>> children;
  // For index i, childInJoint[i]->child = links[i], unless that link is not
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

  void PrintGraph()
  {
    std::cout << "digraph << " << name << " {\n";
    for (auto i = 0; i < links.size(); ++i)
    {
      if (!parents[i])
      {
        std::cout << "world";
      }
      else
      {
        std::cout << parents[i]->Name();
      }
      if (childInJoint[i])
      {
        std::cout << " -> " <<  childInJoint[i]->Name();
      }
      std::cout << " -> " <<  links[i]->Name() << "\n";
    }
    std::cout << "}\n";
  }

  mjsGeom * AddMesh(mjSpec *_spec, mjsBody *_body, const ::sdf::Mesh *_meshSdf)
  {
    auto &meshManager = *gz::common::MeshManager::Instance();
    auto *mesh = meshManager.Load(_meshSdf->Uri());
    if (nullptr == mesh)
    {
      gzwarn << "Failed to load mesh from [" << _meshSdf->Uri() << "]."
             << std::endl;
      return nullptr;
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
    std::transform(verts, verts + 3 * nverts, muMesh->uservert->begin(),
        [](double val) {return static_cast<float>(val);});

    mjs_setInt(muMesh->userface, indices, mesh->IndexCount());

    delete[] verts;
    delete[] indices;
    return geom;
  }

  void AddToSpec(Base &_base, const ::sdf::Model &_sdfModel, mjSpec *_spec,
                 std::size_t _index,
                 const std::shared_ptr<ModelInfo> &_modelInfo,
                 mjsBody *_parentBody)
  {
    auto worldInfo = _modelInfo->worldInfo;

    const auto *link = this->links[_index];
    auto child = mjs_addBody(_parentBody, nullptr);
    const std::string body_name =
        ::sdf::JoinName(_modelInfo->name, link->Name());
    mjs_setName(child->element, body_name.c_str());
    auto linkInfo =
        std::make_shared<LinkInfo>(_base.GetNextEntity(), _modelInfo);
    linkInfo->body = child;
    linkInfo->name = link->Name();
    linkInfo->modelInfo = _modelInfo;
    linkInfo->worldInfo = worldInfo;

    auto childSite = mjs_addSite(child, nullptr);
    _base.frames[linkInfo->entityId] =
        std::make_shared<FrameInfo>(childSite, worldInfo);

    _modelInfo->links.AddEntity(linkInfo->entityId, linkInfo, child,
                                _modelInfo->entityId);
    // TODO(azeey) This will end up assigning the first root level link as the
    // body associated with the model. We should probably consider using the
    // canonical link here instead.
    if (!_modelInfo->body)
    {
      _modelInfo->body = child;
      // TODO(azeey): Resolve link poses
      const auto &pose = _sdfModel.RawPose() * link->RawPose();
      // gzdbg << "--- Pose: " << pose << "\n";
      child->pos[0] = pose.X();
      child->pos[1] = pose.Y();
      child->pos[2] = pose.Z();

      child->quat[0] = pose.Rot().W();
      child->quat[1] = pose.Rot().X();
      child->quat[2] = pose.Rot().Y();
      child->quat[3] = pose.Rot().Z();

      _base.frames[_modelInfo->entityId] =
          std::make_shared<FrameInfo>(childSite, worldInfo);
    }

    child->explicitinertial = true;
    const auto &massM = link->Inertial().MassMatrix();
    const math::Matrix3d &moi = link->Inertial().Moi();
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

    if (_modelInfo->body != child)
    {
      std::string resolveTo = "";
      if (this->parents[_index])
      {
        resolveTo = this->parents[_index]->Name();
      }
      const auto pose = resolveSdfPose(link->SemanticPose(), resolveTo);
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
          geom = this->AddMesh(worldInfo->mjSpecObj, child, shape->MeshShape());
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
        auto pose = resolveSdfPose(collision->SemanticPose());
        std::copy(pose.Pos().Data(), pose.Pos().Data()+3, geom->pos);
        geom->quat[0] = pose.Rot().W();
        geom->quat[1] = pose.Rot().X();
        geom->quat[2] = pose.Rot().Y();
        geom->quat[3] = pose.Rot().Z();
        shapeInfo->geom = geom;
        shapeInfo->name = collision->Name();
        linkInfo->shapes.AddEntity(shapeInfo->entityId, shapeInfo, geom,
                                   linkInfo->entityId);
      }
    }

    // Add joints
    if (!_sdfModel.Static())
    {
      const auto *sdfJoint = childInJoint[_index];
      if (!sdfJoint)
      {
        // No joint has this link as a child, so we add a freejoint.
        mjs_addFreeJoint(child);
      }
      else
      {
        mjsJoint * joint{nullptr};
        if (sdfJoint->Type() == ::sdf::JointType::REVOLUTE)
        {
          joint = mjs_addJoint(child, nullptr);
          joint->type = mjJNT_HINGE;
          const auto *sdfAxis = sdfJoint->Axis(0);
          convertJointAxis(sdfAxis, joint->axis);
          copyStandardJointAxisProperties(joint, sdfAxis);
        }
        else if (sdfJoint->Type() != ::sdf::JointType::FIXED)
        {
          gzwarn << "Joint type " << static_cast<int>(sdfJoint->Type())
                 << " in joint [" << sdfJoint->Name() << "] not supported\n";
          return;
        }

        // Note that no joints will be created when processing a fixed joint.
        if (joint)
        {
          const std::string mjJointName =
            ::sdf::JoinName(_modelInfo->name, sdfJoint->Name());
          mjs_setName(joint->element, mjJointName.c_str());

          // Resolve the position of the joint relative to the body with which
          // it's associated. Note that this body is the child link of the joint
          // in SDF terms.
          auto anchor = resolveSdfPose(sdfJoint->SemanticPose()).Pos();
          std::copy(anchor.Data(), anchor.Data() + 3, joint->pos);
        }
        auto jointInfo =
          std::make_shared<JointInfo>(_base.GetNextEntity(), _modelInfo);
        jointInfo->name = sdfJoint->Name();
        jointInfo->joint = joint;
        jointInfo->worldInfo = worldInfo;

        _modelInfo->joints.AddEntity(jointInfo->entityId, jointInfo,
                                     jointInfo->name, _modelInfo->entityId);
      }
    }


    // Recursively add children
    for (std::size_t i = 0; i < this->children[_index].size(); ++i)
    {
      this->AddToSpec(_base, _sdfModel, _spec, this->children[_index][i],
                      _modelInfo, child);
    }
  }
};

}
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
  worldInfo->specDirty = true;
  ModelKinematicStructure kinTree;
  kinTree.name = _sdfModel.Name();
  kinTree.links.reserve(_sdfModel.LinkCount());
  for (std::size_t i = 0; i < _sdfModel.LinkCount(); ++i)
  {
    kinTree.links.push_back(_sdfModel.LinkByIndex(i));
  }
  kinTree.parents.resize(_sdfModel.LinkCount(), nullptr);
  kinTree.childInJoint.resize(_sdfModel.LinkCount(), nullptr);
  kinTree.children.resize(_sdfModel.LinkCount(), {});

  // Now go through the joints and update parent and children
  for (std::size_t i = 0; i < _sdfModel.JointCount(); ++i)
  {
    const auto *joint = _sdfModel.JointByIndex(i);
    std::string childLinkName;
    // TODO(azeey) Handle errors
    joint->ResolveChildLink(childLinkName);
    auto childIndex = kinTree.FindLinkByName(childLinkName);
    if (!childIndex)
    {
      gzerr << "Error finding link " << childLinkName << "\n";
      return this->GenerateInvalidId();
    }

    std::string parentLinkName;
    // TODO(azeey) Handle errors
    joint->ResolveParentLink(parentLinkName);
    if (parentLinkName == "world")
    {
      kinTree.childInJoint[*childIndex] = joint;
      continue;
    }
    auto parentIndex = kinTree.FindLinkByName(parentLinkName);
    if (!parentIndex)
    {
      gzerr << "Error finding link " << parentLinkName << "\n";
      return this->GenerateInvalidId();
    }

    kinTree.parents[*childIndex] = kinTree.links[*parentIndex];
    kinTree.children[*parentIndex].push_back(*childIndex);
    kinTree.childInJoint[*childIndex] = joint;
  }

  auto modelInfo = std::make_shared<ModelInfo>(
      this->GetNextEntity(), this->ReferenceInterface<WorldInfo>(_parentID));

  auto *worldBody = mjs_findBody(spec, "world");
  modelInfo->name = _sdfModel.Name();
  // TODO(azeey) Change this when we support nested models.
  modelInfo->parentBody = worldBody;
  worldInfo->models.AddEntity(modelInfo->entityId, modelInfo,
                              JoinNames(worldInfo->name, modelInfo->name),
                              worldInfo->entityId);

  kinTree.PrintGraph();

  for (std::size_t i = 0; i < kinTree.parents.size(); ++i)
  {
    if (!kinTree.parents[i])
    {
      kinTree.AddToSpec(*this, _sdfModel, spec, i, modelInfo, worldBody);
    }
  }
  if (!modelInfo->body)
  {
    gzerr << "There was no body associated with the model\n";
    return this->GenerateInvalidId();
  }


  if (!_sdfModel.SelfCollide())
  {
    // Mujoco requires explicitly declaring contact exclusions for body pairs
    // in order to implement self-collide. If self collision is disabled, we
    // need to add exclusions for every body pair in the model.
    // Bodies in a parent-child relationship are already excluded, however, if
    // the parent is attached to worldbody, then we need to explicitly add an
    // exclusion since the geoms of the parent are considered part of worldbody.
    // See https://mujoco.readthedocs.io/en/stable/computation/index.html#collision
    // To avoid the complexity of determining that, we just add exclusions for
    // every pair

    // Avoid duplicating exclusions by tracking history. The key of this set is
    // a pair where the first element is always less than the second.
    std::set<std::pair<std::size_t, std::size_t>> history;
    for (const auto [body1, id1] : modelInfo->links.objectToID)
    {
      for (const auto [body2, id2] : modelInfo->links.objectToID)
      {
        if (body1 == body2)
          continue;

        auto histKey = std::minmax(id1, id2);
        if (history.count(histKey) > 0)
          continue;

        mjsExclude *exclude = mjs_addExclude(worldInfo->mjSpecObj);
        mjs_setString(exclude->bodyname1,
                      mjs_getString(mjs_getName(body1->element)));
        mjs_setString(exclude->bodyname2,
                      mjs_getString(mjs_getName(body2->element)));
        history.insert(histKey);
      }
    }
  }
  auto end = std::chrono::high_resolution_clock::now();
  gztrace << "Model: " << _sdfModel.Name() << " constructed in "
            << std::chrono::duration<double>(end - start).count() << "\n";
  return this->GenerateIdentity(modelInfo->entityId, modelInfo);
}

/////////////////////////////////////////////////
Identity SDFFeatures::ConstructSdfWorld(const Identity &_engine,
                                        const ::sdf::World &_sdfWorld)
{
  const Identity worldID =
      this->ConstructEmptyWorld(_engine, _sdfWorld.Name());

  // Set gravity to value from SDF.
  // Note: there is a small difference between the default gravity
  // z magnitude in mujoco (-9.81) vs the  default value in SDF (-9.8).
  auto *worldInfo = this->ReferenceInterface<WorldInfo>(worldID);
  if (worldInfo && worldInfo->mjModelObj)
  {
    // Update both the model and the spec so that gravity persists across
    // spec recompilation.
    const auto &gravity = _sdfWorld.Gravity();
    worldInfo->mjModelObj->opt.gravity[0] = gravity[0];
    worldInfo->mjModelObj->opt.gravity[1] = gravity[1];
    worldInfo->mjModelObj->opt.gravity[2] = gravity[2];
    worldInfo->mjSpecObj->option.gravity[0] = gravity[0];
    worldInfo->mjSpecObj->option.gravity[1] = gravity[1];
    worldInfo->mjSpecObj->option.gravity[2] = gravity[2];
  }

  for (std::size_t i = 0; i < _sdfWorld.ModelCount(); ++i)
  {
    const ::sdf::Model *model = _sdfWorld.ModelByIndex(i);

    if (!model)
      continue;
    this->ConstructSdfModel(worldID, *model);
  }

  this->RecompileSpec(*worldInfo);

  return worldID;
}

}  // namespace mujoco
}  // namespace physics
}  // namespace gz
