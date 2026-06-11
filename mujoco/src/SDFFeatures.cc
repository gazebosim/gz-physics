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
  _joint->limited = static_cast<int>(!std::isinf(_sdfAxis->Lower()) &&
                                     !std::isinf(_sdfAxis->Upper()));
  _joint->range[0] = _sdfAxis->Lower();
  _joint->range[1] = _sdfAxis->Upper();

  _joint->actfrclimited =
      static_cast<int>(!std::isinf(infIfNeg(_sdfAxis->Effort())));

  _joint->actfrcrange[0] = -infIfNeg(_sdfAxis->Effort());
  _joint->actfrcrange[1] = infIfNeg(_sdfAxis->Effort());

  // TODO(azeey) MuJoCo does not natively support velocity limits.
  // See https://github.com/google-deepmind/mujoco/discussions/2367
  if (!std::isinf(_sdfAxis->MaxVelocity()))
  {
    gzwarn << "The MuJoCo physics engine plugin does not support velocity "
             "limits\n";
  }
}
/////////////////////////////////////////////////
/// \brief Convert SDFormat screw thread pitch into MuJoCo coordinate ratio.
/// \param[in] _pitch Screw thread pitch defined in meters per revolution
/// (m/rev).
/// \return Pitch coordinate ratio in meters per radian (m/rad).
double convertScrewThreadPitch(const double _pitch)
{
  return _pitch / (2.0 * GZ_PI);
}
/////////////////////////////////////////////////
struct ModelKinematicStructure
{
  std::string name;

  std::vector<const ::sdf::Link *> links;
  std::vector<std::shared_ptr<ModelInfo>> modelInfos;
  std::vector<const ::sdf::Model *> sdfModels;
  // For index i, parents[i] is the parent link of link[i]
  std::vector<const ::sdf::Link *> parents;
  // For index i, children[i] contains the list of children of link[i]
  std::vector<std::vector<std::size_t>> children;
  // For index i, childInJoint[i]->child = links[i], unless that link is not
  // referenced by any joint as a child.
  std::vector<const ::sdf::Joint *> childInJoint;
  // For index i, jointModelInfos[i] is the ModelInfo of the model that
  // defines childInJoint[i]
  std::vector<std::shared_ptr<ModelInfo>> jointModelInfos;

  std::optional<std::size_t> FindLinkByName(
      const std::string &_name, const ::sdf::Model *_model)
  {
    auto link = _model->LinkByName(_name);
    if (!link)
      return {};
    auto it = std::find(links.begin(), links.end(), link);
    if (it == links.end())
    {
      return {};
    }
    return std::distance(links.begin(), it);
  }

  void PrintGraph()
  {
    std::cout << "digraph << " << name << " {\n";
    for (std::size_t i = 0; i < links.size(); ++i)
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
    if (!geom)
    {
      return nullptr;
    }
    geom->type = mjGEOM_MESH;
    auto meshName = mesh->Name();
    mjs_setString(geom->meshname, meshName.c_str());

    auto *muMesh = mjs_addMesh(_spec, nullptr);
    if (!muMesh)
    {
      gzerr << "Failed to add mesh spec for " << meshName << std::endl;
      return nullptr;
    }

    if (!muMesh->uservert || !muMesh->userface)
    {
      gzerr << "muMesh uservert or userface array is null for "
            << meshName << std::endl;
      return nullptr;
    }

    mjs_setName(muMesh->element, meshName.c_str());

    muMesh->scale[0] = _meshSdf->Scale().X();
    muMesh->scale[1] = _meshSdf->Scale().Y();
    muMesh->scale[2] = _meshSdf->Scale().Z();
    double *verts{nullptr};
    int *indices{nullptr};

    mesh->FillArrays(&verts, &indices);
    auto nverts = mesh->VertexCount();

    if (nverts > 0 && nullptr == verts)
    {
      gzerr << "Mesh [" << _meshSdf->Uri() << "] has " << nverts
            << " vertices but FillArrays returned null vertices pointer."
            << std::endl;
      delete[] indices;
      return nullptr;
    }

    if (mesh->IndexCount() > 0 && nullptr == indices)
    {
      gzerr << "Mesh [" << _meshSdf->Uri() << "] has " << mesh->IndexCount()
            << " indices but FillArrays returned null indices pointer."
            << std::endl;
      delete[] verts;
      return nullptr;
    }

    muMesh->uservert->assign(3 * nverts, 0.0);
    std::transform(verts, verts + 3 * nverts, muMesh->uservert->begin(),
        [](double val) {return static_cast<float>(val);});

    mjs_setInt(muMesh->userface, indices, mesh->IndexCount());

    delete[] verts;
    delete[] indices;
    return geom;
  }

  void AddJoint(Base &_base, mjSpec *_spec, const ::sdf::Joint *sdfJoint,
                mjsBody *child, const std::shared_ptr<ModelInfo> &_modelInfo)
  {
    auto worldInfo = _modelInfo->worldInfo;
    if (!sdfJoint)
    {
      // No joint has this link as a child, so we add a freejoint.
      // However, Mujoco only allows free joints on top-level bodies.
      if (_modelInfo->parentBody == mjs_findBody(_spec, "world"))
      {
        mjs_addFreeJoint(child);
      }
    }
    else
    {
      mjsJoint *joint{nullptr};
      mjsJoint *joint2{nullptr};
      // It is possible to apply joint forces using `qfrc_applied`, but this
      // makes it harder to retrieve the last applied forces on a joint when
      // implementing GetJoint. Instead, we use actuators and `mjData::ctrl`.
      // This allows us to use the same interface for setting velocity servo
      // commands as well.
      mjsActuator *actuator{nullptr};
      if (sdfJoint->Type() == ::sdf::JointType::PRISMATIC)
      {
        joint = mjs_addJoint(child, nullptr);
        joint->type = mjJNT_SLIDE;
        const auto *sdfAxis = sdfJoint->Axis(0);
        convertJointAxis(sdfAxis, joint->axis);
        copyStandardJointAxisProperties(joint, sdfAxis);
      }
      else if (sdfJoint->Type() == ::sdf::JointType::REVOLUTE)
      {
        joint = mjs_addJoint(child, nullptr);
        joint->type = mjJNT_HINGE;
        const auto *sdfAxis = sdfJoint->Axis(0);
        convertJointAxis(sdfAxis, joint->axis);
        copyStandardJointAxisProperties(joint, sdfAxis);
      }
      else if (sdfJoint->Type() == ::sdf::JointType::BALL)
      {
        joint = mjs_addJoint(child, nullptr);
        joint->type = mjJNT_BALL;
        const auto *sdfAxis = sdfJoint->Axis(0);
        if (sdfAxis)
        {
          convertJointAxis(sdfAxis, joint->axis);
          copyStandardJointAxisProperties(joint, sdfAxis);
          // For ball joints, the first range parameter should always be set to
          // zero.
          if (joint->limited && std::abs(joint->range[0]) > 0.0)
          {
            gzwarn << "MuJoCo requires the lower joint position limit of ball "
                      "joints to be zero.\n";
            joint->range[0] = 0;
          }
        }
      }
      else if (sdfJoint->Type() == ::sdf::JointType::SCREW)
      {
        // Screw joints in MuJoCo are modeled by coupling a hinge joint
        // (rotation) and a slide joint (translation) along the same axis on
        // the child body using a joint equality constraint (mjEQ_JOINT).
        // We store the hinge joint (`joint`) as the primary joint in
        // JointInfo. This matches DART's choice of using the rotational DOF
        // as the primary, ensuring both physics plugins expose consistent
        // angular units (radians and rad/s) for screw joints across the
        // gz-physics API.
        // Like the universal joint, `joint` and `joint2` are compiled
        // contiguously on the same child body.
        joint = mjs_addJoint(child, nullptr);
        joint->type = mjJNT_HINGE;
        const auto *sdfAxis1 = sdfJoint->Axis(0);
        if (sdfAxis1)
        {
          convertJointAxis(sdfAxis1, joint->axis);
          copyStandardJointAxisProperties(joint, sdfAxis1);
          // Disable independent position limits on the primary rotational
          // hinge joint. In MuJoCo's soft constraint solver, enforcing limits
          // on the hinge joint when coupled with a small thread pitch causes
          // premature solver clamping and massive numerical damping. By mapping
          // position limits purely onto the translational slide joint, we
          // ensure exact kinematic limit enforcement without solver resistance.
          joint->limited = false;
        }

        joint2 = mjs_addJoint(child, nullptr);
        joint2->type = mjJNT_SLIDE;
        if (sdfAxis1)
        {
          convertJointAxis(sdfAxis1, joint2->axis);
          // We only copy position limits and range to the secondary slide
          // joint. All passive dynamics (damping, frictionloss, stiffness) and
          // actuator effort limits are enforced purely on the primary
          // rotational hinge joint to avoid double-counting and physical unit
          // mismatches.
          joint2->limited = static_cast<int>(!std::isinf(sdfAxis1->Lower()) &&
                                             !std::isinf(sdfAxis1->Upper()));
          if (joint2->limited)
          {
            const double pitch =
                convertScrewThreadPitch(sdfJoint->ScrewThreadPitch());
            joint2->range[0] = sdfAxis1->Lower() * pitch;
            joint2->range[1] = sdfAxis1->Upper() * pitch;
          }
        }
      }
      else if (sdfJoint->Type() == ::sdf::JointType::UNIVERSAL)
      {
        // Universal joints in MuJoCo are modeled as two hinge joints in
        // series on the child body. We only need to keep a pointer to the
        // first hinge joint (`joint`) in JointInfo. Since MuJoCo compiles
        // joints on the same body contiguously in memory, all getters/setters
        // can safely access the second joint (`joint2`)'s state data using
        // `nq_index + 1` and `nv_index + 1` without needing to store it
        // separately in JointInfo.
        joint = mjs_addJoint(child, nullptr);
        joint->type = mjJNT_HINGE;
        const auto *sdfAxis1 = sdfJoint->Axis(0);
        if (sdfAxis1)
        {
          convertJointAxis(sdfAxis1, joint->axis);
          copyStandardJointAxisProperties(joint, sdfAxis1);
        }

        joint2 = mjs_addJoint(child, nullptr);
        joint2->type = mjJNT_HINGE;
        const auto *sdfAxis2 = sdfJoint->Axis(1);
        if (sdfAxis2)
        {
          convertJointAxis(sdfAxis2, joint2->axis);
          copyStandardJointAxisProperties(joint2, sdfAxis2);
        }
      }
      else if (sdfJoint->Type() != ::sdf::JointType::FIXED)
      {
        gzwarn << "Joint type " << static_cast<int>(sdfJoint->Type())
               << " in joint [" << sdfJoint->Name() << "] not supported\n";
        return;
      }

      // Resolve the pose of the joint relative to the body with which
      // it's associated. Note that this body is the child link of the joint
      // in SDF terms.
      auto jointPose = resolveSdfPose(sdfJoint->SemanticPose());
      mjsEquality *eq = nullptr;
      // Note that no joints will be created when processing a fixed joint.
      if (joint)
      {
        const std::string mjJointName =
            ::sdf::JoinName(_modelInfo->name, sdfJoint->Name());
        mjs_setName(joint->element, mjJointName.c_str());
        actuator = mjs_addActuator(_spec, nullptr);
        actuator->trntype = mjtTrn::mjTRN_JOINT;

        mjs_setString(actuator->target, mjJointName.c_str());

        copyPos(jointPose.Pos(), joint->pos);

        if (joint2)
        {
          // We uniquely name the second axis using getJointAxisName helper
          // with a flat suffix. This flat name avoids indicating any
          // Kinematic/SDF nesting (`::axis2`) while still satisfying
          // MuJoCo's requirement that joints must be uniquely named.
          const std::string mjJointName2 = getJointAxisName(mjJointName, 1);
          mjs_setName(joint2->element, mjJointName2.c_str());
          mjsActuator *actuator2 = mjs_addActuator(_spec, nullptr);
          actuator2->trntype = mjtTrn::mjTRN_JOINT;
          mjs_setString(actuator2->target, mjJointName2.c_str());

          copyPos(jointPose.Pos(), joint2->pos);

          // If this is a screw joint, couple the slide and hinge axes using
          // a joint equality constraint (mjEQ_JOINT) with the specified pitch.
          if (sdfJoint->Type() == ::sdf::JointType::SCREW)
          {
            eq = mjs_addEquality(_spec, nullptr);
            eq->type = mjEQ_JOINT;
            eq->active = 1;
            mjs_setString(eq->name1, mjJointName2.c_str());
            mjs_setString(eq->name2, mjJointName.c_str());

            // dif = pos[1] - ref[1] = hinge_pos - hinge_ref
            // cpos = pos[0] - ref[0] - data[0] - data[1]*dif = 0
            // enforces: slide_pos - slide_ref =
            // data[1] * (hinge_pos - hinge_ref)
            // where data[1] = pitch (meters/rad) = ScrewThreadPitch/2pi
            std::fill(std::begin(eq->data), std::end(eq->data), 0.0);
            eq->data[1] =
                convertScrewThreadPitch(sdfJoint->ScrewThreadPitch());
          }
        }
      }
      auto jointInfo =
          std::make_shared<JointInfo>(_base.GetNextEntity(), _modelInfo);
      jointInfo->name = sdfJoint->Name();
      jointInfo->joint = joint;
      jointInfo->childBody = child;
      jointInfo->actuator = actuator;
      jointInfo->worldInfo = worldInfo;
      if (sdfJoint->Type() == ::sdf::JointType::SCREW)
      {
        jointInfo->screwConstraintSpec = eq;
      }
      if (sdfJoint->Type() == ::sdf::JointType::BALL)
      {
        jointInfo->worldInfo->ballJointPositionsCache.push_back(std::nullopt);
        jointInfo->ballJointCacheIndex =
            jointInfo->worldInfo->ballJointPositionsCache.size() - 1;
      }

      auto jointSite = mjs_addSite(child, nullptr);
      copyPos(jointPose.Pos(), jointSite->pos);
      copyQuat(jointPose.Rot(), jointSite->quat);
      _base.frames[jointInfo->entityId] =
          std::make_shared<FrameInfo>(jointSite, worldInfo);

      _modelInfo->joints.AddEntity(jointInfo->entityId, jointInfo,
                                   jointInfo->name, _modelInfo->entityId);
    }
  }

std::string getRelativeScopedName(
    const std::string &modelName, const std::string &rootModelName)
{
  if (modelName == rootModelName)
    return "";
  if (modelName.rfind(rootModelName + "::", 0) == 0)
  {
    return modelName.substr(rootModelName.length() + 2);
  }
  return modelName;
}

  math::Pose3d resolvePoseInRoot(const std::string &_scopedName)
  {
    const auto *refLink = this->links[0];
    math::Pose3d refInRoot;
    refLink->SemanticPose().Resolve(refInRoot, "__model__");
    math::Pose3d refInLink;
    auto errors = refLink->SemanticPose().Resolve(refInLink, _scopedName);
    if (!errors.empty())
    {
      gzerr << "Failed to resolve " << refLink->Name() << " relative to "
            << _scopedName << ": " << errors << std::endl;
    }
    return refInRoot * refInLink.Inverse();
  }

  void AddToSpec(Base &_base, const ::sdf::Model &_rootModel, mjSpec *_spec,
                 std::size_t _index,
                 mjsBody *_parentBody)
  {
    auto _modelInfo = this->modelInfos[_index];
    auto _sdfModel = this->sdfModels[_index];
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
    // Determine the pose of this child body relative to its parent body in
    // MuJoCo.
    math::Pose3d pose;
    std::string childLinkScopedName = ::sdf::JoinName(
        getRelativeScopedName(_modelInfo->name, this->modelInfos[0]->name),
        link->Name());

    math::Pose3d childPoseInRoot = this->resolvePoseInRoot(childLinkScopedName);

    if (this->parents[_index])
    {
      // Parent body is another link body.
      std::shared_ptr<ModelInfo> parentModelInfo = nullptr;
      auto parentIt = std::find(
          this->links.begin(), this->links.end(), this->parents[_index]);
      if (parentIt != this->links.end())
      {
        std::size_t parentIdx = std::distance(this->links.begin(), parentIt);
        parentModelInfo = this->modelInfos[parentIdx];
      }

      if (parentModelInfo)
      {
        std::string parentLinkScopedName = ::sdf::JoinName(
            getRelativeScopedName(
                parentModelInfo->name, this->modelInfos[0]->name),
            this->parents[_index]->Name());

        math::Pose3d parentPoseInRoot =
            this->resolvePoseInRoot(parentLinkScopedName);
        pose = parentPoseInRoot.Inverse() * childPoseInRoot;
      }
      else
      {
        // Fallback
        pose = _sdfModel->RawPose() * link->RawPose();
      }
    }
    else
    {
      // Parent body is worldbody.
      // Resolve child link pose relative to world.
      pose = _rootModel.RawPose() * childPoseInRoot;
    }

    copyPos(pose.Pos(), child->pos);
    copyQuat(pose.Rot(), child->quat);

    if (!_modelInfo->body)
    {
      _modelInfo->body = child;

      auto modelFrameSite = mjs_addSite(child, nullptr);
      const auto modelFramePose = link->RawPose().Inverse();
      copyPos(modelFramePose.Pos(), modelFrameSite->pos);
      copyQuat(modelFramePose.Rot(), modelFrameSite->quat);
      _base.frames[_modelInfo->entityId] =
          std::make_shared<FrameInfo>(modelFrameSite, worldInfo);
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
    copyPos(inertialPose.Pos(), child->ipos);
    copyQuat(inertialPose.Rot(), child->iquat);

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
        uint16_t conaffinity = std::numeric_limits<uint16_t>::max();
        std::optional<uint16_t> contypeOpt;

        ::sdf::ElementPtr elem = collision->Element();
        if (auto surfaceElem = elem->FindElement("surface"))
        {
          if (auto contactElem = surfaceElem->FindElement("contact"))
          {
            if (auto categoryBitmaskElem =
                contactElem->FindElement("category_bitmask"))
            {
              // sdformat only supports uint32_t so cast back to uint16_t
              contypeOpt = static_cast<uint16_t>(
                  categoryBitmaskElem->Get<uint32_t>());
            }
            if (auto collideBitmaskElem =
                contactElem->FindElement("collide_bitmask"))
            {
              conaffinity = static_cast<uint16_t>(
                  collideBitmaskElem->Get<uint32_t>());
            }
          }
        }

        uint16_t contype = contypeOpt.value_or(conaffinity);

        geom->contype = static_cast<int>(contype);
        geom->conaffinity = static_cast<int>(conaffinity);
        mjs_setName(geom->element,
                    ::sdf::JoinName(body_name, collision->Name()).c_str());
        auto shapeInfo =
            std::make_shared<ShapeInfo>(_base.GetNextEntity(), linkInfo);
        shapeInfo->worldInfo = worldInfo;
        auto collisionPose = resolveSdfPose(collision->SemanticPose());
        copyPos(collisionPose.Pos(), geom->pos);
        copyQuat(collisionPose.Rot(), geom->quat);
        shapeInfo->geom = geom;
        shapeInfo->name = collision->Name();
        shapeInfo->categoryMask = contypeOpt;
        linkInfo->shapes.AddEntity(shapeInfo->entityId, shapeInfo, geom,
                                   linkInfo->entityId);

        // Add a site for the shape and register it in the frames map. This is
        // required for FrameSemantics to correctly transform the local
        // axis-aligned bounding box of the shape.
        auto shapeSite = mjs_addSite(child, nullptr);
        mju_copy3(shapeSite->pos, geom->pos);
        mju_copy4(shapeSite->quat, geom->quat);
        _base.frames[shapeInfo->entityId] =
            std::make_shared<FrameInfo>(shapeSite, worldInfo);
      }
    }

    // Add joints
    if (!_sdfModel->Static())
    {
      auto jointModelInfo = this->jointModelInfos[_index]
                                ? this->jointModelInfos[_index]
                                : _modelInfo;
      this->AddJoint(_base, _spec, childInJoint[_index], child, jointModelInfo);
    }

    // Recursively add children
    for (std::size_t i = 0; i < this->children[_index].size(); ++i)
    {
      this->AddToSpec(
          _base, _rootModel, _spec, this->children[_index][i], child);
    }
  }
};

}
/////////////////////////////////////////////////
Identity SDFFeatures::ConstructSdfNestedModel(const Identity &_parentID,
                                              const ::sdf::Model &_sdfModel)
{
  return this->ConstructSdfModelImpl(_parentID, _sdfModel);
}

/////////////////////////////////////////////////
Identity SDFFeatures::ConstructSdfModelImpl(Identity _parentID,
                                            const ::sdf::Model &_sdfModel)
{
  auto start = std::chrono::high_resolution_clock::now();
  WorldInfo *worldInfo{nullptr};
  ModelInfo *parentModelInfo{nullptr};

  if (this->worlds.HasEntity(_parentID))
  {
    worldInfo = this->ReferenceInterface<WorldInfo>(_parentID);
  }
  else
  {
    for (const auto &[worldId, wInfo] : this->worlds.idToObject)
    {
      if (wInfo->models.HasEntity(_parentID))
      {
        parentModelInfo = this->ReferenceInterface<ModelInfo>(_parentID);
        worldInfo = parentModelInfo->worldInfo;
        break;
      }
    }
  }

  if (!worldInfo)
  {
    gzerr << "Parent of model is neither a world nor a model\n";
    return this->GenerateInvalidId();
  }

  auto *spec = worldInfo->mjSpecObj;
  worldInfo->specDirty = true;

  if (parentModelInfo &&
      parentModelInfo->nestedModelNameToEntityId.find(_sdfModel.Name()) !=
          parentModelInfo->nestedModelNameToEntityId.end())
  {
     std::size_t nestedModelID =
         parentModelInfo->nestedModelNameToEntityId.at(_sdfModel.Name());
     return this->GenerateIdentity(
         nestedModelID, worldInfo->models.at(nestedModelID));
  }

  ModelKinematicStructure kinTree;
  kinTree.name = _sdfModel.Name();

  std::vector<const ::sdf::Model*> allModels;
  std::vector<const ::sdf::Joint*> allJoints;
  std::vector<std::shared_ptr<ModelInfo>> allModelInfos;

  std::function<void(const ::sdf::Model *, std::shared_ptr<ModelInfo>)>
      collectSdf =
          [&](const ::sdf::Model *model, std::shared_ptr<ModelInfo> mInfo)
  {
    allModels.push_back(model);
    allModelInfos.push_back(mInfo);
    for (std::size_t i = 0; i < model->LinkCount(); ++i)
    {
      kinTree.links.push_back(model->LinkByIndex(i));
      kinTree.modelInfos.push_back(mInfo);
      kinTree.sdfModels.push_back(model);
    }
    for (std::size_t i = 0; i < model->JointCount(); ++i)
    {
      allJoints.push_back(model->JointByIndex(i));
    }
    for (std::size_t i = 0; i < model->ModelCount(); ++i)
    {
      auto childModel = model->ModelByIndex(i);
      if (!childModel) continue;

      auto childModelInfo =
          std::make_shared<ModelInfo>(this->GetNextEntity(), worldInfo);
      childModelInfo->name = ::sdf::JoinName(mInfo->name, childModel->Name());
      childModelInfo->parentBody = mInfo->parentBody;
      mInfo->nestedModelNameToEntityId[childModel->Name()] =
          childModelInfo->entityId;
      worldInfo->models.AddEntity(
          childModelInfo->entityId, childModelInfo,
          JoinNames(worldInfo->name, childModelInfo->name),
          worldInfo->entityId);
      collectSdf(childModel, childModelInfo);
    }
  };

  auto rootModelInfo =
      std::make_shared<ModelInfo>(this->GetNextEntity(), worldInfo);
  rootModelInfo->name = _sdfModel.Name();
  if (parentModelInfo)
  {
    rootModelInfo->name =
        ::sdf::JoinName(parentModelInfo->name, _sdfModel.Name());
    rootModelInfo->parentBody = parentModelInfo->body;
    parentModelInfo->nestedModelNameToEntityId[_sdfModel.Name()] =
        rootModelInfo->entityId;
  }
  else
  {
    rootModelInfo->parentBody = mjs_findBody(spec, "world");
  }
  worldInfo->models.AddEntity(rootModelInfo->entityId, rootModelInfo,
                              JoinNames(worldInfo->name, rootModelInfo->name),
                              worldInfo->entityId);

  collectSdf(&_sdfModel, rootModelInfo);

  kinTree.parents.resize(kinTree.links.size(), nullptr);
  kinTree.childInJoint.resize(kinTree.links.size(), nullptr);
  kinTree.jointModelInfos.resize(kinTree.links.size(), nullptr);
  kinTree.children.resize(kinTree.links.size(), {});

  // Now go through the joints and update parent and children
  for (std::size_t i = 0; i < allJoints.size(); ++i)
  {
    const auto *joint = allJoints[i];
    std::string childLinkName;
    // TODO(azeey) Handle errors
    joint->ResolveChildLink(childLinkName);

    // The joint could belong to any model, we need to find which model it
    // belongs to by matching the joint in allModels.
    const ::sdf::Model *jointModel = nullptr;
    std::shared_ptr<ModelInfo> jointModelInfo = nullptr;
    for (std::size_t j = 0; j < allModels.size(); ++j)
    {
      if (allModels[j]->JointByName(joint->Name()) == joint)
      {
        jointModel = allModels[j];
        jointModelInfo = allModelInfos[j];
        break;
      }
    }

    auto childIndex = kinTree.FindLinkByName(childLinkName, jointModel);
    if (!childIndex)
    {
      gzerr << "Error finding link " << childLinkName << " in model "
            << jointModel->Name() << "\n";
      return this->GenerateInvalidId();
    }

    std::string parentLinkName;
    // TODO(azeey) Handle errors
    joint->ResolveParentLink(parentLinkName);
    if (parentLinkName == "world")
    {
      kinTree.childInJoint[*childIndex] = joint;
      kinTree.jointModelInfos[*childIndex] = jointModelInfo;
      continue;
    }
    auto parentIndex = kinTree.FindLinkByName(parentLinkName, jointModel);
    if (!parentIndex)
    {
      gzerr << "Error finding link " << parentLinkName << " in model "
            << jointModel->Name() << "\n";
      return this->GenerateInvalidId();
    }

    kinTree.parents[*childIndex] = kinTree.links[*parentIndex];
    kinTree.children[*parentIndex].push_back(*childIndex);
    kinTree.childInJoint[*childIndex] = joint;
    kinTree.jointModelInfos[*childIndex] = jointModelInfo;
  }

  for (std::size_t i = 0; i < kinTree.parents.size(); ++i)
  {
    if (!kinTree.parents[i])
    {
      kinTree.AddToSpec(
          *this, _sdfModel, spec, i, kinTree.modelInfos[i]->parentBody);
    }
  }
  if (!rootModelInfo->body)
  {
    gzerr << "There was no body associated with the root model\n";
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
    std::vector<const mjsBody *> allBodies;
    std::function<void(const ModelInfo *)> collectBodies =
        [&](const ModelInfo *m) {
      for (const auto &[body, id] : m->links.objectToID)
        allBodies.push_back(body);
      for (const auto &[name, nestedId] : m->nestedModelNameToEntityId)
        collectBodies(worldInfo->models.at(nestedId).get());
    };
    collectBodies(rootModelInfo.get());

    for (auto it1 = allBodies.begin(); it1 != allBodies.end(); ++it1)
    {
      for (auto it2 = std::next(it1); it2 != allBodies.end(); ++it2)
      {
        mjsExclude *exclude = mjs_addExclude(worldInfo->mjSpecObj);
        mjs_setString(exclude->bodyname1,
                      mjs_getString(mjs_getName((*it1)->element)));
        mjs_setString(exclude->bodyname2,
                      mjs_getString(mjs_getName((*it2)->element)));
      }
    }
  }
  auto end = std::chrono::high_resolution_clock::now();
  gztrace << "Model: " << _sdfModel.Name() << " constructed in "
            << std::chrono::duration<double>(end - start).count() << "\n";
  return this->GenerateIdentity(rootModelInfo->entityId, rootModelInfo);
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
