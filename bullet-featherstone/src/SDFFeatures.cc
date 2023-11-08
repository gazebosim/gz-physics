/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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
#include <BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h>
#include <gz/common/Mesh.hh>
#include <gz/common/MeshManager.hh>
#include <gz/common/SubMesh.hh>
#include <gz/math/eigen3/Conversions.hh>
#include <gz/math/Helpers.hh>

#include <sdf/Box.hh>
#include <sdf/Capsule.hh>
#include <sdf/Cylinder.hh>
#include <sdf/Ellipsoid.hh>
#include <sdf/Geometry.hh>
#include <sdf/Joint.hh>
#include <sdf/JointAxis.hh>
#include <sdf/Link.hh>
#include <sdf/Mesh.hh>
#include <sdf/Plane.hh>
#include <sdf/Sphere.hh>
#include <sdf/Surface.hh>

#include <BulletDynamics/Featherstone/btMultiBodyLinkCollider.h>
#include <BulletDynamics/Featherstone/btMultiBodyJointLimitConstraint.h>

#include <LinearMath/btQuaternion.h>

#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

namespace gz {
namespace physics {
namespace bullet_featherstone {

/////////////////////////////////////////////////
/// \brief Resolve the pose of an SDF DOM object with respect to its relative_to
/// frame. If that fails, return the raw pose
static std::optional<Eigen::Isometry3d> ResolveSdfPose(
  const ::sdf::SemanticPose &_semPose)
{
  math::Pose3d pose;
  ::sdf::Errors errors = _semPose.Resolve(pose);
  if (!errors.empty())
  {
    if (!_semPose.RelativeTo().empty())
    {
      gzerr << "There was an error in SemanticPose::Resolve:\n";
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

  return math::eigen3::convert(pose);
}

/////////////////////////////////////////////////
Identity SDFFeatures::ConstructSdfWorld(
    const Identity &_engine,
    const ::sdf::World &_sdfWorld)
{
  const Identity worldID = this->ConstructEmptyWorld(_engine, _sdfWorld.Name());

  const WorldInfoPtr &worldInfo = this->worlds.at(worldID);

  worldInfo->world->setGravity(convertVec(_sdfWorld.Gravity()));

  for (std::size_t i = 0; i < _sdfWorld.ModelCount(); ++i)
  {
    const ::sdf::Model *model = _sdfWorld.ModelByIndex(i);

    if (!model)
      continue;

    this->ConstructSdfModel(worldID, *model);
  }

  return worldID;
}

/////////////////////////////////////////////////
struct ParentInfo
{
  const ::sdf::Joint *joint;
  const ::sdf::Model *model;
};

/////////////////////////////////////////////////
struct Structure
{
  /// The root link of the model
  const ::sdf::Link *rootLink;
  const ::sdf::Joint *rootJoint;
  btScalar mass;
  btVector3 inertia;

  /// Is the root link fixed
  bool fixedBase;

  /// Get the parent joint of the link
  std::unordered_map<const ::sdf::Link*, ParentInfo> parentOf;

  /// This contains all the links except the root link
  std::vector<const ::sdf::Link*> flatLinks;

  std::vector<Structure> nestedStructure;
};

/////////////////////////////////////////////////
std::optional<Structure> ValidateModel(const ::sdf::Model &_sdfModel)
{
  std::unordered_map<const ::sdf::Link*, ParentInfo> parentOf;
  const ::sdf::Link *rootLink = nullptr;
  const ::sdf::Joint *rootJoint = nullptr;
  bool fixed = false;
  const std::string &rootModelName = _sdfModel.Name();
  // a map of parent link to a vector of its child links
  std::unordered_map<const ::sdf::Link*, std::vector<const ::sdf::Link*>>
    linkTree;

  const auto checkModel =
      [&rootLink, &rootJoint, &fixed, &parentOf, &rootModelName, &linkTree](
      const ::sdf::Model &model) -> bool
    {
      for (std::size_t i = 0; i < model.JointCount(); ++i)
      {
        const auto *joint = model.JointByIndex(i);
        const auto &parentLinkName = joint->ParentName();
        const auto *parent = model.LinkByName(parentLinkName);
        const auto &childLinkName = joint->ChildName();
        const auto *child = model.LinkByName(childLinkName);

        switch (joint->Type())
        {
          case ::sdf::JointType::FIXED:
          case ::sdf::JointType::REVOLUTE:
          case ::sdf::JointType::PRISMATIC:
          case ::sdf::JointType::BALL:
            break;
          default:
            gzerr << "Joint type [" << (std::size_t)(joint->Type())
                  << "] is not supported by "
                  << "gz-physics-bullet-featherstone-plugin. "
                  << "Replaced by a fixed joint.\n";
        }

        if (child == parent)
        {
          gzerr << "The Link [" << parentLinkName << "] is being attached to "
                << "itself by Joint [" << joint->Name() << "] in Model ["
                << rootModelName << "]. That is not allowed.\n";
          return false;
        }

        if (nullptr == parent && parentLinkName != "world")
        {
          gzerr << "The link [" << parentLinkName << "] cannot be found in "
                << "Model [" << rootModelName << "], but joint ["
                << joint->Name() << "] wants to use it as its parent link\n";
          return false;
        }
        else if (nullptr == parent)
        {
          // This link is attached to the world, making it the root
          if (nullptr != rootLink)
          {
            // A root already exists for this model
            gzerr << "Two root links were found for Model [" << rootModelName
                  << "]: [" << rootLink->Name() << "] and [" << childLinkName
                  << "], but gz-physics-bullet-featherstone-plugin only "
                  << "supports one root per Model.\n";
            return false;
          }

          if (joint->Type() != ::sdf::JointType::FIXED)
          {
            gzerr << "Link [" << child->Name() << "] in Model ["
                  << rootModelName << "] is being connected to the "
                  << "world by Joint [" << joint->Name() << "] with a ["
                  << (std::size_t)(joint->Type()) << "] joint type, but only "
                  << "Fixed (" << (std::size_t)(::sdf::JointType::FIXED)
                  << ") is supported by "
                  << "gz-physics-bullet-featherstone-plugin\n";
            return false;
          }

          rootLink = child;
          rootJoint = joint;
          fixed = true;

          // Do not add the root link to the set of links that have parents
          continue;
        }

        if (!parentOf.insert(
          std::make_pair(child, ParentInfo{joint, &model})).second)
        {
          gzerr << "The Link [" << childLinkName << "] in Model ["
                << rootModelName << "] has multiple parent joints. That is not "
                << "supported by the gz-physics-bullet-featherstone plugin.\n";
        }
        if (parent != nullptr)
        {
          linkTree[parent].push_back(child);
        }
      }

      return true;
    };

  if (!checkModel(_sdfModel))
    return std::nullopt;

  for (std::size_t i = 0; i < _sdfModel.ModelCount(); ++i)
  {
    if (!checkModel(*_sdfModel.ModelByIndex(i)))
      return std::nullopt;
  }

  // Find root link in model and verify that there is only one root link in
  // the model. Returns false if more than one root link is found
  const auto findRootLink =
      [&rootLink, &parentOf, &rootModelName](const ::sdf::Model &model) -> bool
    {
      for (std::size_t i = 0; i < model.LinkCount(); ++i)
      {
        const auto *link = model.LinkByIndex(i);
        if (parentOf.count(link) == 0)
        {
          // This link must be the root. If a different link was already
          // identified as the root then we have a conflict.
          if (rootLink && rootLink != link)
          {
            gzerr << "Two root links were found for Model [" << rootModelName
                  << "]: [" << rootLink->Name() << "] and [" << link->Name()
                  << "]. The Link [" << link->Name() << "] is implicitly a "
                  << "root because it has no parent joint.\n";
            return false;
          }

          rootLink = link;
        }
      }

      return true;
    };

  if (rootLink == nullptr && !findRootLink(_sdfModel))
  {
    // No root link was found in this model
    return std::nullopt;
  }

  // find root link in nested models if one was not already found
  for (std::size_t i = 0; i < _sdfModel.ModelCount(); ++i)
  {
    if (rootLink != nullptr)
    {
      break;
    }
    if (!findRootLink(*_sdfModel.ModelByIndex(i)))
    {
      return std::nullopt;
    }
  }

  if (!rootLink)
  {
    gzerr << "No root link was found for model [" << _sdfModel.Name() << "\n";
    return std::nullopt;
  }

  // The documentation for bullet does not mention whether parent links must
  // have a lower index than their child links, but the Featherstone Algorithm
  // needs to crawl up and down the tree systematically, and so the flattened
  // tree structures used by the algorithm usually do expect the parents to
  // come before their children in the array, and do not work correctly if that
  // ordering is not held. Out of an abundance of caution we will assume that
  // ordering is necessary.
  std::vector<const ::sdf::Link*> flatLinks;
  std::function<void(const ::sdf::Link *)> flattenLinkTree =
      [&](const ::sdf::Link *link)
  {
    if (link != rootLink)
    {
      flatLinks.push_back(link);
    }
    if (auto it = linkTree.find(link); it != linkTree.end())
    {
      for (const auto &child_link : it->second)
      {
        flattenLinkTree(child_link);
      }
    }
  };
  flattenLinkTree(rootLink);

  const auto &M = rootLink->Inertial().MassMatrix();
  const auto mass = static_cast<btScalar>(M.Mass());
  btVector3 inertia(convertVec(M.DiagonalMoments()));
  for (const double &I : {M.Ixy(), M.Ixz(), M.Iyz()})
  {
    if (std::abs(I) > 1e-3)
    {
      gzerr << "The base link of the model is required to have a diagonal "
            << "inertia matrix by gz-physics-bullet-featherstone, but the "
            << "Model [" << _sdfModel.Name() << "] has a non-zero diagonal "
            << "value: " << I << "\n";
      return std::nullopt;
    }
  }

  return Structure{
    rootLink, rootJoint, mass, inertia, fixed, parentOf, flatLinks};
}

/////////////////////////////////////////////////
Identity SDFFeatures::ConstructSdfNestedModel(const Identity &_parentID,
                                              const ::sdf::Model &_sdfModel)
{
  return this->ConstructSdfModelImpl(_parentID, _sdfModel);
}

/////////////////////////////////////////////////
Identity SDFFeatures::ConstructSdfModelImpl(
    std::size_t _parentID,
    const ::sdf::Model &_sdfModel)
{
  const auto validation = ValidateModel(_sdfModel);
  if (!validation.has_value())
    return this->GenerateInvalidId();

  const auto &structure = *validation;
  const bool isStatic = _sdfModel.Static();

  // Identity worldIdentity;
  auto parentModelIt = this->models.find(_parentID);
  const bool isNested = (parentModelIt != this->models.end());

  // auto worldID = _parentID;
  // WorldInfo *world = nullptr;
  const auto &worldIdentity = isNested ?
        parentModelIt->second->world :
        this->GenerateIdentity(_parentID, this->worlds[_parentID]);
/*  else
  {
    const WorldInfoPtr &worldInfo - this->worlds.at(_parentID);
  }
*/
  // const auto *world = this->ReferenceInterface<WorldInfo>(worldIdentity);
 // auto worldIdentity = this->GenerateIdentity(worldID, this->worlds[worldID]);

  const auto rootInertialToLink =
    gz::math::eigen3::convert(structure.rootLink->Inertial().Pose()).inverse();

  auto modelID = [&] {
    if (isNested)
    {
      auto modelIdentity = this->GenerateIdentity(_parentID, this->models[_parentID]);
      return this->AddNestedModel(
          _sdfModel.Name(), modelIdentity, worldIdentity, rootInertialToLink,
          std::make_unique<btMultiBody>(
            static_cast<int>(structure.flatLinks.size()),
            structure.mass,
            structure.inertia,
            structure.fixedBase || isStatic,
            true));
    }
    else
    {
      return this->AddModel(
          _sdfModel.Name(), worldIdentity, rootInertialToLink,
          std::make_unique<btMultiBody>(
            static_cast<int>(structure.flatLinks.size()),
            structure.mass,
            structure.inertia,
            structure.fixedBase || isStatic,
            true));
    }
  }();

  const auto rootID =
    this->AddLink(LinkInfo{
      structure.rootLink->Name(), std::nullopt, modelID, rootInertialToLink
    });
  const auto *model = this->ReferenceInterface<ModelInfo>(modelID);

  return modelID;
}

/////////////////////////////////////////////////
Identity SDFFeatures::ConstructSdfModel(
    const Identity &_worldID,
    const ::sdf::Model &_sdfModel)
{
  const auto validation = ValidateModel(_sdfModel);
  if (!validation.has_value())
    return this->GenerateInvalidId();

  const auto &structure = *validation;
  const bool isStatic = _sdfModel.Static();

  const auto *world = this->ReferenceInterface<WorldInfo>(_worldID);

  const auto rootInertialToLink =
    gz::math::eigen3::convert(structure.rootLink->Inertial().Pose()).inverse();
  const auto modelID = this->AddModel(
    _sdfModel.Name(), _worldID, rootInertialToLink,
    std::make_unique<btMultiBody>(
      static_cast<int>(structure.flatLinks.size()),
      structure.mass,
      structure.inertia,
      structure.fixedBase || isStatic,
      true));

  const auto rootID =
    this->AddLink(LinkInfo{
      structure.rootLink->Name(), std::nullopt, modelID, rootInertialToLink
    });
  const auto *model = this->ReferenceInterface<ModelInfo>(modelID);

  if (structure.rootJoint)
  {
    this->AddJoint(
          JointInfo{
            structure.rootJoint->Name(),
            RootJoint{},
            std::nullopt,
            rootID,
            Eigen::Isometry3d::Identity(),
            Eigen::Isometry3d::Identity(),
            modelID
          });
  }

  model->body->setLinearDamping(0.0);
  model->body->setAngularDamping(0.0);

  std::unordered_map<const ::sdf::Link*, Identity> linkIDs;
  linkIDs.insert(std::make_pair(structure.rootLink, rootID));

  for (int i = 0; i < static_cast<int>(structure.flatLinks.size()); ++i)
  {
    const auto *link = structure.flatLinks[static_cast<std::size_t>(i)];
    const Eigen::Isometry3d linkToComTf = gz::math::eigen3::convert(
          link->Inertial().Pose());

    if (linkIDs.find(link) == linkIDs.end())
    {
      const auto linkID = this->AddLink(
        LinkInfo{link->Name(), i, modelID, linkToComTf.inverse()});
      linkIDs.insert(std::make_pair(link, linkID));
    }

    const auto &M = link->Inertial().MassMatrix();
    const btScalar mass = static_cast<btScalar>(M.Mass());
    const auto inertia = btVector3(convertVec(M.DiagonalMoments()));

    for (const double I : {M.Ixy(), M.Ixz(), M.Iyz()})
    {
      if (std::abs(I) > 1e-3)
      {
        gzerr << "Links are required to have a diagonal inertia matrix in "
              << "gz-physics-bullet-featherstone, but Link [" << link->Name()
              << "] in Model [" << model->name << "] has a non-zero off "
              << "diagonal value in its inertia matrix\n";
        return this->GenerateInvalidId();
      }
    }

    if (structure.parentOf.size())
    {
      const auto &parentInfo = structure.parentOf.at(link);
      const auto *joint = parentInfo.joint;
      const auto &parentLinkID = linkIDs.at(
        parentInfo.model->LinkByName(joint->ParentName()));
      const auto *parentLinkInfo = this->ReferenceInterface<LinkInfo>(
        parentLinkID);

      int parentIndex = -1;
      if (parentLinkInfo->indexInModel.has_value())
        parentIndex = *parentLinkInfo->indexInModel;

      Eigen::Isometry3d poseParentLinkToJoint;
      Eigen::Isometry3d poseParentComToJoint;
      {
        gz::math::Pose3d gzPoseParentToJoint;
        const auto errors = joint->SemanticPose().Resolve(
          gzPoseParentToJoint, joint->ParentName());
        if (!errors.empty())
        {
          gzerr << "An error occurred while resolving the transform of Joint ["
                << joint->Name() << "] in Model [" << model->name << "]:\n";
          for (const auto &error : errors)
          {
            gzerr << error << "\n";
          }

          return this->GenerateInvalidId();
        }

        poseParentLinkToJoint = gz::math::eigen3::convert(gzPoseParentToJoint);
        poseParentComToJoint =
          poseParentLinkToJoint * parentLinkInfo->inertiaToLinkFrame;
      }

      Eigen::Isometry3d poseJointToChild;
      {
        gz::math::Pose3d gzPoseJointToChild;
        const auto errors =
          link->SemanticPose().Resolve(gzPoseJointToChild, joint->Name());
        if (!errors.empty())
        {
          gzerr << "An error occured while resolving the transform of Link ["
                << link->Name() << "]:\n";
          for (const auto &error : errors)
          {
            gzerr << error << "\n";
          }

          return this->GenerateInvalidId();
        }

        poseJointToChild = gz::math::eigen3::convert(gzPoseJointToChild);
      }

      btQuaternion btRotParentComToJoint;
      convertMat(poseParentComToJoint.linear())
        .getRotation(btRotParentComToJoint);

       auto linkParent = _sdfModel.LinkByName(joint->ParentName());
       gz::math::Pose3d parentTransformInWorldSpace;
       const auto errors = linkParent->SemanticPose().Resolve(
         parentTransformInWorldSpace);

       gz::math::Pose3d parent2joint;
       const auto errors2 = linkParent->SemanticPose().Resolve(
         parent2joint, joint->Name());  // X_JP

       btTransform parentLocalInertialFrame = convertTf(
         parentLinkInfo->inertiaToLinkFrame);
       btTransform parent2jointBt = convertTf(
         gz::math::eigen3::convert(parent2joint.Inverse()));  // X_PJ

       // offsetInABt = parent COM to pivot (X_IpJ)
       // offsetInBBt = current COM to pivot (X_IcJ)
       //
       // X_PIp
       // X_PJ
       // X_IpJ = X_PIp^-1 * X_PJ
       //
       // X_IcJ = X_CIc^-1 * X_CJ
       btTransform offsetInABt, offsetInBBt;
       offsetInABt = parentLocalInertialFrame * parent2jointBt;
       offsetInBBt =
          convertTf(linkToComTf.inverse() * poseJointToChild.inverse());
       // R_IcJ * R_IpJ ^ -1 = R_IcIp;
       btQuaternion parentRotToThis =
         offsetInBBt.getRotation() * offsetInABt.inverse().getRotation();

      auto jointID = this->AddJoint(
        JointInfo{
          joint->Name(),
          InternalJoint{i},
          model->linkEntityIds[static_cast<std::size_t>(parentIndex+1)],
          linkIDs.find(link)->second,
          poseParentLinkToJoint,
          poseJointToChild,
          modelID
        });
      auto jointInfo = this->ReferenceInterface<JointInfo>(jointID);

      if (::sdf::JointType::PRISMATIC == joint->Type() ||
          ::sdf::JointType::REVOLUTE == joint->Type() ||
          ::sdf::JointType::BALL == joint->Type())
      {
        if (::sdf::JointType::REVOLUTE == joint->Type())
        {
          const auto axis = convertVec(joint->Axis()->Xyz());
          model->body->setupRevolute(
            i, mass, inertia, parentIndex,
            parentRotToThis,
            quatRotate(offsetInBBt.getRotation(), axis),
            offsetInABt.getOrigin(),
            -offsetInBBt.getOrigin(),
            true);
        }
        else if (::sdf::JointType::PRISMATIC == joint->Type())
        {
          const auto axis = convertVec(joint->Axis()->Xyz());
          model->body->setupPrismatic(
            i, mass, inertia, parentIndex,
            parentRotToThis,
            quatRotate(offsetInBBt.getRotation(), axis),
            offsetInABt.getOrigin(),
            -offsetInBBt.getOrigin(),
            true);
        }
        else if (::sdf::JointType::BALL == joint->Type())
        {
          model->body->setupSpherical(
            i, mass, inertia, parentIndex,
            parentRotToThis,
            offsetInABt.getOrigin(),
            -offsetInBBt.getOrigin(),
            true);
        }
      }
      else
      {
        model->body->setupFixed(
          i, mass, inertia, parentIndex,
          parentRotToThis,
          offsetInABt.getOrigin(),
          -offsetInBBt.getOrigin());
      }

      if (::sdf::JointType::PRISMATIC == joint->Type() ||
          ::sdf::JointType::REVOLUTE == joint->Type())
      {
        model->body->getLink(i).m_jointLowerLimit =
            static_cast<btScalar>(joint->Axis()->Lower());
        model->body->getLink(i).m_jointUpperLimit =
            static_cast<btScalar>(joint->Axis()->Upper());
        model->body->getLink(i).m_jointDamping =
            static_cast<btScalar>(joint->Axis()->Damping());
        model->body->getLink(i).m_jointFriction =
            static_cast<btScalar>(joint->Axis()->Friction());
        model->body->getLink(i).m_jointMaxVelocity =
            static_cast<btScalar>(joint->Axis()->MaxVelocity());
        model->body->getLink(i).m_jointMaxForce =
            static_cast<btScalar>(joint->Axis()->Effort());
        jointInfo->effort = static_cast<btScalar>(joint->Axis()->Effort());

        jointInfo->jointLimits =
          std::make_shared<btMultiBodyJointLimitConstraint>(
            model->body.get(), i, static_cast<btScalar>(joint->Axis()->Lower()),
            static_cast<btScalar>(joint->Axis()->Upper()));
        world->world->addMultiBodyConstraint(jointInfo->jointLimits.get());
      }

      jointInfo->jointFeedback = std::make_shared<btMultiBodyJointFeedback>();
      jointInfo->jointFeedback->m_reactionForces.setZero();
      model->body->getLink(i).m_jointFeedback = jointInfo->jointFeedback.get();
    }
  }

  model->body->setHasSelfCollision(_sdfModel.SelfCollide());
  model->body->finalizeMultiDof();

  const auto worldToModel = ResolveSdfPose(_sdfModel.SemanticPose());
  if (!worldToModel)
    return this->GenerateInvalidId();

  const auto modelToRootLink =
    ResolveSdfPose(structure.rootLink->SemanticPose());
  if (!modelToRootLink)
    return this->GenerateInvalidId();

  const auto worldToRootCom =
    *worldToModel * *modelToRootLink * rootInertialToLink.inverse();

  model->body->setBaseWorldTransform(convertTf(worldToRootCom));
  model->body->setBaseVel(btVector3(0, 0, 0));
  model->body->setBaseOmega(btVector3(0, 0, 0));

  {
    const auto *link = structure.rootLink;
    const auto &M = link->Inertial().MassMatrix();
    const btScalar mass = static_cast<btScalar>(M.Mass());
    const auto inertia = convertVec(M.DiagonalMoments());

    for (const double I : {M.Ixy(), M.Ixz(), M.Iyz()})
    {
      if (std::abs(I) > 1e-3)
      {
        gzerr << "Links are required to have a diagonal inertia matrix in "
              << "gz-physics-bullet-featherstone, but Link [" << link->Name()
              << "] in Model [" << model->name << "] has a non-zero off "
              << "diagonal value in its inertia matrix\n";
      }
      else
      {
        model->body->setBaseMass(mass);
        model->body->setBaseInertia(inertia);
      }
    }
  }

  world->world->addMultiBody(model->body.get());

  for (const auto& [linkSdf, linkID] : linkIDs)
  {
    for (std::size_t c = 0; c < linkSdf->CollisionCount(); ++c)
    {
      // If we fail to add the collision, just keep building the model. It may
      // need to be constructed outside of the SDF generation pipeline, e.g.
      // with AttachHeightmap.
      this->AddSdfCollision(linkID, *linkSdf->CollisionByIndex(c), isStatic);
    }
  }

  return modelID;
}

/////////////////////////////////////////////////
bool SDFFeatures::AddSdfCollision(
    const Identity &_linkID,
    const ::sdf::Collision &_collision,
    bool isStatic)
{
  if (!_collision.Geom())
  {
    gzerr << "The geometry element of collision [" << _collision.Name() << "] "
           << "was a nullptr\n";
    return false;
  }

  auto *linkInfo = this->ReferenceInterface<LinkInfo>(_linkID);
  const auto *model = this->ReferenceInterface<ModelInfo>(linkInfo->model);

  const auto &geom = _collision.Geom();
  std::unique_ptr<btCollisionShape> shape;

  if (const auto *box = geom->BoxShape())
  {
    const auto size = math::eigen3::convert(box->Size());
    const auto halfExtents = convertVec(size)*0.5;
    shape = std::make_unique<btBoxShape>(halfExtents);
  }
  else if (const auto *sphere = geom->SphereShape())
  {
    const auto radius = sphere->Radius();
    shape = std::make_unique<btSphereShape>(static_cast<btScalar>(radius));
  }
  else if (const auto *cylinder = geom->CylinderShape())
  {
    const auto radius = static_cast<btScalar>(cylinder->Radius());
    const auto halfLength = static_cast<btScalar>(cylinder->Length()*0.5);
    shape =
      std::make_unique<btCylinderShapeZ>(btVector3(radius, radius, halfLength));
  }
  else if (const auto *plane = geom->PlaneShape())
  {
    const auto normal = convertVec(math::eigen3::convert(plane->Normal()));
    shape = std::make_unique<btStaticPlaneShape>(normal, btScalar(0));
  }
  else if (const auto *capsule = geom->CapsuleShape())
  {
    shape = std::make_unique<btCapsuleShapeZ>(
        static_cast<btScalar>(capsule->Radius()),
        static_cast<btScalar>(capsule->Length()));
  }
  else if (const auto *ellipsoid = geom->EllipsoidShape())
  {
    // This code is from bullet3 examples/SoftDemo/SoftDemo.cpp
    struct Hammersley
    {
      static void Generate(btVector3* x, int n)
      {
        for (int i = 0; i < n; i++)
        {
          btScalar p = 0.5, t = 0;
          for (int j = i; j; p *= btScalar(0.5), j >>= 1)
            if (j & 1) t += p;
          btScalar w = 2 * t - 1;
          btScalar a =
              (SIMD_PI + btScalar(2.0) * static_cast<btScalar>(i) * SIMD_PI) /
              static_cast<btScalar>(n);
          btScalar s = btSqrt(1 - w * w);
          *x++ = btVector3(s * btCos(a), s * btSin(a), w);
        }
      }
    };
    btAlignedObjectArray<btVector3> vtx;
    vtx.resize(3 + 128);
    Hammersley::Generate(&vtx[0], vtx.size());
    btVector3 center(0, 0, 0);
    btVector3 radius = convertVec(ellipsoid->Radii());
    for (int i = 0; i < vtx.size(); ++i)
    {
      vtx[i] = vtx[i] * radius + center;
    }

    this->triangleMeshes.push_back(std::make_unique<btTriangleMesh>());

    for (int i = 0; i < vtx.size()/3; i++)
    {
      const btVector3& v0 = vtx[i * 3 + 0];
      const btVector3& v1 = vtx[i * 3 + 1];
      const btVector3& v2 = vtx[i * 3 + 2];
      this->triangleMeshes.back()->addTriangle(v0, v1, v2);
    }
    auto compoundShape = std::make_unique<btCompoundShape>();

    this->meshesGImpact.push_back(
      std::make_unique<btGImpactMeshShape>(
        this->triangleMeshes.back().get()));
    this->meshesGImpact.back()->updateBound();
    this->meshesGImpact.back()->setMargin(btScalar(0.001));
    compoundShape->addChildShape(btTransform::getIdentity(),
      this->meshesGImpact.back().get());
    shape = std::move(compoundShape);
  }
  else if (const auto *meshSdf = geom->MeshShape())
  {
    auto &meshManager = *gz::common::MeshManager::Instance();
    auto *mesh = meshManager.Load(meshSdf->Uri());
    const btVector3 scale = convertVec(meshSdf->Scale());
    if (nullptr == mesh)
    {
      gzwarn << "Failed to load mesh from [" << meshSdf->Uri()
             << "]." << std::endl;
      return false;
    }

    auto compoundShape = std::make_unique<btCompoundShape>();

    for (unsigned int submeshIdx = 0;
         submeshIdx < mesh->SubMeshCount();
         ++submeshIdx)
    {
      auto s = mesh->SubMeshByIndex(submeshIdx).lock();
      auto vertexCount = s->VertexCount();
      auto indexCount = s->IndexCount();
      btAlignedObjectArray<btVector3> convertedVerts;
      convertedVerts.reserve(static_cast<int>(vertexCount));
      for (unsigned int i = 0; i < vertexCount; i++)
      {
        convertedVerts.push_back(btVector3(
              static_cast<btScalar>(s->Vertex(i).X()) * scale[0],
              static_cast<btScalar>(s->Vertex(i).Y()) * scale[1],
              static_cast<btScalar>(s->Vertex(i).Z()) * scale[2]));
      }

      this->triangleMeshes.push_back(std::make_unique<btTriangleMesh>());
      for (unsigned int i = 0; i < indexCount/3; i++)
      {
        const btVector3& v0 = convertedVerts[s->Index(i*3)];
        const btVector3& v1 = convertedVerts[s->Index(i*3 + 1)];
        const btVector3& v2 = convertedVerts[s->Index(i*3 + 2)];
        this->triangleMeshes.back()->addTriangle(v0, v1, v2);
      }

      this->meshesGImpact.push_back(
        std::make_unique<btGImpactMeshShape>(
          this->triangleMeshes.back().get()));
      this->meshesGImpact.back()->updateBound();
      this->meshesGImpact.back()->setMargin(btScalar(0.001));
      compoundShape->addChildShape(btTransform::getIdentity(),
        this->meshesGImpact.back().get());
    }
    shape = std::move(compoundShape);
  }
  else
  {
    // TODO(MXG) Support mesh collisions
    gzerr << "Unsupported collision geometry type ["
          << (std::size_t)(geom->Type()) << "] for collision ["
          << _collision.Name() << "] in Link [" << linkInfo->name
          << "] of Model [" << model->name << "]\n";
    return false;
  }

  double mu = 1.0;
  double mu2 = 1.0;
  double restitution = 0.0;

  double rollingFriction = 0.0;
  if (const auto *surface = _collision.Surface())
  {
    if (const auto *friction = surface->Friction())
    {
      if (const auto frictionElement = friction->Element())
      {
        if (const auto bullet = frictionElement->GetElement("bullet"))
        {
          if (const auto f1 = bullet->GetElement("friction"))
            mu = f1->Get<double>();

          if (const auto f2 = bullet->GetElement("friction2"))
            mu2 = f2->Get<double>();

          // What is fdir1 for in the SDF's <bullet> spec?

          if (const auto rolling = bullet->GetElement("rolling_friction"))
            rollingFriction = rolling->Get<double>();
        }
      }
    }

    if (const auto surfaceElement = surface->Element())
    {
      if (const auto bounce = surfaceElement->GetElement("bounce"))
      {
        if (const auto r = bounce->GetElement("restitution_coefficient"))
          restitution = r->Get<double>();
      }
    }
  }

  Eigen::Isometry3d linkFrameToCollision;
  if (shape != nullptr)
  {
    {
      gz::math::Pose3d gzLinkToCollision;
      const auto errors =
        _collision.SemanticPose().Resolve(gzLinkToCollision, linkInfo->name);
      if (!errors.empty())
      {
        gzerr << "An error occurred while resolving the transform of the "
          << "collider [" << _collision.Name() << "] in Link ["
          << linkInfo->name << "] in Model [" << model->name << "]:\n";
        for (const auto &error : errors)
        {
          gzerr << error << "\n";
        }

        return false;
      }

      linkFrameToCollision = gz::math::eigen3::convert(gzLinkToCollision);
    }

    const btTransform btInertialToCollision =
      convertTf(linkInfo->inertiaToLinkFrame * linkFrameToCollision);

    int linkIndexInModel = -1;
    if (linkInfo->indexInModel.has_value())
      linkIndexInModel = *linkInfo->indexInModel;

    if (!linkInfo->collider)
    {
      linkInfo->shape = std::make_unique<btCompoundShape>();

      // NOTE: Bullet does not appear to support different surface properties
      // for different shapes attached to the same link.
      linkInfo->collider = std::make_unique<btMultiBodyLinkCollider>(
        model->body.get(), linkIndexInModel);

      linkInfo->shape->addChildShape(btInertialToCollision, shape.get());

      linkInfo->collider->setCollisionShape(linkInfo->shape.get());
      linkInfo->collider->setRestitution(static_cast<btScalar>(restitution));
      linkInfo->collider->setRollingFriction(
        static_cast<btScalar>(rollingFriction));
      linkInfo->collider->setFriction(static_cast<btScalar>(mu));
      linkInfo->collider->setAnisotropicFriction(
        btVector3(static_cast<btScalar>(mu), static_cast<btScalar>(mu2), 1),
        btCollisionObject::CF_ANISOTROPIC_FRICTION);

      if (linkIndexInModel >= 0)
      {
        model->body->getLink(linkIndexInModel).m_collider =
          linkInfo->collider.get();
        const auto p = model->body->localPosToWorld(
          linkIndexInModel, btVector3(0, 0, 0));
        const auto rot = model->body->localFrameToWorld(
          linkIndexInModel, btMatrix3x3::getIdentity());
        linkInfo->collider->setWorldTransform(btTransform(rot, p));
      }
      else
      {
        model->body->setBaseCollider(linkInfo->collider.get());
        linkInfo->collider->setWorldTransform(
          model->body->getBaseWorldTransform());
      }

      auto *world = this->ReferenceInterface<WorldInfo>(model->world);

      if (isStatic)
      {
        world->world->addCollisionObject(
          linkInfo->collider.get(),
          btBroadphaseProxy::StaticFilter,
          btBroadphaseProxy::AllFilter ^ btBroadphaseProxy::StaticFilter);
      }
      else
      {
        world->world->addCollisionObject(
          linkInfo->collider.get(),
          btBroadphaseProxy::DefaultFilter,
          btBroadphaseProxy::AllFilter);
      }
    }
    else if (linkInfo->shape)
    {
      linkInfo->shape->addChildShape(btInertialToCollision, shape.get());
    }
    else
    {
      // TODO(MXG): Maybe we should check if the new collider's properties
      // match the existing collider and issue a warning if they don't.
    }

    this->AddCollision(
      CollisionInfo{
        _collision.Name(),
        std::move(shape),
        _linkID,
        linkFrameToCollision});
  }

  return true;
}

Identity SDFFeatures::ConstructSdfCollision(
    const Identity &_linkID,
    const ::sdf::Collision &_collision)
{
  if(this->AddSdfCollision(_linkID, _collision, false))
  {
    for (const auto& collision : this->collisions)
    {
      if (collision.second->link.id == _linkID.id)
      {
        return this->GenerateIdentity(
          collision.first, this->collisions.at(collision.first));
      }
    }
  }
  return this->GenerateInvalidId();
}

}  // namespace bullet_featherstone
}  // namespace physics
}  // namespace gz
