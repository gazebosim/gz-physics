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
#include <sdf/Physics.hh>
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

  const ::sdf::Physics *physics = _sdfWorld.PhysicsByIndex(0);
  if (physics)
    worldInfo->stepSize = physics->MaxStepSize();
  else
    worldInfo->stepSize = 0.001;

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
  const ::sdf::Link *link;
  const ::sdf::Link *parent;
};

/////////////////////////////////////////////////
struct Structure
{
  /// The root link of the model
  const ::sdf::Link *rootLink;
  const ::sdf::Model *model;
  const ::sdf::Joint *rootJoint;
  btScalar mass;
  btVector3 inertia;
  math::Pose3d linkToPrincipalAxesPose;

  /// Is the root link fixed
  bool fixedBase;

  /// Get the parent joint of the link
  std::unordered_map<const ::sdf::Link*, ParentInfo> parentOf;

  /// This contains all the links except the root link
  std::vector<const ::sdf::Link*> flatLinks;
};

/////////////////////////////////////////////////
void extractInertial(
    const math::Inertiald &_inertial,
    btScalar &_mass,
    btVector3 &_principalInertiaMoments,
    math::Pose3d &_linkToPrincipalAxesPose)
{
  const auto &M = _inertial.MassMatrix();
  _mass = static_cast<btScalar>(M.Mass());
  _principalInertiaMoments = convertVec(M.PrincipalMoments());
  _linkToPrincipalAxesPose = _inertial.Pose();
  _linkToPrincipalAxesPose.Rot() *= M.PrincipalAxesOffset();
}

/////////////////////////////////////////////////
/// \brief Get pose of joint relative to link
/// \param[out] _resolvedPose Pose of joint relative to link
/// \param[in] _model Parent model of joint
/// \param[in] _joint Joint name
/// \param[in] _link Scoped link name
::sdf::Errors resolveJointPoseRelToLink(math::Pose3d &_resolvedPose,
    const ::sdf::Model *_model,
    const std::string &_joint, const std::string &_link)
{
  ::sdf::Errors errors;
  const auto *joint = _model->JointByName(_joint);
  if (!joint)
  {
    gzerr << "No joint [" << _joint << "] found in model ["
          << _model->Name() << "]" << std::endl;
    return errors;
  }

  std::string childLinkName;
  errors = joint->ResolveChildLink(childLinkName);
  if (!errors.empty())
  {
    childLinkName = joint->ChildName();
  }

  // joint pose is expressed relative to child link so return joint pose
  // if input link is the child link
  if (childLinkName == _link)
  {
    errors = joint->SemanticPose().Resolve(_resolvedPose);
    return errors;
  }

  // compute joint pose relative to specified link
  const auto *link = _model->LinkByName(_link);
  if (!link)
  {
    gzerr << "No link [" << _link << "] found in model ["
          << _model->Name() << "]" << std::endl;
   return errors;
  }

  math::Pose3d jointPoseRelToModel;
  errors = _model->SemanticPose().Resolve(jointPoseRelToModel,
      _model->Name() + "::" + _joint);
  jointPoseRelToModel = jointPoseRelToModel.Inverse();

  math::Pose3d modelPoseRelToLink;
  errors = _model->SemanticPose().Resolve(modelPoseRelToLink,
      _model->Name() + "::" + _link);

  _resolvedPose = modelPoseRelToLink * jointPoseRelToModel;

  return errors;
}

/////////////////////////////////////////////////
/// \brief Recursively build a tree of parent-child data structures from the
/// input Model SDF.
/// \param[in] _sdfModel input Model SDF.
/// \param[out] _parentOf A map of child link to its parent
/// \param[out] _linkTree A map of parent link to its child links
bool buildTrees(const ::sdf::Model *_sdfModel,
    std::unordered_map<const ::sdf::Link*, ParentInfo> &_parentOf,
    std::unordered_map<const ::sdf::Link*,
    std::vector<const ::sdf::Link*>> &_linkTree)
{
  for (std::size_t i = 0; i < _sdfModel->JointCount(); ++i)
  {
    const auto *joint = _sdfModel->JointByIndex(i);
    std::string parentLinkName;
    ::sdf::Errors errors = joint->ResolveParentLink(parentLinkName);
    if (!errors.empty())
    {
      parentLinkName = joint->ParentName();
    }
    std::string childLinkName;
    errors = joint->ResolveChildLink(childLinkName);
    if (!errors.empty())
    {
      childLinkName = joint->ChildName();
    }
    const std::string &modelName = _sdfModel->Name();
    const auto *parent = _sdfModel->LinkByName(parentLinkName);
    const auto *child = _sdfModel->LinkByName(childLinkName);

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
            << modelName << "]. That is not allowed.\n";
      return false;
    }
    if (nullptr == parent && parentLinkName != "world")
    {
      gzerr << "The link [" << parentLinkName << "] cannot be found in "
            << "Model [" << modelName << "], but joint ["
            << joint->Name() << "] wants to use it as its parent link\n";
      return false;
    }
    else if (nullptr == parent)
    {
      // This link is attached to the world, making it the root
      if (joint->Type() != ::sdf::JointType::FIXED)
      {
        gzerr << "Link [" << child->Name() << "] in Model ["
              << modelName << "] is being connected to the "
              << "world by Joint [" << joint->Name() << "] with a ["
              << (std::size_t)(joint->Type()) << "] joint type, but only "
              << "Fixed (" << (std::size_t)(::sdf::JointType::FIXED)
              << ") is supported by "
              << "gz-physics-bullet-featherstone-plugin\n";
        return false;
      }
    }

    if (!_parentOf.insert(
      std::make_pair(child, ParentInfo{joint, _sdfModel, child, parent}))
      .second)
    {
      gzerr << "The Link [" << childLinkName << "] in Model ["
            << modelName << "] has multiple parent joints. That is not "
            << "supported by the gz-physics-bullet-featherstone plugin.\n";
    }
    if (parent != nullptr)
    {
      _linkTree[parent].push_back(child);
    }
  }

  // Recursively build tree from nested models
  for (std::size_t i = 0; i < _sdfModel->ModelCount(); ++i)
  {
    const auto *model = _sdfModel->ModelByIndex(i);
    if (!buildTrees(model, _parentOf, _linkTree))
      return false;
  }
  return true;
}

/////////////////////////////////////////////////
/// \brief Find all the root links given a model SDF
/// \param[in] _sdfModel Model SDF
/// \param[in] _parentOf A map of child link to its parent info
/// \param[out] _rootLinks A vector of root links paired with their
/// immediate parent model
void findRootLinks(const ::sdf::Model *_sdfModel,
    const std::unordered_map<const ::sdf::Link*, ParentInfo> &_parentOf,
    std::vector<std::pair<const ::sdf::Link*, const ::sdf::Model*>> &_rootLinks)
{
  for (std::size_t i = 0; i < _sdfModel->LinkCount(); ++i)
  {
    const auto *link = _sdfModel->LinkByIndex(i);
    auto parentOfIt = _parentOf.find(link);
    if (parentOfIt == _parentOf.end() || !parentOfIt->second.parent)
    {
      // If there is not parent or parent is null (world),
      // this link must be a root.
      _rootLinks.push_back({link, _sdfModel});
    }
  }

  for (std::size_t i = 0; i < _sdfModel->ModelCount(); ++i)
  {
    const auto *model = _sdfModel->ModelByIndex(i);
    findRootLinks(model, _parentOf, _rootLinks);
  }
}

/////////////////////////////////////////////////
/// \brief Build a structure for each root link
/// \param[in] _rootLink Root link in a model tree
/// \param[in] _parentOf A map of child link to its parent
/// \param[in] _linkTree A map of parent link to its child links
std::optional<Structure> buildStructure(
    const ::sdf::Link * _rootLink,
    const ::sdf::Model * _model,
    const std::unordered_map<const ::sdf::Link*, ParentInfo> &_parentOf,
    std::unordered_map<const ::sdf::Link*, std::vector<const ::sdf::Link*>>
    &_linkTree)
{
  bool fixed = false;
  const ::sdf::Joint *rootJoint = nullptr;
  // rootJoint only exists if root link is connected to world by a joint
  auto linkIt = _parentOf.find(_rootLink);
  if (linkIt != _parentOf.end())
  {
    const auto &parentInfo = linkIt->second;
    rootJoint = parentInfo.joint;
    fixed = true;
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
    if (link != _rootLink)
    {
      flatLinks.push_back(link);
    }
    if (auto it = _linkTree.find(link); it != _linkTree.end())
    {
      for (const auto &childLink : it->second)
      {
        flattenLinkTree(childLink);
      }
    }
  };
  flattenLinkTree(_rootLink);

  btScalar mass;
  btVector3 inertia;
  math::Pose3d linkToPrincipalAxesPose;
  extractInertial(_rootLink->Inertial(), mass, inertia,
                  linkToPrincipalAxesPose);

  // Uncomment to debug structure
  // std::cout << "Structure: " << std::endl;
  // std::cout << "  model:  " << _model->Name() << std::endl;
  // std::cout << "  root link:  " << _rootLink->Name() << std::endl;
  // std::cout << "  root joint:  " << ((rootJoint) ? rootJoint->Name() : "N/A")
  //           << std::endl;
  // std::cout << "  mass: " << mass << std::endl;
  // std::cout << "  fixed:  " << fixed << std::endl;
  // std::cout << "  flatLinks size:  " << flatLinks.size() << std::endl;

  return Structure{
    _rootLink, _model, rootJoint, mass, inertia, linkToPrincipalAxesPose, fixed,
    _parentOf, flatLinks};
}

/////////////////////////////////////////////////
/// \brief Validate input model SDF and build a vector of structures.
/// Each structure represents a single kinematic tree in the model
/// \param[in] _sdfModel Input model SDF
/// \return A vector of structures
std::vector<Structure> validateModel(const ::sdf::Model &_sdfModel)
{
  // A map of child link and its parent info
  std::unordered_map<const ::sdf::Link*, ParentInfo> parentOf;

  // A map of parent link to a vector of its child links
  std::unordered_map<const ::sdf::Link*, std::vector<const ::sdf::Link*>>
    linkTree;

  // A vector of root link of its parent model
  // Use a vector to preseve order of entities defined in sdf
  std::vector<std::pair<const ::sdf::Link*, const ::sdf::Model*>> rootLinks;

  buildTrees(&_sdfModel, parentOf, linkTree);
  findRootLinks(&_sdfModel, parentOf, rootLinks);

  std::vector<Structure> structures;
  if (rootLinks.empty())
  {
    // No root link was found in this model
    gzerr << "No root links are found in this model" << std::endl;
    return structures;
  }

  // Build subtrees
  for (const auto &rootLinkIt : rootLinks)
  {
    auto structure = buildStructure(rootLinkIt.first, rootLinkIt.second,
        parentOf, linkTree);
    if (structure.has_value())
    {
      structures.push_back(*structure);
    }
  }

  return structures;
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
  // The ConstructSdfModelImpl function constructs the entire sdf model tree,
  // including nested models So return the nested model identity if it is
  // constructed already
  auto wIt = this->worlds.find(_parentID);
  if (wIt == this->worlds.end())
  {
    auto mIt = this->models.find(_parentID);
    std::size_t nestedModelID = mIt->second->nestedModelNameToEntityId.at(
        _sdfModel.Name());
    return this->GenerateIdentity(nestedModelID,
                                  this->models.at(nestedModelID));
  }

  auto structures = validateModel(_sdfModel);
  if (structures.empty())
    return this->GenerateInvalidId();

  if (structures.size() > 1)
  {
    // multiple sub-trees detected in model
    // \todo(iche033) support multiple kinematic trees and
    // multiple floating links in a single model
    // https://github.com/gazebosim/gz-physics/issues/550
    gzerr << "Multiple sub-trees / floating links detected in a model. "
          << "This is not supported in bullet-featherstone implementation yet."
          << std::endl;
  }
  // Create model for the first structure.
  auto &structure = structures[0];

  const bool isStatic = _sdfModel.Static();
  WorldInfo *world = nullptr;
  const auto rootInertialToLink =
    gz::math::eigen3::convert(structure.linkToPrincipalAxesPose).inverse();

  // A map of link sdf to parent model identity
  std::unordered_map<const ::sdf::Link*, std::size_t> linkParentModelIds;

  std::unordered_map<const ::sdf::Model*, Identity> modelIDs;
  std::size_t rootModelID = 0u;
  std::shared_ptr<btMultiBody> rootMultiBody;
  // Add all  models, including nested models
  auto addModels = [&](std::size_t _modelOrWorldID, const ::sdf::Model *_model,
                       auto &&_addModels)
    {
      if (!_model)
        return false;

      auto worldIt = this->worlds.find(_modelOrWorldID);
      const bool isNested = worldIt == this->worlds.end();
      auto modelID = [&] {
        if (isNested)
        {
          auto modelIt = this->models.find(_modelOrWorldID);
          auto worldIdentity = modelIt->second->world;
          auto modelIdentity =
              this->GenerateIdentity(_modelOrWorldID, modelIt->second);
          return this->AddNestedModel(
              _model->Name(), modelIdentity, worldIdentity, rootInertialToLink,
              rootMultiBody);
        }
        else
        {
          auto worldIdentity = this->GenerateIdentity(
              _modelOrWorldID, worldIt->second);
          rootMultiBody = std::make_shared<btMultiBody>(
                static_cast<int>(structure.flatLinks.size()),
                structure.mass,
                structure.inertia,
                structure.fixedBase || isStatic,
                true);
          world = this->ReferenceInterface<WorldInfo>(worldIdentity);
          auto id = this->AddModel(
              _model->Name(), worldIdentity, rootInertialToLink,
              rootMultiBody);
          rootModelID = id;
          return id;
        }
      }();

      // build link to model map for use later when adding links
      for (std::size_t i = 0; i < _model->LinkCount(); ++i)
      {
        const ::sdf::Link *link = _model->LinkByIndex(i);
        linkParentModelIds[link] = modelID;
      }
      modelIDs.insert(std::make_pair(_model, modelID));

      // recursively add nested models
      for (std::size_t i = 0; i < _model->ModelCount(); ++i)
      {
        if (!_addModels(modelID, _model->ModelByIndex(i), _addModels))
          return false;
      }
      return true;
    };
  if (!addModels(_parentID, &_sdfModel, addModels))
  {
    return this->GenerateInvalidId();
  }

  // Add base link
  std::size_t baseLinkModelID = linkParentModelIds.at(structure.rootLink);
  const auto rootID =
    this->AddLink(LinkInfo{
      structure.rootLink->Name(), std::nullopt,
      this->GenerateIdentity(baseLinkModelID, this->models.at(baseLinkModelID)),
      rootInertialToLink
    });
  rootMultiBody->setUserIndex(std::size_t(rootID));

  auto modelID =
      this->GenerateIdentity(rootModelID, this->models[rootModelID]);
  const auto *model = this->ReferenceInterface<ModelInfo>(modelID);

  // Add world joint
  if (structure.rootJoint)
  {
    const auto &parentInfo = structure.parentOf.at(structure.rootLink);
    this->AddJoint(
          JointInfo{
            structure.rootJoint->Name(),
            RootJoint{},
            std::nullopt,
            rootID,
            Eigen::Isometry3d::Identity(),
            Eigen::Isometry3d::Identity(),
            modelIDs.find(parentInfo.model)->second
          });
  }

  model->body->setLinearDamping(0.0);
  model->body->setAngularDamping(0.0);

  std::unordered_map<const ::sdf::Link*, Identity> linkIDs;
  linkIDs.insert(std::make_pair(structure.rootLink, rootID));

  // Add all links and joints
  for (int i = 0; i < static_cast<int>(structure.flatLinks.size()); ++i)
  {
    const auto *link = structure.flatLinks[static_cast<std::size_t>(i)];
    btScalar mass;
    btVector3 inertia;
    math::Pose3d linkToPrincipalAxesPose;
    extractInertial(link->Inertial(), mass, inertia, linkToPrincipalAxesPose);
    const Eigen::Isometry3d linkToComTf = gz::math::eigen3::convert(
          linkToPrincipalAxesPose);

    if (linkIDs.find(link) == linkIDs.end())
    {
      std::size_t parentModelID = linkParentModelIds[link];
      const auto linkID = this->AddLink(
        LinkInfo{link->Name(), i,
        this->GenerateIdentity(parentModelID, this->models.at(parentModelID)),
        linkToComTf.inverse()});
      linkIDs.insert(std::make_pair(link, linkID));
    }

    if (!structure.parentOf.empty())
    {
      const auto &parentInfo = structure.parentOf.at(link);
      const auto *joint = parentInfo.joint;
      std::string parentLinkName;
      ::sdf::Errors errors = joint->ResolveParentLink(parentLinkName);
      if (!errors.empty())
      {
        parentLinkName = joint->ParentName();
      }
      const auto &parentLinkID = linkIDs.at(
        parentInfo.model->LinkByName(parentLinkName));
      const auto *parentLinkInfo = this->ReferenceInterface<LinkInfo>(
        parentLinkID);

      int parentIndex = -1;
      if (parentLinkInfo->indexInModel.has_value())
        parentIndex = *parentLinkInfo->indexInModel;

      Eigen::Isometry3d poseParentLinkToJoint;
      Eigen::Isometry3d poseParentComToJoint;
      {
        gz::math::Pose3d gzPoseParentToJoint;
        errors = resolveJointPoseRelToLink(gzPoseParentToJoint,
            parentInfo.model, joint->Name(), parentLinkName);

        if (!errors.empty())
        {
          gzerr << "An error occurred while resolving the transform of Joint ["
                << joint->Name() << "] in Model [" << parentInfo.model->Name()
                << "]:\n";
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
        gz::math::Pose3d gzPoseChildToJoint;
        // this retrieves the joint pose relative to link
        std::string childLinkName;
        errors = joint->ResolveChildLink(childLinkName);
        if (!errors.empty())
        {
          childLinkName = joint->ChildName();
        }
        errors = resolveJointPoseRelToLink(gzPoseChildToJoint,
            parentInfo.model, joint->Name(), childLinkName);

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
        gz::math::Pose3d gzPoseJointToChild;
        gzPoseJointToChild = gzPoseChildToJoint.Inverse();
        poseJointToChild = gz::math::eigen3::convert(gzPoseJointToChild);
      }

      btQuaternion btRotParentComToJoint;
      convertMat(poseParentComToJoint.linear())
        .getRotation(btRotParentComToJoint);

      btTransform parentLocalInertialFrame = convertTf(
        parentLinkInfo->inertiaToLinkFrame);
      btTransform parent2jointBt = convertTf(
        poseParentLinkToJoint);  // X_PJ

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
          linkIDs.find(parentInfo.parent)->second,
          linkIDs.find(link)->second,
          poseParentLinkToJoint,
          poseJointToChild,
          modelIDs.find(parentInfo.model)->second
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
        // Note: These m_joint* properties below are currently not supported by
        // bullet-featherstone and so setting them does not have any effect.
        // The lower and uppper limit is supported via the
        // btMultiBodyJointLimitConstraint.
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

        jointInfo->minEffort = -joint->Axis()->Effort();
        jointInfo->maxEffort = joint->Axis()->Effort();
        jointInfo->minVelocity = -joint->Axis()->MaxVelocity();
        jointInfo->maxVelocity = joint->Axis()->MaxVelocity();
        jointInfo->axisLower = joint->Axis()->Lower();
        jointInfo->axisUpper = joint->Axis()->Upper();

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

  auto modelToNestedModel = Eigen::Isometry3d::Identity();
  // if the model of the root link is nested in a top level model, compute
  // its pose relative to top level model
  if (structure.model != &_sdfModel)
  {
    // get the scoped name of the model
    // This is used to resolve the model pose
    std::string modelScopedName;
    auto getModelScopedName = [&](const ::sdf::Model *_targetModel,
      const ::sdf::Model *_parentModel, const std::string &_prefix,
      auto &&_getModelScopedName)
    {
      for (std::size_t i = 0u; i < _parentModel->ModelCount(); ++i)
      {
        auto childModel = _parentModel->ModelByIndex(i);
        if (childModel == _targetModel)
        {
          modelScopedName = _prefix + childModel->Name();
          return true;
        }
        else
        {
          if (_getModelScopedName(_targetModel, childModel,
              _prefix + childModel->Name() + "::", _getModelScopedName))
            return true;
        }
      }
      return false;
    };

    math::Pose3d modelPose;
    if (!getModelScopedName(structure.model, &_sdfModel,
        _sdfModel.Name() + "::", getModelScopedName))
      return this->GenerateInvalidId();

    auto errors = _sdfModel.SemanticPose().Resolve(modelPose,
        modelScopedName);
    if (!errors.empty())
      return this->GenerateInvalidId();
    modelToNestedModel = math::eigen3::convert(modelPose).inverse();
  }

  const auto modelToRootLink =
    ResolveSdfPose(structure.rootLink->SemanticPose());
  if (!modelToRootLink)
    return this->GenerateInvalidId();

  const auto worldToRootCom =
    *worldToModel * modelToNestedModel * *modelToRootLink *
    rootInertialToLink.inverse();

  model->body->setBaseWorldTransform(convertTf(worldToRootCom));
  model->body->setBaseVel(btVector3(0, 0, 0));
  model->body->setBaseOmega(btVector3(0, 0, 0));

  {
    const auto *link = structure.rootLink;
    btScalar mass;
    btVector3 inertia;
    math::Pose3d linkToPrincipalAxesPose;
    extractInertial(link->Inertial(), mass, inertia, linkToPrincipalAxesPose);
    model->body->setBaseMass(mass);
    model->body->setBaseInertia(inertia);
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

  // Add the remaining links in the model without constructing the bullet
  // objects. These are dummy links for book-keeping purposes
  // \todo(iche033) The code will need to be updated when multiple subtrees in
  // a single model is supported.
  for (std::size_t i = 1u; i < structures.size(); ++i)
  {
    auto otherStructure = structures[i];
    // Add base link
    std::size_t rootLinkModelID = linkParentModelIds[otherStructure.rootLink];
    auto rootLinkModelInfo = this->models.at(rootLinkModelID);
    this->AddLink(LinkInfo{
      otherStructure.rootLink->Name(),
      std::nullopt,
      this->GenerateIdentity(rootLinkModelID, rootLinkModelInfo),
      gz::math::eigen3::convert(
      otherStructure.linkToPrincipalAxesPose).inverse()
    });
    gzwarn << "Floating body / sub-tree detected. Disabling link: '"
           << otherStructure.rootLink->Name() << "' "
           << "from model '" << rootLinkModelInfo->name << "'." << std::endl;
    // other links
    for (int j = 0; j < static_cast<int>(otherStructure.flatLinks.size()); ++j)
    {
      const auto *link = otherStructure.flatLinks[static_cast<std::size_t>(j)];
      std::size_t parentModelID = linkParentModelIds[link];
      btScalar mass;
      btVector3 inertia;
      math::Pose3d linkToPrincipalAxesPose;
      extractInertial(link->Inertial(), mass, inertia, linkToPrincipalAxesPose);
      auto parentModelInfo = this->models.at(parentModelID);
      const auto linkID = this->AddLink(
          LinkInfo{link->Name(), std::nullopt,
          this->GenerateIdentity(parentModelID, parentModelInfo),
          gz::math::eigen3::convert(linkToPrincipalAxesPose).inverse()});
      gzwarn << "Floating body / sub-tree detected. Disabling link: '"
             << link->Name() << "' " << "from model '" << parentModelInfo->name
             << "'." << std::endl;

    }
  }

  return modelID;
}

/////////////////////////////////////////////////
Identity SDFFeatures::ConstructSdfModel(
    const Identity &_worldID,
    const ::sdf::Model &_sdfModel)
{
  return this->ConstructSdfModelImpl(_worldID, _sdfModel);
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
    if (nullptr == mesh)
    {
      gzwarn << "Failed to load mesh from [" << meshSdf->Uri()
             << "]." << std::endl;
      return false;
    }
    const btVector3 scale = convertVec(meshSdf->Scale());

    auto compoundShape = std::make_unique<btCompoundShape>();

    bool meshCreated = false;
    if (meshSdf->Optimization() ==
        ::sdf::MeshOptimization::CONVEX_DECOMPOSITION ||
        meshSdf->Optimization() ==
        ::sdf::MeshOptimization::CONVEX_HULL)
    {
      std::size_t maxConvexHulls = 16u;
      std::size_t voxelResolution = 200000u;
      if (meshSdf->ConvexDecomposition())
      {
        // limit max number of convex hulls to generate
        maxConvexHulls = meshSdf->ConvexDecomposition()->MaxConvexHulls();
        voxelResolution = meshSdf->ConvexDecomposition()->VoxelResolution();
      }
      if (meshSdf->Optimization() == ::sdf::MeshOptimization::CONVEX_HULL)
      {
        /// create 1 convex hull for the whole submesh
        maxConvexHulls = 1u;
      }

      // Check if MeshManager contains the decomposed mesh already. If not
      // add it to the MeshManager so we do not need to decompose it again.
      const std::string convexMeshName =
          mesh->Name() + "_" + meshSdf->Submesh() + "_CONVEX_" +
          std::to_string(maxConvexHulls) + "_" +
          std::to_string(voxelResolution);
      auto *decomposedMesh = meshManager.MeshByName(convexMeshName);
      if (!decomposedMesh)
      {
        // Merge meshes before convex decomposition
        auto mergedMesh = gz::common::MeshManager::MergeSubMeshes(*mesh);
        if (mergedMesh && mergedMesh->SubMeshCount() == 1u)
        {
          // Decompose and add mesh to MeshManager
          auto mergedSubmesh = mergedMesh->SubMeshByIndex(0u).lock();
          std::vector<common::SubMesh> decomposed =
            gz::common::MeshManager::ConvexDecomposition(
            *mergedSubmesh.get(), maxConvexHulls, voxelResolution);
          gzdbg << "Optimizing mesh (" << meshSdf->OptimizationStr() << "): "
                <<  mesh->Name() << std::endl;
          // Create decomposed mesh and add it to MeshManager
          // Note: MeshManager will call delete on this mesh in its destructor
          // \todo(iche033) Consider updating MeshManager to accept
          // unique pointers instead
          common::Mesh *convexMesh = new common::Mesh;
          convexMesh->SetName(convexMeshName);
          for (const auto & submesh : decomposed)
            convexMesh->AddSubMesh(submesh);
          meshManager.AddMesh(convexMesh);
          if (decomposed.empty())
          {
            // Print an error if convex decomposition returned empty submeshes
            // but still add it to MeshManager to avoid going through the
            // expensive convex decomposition process for the same mesh again
            gzerr << "Convex decomposition generated zero meshes: "
                   << mesh->Name() << std::endl;
          }
          decomposedMesh = meshManager.MeshByName(convexMeshName);
        }
      }

      if (decomposedMesh)
      {
        for (std::size_t j = 0u; j < decomposedMesh->SubMeshCount(); ++j)
        {
          auto submesh = decomposedMesh->SubMeshByIndex(j).lock();
          gz::math::Vector3d centroid;
          for (std::size_t i = 0; i < submesh->VertexCount(); ++i)
              centroid += submesh->Vertex(i);
          centroid *= 1.0/static_cast<double>(submesh->VertexCount());
          btAlignedObjectArray<btVector3> vertices;
          for (std::size_t i = 0; i < submesh->VertexCount(); ++i)
          {
            gz::math::Vector3d v = submesh->Vertex(i) - centroid;
            vertices.push_back(convertVec(v) * scale);
          }

          float collisionMargin = 0.001f;
          this->meshesConvex.push_back(std::make_unique<btConvexHullShape>(
              &(vertices[0].getX()), vertices.size()));
          auto *convexShape = this->meshesConvex.back().get();
          convexShape->setMargin(collisionMargin);
          convexShape->recalcLocalAabb();
          convexShape->optimizeConvexHull();

          btTransform trans;
          trans.setIdentity();
          trans.setOrigin(convertVec(centroid) * scale);
          compoundShape->addChildShape(trans, convexShape);
        }
        meshCreated = true;
      }
    }

    if (!meshCreated)
    {
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
        this->meshesGImpact.back()->setMargin(btScalar(0.01));
        compoundShape->addChildShape(btTransform::getIdentity(),
          this->meshesGImpact.back().get());
      }
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
  double torsionalCoefficient = 1.0;
  double rollingFriction = 0.0;
  if (const auto *surface = _collision.Surface())
  {
    if (const auto *friction = surface->Friction())
    {
      if (const auto frictionElement = friction->Element())
      {
        if (const auto bullet = frictionElement->FindElement("bullet"))
        {
          if (const auto f1 = bullet->FindElement("friction"))
            mu = f1->Get<double>();

          if (const auto f2 = bullet->FindElement("friction2"))
            mu2 = f2->Get<double>();

          // What is fdir1 for in the SDF's <bullet> spec?

          if (const auto rolling = bullet->FindElement("rolling_friction"))
            rollingFriction = rolling->Get<double>();
        }
        if (const auto torsional = frictionElement->FindElement("torsional"))
        {
          if (const auto coefficient = torsional->FindElement("coefficient"))
            torsionalCoefficient = coefficient->Get<double>();
        }
      }
    }

    if (const auto surfaceElement = surface->Element())
    {
      if (const auto bounce = surfaceElement->FindElement("bounce"))
      {
        if (const auto r = bounce->FindElement("restitution_coefficient"))
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
      linkInfo->collider = std::make_unique<GzMultiBodyLinkCollider>(
        model->body.get(), linkIndexInModel);

      linkInfo->shape->addChildShape(btInertialToCollision, shape.get());

      linkInfo->collider->setCollisionShape(linkInfo->shape.get());
      linkInfo->collider->setRestitution(static_cast<btScalar>(restitution));
      linkInfo->collider->setRollingFriction(
        static_cast<btScalar>(rollingFriction));
      linkInfo->collider->setSpinningFriction(
        static_cast<btScalar>(torsionalCoefficient));
      linkInfo->collider->setFriction(static_cast<btScalar>(mu));
      linkInfo->collider->setAnisotropicFriction(
        btVector3(static_cast<btScalar>(mu), static_cast<btScalar>(mu2), 1),
        btCollisionObject::CF_ANISOTROPIC_FRICTION);

      if (geom->MeshShape())
      {
        // Set meshes to use softer contacts for stability
        // \todo(iche033) load <kp> and <kd> values from SDF
        const btScalar kp = btScalar(1e15);
        const btScalar kd = btScalar(1e14);
        linkInfo->collider->setContactStiffnessAndDamping(kp, kd);
      }

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

      // Set static filter for collisions in
      // 1) a static model
      // 2) a fixed base link
      // 3) a (non-base) link with zero dofs
      bool isFixed = false;
      if (model->body->hasFixedBase())
      {
        // check if it's a base link
        isFixed = std::size_t(_linkID) ==
            static_cast<std::size_t>(model->body->getUserIndex());
        // check if link has zero dofs
        if (!isFixed && linkInfo->indexInModel.has_value())
        {
           isFixed = model->body->getLink(
              linkInfo->indexInModel.value()).m_dofCount == 0;
        }
      }
      if (isStatic || isFixed)
      {
        world->world->addCollisionObject(
          linkInfo->collider.get(),
          btBroadphaseProxy::StaticFilter,
          btBroadphaseProxy::AllFilter ^ btBroadphaseProxy::StaticFilter);
          linkInfo->isStaticOrFixed = true;

        // Set collider collision flags
#if BT_BULLET_VERSION >= 307
        linkInfo->collider->setDynamicType(btCollisionObject::CF_STATIC_OBJECT);
#endif
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

/////////////////////////////////////////////////
Identity SDFFeatures::ConstructSdfJoint(
    const Identity &_modelID,
    const ::sdf::Joint &_sdfJoint)
{
#if BT_BULLET_VERSION < 289
  // The btMultiBody::setFixedBase function is only available in versions
  // >= 2.89. This is needed for dynamically creating world joints,
  // i.e. setting the btMultiBody to be fixed. So output an error letting
  // users know the joint will not be created.
  // \todo(iche033) A workaround for this is to loop through all the joints
  // in the world first in ConstructSdfWorld, keep track of the models who are
  // a child of the world joint, then when creating the btMultiBody
  // in ConstructSdfModelImpl, pass fixedBase as true in its constructor.
  (void)_modelID;
  gzerr << "ConstructSdfJoint feature is not supported for bullet version "
        << "less than 2.89. Joint '" << _sdfJoint.Name() << "' will not "
        << "be created." << std::endl;
#else
  auto modelInfo = this->ReferenceInterface<ModelInfo>(_modelID);
  if (_sdfJoint.ChildName() == "world")
  {
    gzerr << "Asked to create a joint with the world as the child in model "
           << "[" << modelInfo->name << "]. This is currently not "
           << "supported\n";

    return this->GenerateInvalidId();
  }

  std::string parentLinkName;
  const auto resolveParentErrors = _sdfJoint.ResolveParentLink(parentLinkName);
  if (!resolveParentErrors.empty()) {
    parentLinkName = _sdfJoint.ParentName();
  }
  std::string childLinkName;
  const auto childResolveErrors = _sdfJoint.ResolveChildLink(childLinkName);
  if (!childResolveErrors.empty()) {
    childLinkName = _sdfJoint.ChildName();
  }

  // Currently only supports constructing fixed joint with world as parent
  if (parentLinkName == "world" && _sdfJoint.Type() == ::sdf::JointType::FIXED)
  {
    auto worldModelIt = this->models.find(_modelID);
    if (worldModelIt == this->models.end())
      return this->GenerateInvalidId();
    const auto worldModel = worldModelIt->second;
    std::size_t idx = childLinkName.find("::");
    if (idx == std::string::npos)
      return this->GenerateInvalidId();

    const std::string modelName = childLinkName.substr(0, idx);
    std::size_t modelID = worldModel->nestedModelNameToEntityId.at(modelName);
    auto model = this->models.at(modelID);

    model->body->setFixedBase(true);
    std::size_t linkID = model->body->getUserIndex();
    auto rootID = this->GenerateIdentity(linkID, this->links.at(linkID));
    return this->AddJoint(
          JointInfo{
            _sdfJoint.Name(),
            RootJoint{},
            std::nullopt,
            rootID,
            Eigen::Isometry3d::Identity(),
            Eigen::Isometry3d::Identity(),
            _modelID
          });
  }

  // \todo(iche033) Support fixed joint between 2 different models
  gzerr << "Unable to create joint between parent: " << parentLinkName << " "
        << "and child: " << childLinkName << ". "
        << "ConstructSdfJoint in bullet-featherstone implementation currently "
        << "only supports creating a fixed joint with the world as parent link."
        << std::endl;
#endif

  return this->GenerateInvalidId();
}


}  // namespace bullet_featherstone
}  // namespace physics
}  // namespace gz
