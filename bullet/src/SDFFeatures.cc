/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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
#include <ignition/math/eigen3/Conversions.hh>

#include <sdf/JointAxis.hh>
#include <sdf/Box.hh>
#include <sdf/Geometry.hh>

namespace ignition {
namespace physics {
namespace bullet {

inline btMatrix3x3 convertMat(Eigen::Matrix3d mat)
{
  return btMatrix3x3(mat(0, 0), mat(0, 1), mat(0, 2),
                     mat(1, 0), mat(1, 1), mat(1, 2),
                     mat(2, 0), mat(2, 1), mat(2, 2));
}

inline btVector3 convertVec(Eigen::Vector3d vec)
{
  return btVector3(vec(0), vec(1), vec(2));
}

/////////////////////////////////////////////////
Identity SDFFeatures::ConstructSdfWorld(
    const Identity &_engine,
    const ::sdf::World &_sdfWorld)
{
  const Identity worldID = this->ConstructEmptyWorld(_engine, _sdfWorld.Name());

  const WorldInfoPtr &worldInfo = this->worlds.at(worldID);

  auto gravity = _sdfWorld.Gravity();
  worldInfo->world->setGravity(btVector3(gravity[0], gravity[1], gravity[2]));
  return worldID;
}

Identity SDFFeatures::ConstructSdfModel(
  const Identity &_worldID,
  const ::sdf::Model &_sdfModel)
{
  // Read sdf params
  const std::string name = _sdfModel.Name();
  const auto pose = _sdfModel.Pose();
  const bool isStatic = _sdfModel.Static();
  const bool selfCollide = _sdfModel.SelfCollide();

  // Initially assume no link attached
  const int numLinks = -1;
  const btScalar baseMass = 0;
  const btVector3 baseInertiaDiag(0, 0, 0);
  const bool fixedBase = isStatic;
  const bool canSleep = false; // To Do: experiment with it

  // Set base transform
  const auto poseIsometry = ignition::math::eigen3::convert(pose);
  const auto poseTranslation = poseIsometry.translation();
  const auto poseLinear = poseIsometry.linear();
  btTransform baseTransform;
  baseTransform.setOrigin(convertVec(poseTranslation));
  baseTransform.setBasis(convertMat(poseLinear));

  // Add model params
  btMultiBody* model = nullptr;
  return this->AddModel({model, name, numLinks, baseMass, baseInertiaDiag,
                         fixedBase, canSleep, baseTransform, selfCollide,
                         false, _worldID});
}

Identity SDFFeatures::ConstructSdfLink(
  const Identity &_modelID,
  const ::sdf::Link &_sdfLink)
{
  // Read sdf params
  const std::string name = _sdfLink.Name();
  const auto pose = _sdfLink.Pose();
  const auto inertial = _sdfLink.Inertial();
  const auto mass = inertial.MassMatrix().Mass();
  const auto diagonalMoments = inertial.MassMatrix().DiagonalMoments();

  // Get link properties
  const btScalar linkMass = mass;
  const btVector3 linkInertiaDiag = convertVec(ignition::math::eigen3::convert(diagonalMoments));
  int linkIndex = 0;
  for (const auto &entry : this->links)
  {
    if (entry.second->model.id == _modelID.id)
      linkIndex += 1;
  }
  const auto poseIsometry = ignition::math::eigen3::convert(pose);

  return this->AddLink({name, linkIndex, linkMass, linkInertiaDiag,
                        poseIsometry, _modelID});
}

Identity SDFFeatures::ConstructSdfJoint(
  const Identity &_modelID,
  const ::sdf::Joint &_sdfJoint)
{
  // Read properties
  const std::string name = _sdfJoint.Name();
  const auto type = _sdfJoint.Type();
  const auto pose = _sdfJoint.Pose();
  const auto threadPitch = _sdfJoint.ThreadPitch();
  const auto parentLinkName = _sdfJoint.ParentLinkName();
  const auto childLinkName = _sdfJoint.ChildLinkName();
  const auto firstAxis = _sdfJoint.Axis(0);
  const auto secondAxis = _sdfJoint.Axis(1);

  // Find parent and child link
  LinkInfoPtr parentLinkInfo;
  LinkInfoPtr childLinkInfo;
  std::size_t parentID;
  std::size_t childID;
  for (const auto &entry : this->links)
  {
    const LinkInfoPtr &linkInfo = entry.second;
    if (linkInfo->model.id == _modelID.id)
    {
      if (linkInfo->name == parentLinkName)
      {
        parentLinkInfo = linkInfo;
        parentID = entry.first;
      }
      if (linkInfo->name == childLinkName)
      {
        childLinkInfo = linkInfo;
        childID = entry.first;
      }
    }
  }

  const bool worldParent = (!parentLinkInfo) && (parentLinkName == "world");
  if (!parentLinkInfo && !worldParent)
  {
    ignerr << "Parent link with name [" << parentLinkName << "] could not be "
           << "found in the model\n";

    return this->GenerateInvalidId();
  }
  if (!childLinkInfo)
  {
    ignerr << "Child link with name [" << childLinkName << "] could not be "
           << "found in the model\n";

    return this->GenerateInvalidId();
  }

  const int parentIndex = worldParent ? -1 : parentLinkInfo->linkIndex;
  const int childIndex = childLinkInfo->linkIndex;

  // Get child link properties
  btVector3 jointAxis1 = btVector3(0, 0, 0);
  if (firstAxis != nullptr)
  {
    jointAxis1 = btVector3(firstAxis->Xyz()[0],
                           firstAxis->Xyz()[1],
                           firstAxis->Xyz()[2]);
  }
  btVector3 jointAxis2 = btVector3(0, 0, 0);
  if (secondAxis != nullptr)
  {
    jointAxis2 = btVector3(secondAxis->Xyz()[0],
                           secondAxis->Xyz()[1],
                           secondAxis->Xyz()[2]);
  }

  const auto poseIsometry = ignition::math::eigen3::convert(pose);

  return this->AddJoint({name, type, childIndex, parentIndex, jointAxis1,
                         jointAxis2, poseIsometry, childID, parentID,
                         _modelID});
}

Identity SDFFeatures::ConstructSdfCollision(
      const Identity &_linkID,
      const ::sdf::Collision &_collision)
{
  if (!_collision.Geom())
  {
    ignerr << "The geometry element of collision [" << _collision.Name() << "] "
           << "was a nullptr\n";
    return this->GenerateInvalidId();
  }

  const auto &geom = _collision.Geom();
  btCollisionShape* shape = nullptr;
  bool isDynamic = true;
  btTransform transform = btTransform().getIdentity();

  if (geom->BoxShape())
  {
    const auto box = geom->BoxShape();
    const auto size = math::eigen3::convert(box->Size());
    const auto halfExtents = convertVec(size)*0.5;
    shape = new btBoxShape(halfExtents);
  }
  else if (geom->SphereShape())
  {
    const auto sphere = geom->SphereShape();
    const auto radius = sphere->Radius();
    shape = new btSphereShape(radius);
  }
  else if (geom->CylinderShape())
  {
    const auto cylinder = geom->CylinderShape();
    const auto radius = cylinder->Radius();
    const auto halfLength = cylinder->Length()*0.5;
    shape = new btCylinderShapeZ(btVector3(radius, 0, halfLength));
  }
  else if (geom->PlaneShape())
  {
    const auto plane = geom->PlaneShape();
    const auto normal = convertVec(math::eigen3::convert(plane->Normal()));
    shape = new btStaticPlaneShape(normal, 0);
    isDynamic = false;
  }

  // Get friction
  const auto &odeFriction = _collision.Element()
                                ->GetElement("surface")
                                ->GetElement("friction")
                                ->GetElement("ode");
  const auto mu = odeFriction->Get<double>("mu");

  if (shape != nullptr)
  {
    const auto &modelID = this->links.at(_linkID)->model;
    return this->AddCollision({shape, nullptr, transform, mu, isDynamic,
                               _linkID, modelID});
  }
}

void SDFFeatures::FinalizeSdfModels(
  const Identity &/*_engine*/)
{
  for (const auto &modelEntry : this->models)
  {
    const auto &modelInfo = modelEntry.second;
    if (!modelInfo->finalized)
    {
      int numLinks = 0;
      for (const auto &linkEntry : this->links)
      {
        if (linkEntry.second->model.id == modelEntry.first)
          numLinks += 1;
      }
      modelInfo->numLinks = numLinks;

      // Find if has fixed world joint
      for (const auto &jointEntry : this->joints)
      {
        if (jointEntry.second->model.id == modelEntry.first &&
            jointEntry.second->parentIndex == -1)
        {
          modelInfo->fixedBase = true;
          break;
        }
      }

      modelInfo->model = new btMultiBody(modelInfo->numLinks,
                                         modelInfo->baseMass,
                                         modelInfo->baseInertiaDiag,
                                         modelInfo->fixedBase,
                                         modelInfo->canSleep);
      const auto &model = modelInfo->model;
      model->setBaseWorldTransform(modelInfo->baseTransform);

      model->setHasSelfCollision(modelInfo->selfCollide);
      model->setUseGyroTerm(true);

      model->setLinearDamping(0);
      model->setAngularDamping(0);

      const auto &world = this->worlds.at(modelInfo->world)->world;
      world->addMultiBody(model);

      // Add fixed joints to links unless replaced by joints
      for (const auto &linkEntry : this->links)
      {
        const auto &linkInfo = linkEntry.second;
        if (linkInfo->model.id == modelEntry.first)
        {
          // Find translation
          const btVector3 parentComToCurrentCom = convertVec(
            linkInfo->poseIsometry.translation());
          const btVector3 currentPivotToCurrentCom(0, 0, 0);
          const btVector3 parentComToCurrentPivot = parentComToCurrentCom -
            currentPivotToCurrentCom;
          // Find rotation
          btQuaternion rotParentToThis;
          const btMatrix3x3 mat = convertMat(linkInfo->poseIsometry.linear());
          mat.getRotation(rotParentToThis);

          // Set up link
          const int parentIndex = -1;
          model->setupFixed(linkInfo->linkIndex, linkInfo->linkMass,
                            linkInfo->linkInertiaDiag, parentIndex,
                            rotParentToThis, parentComToCurrentPivot,
                            currentPivotToCurrentCom);
        }
      }

      // Add joints
      for (const auto &jointEntry : this->joints)
      {
        const auto &jointInfo = jointEntry.second;
        if (jointInfo->model.id == modelEntry.first)
        {
          LinkInfoPtr childLinkInfo = this->links.at(jointInfo->childID);
          LinkInfoPtr parentLinkInfo = jointInfo->parentIndex == -1 ?
                            nullptr : this->links.at(jointInfo->parentID);

          // Obtain translation and rotation in world frame
          const btVector3 worldComToCurrentCom = convertVec(
            childLinkInfo->poseIsometry.translation());
          btVector3 worldComToParentCom = btVector3(0, 0, 0);
          if (parentLinkInfo != nullptr)
          {
            worldComToParentCom = convertVec(
              parentLinkInfo->poseIsometry.translation());
          }
          const btMatrix3x3 matWorldToThis = convertMat(
            childLinkInfo->poseIsometry.linear());
          btMatrix3x3 matWorldToParent = btMatrix3x3().getIdentity();
          if (parentLinkInfo != nullptr)
          {
            matWorldToParent = convertMat(
              parentLinkInfo->poseIsometry.linear());
          }

          const btMatrix3x3 matParentToThis = matWorldToParent.inverse() *
                                              matWorldToThis;

          // Obtain translation from parent to child
          btVector3 parentComToCurrentCom = worldComToCurrentCom -
                                                worldComToParentCom;
          parentComToCurrentCom = matWorldToParent.inverse() *
                                    parentComToCurrentCom;

          const btVector3 currentComToCurrentPivot = convertVec(
            jointInfo->poseIsometry.translation());
          const btVector3 parentComToCurrentPivot = parentComToCurrentCom +
                  matParentToThis * currentComToCurrentPivot;
          // Expressed in child frame
          const btVector3 currentPivotToCurrentCom = -currentComToCurrentPivot;

          // Obtain rotation that expresses vectors from parent frame in child
          // frame, which is different from the rotation that rotates vectors
          // from parent to child frame, hence the inverse().
          btQuaternion rotParentToThis;
          matParentToThis.inverse().getRotation(rotParentToThis);

          // Set up joints
          if (::sdf::JointType::REVOLUTE == jointInfo->type)
          {
            model->setupRevolute(jointInfo->childIndex, childLinkInfo->linkMass,
              childLinkInfo->linkInertiaDiag, jointInfo->parentIndex,
              rotParentToThis, jointInfo->axis1, parentComToCurrentPivot,
              currentPivotToCurrentCom, !model->hasSelfCollision());
          }
          else if (::sdf::JointType::BALL == jointInfo->type)
          {
            model->setupSpherical(jointInfo->childIndex,
              childLinkInfo->linkMass, childLinkInfo->linkInertiaDiag,
              jointInfo->parentIndex, rotParentToThis, parentComToCurrentPivot,
              currentPivotToCurrentCom, !model->hasSelfCollision());
          }
        }
      }

      // Add collisions
      for (const auto &collisionEntry : this->collisions)
      {
        const auto &collisionInfo = collisionEntry.second;
        const auto &linkInfo = this->links.at(collisionInfo->link);
        if (linkInfo->model.id == modelEntry.first)
        {
          const auto &WorldInfo = this->worlds.at(modelInfo->world);

          btMultiBodyLinkCollider* col = new btMultiBodyLinkCollider(model, linkInfo->linkIndex);
          collisionInfo->collider = col;
          col->setCollisionShape(collisionInfo->shape);
          col->setWorldTransform(collisionInfo->transform);
          col->setFriction(collisionInfo->mu);
          bool isDynamic = 1;
          int collisionFilterGroup = isDynamic ? int(btBroadphaseProxy::DefaultFilter) : int(btBroadphaseProxy::StaticFilter);
          int collisionFilterMask = isDynamic ? int (btBroadphaseProxy::AllFilter) : int(btBroadphaseProxy::AllFilter ^ btBroadphaseProxy::StaticFilter);

          WorldInfo->world->addCollisionObject(col, collisionFilterGroup, collisionFilterMask);
          model->getLink(linkInfo->linkIndex).m_collider = col;
        }
      }
      btAlignedObjectArray<btQuaternion> scratch_q;
		  btAlignedObjectArray<btVector3> scratch_m;
		  model->forwardKinematics(scratch_q, scratch_m);
		  btAlignedObjectArray<btQuaternion> world_to_local;
		  btAlignedObjectArray<btVector3> local_origin;
		  model->updateCollisionObjectWorldTransforms(world_to_local, local_origin);

      model->finalizeMultiDof();
      modelInfo->finalized = true;
    }
  }
}

}
}
}
