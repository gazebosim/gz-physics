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

#include "FreeGroupFeatures.hh"

#include <dart/constraint/ConstraintSolver.hpp>
#include <dart/dynamics/FreeJoint.hpp>

namespace ignition {
namespace physics {
namespace dartsim {

/////////////////////////////////////////////////
Identity FreeGroupFeatures::FindFreeGroupForModel(
    const Identity &_modelID) const
{
  // Verify that the model qualifies as a FreeGroup
  const dart::dynamics::ConstSkeletonPtr &skeleton =
      this->models.at(_modelID)->model;

  // If there are no bodies at all in this model, then the FreeGroup functions
  // will not work properly, so we'll just reject these cases.
  if (skeleton->getNumBodyNodes() == 0)
    return this->GenerateInvalidId();

  // Verify that all root joints are FreeJoints
  for (std::size_t i = 0; i < skeleton->getNumTrees(); ++i)
  {
    if (skeleton->getRootJoint(i)->getType()
        != dart::dynamics::FreeJoint::getStaticType())
    {
      return this->GenerateInvalidId();
    }
  }

  // TODO(MXG): When the dartsim plugin supports closed-loop constraints, verify
  // that this model is not attached to the world or any other models. If it's
  // attached to anything external, then we should return an invalid identity.

  return _modelID;
}

/////////////////////////////////////////////////
Identity FreeGroupFeatures::FindFreeGroupForLink(
    const Identity &_linkID) const
{
  const dart::dynamics::BodyNode* bn = this->links.at(_linkID)->link;

  // Move towards the root of the tree looking for a FreeJoint
  while (bn)
  {
    if (bn->getParentJoint()->getType()
        == dart::dynamics::FreeJoint::getStaticType())
    {
      break;
    }

    bn = bn->getParentBodyNode();
  }

  if (bn == nullptr)
    return this->GenerateInvalidId();

  // TODO(MXG): When the dartsim plugin supports closed-loop constraints, verify
  // that this sub-tree does not have any constraints that attach it to any
  // links outside of the tree.
  return this->GenerateIdentity(this->links.IdentityOf(bn));
}

/////////////////////////////////////////////////
Identity FreeGroupFeatures::GetFreeGroupUnconstrainedLink(
    const Identity &_groupID) const
{
  const FreeGroupInfo &info = GetCanonicalInfo(_groupID);
  return this->GenerateIdentity(this->links.IdentityOf(info.link));
}

/////////////////////////////////////////////////
FreeGroupFeatures::FreeGroupInfo FreeGroupFeatures::GetCanonicalInfo(
    const Identity &_groupID) const
{
  const auto model_it = this->models.idToObject.find(_groupID);
  if (model_it != this->models.idToObject.end())
  {
    return FreeGroupInfo{
      model_it->second->model->getRootBodyNode(),
      model_it->second->model.get()};
  }

  return FreeGroupInfo{this->links.at(_groupID)->link, nullptr};
}

/////////////////////////////////////////////////
void FreeGroupFeatures::SetFreeGroupWorldPose(
    const Identity &_groupID,
    const PoseType &_pose)
{
  const FreeGroupInfo &info = GetCanonicalInfo(_groupID);
  if (!info.model)
  {
    static_cast<dart::dynamics::FreeJoint*>(info.link->getParentJoint())
        ->setTransform(_pose);
    return;
  }

  const Eigen::Isometry3d tf_change =
      _pose * info.link->getWorldTransform().inverse();

  for (std::size_t i = 0; i < info.model->getNumTrees(); ++i)
  {
    auto *bn = info.model->getRootBodyNode(i);
    const Eigen::Isometry3d new_tf = tf_change * bn->getTransform();

    static_cast<dart::dynamics::FreeJoint*>(bn->getParentJoint())
        ->setTransform(new_tf);
  }
}

/////////////////////////////////////////////////
void FreeGroupFeatures::SetFreeGroupWorldLinearVelocity(
    const Identity &_groupID, const LinearVelocity &_linearVelocity)
{
  const FreeGroupInfo &info = GetCanonicalInfo(_groupID);
  if (!info.model)
  {
    static_cast<dart::dynamics::FreeJoint*>(info.link->getParentJoint())
        ->setLinearVelocity(_linearVelocity);
    return;
  }

  const Eigen::Vector3d delta_v =
      _linearVelocity - info.link->getLinearVelocity();

  for (std::size_t i = 0; i < info.model->getNumTrees(); ++i)
  {
    auto *bn = info.model->getRootBodyNode(i);
    const Eigen::Vector3d new_v = bn->getLinearVelocity() + delta_v;

    static_cast<dart::dynamics::FreeJoint*>(bn->getParentJoint())
        ->setLinearVelocity(new_v);
  }
}

/////////////////////////////////////////////////
void FreeGroupFeatures::SetFreeGroupWorldAngularVelocity(
    const Identity &_groupID, const AngularVelocity &_angularVelocity)
{
  const FreeGroupInfo &info = GetCanonicalInfo(_groupID);
  if (!info.model)
  {
    static_cast<dart::dynamics::FreeJoint*>(info.link->getParentJoint())
        ->setAngularVelocity(_angularVelocity);
    return;
  }

  const Eigen::Vector3d delta_w =
      _angularVelocity - info.link->getAngularVelocity();
  const Eigen::Vector3d origin = info.link->getTransform().translation();

  for (std::size_t i = 0; i < info.model->getNumTrees(); ++i)
  {
    auto *bn = info.model->getRootBodyNode(i);
    const Eigen::Vector3d r = bn->getTransform().translation() - origin;
    const Eigen::Vector3d v = bn->getLinearVelocity();
    const Eigen::Vector3d w = bn->getAngularVelocity();

    dart::dynamics::FreeJoint *fj =
        static_cast<dart::dynamics::FreeJoint*>(bn->getParentJoint());

    fj->setLinearVelocity(v + delta_w.cross(r));
    fj->setAngularVelocity(w + delta_w);
  }
}

}
}
}
