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

#include "FreeGroupFeatures.hh"

#include <memory>
#include <unordered_map>

#include <gz/common/Profiler.hh>

namespace gz {
namespace physics {
namespace bullet_featherstone {

/////////////////////////////////////////////////
btTransform getWorldTransformForLink(btMultiBody *_body, int _linkIndex)
{
  if (_linkIndex == -1)
  {
    return _body->getBaseWorldTransform();
  }
  else
  {
    btMultiBodyLinkCollider *collider = _body->getLinkCollider(_linkIndex);
    return collider->getWorldTransform();
  }
}

/////////////////////////////////////////////////
void enforceFixedConstraint(
    btMultiBodyFixedConstraint *_fixedConstraint,
    const std::unordered_map<std::size_t, Base::JointInfoPtr> &_allJoints)
{
  GZ_PROFILE("bullet_featherstone::enforceFixedConstraint");
  // Update fixed constraint's child link pose to maintain a fixed transform
  // from the parent link.
  btMultiBody *parent = _fixedConstraint->getMultiBodyA();
  btMultiBody *child = _fixedConstraint->getMultiBodyB();

  btTransform parentToChildTf;
  parentToChildTf.setOrigin(_fixedConstraint->getPivotInA());
  parentToChildTf.setBasis(_fixedConstraint->getFrameInA());

  int parentLinkIndex = _fixedConstraint->getLinkA();
  int childLinkIndex = _fixedConstraint->getLinkB();

  btTransform parentLinkTf = getWorldTransformForLink(parent, parentLinkIndex);
  btTransform childLinkTf = getWorldTransformForLink(child, childLinkIndex);

  btTransform expectedChildLinkTf = parentLinkTf * parentToChildTf;
  btTransform childBaseTf =  child->getBaseWorldTransform();
  btTransform childBaseToLink =
      childBaseTf.inverse() * childLinkTf;
  btTransform newChildBaseTf =
      expectedChildLinkTf * childBaseToLink.inverse();
  child->setBaseWorldTransform(newChildBaseTf);
  for (const auto &joint : _allJoints)
  {
    if (joint.second->fixedConstraint)
    {
      // Recursively enforce constraints where the child here is a parent to
      // other constraints.
      if (joint.second->fixedConstraint->getMultiBodyA() == child)
      {
        enforceFixedConstraint(joint.second->fixedConstraint.get(), _allJoints);
      }
    }
  }
}

/////////////////////////////////////////////////
/// \brief World linear velocity of the root link frame of a model. Bullet
/// stores the velocity of the center of mass of the root link, while the
/// velocity of a free group is the velocity of the root link frame:
/// v_link = v_com + w x r, with r from the center of mass to the root link
/// frame.
/// \param[in] _model Model to query.
btVector3 getLinkFrameLinearVelocity(const ModelInfo &_model)
{
  const btVector3 comToLink =
      getBaseComToLinkFrame(_model, _model.body->getBaseWorldTransform());
  return _model.body->getBaseVel() +
      _model.body->getBaseOmega().cross(comToLink);
}

/////////////////////////////////////////////////
/// \brief Set the base velocity of a model from the velocity of its root
/// link frame, see getLinkFrameLinearVelocity: v_com = v_link - w x r.
/// \param[in] _model Model to update.
/// \param[in] _linkLinVel World linear velocity of the root link frame.
/// \param[in] _angVel World angular velocity.
void setBaseVelocityFromLinkFrame(const ModelInfo &_model,
    const btVector3 &_linkLinVel, const btVector3 &_angVel)
{
  const btVector3 comToLink =
      getBaseComToLinkFrame(_model, _model.body->getBaseWorldTransform());
  _model.body->setBaseOmega(_angVel);
  _model.body->setBaseVel(_linkLinVel - _angVel.cross(comToLink));
  _model.body->wakeUp();
}

/////////////////////////////////////////////////
Identity FreeGroupFeatures::FindFreeGroupForModel(
    const Identity &_modelID) const
{
  GZ_PROFILE("FreeGroupFeatures::FindFreeGroupForModel");
  const auto *model = this->ReferenceInterface<ModelInfo>(_modelID);

  for (const auto & joint : this->joints)
  {
    // Also reject if the model is a child of a fixed constraint
    // (detachable joint)
    if (joint.second->fixedConstraint)
    {
      if (joint.second->fixedConstraint->getMultiBodyB() == model->body.get())
      {
        return this->GenerateInvalidId();
      }
    }
    // Reject if the model has a world joint
    if (std::size_t(joint.second->model) == std::size_t(_modelID))
    {
      const auto *identifier =
          std::get_if<RootJoint>(&joint.second->identifier);
      if (identifier)
      {
        return this->GenerateInvalidId();
      }
    }
  }

  return _modelID;
}

/////////////////////////////////////////////////
Identity FreeGroupFeatures::FindFreeGroupForLink(
    const Identity &_linkID) const
{
  // Free groups in bullet-featherstone are currently represented by ModelInfo
  const auto *link = this->ReferenceInterface<LinkInfo>(_linkID);
  return this->FindFreeGroupForModel(link->model);
}

/////////////////////////////////////////////////
Identity FreeGroupFeatures::GetFreeGroupRootLink(const Identity &_groupID) const
{
  GZ_PROFILE("FreeGroupFeatures::GetFreeGroupRootLink");
  // Free groups in bullet-featherstone are always represented by ModelInfo
  const auto *model = this->ReferenceInterface<ModelInfo>(_groupID);

  // btMultiBody user index stores the gz-physics model root link id
  std::size_t rootID = static_cast<std::size_t>(model->body->getUserIndex());
  return this->GenerateIdentity(rootID, this->links.at(rootID));
}

/////////////////////////////////////////////////
void FreeGroupFeatures::SetFreeGroupWorldAngularVelocity(
    const Identity &_groupID, const AngularVelocity &_angularVelocity)
{
  // Free groups in bullet-featherstone are always represented by ModelInfo
  const auto *model = this->ReferenceInterface<ModelInfo>(_groupID);

  if (model)
  {
    // Keep the linear velocity of the root link frame, as dartsim does.
    setBaseVelocityFromLinkFrame(*model, getLinkFrameLinearVelocity(*model),
        convertVec(_angularVelocity));
  }
}

/////////////////////////////////////////////////
void FreeGroupFeatures::SetFreeGroupWorldLinearVelocity(
    const Identity &_groupID, const LinearVelocity &_linearVelocity)
{
  // Free groups in bullet-featherstone are always represented by ModelInfo
  const auto *model = this->ReferenceInterface<ModelInfo>(_groupID);

  if (model)
  {
    setBaseVelocityFromLinkFrame(*model, convertVec(_linearVelocity),
        model->body->getBaseOmega());
  }
}

/////////////////////////////////////////////////
void FreeGroupFeatures::SetFreeGroupWorldPose(
    const Identity &_groupID,
    const PoseType &_pose)
{
  GZ_PROFILE("FreeGroupFeatures::SetFreeGroupWorldPose");
  const auto *model = this->ReferenceInterface<ModelInfo>(_groupID);
  if (model)
  {
    model->body->SetBaseWorldTransform(
      convertTf(_pose * model->baseInertiaToLinkFrame.inverse()));

    model->body->wakeUp();

    for (auto & joint : this->joints)
    {
      if (joint.second->fixedConstraint)
      {
        // Only the parent body of a fixed joint can be moved but we need to
        // enforce the fixed constraint by updating the child pose so it
        // remains at a fixed transform from parent. We do this recursively to
        // account for other constraints attached to the child.
        btMultiBody *parent = joint.second->fixedConstraint->getMultiBodyA();
        if (parent == model->body.get())
        {
          enforceFixedConstraint(joint.second->fixedConstraint.get(),
              this->joints);
        }
      }
    }
  }
}

}  // namespace bullet_featherstone
}  // namespace physics
}  // namespace gz
