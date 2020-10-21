#include "FreeGroupFeatures.hh"

namespace ignition {
namespace physics {
namespace bullet {

/////////////////////////////////////////////////
Identity FreeGroupFeatures::FindFreeGroupForModel(
    const Identity &_modelID) const
{
  const auto &model = this->models.at(_modelID)->model;

  // If there are no links at all in this model, then the FreeGroup functions
  // will not work properly, so we'll just reject these cases.
  if (model->getNumLinks() == 0)
    return this->GenerateInvalidId();

  // Reject also if the model has fixed base
  if (model->hasFixedBase())
    return this->GenerateInvalidId();

  return _modelID;
}

/////////////////////////////////////////////////
Identity FreeGroupFeatures::FindFreeGroupForLink(
    const Identity &_linkID) const
{
  const auto &link_it = this->links.find(_linkID);

  if (link_it != this->links.end() && link_it->second != nullptr)
    return this->GenerateIdentity(_linkID.id, link_it->second);
  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
Identity FreeGroupFeatures::GetFreeGroupCanonicalLink(
    const Identity &_groupID) const
{
  / assume no canonical link for now
  // assume groupID ~= modelID
  const auto modelIt = this->models.find(_groupID.id);
  if (modelIt != this->models.end() && modelIt->second != nullptr)
  {
    // assume canonical link is the first link in model
    tpelib::Entity &link = modelIt->second->model->GetCanonicalLink();
    auto linkPtr = std::make_shared<LinkInfo>();
    linkPtr->link = static_cast<tpelib::Link *>(&link);
    return this->GenerateIdentity(link.GetId(), linkPtr);
  }
  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
void FreeGroupFeatures::SetFreeGroupWorldPose(
    const Identity &_groupID,
    const PoseType &_pose)
{
  // Convert pose
  const auto poseTranslation = _pose.translation();
  const auto poseLinear = _pose.linear();
  btTransform baseTransform;
  baseTransform.setOrigin(convertVec(poseTranslation));
  baseTransform.setBasis(convertMat(poseLinear));

  // Set base transform
  const auto &model = this->models.at(_groupID)->model;
  model->setBaseWorldTransform(baseTransform);
}

}
}
}
