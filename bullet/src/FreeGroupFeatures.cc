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
  //  Verify that link exists
  const auto &link_it = this->links.find(_groupID);
  if (link_it != this->links.end() && link_it->second != nullptr)
  {
    auto &model = this->models.at(link_it->second->model)->model;
    //  Search on that link's parents for the base
    for (int link_num = 0; link_num < model->getNumLinks(); link_num++)
    {
      if (model->getParent(link_num) == -1)
      {
        //  TODO(lobotuerk) when joints are supported, fix canonical getter
        // auto link = model->getLink(link_num);
        // auto linkPtr = std::make_shared<LinkInfo>();
        // linkPtr->linkIndex = link_num;
        // linkPtr->name = link->m_linkName;
        // linkPtr->
        // return this->GenerateIdentity(_groupID.id, link_ptr);
      }
    }
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
