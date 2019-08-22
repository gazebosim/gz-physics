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

#include <ignition/math/eigen3/Conversions.hh>

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
