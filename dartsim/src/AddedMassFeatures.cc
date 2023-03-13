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

#include "AddedMassFeatures.hh"
#include <dart/dynamics/Inertia.hpp>
#include <dart/dynamics/WeldJoint.hpp>

namespace gz::physics::dartsim
{

void AddedMassFeatures::SetLinkAddedMass(const Identity &_link,
    const gz::math::Matrix6d &_addedMass)
{
  auto linkInfo = this->ReferenceInterface<LinkInfo>(_link);
  auto bn = linkInfo->link;

  if (linkInfo->inertial.has_value())
  {
    auto &sdfInertia = linkInfo->inertial.value();
    sdfInertia.SetFluidAddedMass(_addedMass);

    dart::dynamics::Inertia newInertia;
    newInertia.setMass(sdfInertia.MassMatrix().Mass());

    const Eigen::Matrix3d I_link = math::eigen3::convert(sdfInertia.Moi());
    newInertia.setMoment(I_link);

    const Eigen::Vector3d localCom =
        math::eigen3::convert(sdfInertia.Pose().Pos());
    newInertia.setLocalCOM(localCom);
    bn->setInertia(newInertia);

    if (sdfInertia.FluidAddedMass().has_value())
    {
      // Note that the ordering of the spatial inertia matrix used in DART is
      // different than the one used in Gazebo and SDF.
      math::Matrix6d featherstoneMatrix;
      featherstoneMatrix.SetSubmatrix(math::Matrix6d::TOP_LEFT,
          sdfInertia.FluidAddedMass().value().Submatrix(
          math::Matrix6d::BOTTOM_RIGHT));
      featherstoneMatrix.SetSubmatrix(math::Matrix6d::TOP_RIGHT,
          sdfInertia.FluidAddedMass().value().Submatrix(
          math::Matrix6d::BOTTOM_LEFT));
      featherstoneMatrix.SetSubmatrix(math::Matrix6d::BOTTOM_LEFT,
          sdfInertia.FluidAddedMass().value().Submatrix(
          math::Matrix6d::TOP_RIGHT));
      featherstoneMatrix.SetSubmatrix(math::Matrix6d::BOTTOM_RIGHT,
          sdfInertia.FluidAddedMass().value().Submatrix(
          math::Matrix6d::TOP_LEFT));

      // If using added mass, gravity needs to be applied as a separate
      // force at the center of mass using F=ma;
      bn->setGravityMode(false);

      dart::dynamics::BodyNode::Properties bodyProperties;
      bodyProperties.mName = bn->getName() + "_fluid_added_mass";
      bodyProperties.mInertia.setSpatialTensor(math::eigen3::convert(featherstoneMatrix));

      dart::dynamics::WeldJoint::Properties jointProperties;
      jointProperties.mName =  bodyProperties.mName + "_WeldJoint";

      auto result = bn->createChildJointAndBodyNodePair<dart::dynamics::WeldJoint>(jointProperties, bodyProperties);
      result.second->setGravityMode(false);

      {
        const std::size_t id = this->GetNextEntity();
        auto newLinkInfo = std::make_shared<LinkInfo>();
        this->links.idToObject[id] = newLinkInfo;
        newLinkInfo->link = result.second;
      }

      {
        const std::size_t id = this->GetNextEntity();
        auto newJointInfo = std::make_shared<JointInfo>();
        this->joints.idToObject[id] = newJointInfo;
        newJointInfo->joint = result.first;
      }
    }
  }
}

gz::math::Matrix6d
AddedMassFeatures::GetLinkAddedMass(const Identity &_link) const
{
  auto linkInfo = this->ReferenceInterface<LinkInfo>(_link);

  if (linkInfo->inertial.has_value() &&
      linkInfo->inertial->FluidAddedMass().has_value())
  {
    return linkInfo->inertial->FluidAddedMass().value();
  }
  else
  {
    return gz::math::Matrix6d::Zero;
  }
}

}  // namespace gz::physics::dartsim
