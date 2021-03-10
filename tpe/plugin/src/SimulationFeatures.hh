/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#ifndef IGNITION_PHYSICS_TPE_PLUGIN_SRC_SIMULATIONFEATURES_HH_
#define IGNITION_PHYSICS_TPE_PLUGIN_SRC_SIMULATIONFEATURES_HH_

#include <unordered_map>
#include <vector>

#include <ignition/math/Pose3.hh>

#include <ignition/physics/CanWriteData.hh>
#include <ignition/physics/ForwardStep.hh>
#include <ignition/physics/GetContacts.hh>

#include "Base.hh"

namespace ignition {
namespace physics {
namespace tpeplugin {

struct SimulationFeatureList : FeatureList<
  ForwardStep,
  GetContactsFromLastStepFeature
> { };

class SimulationFeatures :
  public CanWriteRequiredData<SimulationFeatures, ForwardStep::Output>,
  public virtual Base,
  public virtual Implements3d<SimulationFeatureList>
{
  public: void WorldForwardStep(
    const Identity &_worldID,
    ForwardStep::Output &_h,
    ForwardStep::State &_x,
    const ForwardStep::Input &_u) override;

  public: void Write(JointPositions &_positions) const;

  public: void Write(WorldPoses &_poses) const;

  public: std::vector<ContactInternal> GetContactsFromLastStep(
    const Identity &_worldID) const override;

  /// \brief Get a collision from the canonical link of a model
  /// \param[in] _id Model ID
  /// \return Collision entity
  private: tpelib::Entity &GetModelCollision(std::size_t _id) const;
};

}
}
}

#endif
