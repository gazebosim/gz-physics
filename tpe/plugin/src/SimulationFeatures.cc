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

#include <unordered_map>
#include <utility>

#include <gz/common/Console.hh>
#include <gz/common/Profiler.hh>

#include <gz/math/Pose3.hh>
#include <gz/math/eigen3/Conversions.hh>

#include "SimulationFeatures.hh"

using namespace gz;
using namespace physics;
using namespace tpeplugin;

void SimulationFeatures::WorldForwardStep(
  const Identity &_worldID,
  ForwardStep::Output & _h,
  ForwardStep::State & /*_x*/,
  const ForwardStep::Input & _u)
{
  IGN_PROFILE("SimulationFeatures::WorldForwardStep");
  auto it = this->worlds.find(_worldID);
  if (it == this->worlds.end())
  {
    gzerr << "World with id ["
      << _worldID.id
      << "] not found."
      << std::endl;
    return;
  }
  std::shared_ptr<tpelib::World> world = it->second->world;
  auto *dtDur =
    _u.Query<std::chrono::steady_clock::duration>();
  const double tol = 1e-6;
  if (dtDur)
  {
    std::chrono::duration<double> dt = *dtDur;
    if (std::fabs(dt.count() - world->GetTimeStep()) > tol)
    {
      world->SetTimeStep(dt.count());
      igndbg << "Simulation timestep set to: "
        << world->GetTimeStep()
        << std::endl;
    }
  }
  world->Step();
  this->Write(_h.Get<ChangedWorldPoses>());
}

void SimulationFeatures::Write(ChangedWorldPoses &_changedPoses) const
{
  // remove link poses from the previous iteration
  _changedPoses.entries.clear();
  _changedPoses.entries.reserve(this->links.size());

  std::unordered_map<std::size_t, math::Pose3d> newPoses;
  // Store the updated links to avoid duplicated entries in _changedPoses
  std::unordered_set<std::size_t> updatedLinkIds;

  for (const auto &[id, info] : this->links)
  {
    // make sure the link exists
    if (info)
    {
      const auto nextPose = info->link->GetPose();
      auto iter = this->prevEntityPoses.find(id);

      // If the link's pose is new or has changed, save this new pose and
      // add it to the output poses. Otherwise, keep the existing link pose
      if ((iter == this->prevEntityPoses.end()) ||
          !iter->second.Pos().Equal(nextPose.Pos(), 1e-6) ||
          !iter->second.Rot().Equal(nextPose.Rot(), 1e-6))
      {
        WorldPose wp;
        wp.pose = nextPose;
        wp.body = id;
        _changedPoses.entries.push_back(wp);
        updatedLinkIds.insert(id);
        newPoses[id] = nextPose;
      }
      else
        newPoses[id] = iter->second;
    }
  }

  // Iterate over models to make sure link velocities for moving models
  // are calculated and sent
  for (const auto &[id, info] : this->models)
  {
    // make sure the model exists
    if (info)
    {
      if (info->model->GetStatic())
        continue;
      const auto nextPose = info->model->GetPose();
      // Note, now prevEntityPoses also contains prevModelPoses
      // Data structure has been kept to avoid breaking ABI
      auto iter = this->prevEntityPoses.find(id);

      // If the models's pose is new or has changed, calculate and add all
      // the children links' poses to the output poses
      if ((iter == this->prevEntityPoses.end()) ||
          !iter->second.Pos().Equal(nextPose.Pos(), 1e-6) ||
          !iter->second.Rot().Equal(nextPose.Rot(), 1e-6))
      {
        for (const auto &linkEnt : info->model->GetChildren())
        {
          // Avoid pushing if the link was already updated in the previous loop
          auto linkId = linkEnt.second->GetId();
          if (updatedLinkIds.find(linkId) == updatedLinkIds.end())
          {
            WorldPose wp;
            wp.pose = linkEnt.second->GetPose();
            wp.body = linkId;
            _changedPoses.entries.push_back(wp);
            updatedLinkIds.insert(linkId);
          }
          newPoses[id] = linkEnt.second->GetPose();
        }
      }
      else
        newPoses[id] = iter->second;
    }
  }

  // Save the new poses so that they can be used to check for updates in the
  // next iteration. Re-setting this->prevEntityPoses with the contents of
  // newPoses ensures that we aren't caching data for links that were removed
  this->prevEntityPoses = std::move(newPoses);
}

std::vector<SimulationFeatures::ContactInternal>
SimulationFeatures::GetContactsFromLastStep(const Identity &_worldID) const
{
  IGN_PROFILE("SimulationFeatures::GetContactFromLastStep");
  std::vector<SimulationFeatures::ContactInternal> outContacts;
  auto const world = this->ReferenceInterface<WorldInfo>(_worldID)->world;
  const auto contacts = world->GetContacts();

  for (const auto &c : contacts)
  {
    CompositeData extraData;

    // Contact expects identity to be associated with shapes not models
    // but tpe computes collisions between models
    // Workaround is to return the first shape of a model
    auto s1 = this->GetModelCollision(c.entity1);
    auto s2 = this->GetModelCollision(c.entity2);

    outContacts.push_back(
        {this->GenerateIdentity(s1.GetId(), this->collisions.at(s1.GetId())),
         this->GenerateIdentity(s2.GetId(), this->collisions.at(s2.GetId())),
         math::eigen3::convert(c.point), extraData});
  }

  return outContacts;
}

tpelib::Entity &SimulationFeatures::GetModelCollision(std::size_t _id) const
{
  auto m = this->models.at(_id);
  if (!m || !m->model)
    return tpelib::Entity::kNullEntity;

  tpelib::Entity &link = m->model->GetCanonicalLink();
  if (link.GetChildCount() == 0u)
    return tpelib::Entity::kNullEntity;

  return link.GetChildByIndex(0u);
}
