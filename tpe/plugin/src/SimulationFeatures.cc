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

#include "SimulationFeatures.hh"

#include <ignition/common/Console.hh>
#include <ignition/common/Profiler.hh>

#include <ignition/math/eigen3/Conversions.hh>


using namespace gz;
using namespace physics;
using namespace tpeplugin;

void SimulationFeatures::WorldForwardStep(
  const Identity &_worldID,
  ForwardStep::Output & /*_h*/,
  ForwardStep::State & /*_x*/,
  const ForwardStep::Input & _u)
{
  IGN_PROFILE("SimulationFeatures::WorldForwardStep");
  auto it = this->worlds.find(_worldID);
  if (it == this->worlds.end())
  {
    ignerr << "World with id ["
      << _worldID.id
      << "] not found."
      << std::endl;
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
