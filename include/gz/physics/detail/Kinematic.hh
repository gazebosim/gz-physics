/*
 * Copyright (C) 2024 Open Source Robotics Foundation
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

#ifndef GZ_PHYSICS_DETAIL_KINEMATIC_HH_
#define GZ_PHYSICS_DETAIL_KINEMATIC_HH_

#include <gz/physics/Kinematic.hh>
#include <gz/physics/FeatureList.hh>

namespace gz
{
namespace physics
{
/////////////////////////////////////////////////
template <typename PolicyT, typename FeaturesT>
void Kinematic::Link<PolicyT, FeaturesT>::SetKinematic(bool _kinematic)
{
  this->template Interface<Kinematic>()
      ->SetLinkKinematic(this->identity, _kinematic);
}

/////////////////////////////////////////////////
template <typename PolicyT, typename FeaturesT>
bool Kinematic::Link<PolicyT, FeaturesT>::GetKinematic() const
{
  return this->template Interface<Kinematic>()
      ->GetLinkKinematic(this->identity);
}
}  // namespace physics
}  // namespace gz

#endif
