/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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

#ifndef GZ_PHYSICS_DETAIL_GRAVITY_HH_
#define GZ_PHYSICS_DETAIL_GRAVITY_HH_

#include <gz/physics/Gravity.hh>

namespace gz
{
namespace physics
{
/////////////////////////////////////////////////
template <typename PolicyT, typename FeaturesT>
void LinkGravityEnabled::Link<PolicyT, FeaturesT>::SetGravityEnabled(
    bool _enabled)
{
  this->template Interface<LinkGravityEnabled>()
      ->SetLinkGravityEnabled(this->identity, _enabled);
}

/////////////////////////////////////////////////
template <typename PolicyT, typename FeaturesT>
bool LinkGravityEnabled::Link<PolicyT, FeaturesT>::GetGravityEnabled() const
{
  return this->template Interface<LinkGravityEnabled>()
      ->GetLinkGravityEnabled(this->identity);
}

/////////////////////////////////////////////////
template <typename PolicyT, typename FeaturesT>
void ModelGravityEnabled::Model<PolicyT, FeaturesT>::SetGravityEnabled(
    bool _enabled)
{
  if (auto * linkIface = this->template Interface<GetLinkFromModel>())
  {
    const std::size_t linkCount = linkIface->GetLinkCount(this->identity);
    for (std::size_t i = 0; i < linkCount; ++i)
    {
      auto link = LinkPtr<PolicyT, FeaturesT>(
          this->pimpl, linkIface->GetLink(this->identity, i));
      if (link)
      {
        link->SetGravityEnabled(_enabled);
      }
    }
  }

  if (auto * nestedIface =
      this->template Interface<GetNestedModelFromModel>())
  {
    const std::size_t nestedModelCount =
        nestedIface->GetNestedModelCount(this->identity);
    for (std::size_t i = 0; i < nestedModelCount; ++i)
    {
      auto nestedModel = ModelPtr<PolicyT, FeaturesT>(
          this->pimpl, nestedIface->GetNestedModel(this->identity, i));
      if (nestedModel)
      {
        nestedModel->SetGravityEnabled(_enabled);
      }
    }
  }
}

/////////////////////////////////////////////////
template <typename PolicyT, typename FeaturesT>
bool ModelGravityEnabled::Model<PolicyT, FeaturesT>::GetGravityEnabled() const
{
  if (auto * linkIface = this->template Interface<GetLinkFromModel>())
  {
    const std::size_t linkCount = linkIface->GetLinkCount(this->identity);
    for (std::size_t i = 0; i < linkCount; ++i)
    {
      auto link = LinkPtr<PolicyT, FeaturesT>(
          this->pimpl, linkIface->GetLink(this->identity, i));
      if (link && !link->GetGravityEnabled())
      {
        return false;
      }
    }
  }

  if (auto * nestedIface =
      this->template Interface<GetNestedModelFromModel>())
  {
    const std::size_t nestedModelCount =
        nestedIface->GetNestedModelCount(this->identity);
    for (std::size_t i = 0; i < nestedModelCount; ++i)
    {
      auto nestedModel = ModelPtr<PolicyT, FeaturesT>(
          this->pimpl, nestedIface->GetNestedModel(this->identity, i));
      if (nestedModel && !nestedModel->GetGravityEnabled())
      {
        return false;
      }
    }
  }

  return true;
}

}
}

#endif
