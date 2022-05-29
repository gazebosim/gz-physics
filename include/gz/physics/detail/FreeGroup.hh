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

#ifndef GZ_PHYSICS_DETAIL_FREEGROUP_HH_
#define GZ_PHYSICS_DETAIL_FREEGROUP_HH_

#include <gz/physics/FreeGroup.hh>

namespace gz
{
  namespace physics
  {
    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    FreeGroupPtr<PolicyT, FeaturesT>
    FindFreeGroupFeature::Model<PolicyT, FeaturesT>::FindFreeGroup()
    {
      return FreeGroupPtrType(this->pimpl,
        this->template Interface<FindFreeGroupFeature>()
          ->FindFreeGroupForModel(this->identity));
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    ConstFreeGroupPtr<PolicyT, FeaturesT>
    FindFreeGroupFeature::Model<PolicyT, FeaturesT>::FindFreeGroup() const
    {
      return ConstFreeGroupPtrType(this->pimpl,
        this->template Interface<FindFreeGroupFeature>()
          ->FindFreeGroupForModel(this->identity));
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    FreeGroupPtr<PolicyT, FeaturesT>
    FindFreeGroupFeature::Link<PolicyT, FeaturesT>::FindFreeGroup()
    {
      return FreeGroupPtrType(this->pimpl,
        this->template Interface<FindFreeGroupFeature>()
          ->FindFreeGroupForLink(this->identity));
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    ConstFreeGroupPtr<PolicyT, FeaturesT>
    FindFreeGroupFeature::Link<PolicyT, FeaturesT>::FindFreeGroup() const
    {
      return FreeGroupPtrType(this->pimpl,
        this->template Interface<FindFreeGroupFeature>()
          ->FindFreeGroupForLink(this->identity));
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    LinkPtr<PolicyT, FeaturesT>
    FindFreeGroupFeature::FreeGroup<PolicyT, FeaturesT>::RootLink()
    {
      return LinkPtr<PolicyT, FeaturesT>(this->pimpl,
        this->template Interface<FindFreeGroupFeature>()
          ->GetFreeGroupRootLink(this->identity));
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    ConstLinkPtr<PolicyT, FeaturesT> FindFreeGroupFeature::FreeGroup<
        PolicyT, FeaturesT>::RootLink() const
    {
      return LinkPtr<PolicyT, FeaturesT>(this->pimpl,
        this->template Interface<FindFreeGroupFeature>()
          ->GetFreeGroupRootLink(this->identity));
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    LinkPtr<PolicyT, FeaturesT>
    FindFreeGroupFeature::FreeGroup<PolicyT, FeaturesT>::CanonicalLink()
    {
      return this->RootLink();
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    ConstLinkPtr<PolicyT, FeaturesT>
    FindFreeGroupFeature::FreeGroup<PolicyT, FeaturesT>::CanonicalLink() const
    {
      return this->RootLink();
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    void SetFreeGroupWorldPose::FreeGroup<PolicyT, FeaturesT>::SetWorldPose(
        const PoseType &_pose)
    {
      this->template Interface<SetFreeGroupWorldPose>()
        ->SetFreeGroupWorldPose(this->identity, _pose);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    void SetFreeGroupWorldVelocity::FreeGroup<PolicyT, FeaturesT>::
    SetWorldLinearVelocity(const LinearVelocity &_linearVelocity)
    {
      this->template Interface<SetFreeGroupWorldVelocity>()
        ->SetFreeGroupWorldLinearVelocity(this->identity, _linearVelocity);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    void SetFreeGroupWorldVelocity::FreeGroup<PolicyT, FeaturesT>::
    SetWorldAngularVelocity(const AngularVelocity &_angularVelocity)
    {
      this->template Interface<SetFreeGroupWorldVelocity>()
        ->SetFreeGroupWorldAngularVelocity(this->identity, _angularVelocity);
    }
  }
}

#endif
