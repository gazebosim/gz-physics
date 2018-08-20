/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

#ifndef IGNITION_PHYSICS_DETAIL_REQUESTENGINE_HH_
#define IGNITION_PHYSICS_DETAIL_REQUESTENGINE_HH_

#include <memory>
#include <set>
#include <string>

#include <ignition/physics/RequestEngine.hh>

namespace ignition
{
  namespace physics
  {
    /////////////////////////////////////////////////
    template <typename FeaturePolicyT, typename FeatureListT>
    template <typename PtrT>
    bool RequestEngine<FeaturePolicyT, FeatureListT>::
    Verify(const PtrT &_pimpl)
    {
      return detail::InspectFeatures<FeaturePolicyT, Features>::Verify(_pimpl);
    }

    /////////////////////////////////////////////////
    template <typename FeaturePolicyT, typename FeatureListT>
    template <typename PtrT>
    std::set<std::string> RequestEngine<FeaturePolicyT, FeatureListT>::
    MissingFeatureNames(const PtrT &_pimpl)
    {
      std::set<std::string> names;
      detail::InspectFeatures<FeaturePolicyT, Features>::
          MissingNames(_pimpl, names);

      return names;
    }

    /////////////////////////////////////////////////
    template <typename FeaturePolicyT, typename FeatureListT>
    template <typename PtrT>
    auto RequestEngine<FeaturePolicyT, FeatureListT>::
    From(const PtrT &_pimpl, const std::size_t _engineID)
      -> EnginePtr
    {
      using Pimpl = typename detail::DeterminePlugin<
          FeaturePolicyT, Features>::type;

      if (!detail::InspectFeatures<FeaturePolicyT, Features>::Verify(_pimpl))
        return nullptr;

      std::shared_ptr<Pimpl> pimpl = std::make_shared<Pimpl>(_pimpl);
      Feature::Implementation<FeaturePolicyT> *implBase =
          (*pimpl)->template QueryInterface<
              Feature::Implementation<FeaturePolicyT>>();

      return EnginePtr(pimpl, implBase->InitiateEngine(_engineID));
    }
  }
}

#endif
