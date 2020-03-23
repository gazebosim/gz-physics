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

#ifndef IGNITION_PHYSICS_REQUESTFEATURES_HH_
#define IGNITION_PHYSICS_REQUESTFEATURES_HH_

#include <set>
#include <string>

#include <ignition/physics/Entity.hh>

namespace ignition
{
  namespace physics
  {
    /// \brief This class can be used to request features from an entity, or
    /// identify what features are missing from an entity.
    template <typename FeatureListT>
    struct RequestFeatures
    {
      using Features = typename FeatureListT::Features;

      template <
          typename PolicyT,
          typename FromFeatureList,
          template <typename, typename> class EntityT>
      static EntityPtr<EntityT<PolicyT, FeatureListT>> From(
          const EntityPtr<EntityT<PolicyT, FromFeatureList>>& from);

      template <
          typename PolicyT,
          typename FromFeatureList,
          template <typename, typename> class EntityT>
      static std::set<std::string> MissingFeatureNames(
          const EntityPtr<EntityT<PolicyT, FromFeatureList>>& from);
    };
  }
}

#include <ignition/physics/detail/RequestFeatures.hh>

#endif
