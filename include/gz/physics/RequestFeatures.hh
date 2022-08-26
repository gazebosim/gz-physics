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

#ifndef GZ_PHYSICS_REQUESTFEATURES_HH_
#define GZ_PHYSICS_REQUESTFEATURES_HH_

#include <set>
#include <string>

#include <gz/physics/Entity.hh>

namespace ignition
{
  namespace physics
  {
    /// \brief This class can be used to request features from an entity, or
    /// identify what features are missing from an entity.
    /// \tparam ToFeatureList The list of features being requested.
    template <typename ToFeatureList>
    struct RequestFeatures
    {
      using Features = typename ToFeatureList::Features;

      /// \brief Cast an entity to another entity pointer, with the requested
      /// set of features.
      /// \tparam PolicyT The feature policy, such as
      /// `ignition::physics::FeaturePolicy3d`.
      /// \tparam FromFeatureList The list of features from the original entity.
      /// \tparam ToFeatureList The list of features of the resulting entity.
      /// \param[in] _from Entity to cast from.
      /// \return The casted entity. It will be null if the physics engine
      /// doesn't support all requested features.
      template <
          typename PolicyT,
          typename FromFeatureList,
          template <typename, typename> class EntityT>
      static EntityPtr<EntityT<PolicyT, ToFeatureList>> From(
          const EntityPtr<EntityT<PolicyT, FromFeatureList>> &_from);

      /// \brief Check which features from the requested list are missing from
      /// the entity's plugin.
      /// \tparam PolicyT The feature policy, such as
      /// `ignition::physics::FeaturePolicy3d`.
      /// \tparam FromFeatureList The current list of features from the entity.
      /// \param[in] _entity Entity to cast from.
      /// \return The names of all missing features. Will be empty if no
      /// features are missing. If an invalid entity is given, the list will
      /// contain all features being requested.
      template <
          typename PolicyT,
          typename FromFeatureList,
          template <typename, typename> class EntityT>
      static std::set<std::string> MissingFeatureNames(
          const EntityPtr<EntityT<PolicyT, FromFeatureList>> &_entity);
    };
  }
}

#include <gz/physics/detail/RequestFeatures.hh>

#endif
