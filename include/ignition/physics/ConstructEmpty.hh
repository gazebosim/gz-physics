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

#ifndef IGNITION_PHYSICS_CONSTRUCTEMPTY_HH_
#define IGNITION_PHYSICS_CONSTRUCTEMPTY_HH_

#include <string>

#include <ignition/physics/FeatureList.hh>

namespace ignition {
namespace physics {

/////////////////////////////////////////////////
//! [ConstructEmptyWorld]
/// \brief This feature constructs an empty world and return its pointer
/// from the current physics engine in use.
class ConstructEmptyWorldFeature : public virtual Feature
{
  public: template <typename PolicyT, typename FeaturesT>
  class Engine : public virtual Feature::Engine<PolicyT, FeaturesT>
  {
    public: using WorldPtrType = WorldPtr<PolicyT, FeaturesT>;

    /// \brief Construct an empty world and attach a given name to it.
    /// \return
    ///   The WorldPtrType of the constructed world.
    public: WorldPtrType ConstructEmptyWorld(const std::string &_name);
  };

  public: template <typename PolicyT>
  class Implementation : public virtual Feature::Implementation<PolicyT>
  {
    public: virtual Identity ConstructEmptyWorld(
        const Identity &_engineID, const std::string &_name) = 0;
  };
};
//! [ConstructEmptyWorld]

/////////////////////////////////////////////////
/// \brief This feature constructs an empty model and return its pointer
/// from the given world.
class ConstructEmptyModelFeature : public virtual Feature
{
  public: template <typename PolicyT, typename FeaturesT>
  class World : public virtual Feature::World<PolicyT, FeaturesT>
  {
    public: using ModelPtrType = ModelPtr<PolicyT, FeaturesT>;

    /// \brief Construct an empty model and attach a given name to it.
    /// \return
    ///   The ModelPtrType of the constructed model.
    public: ModelPtrType ConstructEmptyModel(const std::string &_name);
  };

  public: template <typename PolicyT>
  class Implementation : public virtual Feature::Implementation<PolicyT>
  {
    public: virtual Identity ConstructEmptyModel(
        const Identity &_worldID, const std::string &_name) = 0;
  };
};

/////////////////////////////////////////////////
/// \brief This feature constructs an empty link and return its pointer
/// from the given model.
class ConstructEmptyLinkFeature : public virtual Feature
{
  public: template <typename PolicyT, typename FeaturesT>
  class Model : public virtual Feature::Model<PolicyT, FeaturesT>
  {
    public: using LinkPtrType = LinkPtr<PolicyT, FeaturesT>;

    /// \brief Construct an empty link and attach a given name to it.
    /// \return
    ///   The LinkPtrType of the constructed link.
    public: LinkPtrType ConstructEmptyLink(const std::string &_name);
  };

  public: template <typename PolicyT>
  class Implementation : public virtual Feature::Implementation<PolicyT>
  {
    public: virtual Identity ConstructEmptyLink(
        const Identity &_modelID, const std::string &_name) = 0;
  };
};

}
}

#include <ignition/physics/detail/ConstructEmpty.hh>

#endif
