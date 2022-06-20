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

////////////////////////////////////////////////////////////
//! [include statements]
#include <gz/physics/FeatureList.hh>
#include <gz/physics/FeaturePolicy.hh>
#include <gz/physics/GetEntities.hh>
#include <gz/physics/Register.hh>
//! [include statements]

namespace mock
{
  ////////////////////////////////////////////////////////
  //! [feature list]
  // List of all features that this plugin will implement
  struct HelloWorldFeatureList : gz::physics::FeatureList<
      gz::physics::GetEngineInfo
  > { };
  //! [feature list]

  ////////////////////////////////////////////////////////
  //! [implementation]
  // The plugin class, which implements a 3D policy
  class HelloWorldPlugin
      : public gz::physics::Implements3d<HelloWorldFeatureList>
  {
    using Identity = gz::physics::Identity;

    public: Identity InitiateEngine(std::size_t /*_engineID*/) override
    {
      this->engineName = "HelloWorld";

      return this->GenerateIdentity(0);
    }

    public: std::size_t GetEngineIndex(const Identity &/*_id*/) const override
    {
      return 0;
    }

    public: const std::string &GetEngineName(const Identity &/*_id*/) const override
    {
      return this->engineName;
    }

    std::string engineName;
  };
  //! [implementation]

  ////////////////////////////////////////////////////////
  //! [register]
  // Register plugin
  GZ_PHYSICS_ADD_PLUGIN(
      HelloWorldPlugin,
      gz::physics::FeaturePolicy3d,
      HelloWorldFeatureList)
  //! [register]
}
