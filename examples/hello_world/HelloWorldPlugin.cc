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

#include <ignition/physics/FeatureList.hh>
#include <ignition/physics/FeaturePolicy.hh>
#include <ignition/physics/GetEntities.hh>
#include <ignition/physics/Register.hh>

namespace mock
{
  struct HelloWorldFeatureList : ignition::physics::FeatureList<
      ignition::physics::GetEngineInfo
  > { };

  class HelloWorldPlugin
      : public ignition::physics::Implements3d<HelloWorldFeatureList>
  {
    using Identity = ignition::physics::Identity;

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

  IGN_PHYSICS_ADD_PLUGIN(
      HelloWorldPlugin,
      ignition::physics::FeaturePolicy3d,
      HelloWorldFeatureList)
}
