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

#ifndef SRC_GZ_PHYSICS_TESTUTILITIES_HH_
#define SRC_GZ_PHYSICS_TESTUTILITIES_HH_

#include <string>

#include <gz/physics/Feature.hh>
#include <gz/plugin/Loader.hh>

#include <test/PhysicsPluginsList.hh>

using namespace gz;
namespace test
{
  class UnimplementedFeature : public virtual physics::Feature
  {
    public: template <typename PolicyT>
    class Implementation : public virtual Feature::Implementation<PolicyT>
    {
      public: virtual void someUnimplementedVirtualFunction() = 0;

      public: ~Implementation() override;
    };
  };
}

void PrimeTheLoader(plugin::Loader &_loader)
{
  for (const std::string &library
       : physics::test::g_PhysicsPluginLibraries)
  {
    if (!library.empty())
      _loader.LoadLib(library);
  }
}

#endif
