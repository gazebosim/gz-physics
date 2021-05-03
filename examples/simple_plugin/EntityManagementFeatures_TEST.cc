/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include <iostream>

#include <ignition/plugin/Loader.hh>
#include <ignition/physics/RequestEngine.hh>
#include "EntityManagementFeatures.hh"

// Simple executable that loads the simple plugin and constructs a world.

struct TestFeatureList : ignition::physics::FeatureList<
  ignition::physics::simpleplugin::EntityManagementFeatureList
> { };

int main(int argc, char *argv[])
{
  // Load the custom plugin
  ignition::plugin::Loader loader;
  loader.LoadLib(simple_plugin_LIB);

  auto simplePlugin =
    loader.Instantiate("ignition::physics::simpleplugin::Plugin");

  // Get the engine pointer
  auto engine =
      ignition::physics::RequestEngine3d<TestFeatureList>::From(simplePlugin);

  if (nullptr == engine)
  {
    std::cerr << "Something went wrong, the engine is null" << std::endl;
    return -1;
  }

  auto world = engine->ConstructEmptyWorld("empty world");

  if (nullptr == world)
  {
    std::cerr << "Failed to create empty world" << std::endl;
    return -1;
  }

  std::cout << "Created empty world!" << std::endl;

  return 0;
}
