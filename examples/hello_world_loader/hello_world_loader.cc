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

////////////////////////////////////////////////////////////////////
//! [include statements]
#include <iostream>

#include <gz/plugin/Loader.hh>
#include <gz/plugin/PluginPtr.hh>

#include <gz/physics/FindFeatures.hh>
#include <gz/physics/GetEntities.hh>
#include <gz/physics/RequestEngine.hh>

// The features that an engine must have to be loaded by this loader.
using Features = gz::physics::FeatureList<
    gz::physics::GetEngineInfo
>;
//! [include statements]

////////////////////////////////////////////////////////////////////
//! [main]
int main(int argc, char **argv)
{
  // User should provide path to plugin library
  if (argc <= 1)
  {
    std::cerr << "Please provide the path to an engine plugin." << std::endl;
    return 1;
  }

  std::string pluginPath = argv[1];

  gz::plugin::Loader pl;
  auto plugins = pl.LoadLib(pluginPath);

  // Look for 3d plugins
  auto pluginNames = gz::physics::FindFeatures3d<Features>::From(pl);
  if (pluginNames.empty())
  {
    std::cerr << "No plugins with required features found in "
              << pluginPath
              << std::endl;
  }

  for (const std::string &name : pluginNames)
  {
    std::cout << "Testing plugin: " << name << std::endl;
    gz::plugin::PluginPtr plugin = pl.Instantiate(name);

    auto engine = gz::physics::RequestEngine3d<Features>::From(plugin);

    std::cout << "  engine name: " << engine->GetName() << std::endl;
  }
}
//! [main]