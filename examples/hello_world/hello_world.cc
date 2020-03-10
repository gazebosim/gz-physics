/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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
#include <ignition/plugin/PluginPtr.hh>

#include <ignition/physics/FindFeatures.hh>
#include <ignition/physics/GetEntities.hh>
#include <ignition/physics/RequestEngine.hh>

using Features = ignition::physics::FeatureList<
    ignition::physics::GetEngineInfo
>;

int main(int argc, char **argv)
{
  ignition::plugin::Loader pl;
  auto plugins = pl.LoadLib(HelloWorldPlugin_LIB);

  // Look for 3d plugins
  auto pluginNames = ignition::physics::FindFeatures3d<Features>::From(pl);
  if (pluginNames.empty())
  {
    std::cerr << "No plugins with required features found in "
              << HelloWorldPlugin_LIB
              << std::endl;
  }

  for (const std::string & name : pluginNames)
  {
    std::cout << "Testing plugin: " << name << std::endl;
    ignition::plugin::PluginPtr plugin = pl.Instantiate(name);

    auto engine = ignition::physics::RequestEngine3d<Features>::From(plugin);

    std::cout << "  engine name: " << engine->GetName() << std::endl;
  }
}
