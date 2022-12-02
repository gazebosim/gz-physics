/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#include "TestLibLoader.hh"

#include <iostream>
#include <string>
#include <vector>

#include <gz/common/Filesystem.hh>
#include <gz/common/Util.hh>

std::string gz::physics::TestLibLoader::libToTest = std::string("");

namespace gz
{
namespace physics
{
  bool TestLibLoader::init(int argc, char *argv[])
  {
    if (argc != 2)
    {
      std::cerr << "Please provide the path to an engine plugin.\n"
                << "Usage " <<  argv[0] << " <physics engine path>\n";
      return false;
    }
    libToTest = argv[1];
    return true;
  }

  std::string TestLibLoader::GetLibToTest()
  {
    return libToTest;
  }

  std::string TestLibLoader::ResourceDir()
  {
    // First check the GZ_PHYSICS_RESOURCE_DIR envrionment variable and return
    // that value if it is set.
    std::string resourceDirFromEnv;
    if (common::env("GZ_PHYSICS_RESOURCE_DIR", resourceDirFromEnv))
    {
      return resourceDirFromEnv;
    }
    // Otherwise return the configure-time constant.
    return GZ_PHYSICS_RESOURCE_INSTALL_DIR;
  }

  std::string TestLibLoader::PhysicsEngineName(std::string _name)
  {
    std::vector<std::string> tokens = gz::common::split(_name, "::");
    if (tokens.size() == 4)
    {
      std::string physicsEngineName = tokens[2];
      std::string physicsEnginePluginName = physicsEngineName;
      if (physicsEngineName == "bullet_featherstone")
      {
        physicsEnginePluginName = "bullet-featherstone";
      }
      if (physicsEngineName == "tpeplugin")
      {
        physicsEnginePluginName = "tpe";
      }
      return physicsEnginePluginName;
    }
    return "";
  }
}
}
