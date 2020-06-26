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

#ifndef TEST_PHYSICSPLUGINLIST_HH_
#define TEST_PHYSICSPLUGINLIST_HH_

#include <string>
#include <vector>

#include <ignition/common/Filesystem.hh>
#include <ignition/common/Util.hh>

#ifndef dartsim_plugin_LIB
#define dartsim_plugin_LIB "__main__/ign_physics/dartsim/libignition-physics-dartsim-plugin.so"
#endif

#ifndef bullet_plugin_LIB
#define bullet_plugin_LIB ""
#endif

#ifndef tpe_plugin_LIB
#define tpe_plugin_LIB "__main__/ign_physics/tpe/libignition-physics-tpe-plugin.so"
#endif

/////////////////////////////////////////////////
std::string resolveLibrary(const std::string &_path)
{
  auto it = _path.find("__main__");
  if(it != std::string::npos && it == 0)
  {
    std::string dataDir;
    if (ignition::common::env("TEST_SRCDIR", dataDir))
    {
      return ignition::common::joinPaths(dataDir, _path);
    }
  }
  return _path;
}

/////////////////////////////////////////////////
std::string DartsimPluginLib()
{
  return resolveLibrary(dartsim_plugin_LIB);
}

/////////////////////////////////////////////////
std::string TpePluginLib()
{
  return resolveLibrary(tpe_plugin_LIB);
}

/////////////////////////////////////////////////
std::string BulletPluginLib()
{
  return resolveLibrary(bullet_plugin_LIB);
}

namespace ignition
{
  namespace physics
  {
    namespace test
    {
      const std::vector<std::string> g_PhysicsPluginLibraries = {
        dartsim_plugin_LIB,
        bullet_plugin_LIB,
        tpe_plugin_LIB
      };
    }
  }
}

#endif
