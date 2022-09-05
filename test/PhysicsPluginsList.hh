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

#ifndef dartsim_plugin_LIB
#define dartsim_plugin_LIB ""
#endif

#ifndef bullet_plugin_LIB
#define bullet_plugin_LIB ""
#endif

#ifndef tpe_plugin_LIB
#define tpe_plugin_LIB ""
#endif

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
