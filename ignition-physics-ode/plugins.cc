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

#include <ignition/common/PluginMacros.hh>

#include "BuggyDemoController.hh"


IGN_COMMON_BEGIN_ADDING_PLUGINS
  IGN_COMMON_ADD_PLUGIN(ignition::physics::ode::BuggyDemoController,
                        ignition::physics::ForwardStep)
  IGN_COMMON_ADD_PLUGIN(ignition::physics::ode::BuggyDemoController,
                        ignition::physics::SetState)
IGN_COMMON_FINISH_ADDING_PLUGINS
