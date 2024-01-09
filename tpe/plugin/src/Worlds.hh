/*
 * Copyright (C) 2024 Open Source Robotics Foundation
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

#ifndef TPE_PLUGIN_WORLDS_HH_
#define TPE_PLUGIN_WORLDS_HH_

#include <gz/common/testing/TestPaths.hh>

#include <string>

// \brief retrieve a filename from the tpe/plugin/worlds directory
// \param[in] _world filename to retrieve
// \return full path to the request world
inline std::string TpeTestWorld(const std::string &_world)
{
  return gz::common::testing::SourceFile("tpe", "plugin", "worlds", _world);
}

namespace tpe::worlds
{
const auto kNestedModelWorld = TpeTestWorld("nested_model.world");
const auto kShapesWorld = TpeTestWorld("shapes.world");
const auto kShapesBitmaskSdf = TpeTestWorld("shapes_bitmask.sdf");
const auto kTestWorld = TpeTestWorld("test.world");
}  // namespace tpe::worlds
#endif  // TPE_PLUGIN_WORLDS_HH_
