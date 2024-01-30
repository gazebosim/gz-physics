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

#ifndef TEST_RESOURCES_HH_
#define TEST_RESOURCES_HH_

#include <gz/common/testing/TestPaths.hh>

#include <string>

inline std::string TestResource(const std::string &_filename)
{
  return gz::common::testing::TestFile("resources", _filename);
}

namespace gz::physics::test::resources
{
const auto kChassisDae = TestResource("chassis.dae");
const auto kHeightmapBowlPng = TestResource("heightmap_bowl.png");
const auto kRrbotXml = TestResource("rrbot.xml");
const auto kVolcanoTif = TestResource("volcano.tif");
}  // namespace gz::physics::test

#endif  // TEST_RESOURCES_HH_
