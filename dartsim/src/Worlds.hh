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

#ifndef DARTSIM_WORLDS_HH_
#define DARTSIM_WORLDS_HH_

#include <gz/common/testing/TestPaths.hh>

#include <string>

inline std::string DartsimTestWorld(const std::string &_world)
{
  return gz::common::testing::TestFile("dartsim", "worlds", _world);
}

namespace dartsim::worlds
{
const auto kAddedMassSdf = DartsimTestWorld("added_mass.sdf");
const auto kEmptySdf = DartsimTestWorld("empty.sdf");
const auto kFallingWorld = DartsimTestWorld("falling.world");
const auto kJointAcrossNestedModelsSdf =
  DartsimTestWorld("joint_across_nested_models.sdf");
const auto kModelFramesSdf = DartsimTestWorld("kModelFrames.sdf");
const auto kShapesWorld = DartsimTestWorld("shapes.world");
const auto kShapesBitmaskWorld = DartsimTestWorld("shapes_bitmask.sdf");
const auto kTestWorld = DartsimTestWorld("test.world");
const auto kWorldFramesSdf = DartsimTestWorld("world_frames.sdf");
const auto kWorldWithNestedModelSdf =
  DartsimTestWorld("world_with_nested_model.sdf");
const auto kWorldWithNestedModelJointToWorldSdf =
  DartsimTestWorld("world_with_nested_model_joint_to_world.sdf");

}  // namespace dartsim::worlds
#endif  // DARTSIM_WORLDS_HH_
