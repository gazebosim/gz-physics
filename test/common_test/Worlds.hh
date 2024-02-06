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

#ifndef COMMON_TEST_WORLDS_WORLDS_HH_
#define COMMON_TEST_WORLDS_WORLDS_HH_

#include <gz/common/testing/TestPaths.hh>

#include <string>

inline std::string CommonTestWorld(const std::string &_world)
{
  return gz::common::testing::TestFile("common_test", "worlds", _world);
}

namespace common_test::worlds
{
const auto kContactSdf = CommonTestWorld("contact.sdf");
const auto kDetachableJointWorld = CommonTestWorld("detachable_joint.world");
const auto kEmptySdf = CommonTestWorld("empty.sdf");
const auto kFallingWorld = CommonTestWorld("falling.world");
const auto kFallingAddedMassWorld = CommonTestWorld("falling_added_mass.world");
const auto kGroundSdf = CommonTestWorld("ground.sdf");
const auto kJointAcrossModelsSdf = CommonTestWorld("joint_across_models.sdf");
const auto kJointConstraintSdf = CommonTestWorld("joint_constraint.sdf");
const auto kMimicFastSlowPendulumsWorld =
  CommonTestWorld("mimic_fast_slow_pendulums_world.sdf");
const auto kMimicPendulumWorld = CommonTestWorld("mimic_pendulum_world.sdf");
const auto kMimicPrismaticWorld = CommonTestWorld("mimic_prismatic_world.sdf");
const auto kMimicUniversalWorld = CommonTestWorld("mimic_universal_world.sdf");
const auto kMultipleCollisionsSdf = CommonTestWorld("multiple_collisions.sdf");
const auto kPendulumJointWrenchSdf =
  CommonTestWorld("pendulum_joint_wrench.sdf");
const auto kShapesWorld = CommonTestWorld("shapes.world");
const auto kShapesBitmaskWorld = CommonTestWorld("shapes_bitmask.sdf");
const auto kSlipComplianceSdf = CommonTestWorld("slip_compliance.sdf");
const auto kSphereSdf = CommonTestWorld("sphere.sdf");
const auto kStringPendulumSdf = CommonTestWorld("string_pendulum.sdf");
const auto kTestWorld = CommonTestWorld("test.world");
const auto kWorldJointTestSdf = CommonTestWorld("world_joint_test.sdf");
const auto kWorldUnsortedLinksSdf = CommonTestWorld("world_unsorted_links.sdf");
const auto kWorldWithNestedModelSdf =
  CommonTestWorld("world_with_nested_model.sdf");
}  // namespace common_test::worlds
#endif  // COMMON_TEST_WORLDS_WORLDS_HH_
