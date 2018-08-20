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

#include <gtest/gtest.h>

#include <ignition/plugin/Loader.hh>

#include <ignition/physics/Joint.hh>
#include <ignition/physics/RequestEngine.hh>

#include <ignition/physics/sdf/ConstructWorld.hh>

#include <ignition/physics/dartsim/World.hh>

#include <sdf/Root.hh>
#include <sdf/World.hh>

#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/DegreeOfFreedom.hpp>
#include <dart/dynamics/RevoluteJoint.hpp>

using TestFeatureList = ignition::physics::FeatureList<
  ignition::physics::GetBasicJointState,
  ignition::physics::SetBasicJointState,
  ignition::physics::dartsim::RetrieveWorld,
  ignition::physics::sdf::ConstructSdfWorld
>;

using World = ignition::physics::World3d<TestFeatureList>;

World LoadWorld(const std::string &_world)
{
  ignition::plugin::Loader loader;
  loader.LoadLibrary(dartsim_plugin_LIB);
  ignition::plugin::PluginPtr dartsim =
      loader.Instantiate("ignition::physics::dartsim::Plugin");

  auto engine =
      ignition::physics::RequestEngine3d<TestFeatureList>::From(dartsim);
  EXPECT_NE(nullptr, engine);

  sdf::Root root;
  const sdf::Errors &errors = root.Load(_world);
  EXPECT_EQ(0u, errors.size());

  EXPECT_EQ(1u, root.WorldCount());
  const sdf::World *sdfWorld = root.WorldByIndex(0);
  EXPECT_NE(nullptr, sdfWorld);

  auto world = engine->ConstructWorld(*sdfWorld);
  EXPECT_NE(nullptr, world);

  return *world;
}

// Test that the dartsim plugin loaded all the relevant information correctly.
TEST(SDFFeatures_TEST, CheckDartsimData)
{
  World world = LoadWorld(TEST_WORLD_DIR"/test.world");

  dart::simulation::WorldPtr dartWorld = world.GetDartsimWorld();
  ASSERT_NE(nullptr, dartWorld);

  ASSERT_EQ(2u, dartWorld->getNumSkeletons());

  const dart::dynamics::SkeletonPtr skeleton = dartWorld->getSkeleton(1);
  ASSERT_EQ(3u, skeleton->getNumBodyNodes());

  auto verify = [](const dart::dynamics::DegreeOfFreedom * dof,
                   double initialPos, double damping, double friction,
                   double springRest, double stiffness, double lower,
                   double upper, double maxForce, double maxVelocity)
  {
    EXPECT_DOUBLE_EQ(initialPos, dof->getPosition());
    EXPECT_DOUBLE_EQ(initialPos, dof->getInitialPosition());
    EXPECT_DOUBLE_EQ(damping, dof->getDampingCoefficient());
    EXPECT_DOUBLE_EQ(friction, dof->getCoulombFriction());
    EXPECT_DOUBLE_EQ(springRest, dof->getRestPosition());
    EXPECT_DOUBLE_EQ(stiffness, dof->getSpringStiffness());
    EXPECT_DOUBLE_EQ(lower, dof->getPositionLowerLimit());
    EXPECT_DOUBLE_EQ(upper, dof->getPositionUpperLimit());
    EXPECT_DOUBLE_EQ(-maxForce, dof->getForceLowerLimit());
    EXPECT_DOUBLE_EQ( maxForce, dof->getForceUpperLimit());
    EXPECT_DOUBLE_EQ(-maxVelocity, dof->getVelocityLowerLimit());
    EXPECT_DOUBLE_EQ( maxVelocity, dof->getVelocityUpperLimit());
  };

  // Test that things were parsed correctly. These values are either stated or
  // implied in the test.world SDF file.
  verify(skeleton->getJoint(1)->getDof(0),
         1.5706796, 3.0, 0.0, 0.0, 0.0, -1e16, 1e16,
         std::numeric_limits<double>::infinity(),
         std::numeric_limits<double>::infinity());

  verify(skeleton->getJoint(2)->getDof(0),
         -0.429462, 3.0, 0.0, 0.0, 0.0, -1e16, 1e16,
         std::numeric_limits<double>::infinity(),
         std::numeric_limits<double>::infinity());

  for (const auto * joint : {skeleton->getJoint(1), skeleton->getJoint(2)})
  {
    const auto * revolute =
        dynamic_cast<const dart::dynamics::RevoluteJoint*>(joint);
    ASSERT_NE(nullptr, revolute);

    const Eigen::Vector3d &axis = revolute->getAxis();
    EXPECT_DOUBLE_EQ(1.0, axis[0]);
    EXPECT_DOUBLE_EQ(0.0, axis[1]);
    EXPECT_DOUBLE_EQ(0.0, axis[2]);
  }
}

int main(int argc, char *argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
