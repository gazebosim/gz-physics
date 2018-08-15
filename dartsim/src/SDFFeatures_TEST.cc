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
#include <ignition/physics/RequestFeatures.hh>

#include <ignition/physics/sdf/ConstructWorld.hh>

#include <ignition/physics/dartsim/World.hh>

#include <sdf/Root.hh>
#include <sdf/World.hh>

#ifdef DARTSIM_GUI_OSG_AVAILABLE
#include <dart/gui/osg/Viewer.hpp>
#include <dart/gui/osg/WorldNode.hpp>
#include <dart/gui/osg/InteractiveFrame.hpp>
#endif

#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/WeldJoint.hpp>
#include <dart/dynamics/CylinderShape.hpp>

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
      ignition::physics::RequestFeatures3d<TestFeatureList>::From(dartsim);
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

TEST(SDFFeatures_TEST, CheckDartsimData)
{
  World world = LoadWorld(TEST_WORLD_DIR"/test.world");

  dart::simulation::WorldPtr dartWorld = world.GetDartsimWorld();
  ASSERT_NE(nullptr, dartWorld);

  ASSERT_EQ(2u, dartWorld->getNumSkeletons());

  const dart::dynamics::SkeletonPtr skeleton = dartWorld->getSkeleton(0);
  ASSERT_EQ(3u, skeleton->getNumBodyNodes());


  // TODO(MXG): Delete this temporary test code
#ifdef DARTSIM_GUI_OSG_AVAILABLE

  if(false)
  {
    dart::gui::osg::Viewer viewer;
    osg::ref_ptr<dart::gui::osg::WorldNode> worldNode =
        new dart::gui::osg::WorldNode(dartWorld);
    worldNode->setNumStepsPerCycle(200);
    viewer.addWorldNode(worldNode);

    viewer.run();
  }

#endif
}

int main(int argc, char *argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
