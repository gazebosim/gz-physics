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

#include <gz/plugin/Loader.hh>

#include <gz/common/geospatial/Dem.hh>
#include <gz/common/geospatial/ImageHeightmap.hh>
#include <gz/common/MeshManager.hh>
#include <gz/common/Filesystem.hh>

#include <gz/math/eigen3/Conversions.hh>

#include <gz/physics/Joint.hh>
#include <gz/physics/RequestEngine.hh>
#include <gz/physics/RevoluteJoint.hh>

#include "EntityManagementFeatures.hh"
#include "JointFeatures.hh"
#include "KinematicsFeatures.hh"
#include "ShapeFeatures.hh"

#include "test/Resources.hh"

using namespace gz;

struct TestFeatureList : physics::FeatureList<
    physics::dartsim::EntityManagementFeatureList,
    physics::dartsim::JointFeatureList,
    physics::dartsim::KinematicsFeatureList,
    physics::dartsim::ShapeFeatureList
> { };

TEST(EntityManagement_TEST, ConstructEmptyWorld)
{
  plugin::Loader loader;
  loader.LoadLib(dartsim_plugin_LIB);

  plugin::PluginPtr dartsim =
      loader.Instantiate("gz::physics::dartsim::Plugin");

  auto engine =
      physics::RequestEngine3d<TestFeatureList>::From(dartsim);
  ASSERT_NE(nullptr, engine);

  auto world = engine->ConstructEmptyWorld("empty world");
  ASSERT_NE(nullptr, world);

  auto model = world->ConstructEmptyModel("empty model");
  ASSERT_NE(nullptr, model);

  auto child = model->ConstructEmptyLink("child link");
  ASSERT_NE(nullptr, child);

  const math::Pose3d pose(0, 0, 0.2, 0, 0, 0);

  //  dem heightmap
  auto demLink = model->ConstructEmptyLink("dem_link");
  demLink->AttachFixedJoint(child, "dem_joint");

  auto demFilename = gz::physics::test::resources::kVolcanoTif;
  common::Dem dem;
  EXPECT_EQ(0, dem.Load(demFilename));

  math::Vector3d sizeDem;
  sizeDem.X(dem.WorldWidth());
  sizeDem.Y(dem.WorldHeight());
  sizeDem.Z(dem.MaxElevation() - dem.MinElevation());

  auto demShape = demLink->AttachHeightmapShape("dem", dem,
      math::eigen3::convert(pose),
      math::eigen3::convert(sizeDem));

  // there is a loss in precision with large dems since heightmaps use floats
  EXPECT_NEAR(sizeDem.X(), demShape->GetSize()[0], 1e-3);
  EXPECT_NEAR(sizeDem.Y(), demShape->GetSize()[1], 1e-3);
  EXPECT_NEAR(sizeDem.Z(), demShape->GetSize()[2], 1e-6);

  auto demShapeGeneric = demLink->GetShape("dem");
  ASSERT_NE(nullptr, demShapeGeneric);
  EXPECT_EQ(nullptr, demShapeGeneric->CastToBoxShape());
  auto demShapeRecast = demShapeGeneric->CastToHeightmapShape();
  ASSERT_NE(nullptr, demShapeRecast);
  EXPECT_NEAR(sizeDem.X(), demShapeRecast->GetSize()[0], 1e-3);
  EXPECT_NEAR(sizeDem.Y(), demShapeRecast->GetSize()[1], 1e-3);
  EXPECT_NEAR(sizeDem.Z(), demShapeRecast->GetSize()[2], 1e-6);
}

