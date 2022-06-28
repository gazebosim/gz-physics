/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include <test/PhysicsPluginsList.hh>

#include <gz/physics/FindFeatures.hh>
#include <gz/physics/RequestEngine.hh>
#include <gz/plugin/Loader.hh>

// Features
#include <gz/physics/ConstructEmpty.hh>
#include <gz/physics/ForwardStep.hh>
#include <gz/physics/FrameSemantics.hh>
#include <gz/physics/FreeJoint.hh>
#include <gz/physics/GetEntities.hh>
#include <gz/physics/mesh/MeshShape.hh>
#include <gz/physics/PlaneShape.hh>
#include <gz/physics/FixedJoint.hh>

#include <gz/common/MeshManager.hh>

using TestFeatureList = gz::physics::FeatureList<
  gz::physics::LinkFrameSemantics,
  gz::physics::ForwardStep,
  gz::physics::GetEntities,
  gz::physics::ConstructEmptyWorldFeature,
  gz::physics::ConstructEmptyModelFeature,
  gz::physics::ConstructEmptyLinkFeature,
  gz::physics::mesh::AttachMeshShapeFeature,
  gz::physics::AttachPlaneShapeFeature,
  gz::physics::SetFreeJointRelativeTransformFeature,
  gz::physics::AttachFixedJointFeature
>;

using TestWorldPtr = gz::physics::World3dPtr<TestFeatureList>;
using TestEnginePtr = gz::physics::Engine3dPtr<TestFeatureList>;

using WorldConstructor = std::function<TestWorldPtr(const TestEnginePtr&)>;

std::unordered_set<TestWorldPtr> LoadWorlds(
    const std::string &_library,
    const WorldConstructor &_constructor)
{
  gz::plugin::Loader loader;
  loader.LoadLib(_library);

  const std::set<std::string> pluginNames =
      gz::physics::FindFeatures3d<TestFeatureList>::From(loader);

  std::unordered_set<TestWorldPtr> worlds;
  for (const std::string &name : pluginNames)
  {
    gz::plugin::PluginPtr plugin = loader.Instantiate(name);

    std::cout << " -- Plugin name: " << name << std::endl;

    auto engine =
        gz::physics::RequestEngine3d<TestFeatureList>::From(plugin);
    EXPECT_NE(nullptr, engine);

    worlds.insert(_constructor(engine));
  }

  return worlds;
}

class Collisions_TEST
    : public ::testing::Test,
      public ::testing::WithParamInterface<std::string>
{};

INSTANTIATE_TEST_SUITE_P(PhysicsPlugins, Collisions_TEST,
    ::testing::ValuesIn(gz::physics::test::g_PhysicsPluginLibraries));

TestWorldPtr ConstructMeshPlaneWorld(
    const gz::physics::Engine3dPtr<TestFeatureList> &_engine,
    const gz::common::Mesh &_mesh)
{
  auto world = _engine->ConstructEmptyWorld("world");

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation()[2] = 2.0;

  auto model = world->ConstructEmptyModel("mesh");
  auto link = model->ConstructEmptyLink("link");
  // TODO(anyone): This test is somewhat awkward because we lift up the mesh
  // from the center of the link instead of lifting up the link or the model.
  // We're doing this because we don't currently have an API for moving models
  // or links around. See the conversation here for more:
  // https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-physics/pull-requests/46/page/1#comment-87592809
  link->AttachMeshShape("mesh", _mesh, tf);

  model = world->ConstructEmptyModel("plane");
  link = model->ConstructEmptyLink("link");

  link->AttachPlaneShape("plane", gz::physics::LinearVector3d::UnitZ());
  link->AttachFixedJoint(nullptr);

  return world;
}

TEST_P(Collisions_TEST, MeshAndPlane)
{
  const std::string library = GetParam();
  if (library.empty())
    return;

  const std::string meshFilename = GZ_PHYSICS_RESOURCE_DIR "/chassis.dae";
  auto &meshManager = *gz::common::MeshManager::Instance();
  auto *mesh = meshManager.Load(meshFilename);

  std::cout << "Testing library " << library << std::endl;
  auto worlds = LoadWorlds(library, [&](const TestEnginePtr &_engine)
  {
    return ConstructMeshPlaneWorld(_engine, *mesh);
  });

  for (const auto &world : worlds)
  {
    const auto link = world->GetModel(0)->GetLink(0);

    EXPECT_NEAR(
          0.0, link->FrameDataRelativeToWorld().pose.translation()[2], 1e-6);

    gz::physics::ForwardStep::Output output;
    gz::physics::ForwardStep::State state;
    gz::physics::ForwardStep::Input input;
    for (std::size_t i = 0; i < 1000; ++i)
    {
      world->Step(output, state, input);
    }

    // Make sure the mesh was stopped by the plane. In 2000 time steps at the
    // default step size of 0.001, a free falling body should drop approximately
    // 19.6 meters. As long as the body is somewhere near 1.91, then it has been
    // stopped by the plane (the exact value might vary because the body might
    // be rocking side-to-side after falling).
    EXPECT_NEAR(
          -1.91, link->FrameDataRelativeToWorld().pose.translation()[2], 0.05);
  }
}

int main(int argc, char *argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
