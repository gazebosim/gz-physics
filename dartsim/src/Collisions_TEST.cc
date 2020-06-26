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

#include <ignition/physics/FindFeatures.hh>
#include <ignition/physics/RequestEngine.hh>
#include <ignition/plugin/Loader.hh>

// Features
#include <ignition/physics/ConstructEmpty.hh>
#include <ignition/physics/ForwardStep.hh>
#include <ignition/physics/FrameSemantics.hh>
#include <ignition/physics/FreeJoint.hh>
#include <ignition/physics/GetEntities.hh>
#include <ignition/physics/mesh/MeshShape.hh>
#include <ignition/physics/PlaneShape.hh>
#include <ignition/physics/FixedJoint.hh>

#include <ignition/common/MeshManager.hh>

using TestFeatureList = ignition::physics::FeatureList<
  ignition::physics::LinkFrameSemantics,
  ignition::physics::ForwardStep,
  ignition::physics::GetEntities,
  ignition::physics::ConstructEmptyWorldFeature,
  ignition::physics::ConstructEmptyModelFeature,
  ignition::physics::ConstructEmptyLinkFeature,
  ignition::physics::mesh::AttachMeshShapeFeature,
  ignition::physics::AttachPlaneShapeFeature,
  ignition::physics::SetFreeJointRelativeTransformFeature,
  ignition::physics::AttachFixedJointFeature
>;

using TestWorldPtr = ignition::physics::World3dPtr<TestFeatureList>;
using TestEnginePtr = ignition::physics::Engine3dPtr<TestFeatureList>;

using WorldConstructor = std::function<TestWorldPtr(const TestEnginePtr&)>;

std::unordered_set<TestWorldPtr> LoadWorlds(
    const std::string &_library,
    const WorldConstructor &_constructor)
{
  ignition::plugin::Loader loader;
  loader.LoadLib(resolveLibrary(_library));

  const std::set<std::string> pluginNames =
      ignition::physics::FindFeatures3d<TestFeatureList>::From(loader);

  std::unordered_set<TestWorldPtr> worlds;
  for (const std::string &name : pluginNames)
  {
    ignition::plugin::PluginPtr plugin = loader.Instantiate(name);

    std::cout << " -- Plugin name: " << name << std::endl;

    auto engine =
        ignition::physics::RequestEngine3d<TestFeatureList>::From(plugin);
    EXPECT_NE(nullptr, engine);

    worlds.insert(_constructor(engine));
  }

  return worlds;
}

class Collisions_TEST
    : public ::testing::Test,
      public ::testing::WithParamInterface<std::string>
{};

INSTANTIATE_TEST_SUITE_P(
    PhysicsPlugins,
    Collisions_TEST,
    ::testing::ValuesIn(ignition::physics::test::g_PhysicsPluginLibraries)); // NOLINT

TestWorldPtr ConstructMeshPlaneWorld(
    const ignition::physics::Engine3dPtr<TestFeatureList> &_engine,
    const ignition::common::Mesh &_mesh)
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

  link->AttachPlaneShape("plane", ignition::physics::LinearVector3d::UnitZ());
  link->AttachFixedJoint(nullptr);

  return world;
}

TEST_P(Collisions_TEST, MeshAndPlane)
{
  const std::string library = GetParam();
  if (library.empty())
    return;

  const std::string meshFilename = IGNITION_PHYSICS_RESOURCE_DIR "/chassis.dae";
  auto &meshManager = *ignition::common::MeshManager::Instance();
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

    ignition::physics::ForwardStep::Output output;
    ignition::physics::ForwardStep::State state;
    ignition::physics::ForwardStep::Input input;
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
