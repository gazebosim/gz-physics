/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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
 */

#include <gtest/gtest.h>

#include <string>

#include <gz/common/Console.hh>
#include <gz/common/Util.hh>
#include <gz/physics/ConstructEmpty.hh>
#include <gz/physics/ForwardStep.hh>
#include <gz/physics/RequestEngine.hh>
#include <gz/physics/sdf/ConstructModel.hh>
#include <gz/physics/sdf/ConstructWorld.hh>
#include <gz/plugin/Loader.hh>
#include <sdf/Root.hh>
#include <sdf/World.hh>

#include "Base.hh"
#include <mujoco/mujoco.h>

using namespace gz;

struct ThreadPoolFeatureList : physics::FeatureList<
    physics::ConstructEmptyWorldFeature,
    physics::ForwardStep,
    physics::sdf::ConstructSdfModel,
    physics::sdf::ConstructSdfWorld
> { };

using WorldPtr = physics::World3dPtr<ThreadPoolFeatureList>;

namespace {

constexpr const char *kEnvVar = "GZ_PHYSICS_MUJOCO_THREADS";

/// \brief Sets the thread count environment variable for the duration of a
/// test and restores whatever was there before, so that one test's setting
/// cannot leak into the next.
class ScopedThreadEnv
{
  /// \brief Set the variable to _value.
  public: explicit ScopedThreadEnv(const std::string &_value)
  {
    this->Save();
    gz::common::setenv(kEnvVar, _value);
  }

  /// \brief Remove the variable entirely.
  public: ScopedThreadEnv()
  {
    this->Save();
    gz::common::unsetenv(kEnvVar);
  }

  public: ~ScopedThreadEnv()
  {
    if (this->hadValue)
      gz::common::setenv(kEnvVar, this->previousValue);
    else
      gz::common::unsetenv(kEnvVar);
  }

  private: void Save()
  {
    std::string previous;
    this->hadValue = gz::common::env(kEnvVar, previous);
    this->previousValue = previous;
  }

  private: bool hadValue{false};
  private: std::string previousValue;
};

/////////////////////////////////////////////////
auto LoadEngine()
{
  plugin::Loader loader;
  loader.LoadLib(mujoco_plugin_LIB);

  plugin::PluginPtr mujoco =
      loader.Instantiate("gz::physics::mujoco::Plugin");

  return physics::RequestEngine3d<ThreadPoolFeatureList>::From(mujoco);
}

/////////////////////////////////////////////////
/// \brief Reach the plugin's internal WorldInfo so the pool can be observed.
/// The pool pointer is not reachable through the public physics API.
physics::mujoco::WorldInfo *GetWorldInfo(const WorldPtr &_world)
{
  return static_cast<physics::mujoco::WorldInfo *>(
      _world->FullIdentity().ref.get());
}

/////////////////////////////////////////////////
/// \brief A one-model world, enough to force a recompile during construction.
std::string OneModelWorld()
{
  return R"(<?xml version="1.0" ?>
    <sdf version="1.11">
      <world name="threadpool_world">
        <model name="box">
          <link name="link">
            <collision name="collision">
              <geometry><box><size>1 1 1</size></box></geometry>
            </collision>
          </link>
        </model>
      </world>
    </sdf>)";
}

/////////////////////////////////////////////////
/// \brief A standalone model, for adding to an already-constructed world.
std::string ExtraModel(const std::string &_name)
{
  return R"(<?xml version="1.0" ?>
    <sdf version="1.11">
      <model name=")" + _name + R"(">
        <link name="link">
          <collision name="collision">
            <geometry><sphere><radius>0.5</radius></sphere></geometry>
          </collision>
        </link>
      </model>
    </sdf>)";
}

/////////////////////////////////////////////////
/// \brief Advance the world one step. Adding a model only marks the spec
/// dirty; the recompile is deferred until something needs the compiled model,
/// so a step is what actually exercises Base::RecompileSpec.
void StepWorld(const WorldPtr &_world)
{
  physics::ForwardStep::Input input;
  physics::ForwardStep::State state;
  physics::ForwardStep::Output output;
  _world->Step(output, state, input);
}

/////////////////////////////////////////////////
WorldPtr LoadWorldString(const std::string &_worldSdf)
{
  auto engine = LoadEngine();
  EXPECT_NE(nullptr, engine);
  if (nullptr == engine)
    return nullptr;

  ::sdf::Root root;
  const auto errors = root.LoadSdfString(_worldSdf);
  EXPECT_TRUE(errors.empty()) << errors;
  EXPECT_NE(nullptr, root.WorldByIndex(0));

  return engine->ConstructWorld(*root.WorldByIndex(0));
}
}  // namespace

/////////////////////////////////////////////////
// With the variable unset, no pool is created. This is the default and must
// stay byte-identical to the behavior before threading was added.
TEST(ThreadPool, UnsetEnvCreatesNoPool)
{
  common::Console::SetVerbosity(4);
  ScopedThreadEnv guard;

  auto engine = LoadEngine();
  ASSERT_NE(nullptr, engine);
  auto world = engine->ConstructEmptyWorld("empty");
  ASSERT_NE(nullptr, world);

  auto *worldInfo = GetWorldInfo(world);
  ASSERT_NE(nullptr, worldInfo);
  ASSERT_NE(nullptr, worldInfo->mjDataObj);
  EXPECT_EQ(0, worldInfo->threadPoolSize);
  EXPECT_EQ(0u, worldInfo->mjDataObj->threadpool);
}

/////////////////////////////////////////////////
// An explicit 0 is the same as unset.
TEST(ThreadPool, ExplicitZeroCreatesNoPool)
{
  common::Console::SetVerbosity(4);
  ScopedThreadEnv guard{"0"};

  auto engine = LoadEngine();
  ASSERT_NE(nullptr, engine);
  auto world = engine->ConstructEmptyWorld("empty");
  ASSERT_NE(nullptr, world);

  auto *worldInfo = GetWorldInfo(world);
  ASSERT_NE(nullptr, worldInfo);
  EXPECT_EQ(0, worldInfo->threadPoolSize);
  EXPECT_EQ(0u, worldInfo->mjDataObj->threadpool);
}

/////////////////////////////////////////////////
TEST(ThreadPool, PositiveValueCreatesPool)
{
  common::Console::SetVerbosity(4);
  ScopedThreadEnv guard{"2"};

  auto engine = LoadEngine();
  ASSERT_NE(nullptr, engine);
  auto world = engine->ConstructEmptyWorld("empty");
  ASSERT_NE(nullptr, world);

  auto *worldInfo = GetWorldInfo(world);
  ASSERT_NE(nullptr, worldInfo);
  EXPECT_EQ(2, worldInfo->threadPoolSize);
  EXPECT_NE(0u, worldInfo->mjDataObj->threadpool);
}

/////////////////////////////////////////////////
// Garbage must disable threading rather than being fatal or being silently
// coerced into some arbitrary thread count.
TEST(ThreadPool, UnparseableValueCreatesNoPool)
{
  common::Console::SetVerbosity(4);

  for (const auto &bad : {"banana", "-2", "4x", "3.5"})
  {
    ScopedThreadEnv guard{bad};

    auto engine = LoadEngine();
    ASSERT_NE(nullptr, engine);
    auto world = engine->ConstructEmptyWorld("empty");
    ASSERT_NE(nullptr, world) << "value: " << bad;

    auto *worldInfo = GetWorldInfo(world);
    ASSERT_NE(nullptr, worldInfo);
    EXPECT_EQ(0, worldInfo->threadPoolSize) << "value: " << bad;
    EXPECT_EQ(0u, worldInfo->mjDataObj->threadpool) << "value: " << bad;
  }
}

/////////////////////////////////////////////////
// Regression test for the hazard this feature is built around.
//
// mj_recompile reaches mj_makeRawData, which zeroes mjData::threadpool
// without destroying the pool behind it. Base::RecompileSpec runs whenever a
// model is added, including during world construction from SDF. Without the
// explicit teardown/reinstall bracket the worker threads are orphaned and
// stepping silently drops back to a single thread.
//
// Removing the bracket in Base::RecompileSpec makes this fail.
TEST(ThreadPool, PoolSurvivesWorldConstructionRecompile)
{
  common::Console::SetVerbosity(4);
  ScopedThreadEnv guard{"2"};

  auto world = LoadWorldString(OneModelWorld());
  ASSERT_NE(nullptr, world);

  auto *worldInfo = GetWorldInfo(world);
  ASSERT_NE(nullptr, worldInfo);
  ASSERT_NE(nullptr, worldInfo->mjDataObj);
  EXPECT_EQ(2, worldInfo->threadPoolSize);
  EXPECT_NE(0u, worldInfo->mjDataObj->threadpool)
      << "the pool must be reinstalled after the recompile triggered by "
         "adding the world's models";
}

/////////////////////////////////////////////////
// The same hazard, but for recompiles triggered well after construction.
// Several add-then-step cycles must each leave the pool intact.
TEST(ThreadPool, PoolSurvivesRepeatedModelAdditions)
{
  common::Console::SetVerbosity(4);
  ScopedThreadEnv guard{"2"};

  auto engine = LoadEngine();
  ASSERT_NE(nullptr, engine);
  auto world = engine->ConstructEmptyWorld("empty");
  ASSERT_NE(nullptr, world);

  auto *worldInfo = GetWorldInfo(world);
  ASSERT_NE(nullptr, worldInfo);
  ASSERT_NE(0u, worldInfo->mjDataObj->threadpool);

  for (int i = 0; i < 3; ++i)
  {
    const std::string name = "model_" + std::to_string(i);

    ::sdf::Root root;
    const auto errors = root.LoadSdfString(ExtraModel(name));
    ASSERT_TRUE(errors.empty()) << errors;
    ASSERT_NE(nullptr, root.Model());

    auto model = world->ConstructModel(*root.Model());
    ASSERT_NE(nullptr, model) << "iteration " << i;

    // The addition alone only sets specDirty; stepping is what recompiles.
    StepWorld(world);

    EXPECT_NE(0u, worldInfo->mjDataObj->threadpool) << "iteration " << i;
  }
}

/////////////////////////////////////////////////
// Stepping repeatedly must not churn the pool: mju_threadpool treats a
// request for the current size as a no-op, so the same pool should persist.
TEST(ThreadPool, PoolIdentityStableAcrossSteps)
{
  common::Console::SetVerbosity(4);
  ScopedThreadEnv guard{"2"};

  auto world = LoadWorldString(OneModelWorld());
  ASSERT_NE(nullptr, world);

  auto *worldInfo = GetWorldInfo(world);
  ASSERT_NE(nullptr, worldInfo);
  const uintptr_t poolBefore = worldInfo->mjDataObj->threadpool;
  ASSERT_NE(0u, poolBefore);

  for (int i = 0; i < 10; ++i)
    StepWorld(world);

  EXPECT_EQ(poolBefore, worldInfo->mjDataObj->threadpool);
}

/////////////////////////////////////////////////
// A world that never asked for threading must not acquire a pool as a side
// effect of a recompile.
TEST(ThreadPool, RecompileCreatesNoPoolWhenDisabled)
{
  common::Console::SetVerbosity(4);
  ScopedThreadEnv guard;

  auto world = LoadWorldString(OneModelWorld());
  ASSERT_NE(nullptr, world);

  auto *worldInfo = GetWorldInfo(world);
  ASSERT_NE(nullptr, worldInfo);
  ASSERT_NE(nullptr, worldInfo->mjDataObj);
  EXPECT_EQ(0u, worldInfo->mjDataObj->threadpool);
}
