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

/// \file
/// Benchmark and equivalence test for MuJoCo's mju_threadpool, driven
/// entirely through the public gz-physics API.
///
/// The scene is a grid of free boxes dropped onto a ground plane and spaced
/// so they do not touch each other. That produces one constraint island per
/// box, which is what mju_threadpool parallelises. Small scenes are actively
/// misleading here: with one or two islands the pool overhead dominates and
/// the result reads as "threading hurts", which is a property of the scene
/// rather than of the engine.
///
/// Two timings are reported per thread count: the full gz-physics
/// World::Step, and a raw mj_step loop on the same model. The gap between
/// them is the plugin's own per-step cost, which is serial and grows with
/// link count. It matters because it determines how much of MuJoCo's
/// threading gain actually reaches a gz-physics user.
///
/// Timings are reported, never asserted. Wall-clock thresholds flake on
/// loaded or low-core CI machines. The assertion is on equivalence: threading
/// must not change the simulated result.

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Geometry>

#include <gz/common/Console.hh>
#include <gz/common/Util.hh>
#include <gz/physics/FrameSemantics.hh>
#include <gz/physics/ForwardStep.hh>
#include <gz/physics/GetEntities.hh>
#include <gz/physics/RequestEngine.hh>
#include <gz/physics/sdf/ConstructWorld.hh>
#include <gz/plugin/Loader.hh>
#include <sdf/Root.hh>
#include <sdf/World.hh>

#include "Base.hh"
#include <mujoco/mujoco.h>

using namespace gz;

struct ThreadPoolBenchFeatureList : physics::FeatureList<
    physics::ForwardStep,
    physics::GetModelFromWorld,
    physics::GetLinkFromModel,
    physics::LinkFrameSemantics,
    physics::sdf::ConstructSdfWorld
> { };

using WorldPtr = physics::World3dPtr<ThreadPoolBenchFeatureList>;

namespace {

constexpr const char *kEnvVar = "GZ_PHYSICS_MUJOCO_THREADS";

/// \brief Boxes per side of the grid. 20x20 gives ~400 independent islands,
/// enough for island-parallel solving to be measurable without making the
/// single-threaded run unpleasantly slow in CI.
constexpr int kGridSide = 20;

/// \brief Steps used for the reported timing runs.
constexpr int kBenchmarkSteps = 500;

/// \brief Steps used for the equivalence assertion. Deliberately short:
/// island solving can reorder floating point accumulation, and a contact-rich
/// scene amplifies that over a long horizon.
constexpr int kEquivalenceSteps = 50;

/// \brief Position tolerance for the equivalence check, in metres.
constexpr double kEquivalenceTol = 1e-6;

/////////////////////////////////////////////////
/// \brief Restores the thread count environment variable when it goes out of
/// scope, so one case cannot leak into the next.
class ScopedThreadEnv
{
  public: explicit ScopedThreadEnv(int _nthread)
  {
    std::string previous;
    this->hadValue = gz::common::env(kEnvVar, previous);
    this->previousValue = previous;
    gz::common::setenv(kEnvVar, std::to_string(_nthread));
  }

  public: ~ScopedThreadEnv()
  {
    if (this->hadValue)
      gz::common::setenv(kEnvVar, this->previousValue);
    else
      gz::common::unsetenv(kEnvVar);
  }

  private: bool hadValue{false};
  private: std::string previousValue;
};

/////////////////////////////////////////////////
/// \brief Build a world of kGridSide^2 free boxes over a ground plane.
///
/// Boxes are spaced 2 m apart and dropped from 0.6 m so that they settle
/// independently. Each resting box couples only to the static ground, so it
/// forms its own constraint island.
std::string GridWorld()
{
  std::ostringstream world;
  world << R"(<?xml version="1.0" ?>
    <sdf version="1.11">
      <world name="threadpool_bench">
        <model name="ground">
          <static>true</static>
          <link name="link">
            <collision name="collision">
              <geometry><plane><normal>0 0 1</normal></plane></geometry>
            </collision>
          </link>
        </model>)";

  for (int i = 0; i < kGridSide; ++i)
  {
    for (int j = 0; j < kGridSide; ++j)
    {
      world << R"(
        <model name="box_)" << i << "_" << j << R"(">
          <pose>)" << (i * 2.0) << " " << (j * 2.0) << R"( 0.6 0 0 0</pose>
          <link name="link">
            <inertial>
              <mass>1.0</mass>
              <inertia>
                <ixx>0.1</ixx><iyy>0.1</iyy><izz>0.1</izz>
                <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
              </inertia>
            </inertial>
            <collision name="collision">
              <geometry><box><size>1 1 1</size></box></geometry>
            </collision>
          </link>
        </model>)";
    }
  }

  world << R"(
      </world>
    </sdf>)";

  return world.str();
}

/////////////////////////////////////////////////
WorldPtr LoadGridWorld()
{
  plugin::Loader loader;
  loader.LoadLib(mujoco_plugin_LIB);

  plugin::PluginPtr mujoco =
      loader.Instantiate("gz::physics::mujoco::Plugin");

  auto engine =
      physics::RequestEngine3d<ThreadPoolBenchFeatureList>::From(mujoco);
  EXPECT_NE(nullptr, engine);
  if (nullptr == engine)
    return nullptr;

  ::sdf::Root root;
  const auto errors = root.LoadSdfString(GridWorld());
  EXPECT_TRUE(errors.empty()) << errors;
  if (!errors.empty())
    return nullptr;

  EXPECT_NE(nullptr, root.WorldByIndex(0));
  return engine->ConstructWorld(*root.WorldByIndex(0));
}

/////////////////////////////////////////////////
/// \brief Step the world _steps times and return the elapsed wall time.
std::chrono::duration<double> StepFor(const WorldPtr &_world, int _steps)
{
  physics::ForwardStep::Input input;
  physics::ForwardStep::State state;
  physics::ForwardStep::Output output;

  const auto start = std::chrono::steady_clock::now();
  for (int i = 0; i < _steps; ++i)
    _world->Step(output, state, input);
  return std::chrono::steady_clock::now() - start;
}

/////////////////////////////////////////////////
/// \brief Collect the world position of every box link, in model order.
std::vector<Eigen::Vector3d> CollectPositions(const WorldPtr &_world)
{
  std::vector<Eigen::Vector3d> positions;
  const std::size_t modelCount = _world->GetModelCount();
  positions.reserve(modelCount);

  for (std::size_t i = 0; i < modelCount; ++i)
  {
    auto model = _world->GetModel(i);
    if (nullptr == model || 0u == model->GetLinkCount())
      continue;
    auto link = model->GetLink(0);
    if (nullptr == link)
      continue;
    positions.push_back(link->FrameDataRelativeToWorld().pose.translation());
  }

  return positions;
}

/////////////////////////////////////////////////
/// \brief Step the world's mjModel/mjData directly, bypassing the plugin.
///
/// Reaching the compiled model is a cast on the world's reference, the same
/// approach the mujoco unit tests use; no plugin symbol is needed. This
/// isolates MuJoCo's own cost from the plugin's per-step bookkeeping.
std::chrono::duration<double> RawStepFor(const WorldPtr &_world, int _steps)
{
  auto *worldInfo = static_cast<physics::mujoco::WorldInfo *>(
      _world->FullIdentity().ref.get());
  EXPECT_NE(nullptr, worldInfo);
  if (nullptr == worldInfo)
    return {};

  const auto start = std::chrono::steady_clock::now();
  for (int i = 0; i < _steps; ++i)
    mj_step(worldInfo->mjModelObj, worldInfo->mjDataObj);
  return std::chrono::steady_clock::now() - start;
}

/////////////////////////////////////////////////
/// \brief Run the scene for _steps with _nthread workers, reporting the time.
/// \return Final box positions.
std::vector<Eigen::Vector3d> RunCase(int _nthread, int _steps,
                                     std::chrono::duration<double> *_elapsed)
{
  ScopedThreadEnv guard{_nthread};

  auto world = LoadGridWorld();
  EXPECT_NE(nullptr, world);
  if (nullptr == world)
    return {};

  const auto elapsed = StepFor(world, _steps);
  if (nullptr != _elapsed)
    *_elapsed = elapsed;

  return CollectPositions(world);
}

/////////////////////////////////////////////////
/// \brief Time a raw mj_step loop from a freshly constructed world, so that
/// it starts from the same state as the World::Step measurement.
std::chrono::duration<double> RunRawCase(int _nthread, int _steps)
{
  ScopedThreadEnv guard{_nthread};

  auto world = LoadGridWorld();
  EXPECT_NE(nullptr, world);
  if (nullptr == world)
    return {};

  // The first World::Step is what compiles the spec and installs the pool;
  // without it the raw loop would run on an uncompiled model.
  StepFor(world, 1);

  return RawStepFor(world, _steps);
}
}  // namespace

/////////////////////////////////////////////////
// Threading must not change the simulated result.
//
// This is the real gate. It is checked over a short horizon because island
// solving can reorder floating point accumulation; see kEquivalenceSteps.
TEST(ThreadPoolBenchmark, ThreadedResultMatchesSerial)
{
  common::Console::SetVerbosity(0);

  if (std::thread::hardware_concurrency() < 2)
    GTEST_SKIP() << "needs at least 2 hardware threads";

  const auto serial = RunCase(0, kEquivalenceSteps, nullptr);
  ASSERT_FALSE(serial.empty());

  const auto threaded = RunCase(4, kEquivalenceSteps, nullptr);
  ASSERT_EQ(serial.size(), threaded.size());

  for (std::size_t i = 0; i < serial.size(); ++i)
  {
    EXPECT_NEAR(serial[i].x(), threaded[i].x(), kEquivalenceTol) << "body " << i;
    EXPECT_NEAR(serial[i].y(), threaded[i].y(), kEquivalenceTol) << "body " << i;
    EXPECT_NEAR(serial[i].z(), threaded[i].z(), kEquivalenceTol) << "body " << i;
  }
}

/////////////////////////////////////////////////
// Report stepping time across worker counts.
//
// Nothing here is asserted beyond the runs completing: wall-clock thresholds
// flake on loaded or low-core machines. The numbers are for humans deciding
// whether to set GZ_PHYSICS_MUJOCO_THREADS.
TEST(ThreadPoolBenchmark, ReportSpeedup)
{
  common::Console::SetVerbosity(0);

  const unsigned int cores = std::thread::hardware_concurrency();

  std::vector<int> threadCounts{0};
  for (int n : {2, 4, 8})
  {
    if (cores >= static_cast<unsigned int>(n))
      threadCounts.push_back(n);
  }

  std::chrono::duration<double> stepBaseline{0};
  std::chrono::duration<double> rawBaseline{0};

  const auto ratio = [](const std::chrono::duration<double> &_base,
                        const std::chrono::duration<double> &_value)
  {
    return _value.count() > 0.0 ? _base.count() / _value.count() : 0.0;
  };

  std::cout << "\n  MuJoCo thread pool, " << (kGridSide * kGridSide)
            << " free boxes, " << kBenchmarkSteps << " steps"
            << " (" << cores << " hardware threads)\n\n"
            << "           World::Step            raw mj_step\n"
            << "  threads   time (s)  speedup     time (s)  speedup\n";

  for (int nthread : threadCounts)
  {
    std::chrono::duration<double> stepElapsed{0};
    const auto positions = RunCase(nthread, kBenchmarkSteps, &stepElapsed);
    ASSERT_FALSE(positions.empty()) << "threads: " << nthread;

    const auto rawElapsed = RunRawCase(nthread, kBenchmarkSteps);

    if (0 == nthread)
    {
      stepBaseline = stepElapsed;
      rawBaseline = rawElapsed;
    }

    std::cout << "  " << std::setw(7) << (0 == nthread ? 1 : nthread)
              << std::setw(11) << std::fixed << std::setprecision(3)
              << stepElapsed.count()
              << std::setw(9) << std::setprecision(2)
              << ratio(stepBaseline, stepElapsed) << "x"
              << std::setw(13) << std::setprecision(3) << rawElapsed.count()
              << std::setw(9) << std::setprecision(2)
              << ratio(rawBaseline, rawElapsed) << "x\n";
  }

  std::cout << "\n  The gap between the two speedup columns is gz-physics"
               " per-step overhead:\n"
               "  mj_fwdPosition/mj_fwdVelocity plus a per-link pose sweep,"
               " all serial.\n"
            << std::endl;
}
