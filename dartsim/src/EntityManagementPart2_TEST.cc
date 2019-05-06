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

#include <condition_variable>
#include <mutex>
#include <thread>

#include <ignition/plugin/Loader.hh>

#include <ignition/physics/ConstructEmpty.hh>
#include <ignition/physics/RequestEngine.hh>

using TestFeatureList = ignition::physics::FeatureList<
  ignition::physics::ConstructEmptyWorldFeature
>;

/////////////////////////////////////////////////
///Test if race condition exists with one plugin instance but multiple worlds
TEST(EntityManagementPart2_TEST, MultipleWorlds)
{
  ignition::plugin::Loader loader;
  loader.LoadLib(dartsim_plugin_LIB);

  const std::string pluginName = "ignition::physics::dartsim::Plugin";
  auto dartsim = loader.Instantiate(pluginName);

  const std::size_t numThreads = 10;
  std::vector<ignition::physics::Engine3dPtr<TestFeatureList>> engines;
  for (std::size_t i = 0; i < numThreads; ++i)
  {
    engines.push_back(
        ignition::physics::RequestEngine3d<TestFeatureList>::From(dartsim));
    ASSERT_NE(nullptr, engines.back());
  }

  std::condition_variable cv;
  bool ready = false;
  std::mutex mutex;
  std::vector<std::thread> threads;
  for (const auto &engine : engines)
  {
    threads.emplace_back([&]()
      {
        std::unique_lock<std::mutex> lock(mutex);
        cv.wait(lock, [&]{return ready;});
        lock.unlock();
        engine->ConstructEmptyWorld("empty world");
      });
  }
  {
    std::lock_guard<std::mutex> lock(mutex);
    ready = true;
  }
  cv.notify_all();

  for (auto &th : threads)
  {
    th.join();
  }
}

/////////////////////////////////////////////////
///Test if race condition exists with multiple plugin instances each creating a
///single world
TEST(EntityManagementPart2_TEST, MultipleInstances)
{
  ignition::plugin::Loader loader;
  loader.LoadLib(dartsim_plugin_LIB);

  const std::string pluginName = "ignition::physics::dartsim::Plugin";
  std::vector<ignition::plugin::PluginPtr> dartsimInstances;

  const std::size_t numThreads = 10;
  for (std::size_t i = 0; i < numThreads; ++i)
  {
    dartsimInstances.push_back(loader.Instantiate(pluginName));
  }

  std::condition_variable cv;
  bool ready = false;
  std::mutex mutex;
  std::vector<std::thread> threads;
  for (const auto &dartsim : dartsimInstances)
  {
    threads.emplace_back([&]()
      {
        std::unique_lock<std::mutex> lock(mutex);
        cv.wait(lock, [&] { return ready; });
        lock.unlock();
        auto engine =
            ignition::physics::RequestEngine3d<TestFeatureList>::From(dartsim);
        engine->ConstructEmptyWorld("empty world");
      });
  }

  {
    std::lock_guard<std::mutex> lock(mutex);
    ready = true;
  }

  cv.notify_all();

  for (auto &th : threads)
  {
    th.join();
  }
}

int main(int argc, char *argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
