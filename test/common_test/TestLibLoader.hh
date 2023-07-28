/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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
#ifndef GZ_PHYSICS_TESTLIBLOADER_HH_
#define GZ_PHYSICS_TESTLIBLOADER_HH_

#include <iostream>
#include <string>
#include <unordered_set>
#include <vector>

#include <gz/common/Filesystem.hh>
#include <gz/common/Util.hh>

namespace gz
{
namespace physics
{
class TestLibLoader
{
  /// brief Initialize command line arguments
  /// \param[in] argc Number of arguments
  /// \param[in] argv Vector with the arguments
  public: static bool init(int argc, char *argv[])
  {
    if (argc != 2)
    {
      std::cerr << "Please provide the path to an engine plugin.\n"
                << "Usage " <<  argv[0] << " <physics engine path>\n";
      return false;
    }
    std::string &libToTest = LibToTest();
    libToTest = argv[1];
    return true;
  }

  /// \brief Get the name of the library to test
  /// \return Name of the library to test
  public: static std::string GetLibToTest()
  {
    return LibToTest();
  }

  /// \brief Get Physics Engine name based on the plugin name
  /// \param[in] _name Plugin name
  /// \return Name of the Physics Engine
  static std::string PhysicsEngineName(std::string _name)
  {
    std::vector<std::string> tokens = gz::common::split(_name, "::");
    if (tokens.size() == 4)
    {
      std::string physicsEngineName = tokens[2];
      std::string physicsEnginePluginName = physicsEngineName;
      if (physicsEngineName == "bullet_featherstone")
      {
        physicsEnginePluginName = "bullet-featherstone";
      }
      if (physicsEngineName == "tpeplugin")
      {
        physicsEnginePluginName = "tpe";
      }
      return physicsEnginePluginName;
    }
    return "";
  }

  /// \brief Check if the physics engine provided by the plugin is in the
  /// provided list.
  /// \param[in] _pluginName Plugin name. PhysicsEngineName will first be called
  /// to determine the engine name.
  /// \param[in] _engineList List of engine names.
  /// \return True if the engine is in the list.
  public:
  static bool EngineInList(const std::string &_pluginName,
                           const std::unordered_set<std::string> &_engineList)
  {
    return _engineList.count(PhysicsEngineName(_pluginName)) != 0;
  }

  private: static std::string& LibToTest()
  {
    static std::string libToTest = "";
    return libToTest;
  }
};
}
}

/// \brief Check that the current engine being tested is supported.
/// If the engine is not in the set of passed arguments, the test is skipped
/// Adapted from
/// https://github.com/gazebosim/gz-rendering/blob/c2e72ee51a7e4dba5156faa96c972c63ca5ab437/test/common_test/CommonRenderingTest.hh#L127-L138
/// Example:
/// Skip test if engine is not dart or bullet
/// CHECK_SUPPORTED_ENGINE(name, "dart", "bullet");
#define CHECK_SUPPORTED_ENGINE(engineToTest, ...)                             \
  if (!gz::physics::TestLibLoader::EngineInList(engineToTest, {__VA_ARGS__})) \
    GTEST_SKIP() << "Engine '" << engineToTest << "' unsupported";

/// \brief Check that the current engine being tested is unsupported
/// If the engine is in the set of passed arguments, the test is skipped
/// Adapted from
/// https://github.com/gazebosim/gz-rendering/blob/c2e72ee51a7e4dba5156faa96c972c63ca5ab437/test/common_test/CommonRenderingTest.hh#L127-L138
/// Example:
/// Skip test if engine is bullet-featherstone
/// CHECK_UNSUPPORTED_ENGINE(name, "bullet-featherstone");
#define CHECK_UNSUPPORTED_ENGINE(engineToTest, ...)                          \
  if (gz::physics::TestLibLoader::EngineInList(engineToTest, {__VA_ARGS__})) \
    GTEST_SKIP() << "Engine '" << (engineToTest) << "' unsupported";
#endif
