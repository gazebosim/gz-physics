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

#include <string>

#ifndef _WIN32
#  define TestLibLoader_EXPORTS_API
#else
#  if (defined(TestLibLoader_EXPORTS))
#    define TestLibLoader_EXPORTS_API __declspec(dllexport)
#  else
#    define TestLibLoader_EXPORTS_API __declspec(dllimport)
#  endif
#endif

// This is necessary because of using stl types here. It is completely safe, because
// a) the member is not accessible from the outside
// b) there are no inline functions.
#ifdef _WIN32
# pragma warning(push)
# pragma warning(disable:4251)
#endif

namespace gz
{
namespace physics
{
class TestLibLoader_EXPORTS_API TestLibLoader
{
  /// brief Initialize command line arguments
  /// \param[in] argc Number of arguments
  /// \param[in] argv Vector with the arguments
  public: static bool init(int argc, char *argv[]);

  /// \brief Get the name of the library to test
  /// \return Name of the library to test
  public: static std::string GetLibToTest();

  /// \brief Get Physics Engine name based on the plugin name
  /// \param[in] _name Plugin name
  /// \return Name of the Physics Engine
  std::string PhysicsEngineName(std::string _name);

  private: static std::string libToTest;
};
}
}

#ifdef _WIN32
# pragma warning(pop)
#endif

#endif
