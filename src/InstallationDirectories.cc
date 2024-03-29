/*
 * Copyright (C) 2023 Open Source Robotics Foundation
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

#include <regex>

#include <gz/physics/config.hh>
#include <gz/physics/InstallationDirectories.hh>

#ifdef _WIN32
#include <shlwapi.h>
#endif

namespace gz
{
namespace physics
{
inline namespace GZ_PHYSICS_VERSION_NAMESPACE {

// We locally import the gz::common::joinPaths function
// See https://github.com/gazebosim/gz-physics/pull/507#discussion_r1186919267
// for more details

// Function imported from
// https://github.com/gazebosim/gz-common/blob/ignition-common4_4.6.2/src/FilesystemBoost.cc#L507
#ifndef WIN32
static const char preferred_separator = '/';
#else  // Windows
static const char preferred_separator = '\\';
#endif
const std::string separator(const std::string &_p)
{
  return _p + preferred_separator;
}

// Function imported from
// https://github.com/gazebosim/gz-common/blob/ignition-common4_4.6.2/src/Filesystem.cc#L227
std::string checkWindowsPath(const std::string _path)
{
  if (_path.empty())
    return _path;

  // Check if this is a http or https, if so change backslashes generated by
  // jointPaths to '/'
  if ((_path.size() > 7 && 0 == _path.compare(0, 7, "http://")) ||
      (_path.size() > 8 && 0 == _path.compare(0, 8, "https://")))
  {
    return std::regex_replace(_path, std::regex(R"(\\)"), "/");
  }

  // This is a Windows path, convert all '/' into backslashes
  std::string result = std::regex_replace(_path, std::regex(R"(/)"), "\\");
  std::string drive_letters;

  // only Windows contains absolute paths starting with drive letters
  if (result.length() > 3 && 0 == result.compare(1, 2, ":\\"))
  {
    drive_letters = result.substr(0, 3);
    result = result.substr(3);
  }
  result = drive_letters + std::regex_replace(
    result, std::regex("[<>:\"|?*]"), "");
  return result;
}

// Function imported from
// https://github.com/gazebosim/gz-common/blob/ignition-common4_4.6.2/src/Filesystem.cc#L256
std::string joinPaths(const std::string &_path1,
                      const std::string &_path2)
{

  /// This function is used to avoid duplicated path separators at the
  /// beginning/end of the string, and between the two paths being joined.
  /// \param[in] _path This is the string to sanitize.
  /// \param[in] _stripLeading True if the leading separator should be
  /// removed.
  auto sanitizeSlashes = [](const std::string &_path,
                            bool _stripLeading = false)
  {
    // Shortcut
    if (_path.empty())
      return _path;

    std::string result = _path;

    // Use the appropriate character for each platform.
#ifndef _WIN32
    char replacement = '/';
#else
    char replacement = '\\';
#endif

    // Sanitize the start of the path.
    size_t index = 0;
    size_t leadingIndex = _stripLeading ? 0 : 1;
    for (; index < result.length() && result[index] == replacement; ++index)
    {
    }
    if (index > leadingIndex)
      result.erase(leadingIndex, index-leadingIndex);

    // Sanitize the end of the path.
    index = result.length()-1;
    for (; index <  result.length() && result[index] == replacement; --index)
    {
    }
    index += 1;
    if (index < result.length()-1)
        result.erase(index+1);
    return result;
  };

  std::string path;
#ifndef _WIN32
  path = sanitizeSlashes(sanitizeSlashes(separator(_path1)) +
      sanitizeSlashes(_path2, true));
#else  // _WIN32
  std::string path1 = sanitizeSlashes(checkWindowsPath(_path1));
  std::string path2 = sanitizeSlashes(checkWindowsPath(_path2), true);
  std::vector<char> combined(path1.length() + path2.length() + 2);
  if (::PathCombineA(combined.data(), path1.c_str(), path2.c_str()) != NULL)
  {
    path = sanitizeSlashes(checkWindowsPath(std::string(combined.data())));
  }
  else
  {
    path = sanitizeSlashes(checkWindowsPath(separator(path1) + path2));
  }
#endif  // _WIN32
  return path;
}


std::string getEngineInstallDir()
{
  return gz::physics::joinPaths(
      getInstallPrefix(), GZ_PHYSICS_ENGINE_RELATIVE_INSTALL_DIR);
}

}
}
}
