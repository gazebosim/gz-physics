/*
 * Copyright (C) 2025 Open Source Robotics Foundation
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

#include <gz/physics/InstallationDirectories.hh>

namespace gz
{
namespace physics
{
inline namespace GZ_PHYSICS_VERSION_NAMESPACE {

// Generate an install prefix specifically for bazel build.
std::string getInstallPrefix()
{
  return ".";
}
}  // namespace GZ_PHYSICS_VERSION_NAMESPACE
}  // namespace physics
}  // namespace gz

