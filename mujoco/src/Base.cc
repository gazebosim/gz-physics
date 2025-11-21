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

#include "Base.hh"

#include <gz/physics/Implements.hh>

namespace gz
{
namespace physics
{
namespace mujoco
{

Identity Base::InitiateEngine(std::size_t)
{
  return this->GenerateIdentity(0);
}

bool Base::RecompileSpec(const WorldInfo &_worldInfo) const
{
  if (!_worldInfo.specDirety)
    return true;

  int rc = mj_recompile(_worldInfo.mjSpecObj, nullptr, _worldInfo.mjModelObj,
                        _worldInfo.mjDataObj);
  if (rc != 0)
  {
    gzerr << "Error compiling:" << mjs_getError(_worldInfo.mjSpecObj) << "\n";
    return false;
  }
  mj_saveXML(_worldInfo.mjSpecObj, "/tmp/mujoco_model.xml", nullptr, 0);

  mj_forward(_worldInfo.mjModelObj, _worldInfo.mjDataObj);
  return true;
}

}  // namespace mujoco
}  // namespace physics
}  // namespace gz
