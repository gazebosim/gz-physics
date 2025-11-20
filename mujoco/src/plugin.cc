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
#include <gz/physics/Register.hh>


#include "Base.hh"
#include "EntityManagementFeatures.hh"
#include "KinematicsFeatures.hh"
#include "SDFFeatures.hh"

namespace gz
{
namespace physics
{
namespace mujoco
{

struct MujocoFeatures : FeatureList<
  EntityManagementFeatureList,
  // FreeGroupFeatureList,
  // JointFeatureList,
  KinematicsFeatureList,
  // LinkFeatureList,
  SDFFeatureList
  // ShapeFeatureList,
  // SimulationFeatureList,
  // WorldFeatureList
> { };

class Plugin :
    public virtual Base,
    public virtual EntityManagementFeatures,
    // public virtual FreeGroupFeatures,
    // public virtual JointFeatures,
    public virtual KinematicsFeatures,
    // public virtual LinkFeatures,
    public virtual SDFFeatures
    // public virtual ShapeFeatures,
    // public virtual SimulationFeatures,
    // public virtual WorldFeatures
{ };

GZ_PHYSICS_ADD_PLUGIN(Plugin, FeaturePolicy3d, MujocoFeatures)
}  // namespace mujoco
}  // namespace physics
}  // namespace gz
