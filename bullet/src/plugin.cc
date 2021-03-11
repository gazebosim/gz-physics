/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#include <ignition/physics/FeatureList.hh>
#include <ignition/physics/FeaturePolicy.hh>
#include <ignition/physics/GetEntities.hh>
#include <ignition/physics/Register.hh>

#include "Base.hh"
#include "EntityManagementFeatures.hh"
#include "SimulationFeatures.hh"
#include "SDFFeatures.hh"
#include "KinematicsFeatures.hh"
#include "FreeGroupFeatures.hh"
#include "ShapeFeatures.hh"
#include "JointFeatures.hh"

namespace ignition {
namespace physics {
namespace bullet {

struct BulletFeatures : FeatureList <
  EntityManagementFeatureList,
  SimulationFeatureList,
  FreeGroupFeatureList,
  KinematicsFeatureList,
  SDFFeatureList,
  ShapeFeatureList,
  JointFeatureList
> { };

class Plugin :
    public virtual Implements3d<BulletFeatures>,
    public virtual Base,
    public virtual EntityManagementFeatures,
    public virtual SimulationFeatures,
    public virtual FreeGroupFeatures,
    public virtual KinematicsFeatures,
    public virtual SDFFeatures,
    public virtual ShapeFeatures,
    public virtual JointFeatures
{};

IGN_PHYSICS_ADD_PLUGIN(Plugin, FeaturePolicy3d, BulletFeatures)

}  // namespace bullet
}  // namespace physics
}  // namespace ignition
