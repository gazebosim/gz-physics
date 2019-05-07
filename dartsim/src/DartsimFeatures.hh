/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#ifndef IGNITION_PHYSICS_DARTSIM_SRC_DARTSIMFEATURES_HH
#define IGNITION_PHYSICS_DARTSIM_SRC_DARTSIMFEATURES_HH

#include <ignition/physics/Precompile.hh>

#include "Base.hh"
#include "CustomFeatures.hh"
#include "JointFeatures.hh"
#include "KinematicsFeatures.hh"
#include "LinkFeatures.hh"
#include "SDFFeatures.hh"
#include "ShapeFeatures.hh"
#include "SimulationFeatures.hh"
#include "EntityManagementFeatures.hh"
#include "FreeGroupFeatures.hh"

namespace ignition {
namespace physics {
namespace dartsim {

using DartsimFeatures = FeatureList<
  CustomFeatureList,
  EntityManagementFeatureList,
  FreeGroupFeatureList,
  JointFeatureList,
  KinematicsFeatureList,
  LinkFeatureList,
  SDFFeatureList,
  ShapeFeatureList,
  SimulationFeatureList
  // TODO(MXG): Implement more features
>;

}
}
}

#endif
