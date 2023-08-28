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

#include <gz/physics/Register.hh>

#include "Base.hh"
#include "AddedMassFeatures.hh"
#include "CustomFeatures.hh"
#include "JointFeatures.hh"
#include "KinematicsFeatures.hh"
#include "LinkFeatures.hh"
#include "SDFFeatures.hh"
#include "ShapeFeatures.hh"
#include "SimulationFeatures.hh"
#include "EntityManagementFeatures.hh"
#include "FreeGroupFeatures.hh"
#include "WorldFeatures.hh"

namespace gz {
namespace physics {
namespace dartsim {

struct DartsimFeatures : FeatureList<
  AddedMassFeatureList,
  CustomFeatureList,
  EntityManagementFeatureList,
  FreeGroupFeatureList,
  JointFeatureList,
  KinematicsFeatureList,
  LinkFeatureList,
  SDFFeatureList,
  ShapeFeatureList,
  SimulationFeatureList,
  WorldFeatureList
> { };

class Plugin :
    public virtual Base,
    public virtual AddedMassFeatures,
    public virtual CustomFeatures,
    public virtual EntityManagementFeatures,
    public virtual FreeGroupFeatures,
    public virtual JointFeatures,
    public virtual KinematicsFeatures,
    public virtual LinkFeatures,
    public virtual SDFFeatures,
    public virtual ShapeFeatures,
    public virtual SimulationFeatures,
    public virtual WorldFeatures { };

namespace {

// This is done as a partial fix for
// https://github.com/gazebosim/gz-physics/issues/442. The issue seems like the
// destructors for the concrete collision detectors get unloaded and deleted
// from memory before the destructors run. When it's time to actually call the
// destructors, a segfault is generated.
//
// It's not clear why the destructors are deleted prematurely. It might be a
// compiler optimization in new compiler versions.
//
// The solution here is to call the `unregisterAllCreators` function from the
// plugins translation unit in the hopes that it will force the compiler to keep
// the destructors.
struct UnregisterCollisionDetectors
{
  ~UnregisterCollisionDetectors()
  {
    dart::collision::CollisionDetector::getFactory()->unregisterAllCreators();
  }
};

UnregisterCollisionDetectors unregisterAtUnload;
}

GZ_PHYSICS_ADD_PLUGIN(Plugin, FeaturePolicy3d, DartsimFeatures)

}
}
}
