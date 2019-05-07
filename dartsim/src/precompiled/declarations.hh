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

#ifndef IGNITION_PHYSICS_DARTSIM_SRC_PRECOMPILED_DECLARATIONS_HH
#define IGNITION_PHYSICS_DARTSIM_SRC_PRECOMPILED_DECLARATIONS_HH

#ifdef IGNITION_PHYSICS_DARTSIM_PRECOMPILE_STATIC_DEFINE
#  define IGNITION_PHYSICS_DARTSIM_PRECOMPILE_VISIBLE
#  define IGNITION_PHYSICS_DARTSIM_PRECOMPILE_HIDDEN
#else
#  ifndef IGNITION_PHYSICS_DARTSIM_PRECOMPILE_VISIBLE
#    ifdef ignition_physics2_dartsim_precompile_EXPORTS
        /* We are building this library */
#      define IGNITION_PHYSICS_DARTSIM_PRECOMPILE_VISIBLE __attribute__((visibility("default")))
#    else
        /* We are using this library */
#      define IGNITION_PHYSICS_DARTSIM_PRECOMPILE_VISIBLE __attribute__((visibility("hidden")))
#    endif
#  endif

#  ifndef IGNITION_PHYSICS_DARTSIM_PRECOMPILE_HIDDEN
#    define IGNITION_PHYSICS_DARTSIM_PRECOMPILE_HIDDEN __attribute__((visibility("hidden")))
#  endif
#endif

#include "../DartsimFeatures.hh"
#include <ignition/physics/mesh/MeshShape.hh>

namespace ignition {
namespace physics {
namespace dartsim {

//struct ComboFeatures : FeatureList<
//  CustomFeatureList,
//  EntityManagementFeatureList,
//  FreeGroupFeatureList,
//  JointFeatureList,
//  KinematicsFeatureList,
//  LinkFeatureList,
//  SDFFeatureList,
//  ShapeFeatureList,
//  SimulationFeatureList
//> { };



IGN_PHYSICS_NAME_BASIC_OBJECTS(
    IGNITION_PHYSICS_DARTSIM_PRECOMPILE_VISIBLE,
    Dartsim, DartsimFeatures, 3d)

IGN_PHYSICS_NAME_OBJECT(
    IGNITION_PHYSICS_DARTSIM_PRECOMPILE_VISIBLE,
    DartsimRevoluteJoint, RevoluteJoint, DartsimFeatures, 3d)

IGN_PHYSICS_NAME_OBJECT(
    IGNITION_PHYSICS_DARTSIM_PRECOMPILE_VISIBLE,
    DartsimPrismaticJoint, PrismaticJoint, DartsimFeatures, 3d)

IGN_PHYSICS_NAME_OBJECT(
    IGNITION_PHYSICS_DARTSIM_PRECOMPILE_VISIBLE,
    DartsimSphereShape, SphereShape, DartsimFeatures, 3d)

IGN_PHYSICS_NAME_OBJECT(
    IGNITION_PHYSICS_DARTSIM_PRECOMPILE_VISIBLE,
    DartsimMeshShape, mesh::MeshShape, DartsimFeatures, 3d)

IGN_PHYSICS_NAME_OBJECT(
    IGNITION_PHYSICS_DARTSIM_PRECOMPILE_VISIBLE,
    DartsimBoxShape, BoxShape, DartsimFeatures, 3d)
}
}
}

//IGN_PHYSICS_DECLARE_BASIC_OBJECTS(
//    ignition::physics::dartsim::CustomFeatureList, 3d)

//IGN_PHYSICS_DECLARE_BASIC_OBJECTS(
//    ignition::physics::dartsim::EntityManagementFeatureList, 3d)

//IGN_PHYSICS_DECLARE_BASIC_OBJECTS(
//    ignition::physics::dartsim::FreeGroupFeatureList, 3d)

//IGN_PHYSICS_DECLARE_BASIC_OBJECTS(
//    ignition::physics::dartsim::JointFeatureList, 3d)

//IGN_PHYSICS_DECLARE_BASIC_OBJECTS(
//    ignition::physics::dartsim::KinematicsFeatureList, 3d)

//IGN_PHYSICS_DECLARE_BASIC_OBJECTS(
//    ignition::physics::dartsim::LinkFeatureList, 3d)

//IGN_PHYSICS_DECLARE_BASIC_OBJECTS(
//    ignition::physics::dartsim::SDFFeatureList, 3d)

//IGN_PHYSICS_DECLARE_BASIC_OBJECTS(
//    ignition::physics::dartsim::ShapeFeatureList, 3d)

//IGN_PHYSICS_DECLARE_BASIC_OBJECTS(
//    ignition::physics::dartsim::SimulationFeatureList, 3d)

//IGN_PHYSICS_DECLARE_BASIC_OBJECTS(
//    ignition::physics::dartsim::ComboFeatures, 3d)

//IGN_PHYSICS_DECLARE_BASIC_OBJECTS(
//    ignition::physics::dartsim::DartsimFeatures, 3d)


#endif
