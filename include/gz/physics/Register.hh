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

#ifndef GZ_PHYSICS_REGISTER_HH_
#define GZ_PHYSICS_REGISTER_HH_

#include <gz/physics/detail/Register.hh>
#include <gz/physics/Implements.hh>

// -------------------- Add a physics engine plugin ------------------------

/// \brief Add a plugin that can be used as physics engine.
///
/// 1. The first argument is the name of the class that wraps the physics engine
///    into a plugin.
/// 2. The second argument is the Feature Policy for this plugin,
///    e.g. gz::physics::FeaturePolicy3d
/// 3. The third argument is the Feature List, specifying all the features that
///    this plugin provides,
///    e.g. gz::physics::StandardFeatures
///
/// Note that the Feature Policy and Feature List should match the types that
/// you pass to the gz::physics::Implements<P, L> that your plugin class
/// inherits.
///
/// If your library has multiple translation units (.cpp files) and you want to
/// register plugins in multiple translation units, use this
/// gz/physics/Register.hh header in ONE of the translation units, and
/// then the gz/physics/RegisterMore.hh header in all of the rest of the
/// translation units.
#define GZ_PHYSICS_ADD_PLUGIN(PluginType, FeaturePolicyT, FeatureListT) \
  DETAIL_GZ_PHYSICS_ADD_PLUGIN(PluginType, FeaturePolicyT, FeatureListT)

#endif
