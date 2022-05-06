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

#ifndef IGNITION_PHYSICS_REGISTERMORE_HH_
#define IGNITION_PHYSICS_REGISTERMORE_HH_

/// If your library already has a translation unit (.cpp file) containing
/// \code
///     #include <ignition/physics/Register.hh>
/// \endcode
///
/// then any other translation units that want to register plugins should use
/// \code
///     #include <ignition/physics/RegisterMore.hh>
/// \endcode
///
/// But at least one translation unit of your library must contain Register.hh.
#define IGN_PLUGIN_REGISTER_MORE_TRANS_UNITS
#include <ignition/physics/Register.hh>

#endif
