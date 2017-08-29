/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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

#ifndef IGNITION_PHYSICS_COMPOSITEDATAMACROS_HH_
#define IGNITION_PHYSICS_COMPOSITEDATAMACROS_HH_

#include <string>

/// \brief Call this macro inside the class definition of any class or struct
/// that you want to put inside of a CompositeData object. The argument you
/// pass must avoid name collisions with all other data types, so you are
/// advised to use the fully-qualified name of the class, e.g.
///
///     namespace libname {
///
///       class ClassName
///       {
///         public:
///           IGN_PHYSICS_DATA_LABEL(libname::ClassName)
///         // ... rest of class definition...
///       };
///
///     } // namespace libname
///
#define IGN_PHYSICS_DATA_LABEL(label) \
  public: inline static constexpr const char* IgnPhysicsTypeLabel() { \
      return #label; }

#endif
