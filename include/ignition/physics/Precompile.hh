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

#ifndef IGNITION_PHYSICS_PRECOMPILE_HH
#define IGNITION_PHYSICS_PRECOMPILE_HH

#include <ignition/physics/FeatureList.hh>
#include <ignition/physics/FeaturePolicy.hh>

// ------------ Declare feature sets ------------

/// Tell a header that a certain feature set will be precompiled for a specific
/// object type.
/// Note that the library which provides the header must also have
/// \code
/// #define IGN_PHYSICS_DEFINE_OBJECT(CustomFeatures, Object, PolicySuffix)
/// \endcode
/// in a source file.
#define IGN_PHYSICS_DECLARE_OBJECT(CustomFeatures, Object, PolicySuffix) \
  extern template class ::ignition::physics::Object < \
      ::ignition::physics::FeaturePolicy ## PolicySuffix, CustomFeatures>; \
  extern template class ::ignition::physics::EntityPtr< \
      ::ignition::physics::Object< \
          ::ignition::physics::FeaturePolicy ## PolicySuffix, \
          CustomFeatures> >;

/// Tell a header that a certain feature set will be precompiled for the basic
/// objects.
/// Note that the library which provides the header must also have a
///
/// \code
/// #define IGN_PHYSICS_DEFINE_BASIC_OBJECTS(Prefix, CustomFeatures, PolicySuffix)
/// \endcode
#define IGN_PHYSICS_DECLARE_BASIC_OBJECTS(CustomFeatures, PolicySuffix) \
  extern template struct ::ignition::physics::Entity< \
      ::ignition::physics::FeaturePolicy ## PolicySuffix, \
      CustomFeatures>::Pimpl; \
  IGN_PHYSICS_DECLARE_OBJECT(CustomFeatures, Engine, PolicySuffix) \
  IGN_PHYSICS_DECLARE_OBJECT(CustomFeatures, World, PolicySuffix) \
  IGN_PHYSICS_DECLARE_OBJECT(CustomFeatures, Model, PolicySuffix) \
  IGN_PHYSICS_DECLARE_OBJECT(CustomFeatures, Link, PolicySuffix) \
  IGN_PHYSICS_DECLARE_OBJECT(CustomFeatures, Joint, PolicySuffix) \
  IGN_PHYSICS_DECLARE_OBJECT(CustomFeatures, Shape, PolicySuffix)


// ------------ Define feature sets ------------

/// Put this in the source file where an object type should be compiled for the
/// feature list.
#define IGN_PHYSICS_DEFINE_OBJECT(CustomFeatures, Object, PolicySuffix) \
  template class ::ignition::physics::Object < \
      ::ignition::physics::FeaturePolicy ## PolicySuffix, \
      CustomFeatures>; \
  template class ::ignition::physics::EntityPtr< \
      ::ignition::physics::Object< \
          ::ignition::physics::FeaturePolicy ## PolicySuffix, \
          CustomFeatures> >;

// Helper macro
#define IGN_PHYSICS_DEFINE_BASIC_OBJECTS(CustomFeatures, PolicySuffix) \
  template struct ::ignition::physics::Entity< \
      ::ignition::physics::FeaturePolicy ## PolicySuffix, \
      CustomFeatures>::Pimpl; \
  IGN_PHYSICS_DEFINE_OBJECT(CustomFeatures, Engine, PolicySuffix) \
  IGN_PHYSICS_DEFINE_OBJECT(CustomFeatures, World, PolicySuffix) \
  IGN_PHYSICS_DEFINE_OBJECT(CustomFeatures, Model, PolicySuffix) \
  IGN_PHYSICS_DEFINE_OBJECT(CustomFeatures, Link, PolicySuffix) \
  IGN_PHYSICS_DEFINE_OBJECT(CustomFeatures, Joint, PolicySuffix) \
  IGN_PHYSICS_DEFINE_OBJECT(CustomFeatures, Shape, PolicySuffix)


#define IGN_PHYSICS_NAME_OBJECT( \
    Visibility, Name, Object, CustomFeatures, PolicySuffix) \
  struct Visibility Name : \
    ::ignition::physics::Object< \
      ::ignition::physics::FeaturePolicy ## PolicySuffix, CustomFeatures> \
  { \
    using Base = ::ignition::physics::Object< \
        ::ignition::physics::FeaturePolicy ## PolicySuffix, CustomFeatures>; \
    using Base::Base; \
  }; \
  struct Visibility Name ## Ptr : \
    ::ignition::physics::EntityPtr<Name> \
  { \
    using Base = ::ignition::physics::EntityPtr<Name>; \
    using Base::Base; \
  };

#define IGN_PHYSICS_NAME_BASIC_OBJECTS( \
  Visibility, Prefix, CustomFeatures, PolicySuffix) \
  IGN_PHYSICS_NAME_OBJECT(Visibility, Prefix ## Engine, Engine, CustomFeatures, PolicySuffix) \
  IGN_PHYSICS_NAME_OBJECT(Visibility, Prefix ## World, World, CustomFeatures, PolicySuffix) \
  IGN_PHYSICS_NAME_OBJECT(Visibility, Prefix ## Model, Model, CustomFeatures, PolicySuffix) \
  IGN_PHYSICS_NAME_OBJECT(Visibility, Prefix ## Link, Link, CustomFeatures, PolicySuffix) \
  IGN_PHYSICS_NAME_OBJECT(Visibility, Prefix ## Joint, Joint, CustomFeatures, PolicySuffix) \
  IGN_PHYSICS_NAME_OBJECT(Visibility, Prefix ## Shape, Shape, CustomFeatures, PolicySuffix)



#endif
