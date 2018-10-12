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

#ifndef IGNITION_PHYSICS_TEST_MOCKFEATURES_HH_
#define IGNITION_PHYSICS_TEST_MOCKFEATURES_HH_

#include "MockGetByName.hh"
#include "MockSetName.hh"
#include "MockCenterOfMass.hh"

namespace mock
{
  using MockFeatureList = ignition::physics::FeatureList<
      MockGetByName,
      MockSetName,
      MockCenterOfMass
  >;

#define IGN_PHYSICS_MOCK_MACRO_HELPER( Type, X ) \
  using Mock ## Type ## X = ignition::physics:: Type ## X <MockFeatureList>; \
  using Mock ## Type ## X ## Ptr = \
      ignition::physics::EntityPtr< Mock ## Type ## X >;

#define IGN_PHYSICS_MOCK_MACRO(Type) \
  IGN_PHYSICS_MOCK_MACRO_HELPER(Type, 3d) \
  IGN_PHYSICS_MOCK_MACRO_HELPER(Type, 2d) \
  IGN_PHYSICS_MOCK_MACRO_HELPER(Type, 3f) \
  IGN_PHYSICS_MOCK_MACRO_HELPER(Type, 2f)

  IGN_PHYSICS_MOCK_MACRO(Engine)
  IGN_PHYSICS_MOCK_MACRO(World)
  IGN_PHYSICS_MOCK_MACRO(Model)
  IGN_PHYSICS_MOCK_MACRO(Link)
  IGN_PHYSICS_MOCK_MACRO(Joint)
}

#endif
