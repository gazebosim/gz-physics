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

#ifndef GZ_PHYSICS_TEST_MOCKFEATURES_HH_
#define GZ_PHYSICS_TEST_MOCKFEATURES_HH_

#include "MockGetByName.hh"
#include "MockSetName.hh"
#include "MockCenterOfMass.hh"

namespace mock
{
  using MockFeatureList = gz::physics::FeatureList<
      MockGetByName,
      MockSetName,
      MockCenterOfMass
  >;

#define GZ_PHYSICS_MOCK_MACRO_HELPER( Type, X ) \
  using Mock ## Type ## X = gz::physics:: Type ## X <MockFeatureList>; \
  using Mock ## Type ## X ## Ptr = \
      gz::physics::EntityPtr< Mock ## Type ## X >;

#define GZ_PHYSICS_MOCK_MACRO(Type) \
  GZ_PHYSICS_MOCK_MACRO_HELPER(Type, 3d) \
  GZ_PHYSICS_MOCK_MACRO_HELPER(Type, 2d) \
  GZ_PHYSICS_MOCK_MACRO_HELPER(Type, 3f) \
  GZ_PHYSICS_MOCK_MACRO_HELPER(Type, 2f)

  GZ_PHYSICS_MOCK_MACRO(Engine)
  GZ_PHYSICS_MOCK_MACRO(World)
  GZ_PHYSICS_MOCK_MACRO(Model)
  GZ_PHYSICS_MOCK_MACRO(Link)
  GZ_PHYSICS_MOCK_MACRO(Joint)
}

#endif
