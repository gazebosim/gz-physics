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

#include "JointTypes.hh"

/////////////////////////////////////////////////
TEST(JointTypes_TEST, RevoluteJoint2f)
{
  TestRevoluteJoint<FeaturePolicy2f>(1e-16, "2f");
}

/////////////////////////////////////////////////
TEST(JointTypes_TEST, TypeCasts2f)
{
  TestJointTypeCasts<FeaturePolicy2f>("2f");
}
