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

#include "FrameSemantics.hh"

/////////////////////////////////////////////////
TEST(FrameSemantics_TEST, RelativeFrames3f)
{
  TestRelativeFrames<gz::physics::FeaturePolicy3f>(1e-2, "3f");
}

/////////////////////////////////////////////////
TEST(FrameSemantics_TEST, RelativeAlignedBox3f)
{
  TestRelativeAlignedBox<gz::physics::FeaturePolicy3f>(1e-3, "3f");
}

/////////////////////////////////////////////////
TEST(FrameSemantics_TEST, FrameID3f)
{
  TestFrameID<gz::physics::FeaturePolicy3f>(1e-2, "3f");
}

/////////////////////////////////////////////////
TEST(FrameSemantics_TEST, RelativeQuantities3f)
{
  TestRelativeQuantities<gz::physics::FeaturePolicy3f>(1e-2, "3f");
}

/////////////////////////////////////////////////
TEST(FrameSemantics_TEST, RelativeFrameData3f)
{
  TestRelativeFrameData<gz::physics::FeaturePolicy3f>(1e-2, "3f");
}

int main(int argc, char **argv)
{
  // This seed is arbitrary, but we always use the same seed value to ensure
  // that results are reproduceable between runs. You may change this number,
  // but understand that the values generated in these tests will be different
  // each time that you change it. The expected tolerances might need to be
  // adjusted if the seed number is changed.
  gz::math::Rand::Seed(416);

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
