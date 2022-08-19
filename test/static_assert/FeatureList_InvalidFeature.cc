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

#include <ignition/physics/FeatureList.hh>

using namespace gz::physics;

class FeatureA : public virtual Feature { };
class FeatureB : public virtual Feature { };
class FeatureC : public virtual Feature { };

// This object is not a feature, but we will pass it into a FeatureList to see
// that a compilation error occurs.
class NotAFeature { };

using SomeList = FeatureList<
    FeatureA,
    FeatureB,
    NotAFeature,
    FeatureC>;

int main()
{
  SomeList();
}
