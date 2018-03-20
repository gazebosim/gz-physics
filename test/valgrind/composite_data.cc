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
#include <ignition/physics/CompositeData.hh>

int main()
{
  ignition::physics::CompositeData data;

  std::string &s = data.Get<std::string>();

  s = "modified";

  data.Create<std::string>("new string");
  data.Remove<std::string>();

  ignition::physics::CompositeData emptyData;
  data = emptyData;

  ignition::physics::CompositeData requiredData;
  requiredData.MakeRequired<std::string>("I am required");
  return 0;
}
