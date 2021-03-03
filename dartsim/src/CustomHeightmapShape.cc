/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include "CustomHeightmapShape.hh"

#include <vector>
#include <ignition/math/eigen3/Conversions.hh>

namespace ignition {
namespace physics {
namespace dartsim {

/////////////////////////////////////////////////
CustomHeightmapShape::CustomHeightmapShape(
    common::HeightmapData &_input,
    const Eigen::Vector3d &_size,
    int _subSampling)
  : dart::dynamics::HeightmapShape<double>()
{
  double heightmapSizeZ = _input.MaxElevation();
  const bool flipY = false;
  const int vertSize = (_input.Width() * _subSampling) - _subSampling + 1;
  math::Vector3d scale;
  scale.X(_size(0) / vertSize);
  scale.Y(_size(1) / vertSize);

  if (math::equal(heightmapSizeZ, 0.0))
    scale.Z(1.0);
  else
    scale.Z(fabs(_size(2)) / heightmapSizeZ);

  auto sizeIgn = ignition::math::eigen3::convert(_size);

  std::vector<float> heightsFloat;
  _input.FillHeightMap(_subSampling, vertSize, sizeIgn, scale, flipY,
      heightsFloat);

  std::vector<double> heightsDouble(heightsFloat.begin(), heightsFloat.end());

  this->setHeightField(vertSize, vertSize, heightsDouble);
  this->setScale(Vector3(scale.X(), scale.Y(), 1));
}

}
}
}
