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
#include <ignition/common/Console.hh>
#include <ignition/common/ImageHeightmap.hh>
#include <ignition/math/eigen3/Conversions.hh>

namespace ignition {
namespace physics {
namespace dartsim {

/////////////////////////////////////////////////
CustomHeightmapShape::CustomHeightmapShape(
    const common::HeightmapData &_input,
    const Eigen::Vector3d &_size,
    int _subSampling)
  : dart::dynamics::HeightmapShape<float>()
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

  // We need to make a copy here in order to use the non-const FillHeightMap
  // function
  common::ImageHeightmap copyData;
  try
  {
    const auto &image = dynamic_cast<const common::ImageHeightmap &>(_input);
    copyData.Load(image.Filename());
  }
  catch(const std::bad_cast &)
  {
    ignerr << "Only image heightmaps are supported at the moment." << std::endl;
    return;
  }

  std::vector<float> heightsFloat;
  copyData.FillHeightMap(_subSampling, vertSize, sizeIgn, scale, flipY,
      heightsFloat);

  this->setHeightField(vertSize, vertSize, heightsFloat);
  this->setScale(Vector3(scale.X(), scale.Y(), 1));
}
}
}
}
