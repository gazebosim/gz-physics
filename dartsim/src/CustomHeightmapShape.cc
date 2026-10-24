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

#include <memory>

#include <gz/common/Profiler.hh>

namespace gz {
namespace physics {
namespace dartsim {

/////////////////////////////////////////////////
CustomHeightmapShape::CustomHeightmapShape(
    const heightmap::detail::SampledHeightmap &_sampled,
    const Eigen::Vector3d &_size)
  : dart::dynamics::HeightmapShape<float>()
{
  GZ_PROFILE("CustomHeightmapShape::CustomHeightmapShape");
  this->setHeightField(_sampled.vertSize, _sampled.vertSize, _sampled.heights);
  // The n vertices are n - 1 cells apart, so that the grid spans the full size.
  this->setScale(Vector3(_size(0) / (_sampled.vertSize - 1),
                         _size(1) / (_sampled.vertSize - 1), 1));
}

/////////////////////////////////////////////////
dart::dynamics::ShapePtr CustomHeightmapShape::clone() const
{
  std::shared_ptr<CustomHeightmapShape> clone(new CustomHeightmapShape());
  clone->setScale(this->getScale());
  clone->setHeightField(this->getHeightField());
  return clone;
}

/////////////////////////////////////////////////
void CustomHeightmapShape::updateBoundingBox() const
{
  // DART's computeBoundingBox uses width * scale, but the grid only spans
  // (width - 1) * scale: trim half a cell off each side in x and y.
  Eigen::Vector3d min, max;
  this->computeBoundingBox(min, max);
  const Eigen::Vector3d halfCell(
      0.5 * this->getScale().x(), 0.5 * this->getScale().y(), 0);
  this->mBoundingBox.setMin(min + halfCell);
  this->mBoundingBox.setMax(max - halfCell);
  this->mIsBoundingBoxDirty = false;
}
}
}
}
