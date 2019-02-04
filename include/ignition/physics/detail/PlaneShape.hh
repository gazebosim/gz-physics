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

#ifndef IGNITION_PHYSICS_DETAIL_PLANESHAPE_HH_
#define IGNITION_PHYSICS_DETAIL_PLANESHAPE_HH_

#include <string>

#include <ignition/physics/PlaneShape.hh>

namespace ignition
{
namespace physics
{
  /////////////////////////////////////////////////
  template <typename P, typename F>
  auto GetPlaneShapeProperties::PlaneShape<P, F>::GetNormal() const -> Normal
  {
    return this->template Interface<GetPlaneShapeProperties>()
        ->GetPlaneShapeNormal(this->identity);
  }

  /////////////////////////////////////////////////
  template <typename P, typename F>
  auto GetPlaneShapeProperties::PlaneShape<P, F>::GetPoint() const -> Point
  {
    return this->template Interface<GetPlaneShapeProperties>()
        ->GetPlaneShapePoint(this->identity);
  }

  /////////////////////////////////////////////////
  template <typename P, typename F>
  void SetPlaneShapeProperties::PlaneShape<P, F>::SetNormal(
      const Normal &_normal)
  {
    return this->template Interface<SetPlaneShapeProperties>()
        ->SetPlaneShapeNormal(this->identity, _normal);
  }

  /////////////////////////////////////////////////
  template <typename P, typename F>
  void SetPlaneShapeProperties::PlaneShape<P, F>::SetPoint(
      const Point &_point)
  {
    return this->template Interface<SetPlaneShapeProperties>()
        ->SetPlaneShapePoint(this->identity, _point);
  }

  /////////////////////////////////////////////////
  template <typename P, typename F>
  PlaneShapePtr<P, F> AttachPlaneShapeFeature::Link<P, F>::AttachPlaneShape(
      const std::string &_name,
      const Normal &_normal,
      const Point &_offset)
  {
    return this->template Interface<AttachPlaneShapeFeature>()
        ->AttachPlaneShape(this->identity, _name, _normal, _offset);
  }
}
}

#endif // IGNITION_PHYSICS_DETAIL_PLANESHAPE_HH_
