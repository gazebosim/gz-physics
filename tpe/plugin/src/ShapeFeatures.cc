/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#include <ignition/math/eigen3/Conversions.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/common/Console.hh>

#include "ShapeFeatures.hh"

using namespace ignition;
using namespace physics;
using namespace tpe;
using namespace plugin;

/////////////////////////////////////////////////
Identity ShapeFeatures::CastToBoxShape(const Identity &_shapeID) const
{
  // dart::_shapeID = tpe::_collisionID
  auto it = this->collisions.find(_shapeID);
  if (it != this->collisions.end())
  {
    auto shape = it->second->collision->GetShape();
    if (dynamic_cast<tpe::BoxShape*>(shape))
      return this->GenerateIdentity(_shapeID, it->second);
  }
  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
LinearVector3d ShapeFeatures::GetBoxShapeSize(
  const Identity &_boxID) const
{
  // _boxID ~= _collisionID
  auto it = this->collisions.find(_boxID);
  if (it != this->collisions.end())
  {
    tpe::BoxShape *box = static_cast<tpe::BoxShape*>(
      it->second->collision->GetShape());
    return math::eigen3::convert(box->GetSize());
  }
  return math::eigen3::convert(math::Vector3d(-1.0, -1.0, -1.0));
}

/////////////////////////////////////////////////
Identity ShapeFeatures::AttachBoxShape(
  const Identity &_linkID,
  const std::string &_name,
  const LinearVector3d &_size,
  const Pose3d &_pose)
{
  auto it = this->links.find(_linkID);
  if (it == this->links.end())
  {
    return this->GenerateInvalidId();
  }
  auto &collision = static_cast<tpe::lib::Collision&>(
    it->second->link->AddCollision());
  collision.SetName(_name);
  collision.SetPose(math::eigen3::convert(_pose));

  tpe::BoxShape boxshape;
  boxshape.SetSize(math::eigen3::convert(_size));
  collision.SetShape(boxshape);

  return this->AddCollision(_linkID, collision);
}

/////////////////////////////////////////////////
Identity ShapeFeatures::CastToCylinderShape(const Identity &_shapeID) const
{
  auto it = this->collisions.find(_shapeID);
  if (it != this->collisions.end())
  {
    auto shape = it->second->collision->GetShape();
    if (dynamic_cast<tpe::CylinderShape*>(shape))
      return this->GenerateIdentity(_shapeID, it->second);
  }
  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
double ShapeFeatures::GetCylinderShapeRadius(
  const Identity &_cylinderID) const
{
  // assume _cylinderID ~= _collisionID
  auto it = this->collisions.find(_cylinderID);
  if (it != this->collisions.end())
  {
    auto *shape = static_cast<tpe::lib::CylinderShape*>(
      it->second->collision->GetShape());
    return shape->GetRadius();
  }
  return -1.0;
}

/////////////////////////////////////////////////
double ShapeFeatures::GetCylinderShapeHeight(
  const Identity &_cylinderID) const
{
  // assume _cylinderID ~= _collisionID
  auto it = this->collisions.find(_cylinderID);
  if (it != this->collisions.end())
  {
    auto *shape = static_cast<tpe::lib::CylinderShape*>(
      it->second->collision->GetShape());
    return shape->GetLength();
  }
  return -1.0;
}

/////////////////////////////////////////////////
Identity ShapeFeatures::AttachCylinderShape(
  const Identity &_linkID,
  const std::string &_name,
  const double _radius,
  const double _height,
  const Pose3d &_pose)
{
  auto it = this->links.find(_linkID);
  if (it == this->links.end())
  {
    return this->GenerateInvalidId();
  }
  auto &collision = static_cast<tpe::lib::Collision&>(
    it->second->link->AddCollision());
  collision.SetName(_name);
  collision.SetPose(math::eigen3::convert(_pose));

  tpe::CylinderShape cylindershape;
  cylindershape.SetRadius(_radius);
  cylindershape.SetLength(_height);
  collision.SetShape(cylindershape);

  return this->AddCollision(_linkID, collision);
}

/////////////////////////////////////////////////
Identity ShapeFeatures::CastToSphereShape(
  const Identity &_shapeID) const
{
  auto it = this->collisions.find(_shapeID);
  if (it != this->collisions.end())
  {
    auto sphere = it->second->collision->GetShape();
    if (dynamic_cast<tpe::lib::SphereShape*>(sphere))
      return this->GenerateIdentity(_shapeID, it->second);
  }
  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
double ShapeFeatures::GetSphereShapeRadius(const Identity &_sphereID) const
{
  auto it = this->collisions.find(_sphereID);
  if (it != this->collisions.end())
  {
    auto *sphere = static_cast<tpe::lib::SphereShape*>(
      it->second->collision->GetShape());
    return sphere->GetRadius();
  }
  return -1.0;
}

/////////////////////////////////////////////////
Identity ShapeFeatures::AttachSphereShape(
  const Identity &_linkID,
  const std::string &_name,
  const double _radius,
  const Pose3d &_pose)
{
  auto it = this->links.find(_linkID);
  if (it == this->links.end())
  {
    return this->GenerateInvalidId();
  }
  auto &collision = static_cast<tpe::lib::Collision&>(
    it->second->link->AddCollision());
  collision.SetName(_name);
  collision.SetPose(math::eigen3::convert(_pose));

  tpe::SphereShape sphereshape;
  sphereshape.SetRadius(_radius);
  collision.SetShape(sphereshape);

  return this->AddCollision(_linkID, collision);
}

/////////////////////////////////////////////////
Identity ShapeFeatures::CastToMeshShape(
  const Identity &_shapeID) const
{
  auto it = this->collisions.find(_shapeID);
  if (it != this->collisions.end())
  {
    auto mesh = it->second->collision->GetShape();
    if (dynamic_cast<tpe::lib::MeshShape*>(mesh))
      return this->GenerateIdentity(_shapeID, it->second);
  }
  return this->GenerateInvalidId();
}

/////////////////////////////////////////////////
LinearVector3d ShapeFeatures::GetMeshShapeSize(
  const Identity &_meshID) const
{
  auto it = this->collisions.find(_meshID);
  if (it != this->collisions.end())
  {
    auto *mesh = static_cast<tpe::lib::MeshShape*>(
      it->second->collision->GetShape());
    return math::eigen3::convert(mesh->GetBoundingBox().Size());
  }
  return math::eigen3::convert(math::Vector3d(-1.0, -1.0, -1.0));
}

/////////////////////////////////////////////////
LinearVector3d ShapeFeatures::GetMeshShapeScale(
  const Identity &_meshID) const
{
  auto it = this->collisions.find(_meshID);
  if (it != this->collisions.end())
  {
    auto *mesh = static_cast<tpe::lib::MeshShape*>(
      it->second->collision->GetShape());
    return math::eigen3::convert(mesh->GetScale());
  }
  return math::eigen3::convert(math::Vector3d(-1.0, -1.0, -1.0));
}

/////////////////////////////////////////////////
Identity ShapeFeatures::AttachMeshShape(
  const Identity &_linkID,
  const std::string &_name,
  const ignition::common::Mesh &_mesh,
  const Pose3d &_pose,
  const LinearVector3d &_scale)
{
  auto it = this->links.find(_linkID);
  if (it == this->links.end())
  {
    return this->GenerateInvalidId();
  }
  auto &collision = static_cast<tpe::lib::Collision&>(
    it->second->link->AddCollision());
  collision.SetName(_name);
  collision.SetPose(math::eigen3::convert(_pose));

  tpe::lib::MeshShape mesh;
  mesh.SetMesh(_mesh);
  mesh.SetScale(math::eigen3::convert(_scale));
  collision.SetShape(mesh);

  return this->AddCollision(_linkID, collision);
}

/////////////////////////////////////////////////
AlignedBox3d ShapeFeatures::GetShapeAxisAlignedBoundingBox(
  const Identity &_shapeID) const
{
  auto it = this->collisions.find(_shapeID);
  if (it != this->collisions.end())
  {
    auto shape = it->second->collision->GetShape();
    return math::eigen3::convert(shape->GetBoundingBox());
  }
  return math::eigen3::convert(
    math::AxisAlignedBox(
      math::Vector3d(-1.0, -1.0, -1.0),
      math::Vector3d(-1.0, -1.0, -1.0)));
}
