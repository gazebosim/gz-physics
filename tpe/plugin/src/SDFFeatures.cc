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

#include <sdf/Box.hh>
#include <sdf/Geometry.hh>
#include <ignition/common/Console.hh>

#include "SDFFeatures.hh"

using namespace ignition;
using namespace physics;
using namespace tpeplugin;

/////////////////////////////////////////////////
Identity SDFFeatures::ConstructSdfWorld(
    const Identity &_engine,
    const ::sdf::World &_sdfWorld)
{
  const Identity worldID = this->ConstructEmptyWorld(_engine, _sdfWorld.Name());

  // construct models
  for (std::size_t i = 0; i < _sdfWorld.ModelCount(); ++i)
  {
    this->ConstructSdfModel(worldID, *_sdfWorld.ModelByIndex(i));
  }

  return worldID;
}

/////////////////////////////////////////////////
Identity SDFFeatures::ConstructSdfModel(
  const Identity &_worldID,
  const ::sdf::Model &_sdfModel)
{
  // Read sdf params
  const std::string name = _sdfModel.Name();
  std::cerr << "construct model " << name << std::endl;
  const auto pose = _sdfModel.RawPose();

  auto world = this->worlds.at(_worldID)->world;
  tpelib::Entity &ent = world->AddModel();
  tpelib::Model *model = static_cast<tpelib::Model *>(&ent);
  model->SetName(name);
  model->SetPose(pose);
  const auto modelIdentity = this->AddModel(world->GetId(), *model);

  // construct links
  for (std::size_t i = 0; i < _sdfModel.LinkCount(); ++i)
  {
    this->ConstructSdfLink(modelIdentity, *_sdfModel.LinkByIndex(i));
  }

  return modelIdentity;
}

/////////////////////////////////////////////////
Identity SDFFeatures::ConstructSdfLink(
    const Identity &_modelID,
    const ::sdf::Link &_sdfLink)
{
  // Read sdf params
  const std::string name = _sdfLink.Name();
  std::cerr << "construct link " << name << std::endl;
  const auto pose = _sdfLink.RawPose();

  auto model = this->models.at(_modelID)->model;
  tpelib::Entity &ent = model->AddLink();
  tpelib::Link *link = static_cast<tpelib::Link *>(&ent);
  link->SetName(name);
  link->SetPose(pose);
  const auto linkIdentity = this->AddLink(model->GetId(), *link);

  // construct collisions
  for (std::size_t i = 0; i < _sdfLink.CollisionCount(); ++i)
  {
    this->ConstructSdfCollision(linkIdentity, *_sdfLink.CollisionByIndex(i));
  }

  return linkIdentity;
}

/////////////////////////////////////////////////
Identity SDFFeatures::ConstructSdfCollision(
    const Identity &_linkID,
    const ::sdf::Collision &_sdfCollision,
    const common::Mesh *_mesh)
{
  // Read sdf params
  const std::string name = _sdfCollision.Name();
  std::cerr << "construct collision " << name << std::endl;
  const auto pose = _sdfCollision.RawPose();
  const auto geom = _sdfCollision.Geom();

  auto link = this->links.at(_linkID)->link;
  tpelib::Entity &ent = link->AddCollision();
  tpelib::Collision *collision = static_cast<tpelib::Collision *>(&ent);
  collision->SetName(name);
  collision->SetPose(pose);
  if (geom->Type() == ::sdf::GeometryType::BOX)
  {
    const auto boxSdf = geom->BoxShape();
    tpelib::BoxShape shape;
    shape.SetSize(boxSdf->Size());
    collision->SetShape(shape);
  }
  else if (geom->Type() == ::sdf::GeometryType::CYLINDER)
  {
    const auto cylinderSdf = geom->CylinderShape();
    tpelib::CylinderShape shape;
    shape.SetRadius(cylinderSdf->Radius());
    shape.SetLength(cylinderSdf->Length());
    collision->SetShape(shape);
  }
  else if (geom->Type() == ::sdf::GeometryType::SPHERE)
  {
    const auto sphereSdf = geom->SphereShape();
    tpelib::SphereShape shape;
    shape.SetRadius(sphereSdf->Radius());
    collision->SetShape(shape);
  }
  else if (geom->Type() == ::sdf::GeometryType::MESH)
  {
    // \todo(anyone) the code here assumes that the mesh is loaded externally
    // and passed in as argument as there is no logic for searching resources
    // in ign-physics
    if (_mesh)
    {
      tpelib::MeshShape shape;
      shape.SetMesh(*_mesh);
      collision->SetShape(shape);
    }
    else
    {
      ignerr << "Unable to construct mesh shape. Mesh is null" << std::endl;
    }
  }

  const auto collisionIdentity = this->AddCollision(link->GetId(), *collision);
  return collisionIdentity;

/*/////////////////////////////////////////////////
Identity SDFFeatures::BuildSdfLink(
  const Identity &_modelID,
  const ::sdf::Link &_sdfLink,
  const int _linkIndex)
{
  // Read sdf params
  const std::string name = _sdfLink.Name();
  const auto pose = _sdfLink.Pose();
  const auto inertial = _sdfLink.Inertial();
  const auto mass = inertial.MassMatrix().Mass();
  const auto diagonalMoments = inertial.MassMatrix().DiagonalMoments();

  // Get link properties
  const btScalar linkMass = mass;
  const btVector3 linkInertiaDiag =
      convertVec(ignition::math::eigen3::convert(diagonalMoments));
  const auto poseIsometry = ignition::math::eigen3::convert(pose);

  // Add default fixed joints to links unless replaced by other joints
  // Find translation
  const btVector3 parentComToCurrentCom = convertVec(
    poseIsometry.translation());
  const btVector3 currentPivotToCurrentCom(0, 0, 0);
  const btVector3 parentComToCurrentPivot = parentComToCurrentCom -
    currentPivotToCurrentCom;
  // Find rotation
  btQuaternion rotParentToThis;
  const btMatrix3x3 mat = convertMat(poseIsometry.linear());
  mat.getRotation(rotParentToThis);

  // Set up fixed joints
  const int parentIndex = -1;
  const auto &model = this->models.at(_modelID)->model;
  model->setupFixed(_linkIndex, linkMass, linkInertiaDiag, parentIndex,
                     rotParentToThis, parentComToCurrentPivot,
                     currentPivotToCurrentCom);

  const auto linkIdentity = this->AddLink({name, _linkIndex, linkMass,
                                  linkInertiaDiag, poseIsometry, _modelID});

  // Build collisions
  for (std::size_t i = 0; i < _sdfLink.CollisionCount(); ++i)
  {
    this->BuildSdfCollision(linkIdentity, *_sdfLink.CollisionByIndex(i));
  }

  return linkIdentity;
}
*/
}
