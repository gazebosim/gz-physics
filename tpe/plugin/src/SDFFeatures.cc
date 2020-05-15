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
#include <sdf/Cylinder.hh>
#include <sdf/Mesh.hh>
#include <sdf/Sphere.hh>
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
  const auto pose = _sdfModel.RawPose();

  if (this->worlds.find(_worldID.id) == this->worlds.end())
  {
    ignwarn << "World [" << _worldID.id << "] is not found." << std::endl;
    return this->GenerateInvalidId();
  }
  auto world = this->worlds.at(_worldID)->world;
  if (world == nullptr)
  {
    ignwarn << "World is a nullptr" << std::endl;
    return this->GenerateInvalidId();
  }
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
  igndbg << "construct link " << name << std::endl;
  const auto pose = _sdfLink.RawPose();

  if (this->models.find(_modelID) == this->models.end())
  {
    ignwarn << "Model [" << _modelID.id << "] is not found" << std::endl;
    return this->GenerateInvalidId();
  } 
  auto model = this->models.at(_modelID)->model;
  if (model == nullptr)
  {
    ignwarn << "Model is a nullptr" << std::endl;
    return this->GenerateInvalidId();
  }
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
    const ::sdf::Collision &_sdfCollision)
{
  // Read sdf params
  const std::string name = _sdfCollision.Name();
  igndbg << "construct collision " << name << std::endl;
  const auto pose = _sdfCollision.RawPose();
  const auto geom = _sdfCollision.Geom();

  if (this->links.find(_linkID) == this->links.end())
  {
    ignwarn << "Link [" << _linkID.id << "] is not found" << std::endl;
    return this->GenerateInvalidId();
  }
  auto link = this->links.at(_linkID)->link;
  if (link == nullptr)
  {
    ignwarn << "Link is a nullptr" << std::endl;
    return this->GenerateInvalidId();
  }

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
  const auto collisionIdentity = this->AddCollision(link->GetId(), *collision);
  return collisionIdentity;
}

/////////////////////////////////////////////////
Identity SDFFeatures::ConstructSdfCollision(
    const Identity &_linkID,
    const ::sdf::Collision &_sdfCollision,
    const common::Mesh *_mesh)
{
  // Read sdf params
  const std::string name = _sdfCollision.Name();
  igndbg << "construct collision " << name << std::endl;
  const auto pose = _sdfCollision.RawPose();
  const auto geom = _sdfCollision.Geom();

  if (this->links.find(_linkID) == this->links.end())
  {
    ignwarn << "Link [" << _linkID.id << "] is not found" << std::endl;
    return this->GenerateInvalidId();
  }
  auto link = this->links.at(_linkID)->link;
  if (link == nullptr)
  {
    ignwarn << "Link is a nullptr" << std::endl;
    return this->GenerateInvalidId();
  }

  tpelib::Entity &ent = link->AddCollision();
  tpelib::Collision *collision = static_cast<tpelib::Collision *>(&ent);
  collision->SetName(name);
  collision->SetPose(pose);
  if (geom->Type() == ::sdf::GeometryType::MESH)
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
}
