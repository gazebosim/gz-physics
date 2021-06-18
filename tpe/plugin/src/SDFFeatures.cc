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

#include "SDFFeatures.hh"

#include <sdf/Box.hh>
#include <sdf/Capsule.hh>
#include <sdf/Cylinder.hh>
#include <sdf/Ellipsoid.hh>
#include <sdf/Sphere.hh>
#include <sdf/Geometry.hh>
#include <sdf/World.hh>
#include <ignition/common/Console.hh>

namespace ignition {
namespace physics {
namespace tpeplugin {

namespace {
/////////////////////////////////////////////////
/// \brief Resolve the pose of an SDF DOM object with respect to its relative_to
/// frame. If that fails, return the raw pose
static math::Pose3d ResolveSdfPose(const ::sdf::SemanticPose &_semPose)
{
  math::Pose3d pose;
  ::sdf::Errors errors = _semPose.Resolve(pose);
  if (!errors.empty())
  {
    if (!_semPose.RelativeTo().empty())
    {
      ignerr << "There was an error in SemanticPose::Resolve\n";
      for (const auto &err : errors)
      {
        ignerr << err.Message() << std::endl;
      }
      ignerr << "There is no optimal fallback since the relative_to attribute["
             << _semPose.RelativeTo() << "] of the pose is not empty. "
             << "Falling back to using the raw Pose.\n";
    }
    pose = _semPose.RawPose();
  }
  return pose;
}
}  // namespace

/////////////////////////////////////////////////
Identity SDFFeatures::ConstructSdfWorld(
    const Identity &_engine,
    const ::sdf::World &_sdfWorld)
{
  const Identity worldID = this->ConstructEmptyWorld(_engine, _sdfWorld.Name());

  // construct models
  for (std::size_t i = 0; i < _sdfWorld.ModelCount(); ++i)
  {
    const ::sdf::Model *model = _sdfWorld.ModelByIndex(i);
    if (model && model->ModelCount() == 0u)
    {
      this->ConstructSdfModel(worldID, *model);
    }
    else
    {
      this->ConstructSdfNestedModel(worldID, *model);
    }
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
  const auto pose = ResolveSdfPose(_sdfModel.SemanticPose());
  const bool isStatic = _sdfModel.Static();

  auto it = this->worlds.find(_worldID.id);
  if (it == this->worlds.end())
  {
    ignwarn << "World [" << _worldID.id << "] is not found." << std::endl;
    return this->GenerateInvalidId();
  }
  auto world = it->second->world;
  if (world == nullptr)
  {
    ignwarn << "World is a nullptr" << std::endl;
    return this->GenerateInvalidId();
  }
  tpelib::Entity &ent = world->AddModel();
  tpelib::Model *model = static_cast<tpelib::Model *>(&ent);
  model->SetName(name);
  model->SetPose(pose);
  model->SetStatic(isStatic);
  const auto modelIdentity = this->AddModel(world->GetId(), *model);

  // construct links
  for (std::size_t i = 0; i < _sdfModel.LinkCount(); ++i)
  {
    this->ConstructSdfLink(modelIdentity, *_sdfModel.LinkByIndex(i));
  }

  if (_sdfModel.LinkCount() > 0u)
  {
    // set canonical link id
    if (_sdfModel.CanonicalLink() != nullptr)
    {
      std::string canonicalLinkName = _sdfModel.CanonicalLinkName();
      tpelib::Entity &canonicalLink = model->GetChildByName(canonicalLinkName);
      model->SetCanonicalLink(canonicalLink.GetId());
    }
    else
    {
      model->SetCanonicalLink();
    }
  }

  return modelIdentity;
}

/////////////////////////////////////////////////
Identity SDFFeatures::ConstructSdfNestedModel(
  const Identity &_parentID,
  const ::sdf::Model &_sdfModel)
{
  tpelib::Model *model = nullptr;
  std::size_t parentId = 0u;

  // check if parent is world
  auto worldIt = this->worlds.find(_parentID.id);
  if (worldIt != this->worlds.end())
  {
    auto world = worldIt->second->world;
    if (world == nullptr)
    {
      ignwarn << "Parent world is a null" << std::endl;
      return this->GenerateInvalidId();
    }
    parentId = world->GetId();
    tpelib::Entity &ent = world->AddModel();
    model = static_cast<tpelib::Model *>(&ent);
  }
  else
  {
    // check if parent is model
    auto modelIt = this->models.find(_parentID.id);
    if (modelIt != this->models.end())
    {
      auto parent = modelIt->second->model;
      if (parent == nullptr)
      {
        ignwarn << "Parent model is a null" << std::endl;
        return this->GenerateInvalidId();
      }
      parentId = parent->GetId();
      tpelib::Entity &ent = parent->AddModel();
      model = static_cast<tpelib::Model *>(&ent);
    }
  }
  if (!model)
    return this->GenerateInvalidId();

  // Read sdf params
  const std::string name = _sdfModel.Name();
  const auto pose = ResolveSdfPose(_sdfModel.SemanticPose());

  model->SetName(name);
  model->SetPose(pose);
  const auto modelIdentity = this->AddModel(parentId, *model);

  // construct links
  for (std::size_t i = 0; i < _sdfModel.LinkCount(); ++i)
  {
    this->ConstructSdfLink(modelIdentity, *_sdfModel.LinkByIndex(i));
  }

  if (_sdfModel.LinkCount() > 0u)
  {
    // set canonical link id
    if (_sdfModel.CanonicalLink() != nullptr)
    {
      std::string canonicalLinkName = _sdfModel.CanonicalLinkName();
      tpelib::Entity &canonicalLink = model->GetChildByName(canonicalLinkName);
      model->SetCanonicalLink(canonicalLink.GetId());
    }
    else
    {
      model->SetCanonicalLink();
    }
  }

  // construct nested models
  for (std::size_t i = 0; i < _sdfModel.ModelCount(); ++i)
  {
    this->ConstructSdfNestedModel(modelIdentity, *_sdfModel.ModelByIndex(i));
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
  const auto pose = ResolveSdfPose(_sdfLink.SemanticPose());

  auto it = this->models.find(_modelID);
  if (it == this->models.end())
  {
    ignwarn << "Model [" << _modelID.id << "] is not found" << std::endl;
    return this->GenerateInvalidId();
  }
  auto model = it->second->model;
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
  const auto pose = ResolveSdfPose(_sdfCollision.SemanticPose());
  const auto geom = _sdfCollision.Geom();

  auto it = this->links.find(_linkID);
  if (it == this->links.end())
  {
    ignwarn << "Link [" << _linkID.id << "] is not found" << std::endl;
    return this->GenerateInvalidId();
  }
  auto link = it->second->link;
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
  else if (geom->Type() == ::sdf::GeometryType::CAPSULE)
  {
    const auto capsuleSdf = geom->CapsuleShape();
    tpelib::CapsuleShape shape;
    shape.SetRadius(capsuleSdf->Radius());
    shape.SetLength(capsuleSdf->Length());
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
  else if (geom->Type() == ::sdf::GeometryType::ELLIPSOID)
  {
    const auto ellipsoidSdf = geom->EllipsoidShape();
    tpelib::EllipsoidShape shape;
    shape.SetRadii(ellipsoidSdf->Radii());
    collision->SetShape(shape);
  }
  else if (geom->Type() == ::sdf::GeometryType::SPHERE)
  {
    const auto sphereSdf = geom->SphereShape();
    tpelib::SphereShape shape;
    shape.SetRadius(sphereSdf->Radius());
    collision->SetShape(shape);
  }
  else
  {
    ignwarn << "Geometry type not supported for collision [" << name << "]."
            << std::endl;
  }
  // \todo(anyone) add mesh. currently mesh has to be loaded externally
  // and passed in as argument as there is no logic for searching resources
  // in ign-physics
  const auto collisionIdentity = this->AddCollision(link->GetId(), *collision);

  // set collide bitmask
  uint16_t collideBitmask = 0xFF;
  if (_sdfCollision.Element())
  {
    // TODO(anyone) add category_bitmask as well
    auto elem = _sdfCollision.Element();
    if (elem->HasElement("surface"))
    {
      elem = elem->GetElement("surface");
      if (elem->HasElement("contact"))
      {
        elem = elem->GetElement("contact");
        if (elem->HasElement("collide_bitmask"))
        {
          collideBitmask = elem->Get<unsigned int>("collide_bitmask");
          this->SetCollisionFilterMask(collisionIdentity, collideBitmask);
        }
      }
    }
  }

  return collisionIdentity;
}

}
}
}
