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

#ifndef IGNITION_PHYSICS_BULLET_BASE_HH_
#define IGNITION_PHYSICS_BULLET_BASE_HH_

#include "BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h"
#include "BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h"
#include "BulletDynamics/Featherstone/btMultiBody.h"
#include "BulletDynamics/Featherstone/btMultiBodyLinkCollider.h"
#include "btBulletDynamicsCommon.h"

#include <assert.h>
#include <string>

#include <Eigen/Geometry>
#include <sdf/Joint.hh>

#include <ignition/common/Console.hh>
#include <ignition/physics/Implements.hh>

namespace ignition {
namespace physics {
namespace bullet {

/// \brief The structs WorldInfo are used
/// for three reasons:
/// 1) Holding extra information such as the name
///    that will be different from the underlying engine
/// 2) Wrap shared pointers to Bullet entities to use as parameters to
///    GenerateIdentity.
/// 3) Hold explicit copies of raw pointers that can be deallocated

struct WorldInfo
{
  btMultiBodyDynamicsWorld* world;
  std::string name;
  btDefaultCollisionConfiguration* collisionConfiguration;
  btCollisionDispatcher* dispatcher;
  btBroadphaseInterface* broadphase;
  btMultiBodyConstraintSolver* solver;
}; // To Do: Handle cleaning these pointers

struct ModelInfo
{
  btMultiBody* model;
  std::string name;
  int numLinks;
  btScalar baseMass;
  btVector3 baseInertiaDiag;
  bool fixedBase;
  bool canSleep;
  btTransform baseTransform;
  bool selfCollide;
  bool finalized;
  Identity world;
};


struct LinkInfo
{
  std::string name;
  int linkIndex;
  btScalar linkMass;
  btVector3 linkInertiaDiag;
  Eigen::Isometry3d poseIsometry;
  Identity model;
};

struct JointInfo
{
  std::string name;
  ::sdf::JointType type;
  int childIndex;
  int parentIndex;
  btVector3 axis1;
  btVector3 axis2;
  Eigen::Isometry3d poseIsometry;
  std::size_t childID;
  std::size_t parentID;
  Identity model;
};

struct CollisionInfo
{
  btCollisionShape* shape;
  btMultiBodyLinkCollider* collider;
  btTransform transform;
  btScalar mu;
  bool isDynamic;
  Identity link;
  Identity model;
};

class Base : public Implements3d<FeatureList<Feature>>
{
  public: using WorldInfoPtr = std::shared_ptr<WorldInfo>;
  public: using ModelInfoPtr = std::shared_ptr<ModelInfo>;
  public: using LinkInfoPtr = std::shared_ptr<LinkInfo>;
  public: using JointInfoPtr = std::shared_ptr<JointInfo>;
  public: using CollisionInfoPtr = std::shared_ptr<CollisionInfo>;

  public: std::size_t entityCount = 0;

  public: inline std::size_t GetNextEntity()
  {
    return entityCount++;
  }

  public: inline Identity InitiateEngine(std::size_t /*_engineID*/) override
  {
    const auto id = this->GetNextEntity();
    assert(id == 0);

    return this->GenerateIdentity(0);
  }

  public: inline Identity AddWorld(WorldInfo _worldInfo)
  {
    const auto id = this->GetNextEntity();
    this->worlds[id] = std::make_shared<WorldInfo>(_worldInfo);
    return this->GenerateIdentity(id, this->worlds.at(id));
  }

  public: inline Identity AddModel(ModelInfo _modelInfo)
  {
    const auto id = this->GetNextEntity();
    this->models[id] = std::make_shared<ModelInfo>(_modelInfo);

    return this->GenerateIdentity(id, this->models.at(id));
  }

  public: inline Identity AddLink(LinkInfo _linkInfo)
  {
    const auto id = this->GetNextEntity();
    this->links[id] = std::make_shared<LinkInfo>(_linkInfo);

    return this->GenerateIdentity(id, this->links.at(id));
  }

  public: inline Identity AddJoint(JointInfo _jointInfo)
  {
    const auto id = this->GetNextEntity();
    this->joints[id] = std::make_shared<JointInfo>(_jointInfo);

    return this->GenerateIdentity(id, this->joints.at(id));
  }

  public: inline Identity AddCollision(CollisionInfo _collisionInfo)
  {
    const auto id = this->GetNextEntity();
    this->collisions[id] = std::make_shared<CollisionInfo>(_collisionInfo);

    return this->GenerateIdentity(id, this->collisions.at(id));
  }

  public: std::unordered_map<std::size_t, WorldInfoPtr> worlds;
  public: std::unordered_map<std::size_t, ModelInfoPtr> models;
  public: std::unordered_map<std::size_t, LinkInfoPtr> links;
  public: std::unordered_map<std::size_t, JointInfoPtr> joints;
  public: std::unordered_map<std::size_t, CollisionInfoPtr> collisions;
};

}
}
}

#endif