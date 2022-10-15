/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#include "Base.hh"

#include <utility>

namespace gz {
namespace physics {
namespace bullet_featherstone {

/////////////////////////////////////////////////
WorldInfo::WorldInfo(std::string name_)
  : name(std::move(name_))
{
  this->collisionConfiguration =
    std::make_unique<btDefaultCollisionConfiguration>();
  this->dispatcher =
    std::make_unique<btCollisionDispatcher>(collisionConfiguration.get());
  this->broadphase = std::make_unique<btDbvtBroadphase>();
  this->solver = std::make_unique<btMultiBodyConstraintSolver>();
  this->world = std::make_unique<btMultiBodyDynamicsWorld>(
    dispatcher.get(), broadphase.get(), solver.get(),
    collisionConfiguration.get());

  btGImpactCollisionAlgorithm::registerAlgorithm(dispatcher.get());
}

}  // namespace bullet_featherstone
}  // namespace physics
}  // namespace gz
