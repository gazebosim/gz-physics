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

#include "Collision.hh"
#include "Link.hh"

using namespace ignition;
using namespace physics;
using namespace tpe;
using namespace lib;

//////////////////////////////////////////////////
Link::Link() : Entity()
{
}

//////////////////////////////////////////////////
Link::Link(uint64_t _id) : Entity(_id)
{
}

//////////////////////////////////////////////////
Entity &Link::AddCollision()
{
  uint64_t collisionId = Entity::GetNextId();
  const auto [it, success] = this->GetChildren().insert(
    {collisionId, std::make_shared<Collision>(collisionId)});
  return *it->second.get();
}
