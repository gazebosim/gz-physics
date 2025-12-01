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

#ifndef GZ_PHYSICS_BULLET_FEATHERSTONE_SRC_SDFFEATURES_HH_
#define GZ_PHYSICS_BULLET_FEATHERSTONE_SRC_SDFFEATURES_HH_

#include <string>

#include <gz/physics/sdf/ConstructCollision.hh>
#include <gz/physics/sdf/ConstructJoint.hh>
#include <gz/physics/sdf/ConstructModel.hh>
#include <gz/physics/sdf/ConstructNestedModel.hh>
#include <gz/physics/sdf/ConstructWorld.hh>
#include <gz/physics/Implements.hh>

#include <sdf/Collision.hh>

#include "EntityManagementFeatures.hh"

namespace gz {
namespace physics {
namespace bullet_featherstone {

struct SDFFeatureList : gz::physics::FeatureList<
  sdf::ConstructSdfJoint,
  sdf::ConstructSdfModel,
  sdf::ConstructSdfNestedModel,
  sdf::ConstructSdfWorld,
  sdf::ConstructSdfCollision
> { };

class SDFFeatures :
    public virtual EntityManagementFeatures,
    public virtual Implements3d<SDFFeatureList>
{
  public: Identity ConstructSdfWorld(
      const Identity &/*_engine*/,
      const ::sdf::World &_sdfWorld) override;

  public: Identity ConstructSdfModel(
      const Identity &_worldID,
      const ::sdf::Model &_sdfModel) override;

  public: Identity ConstructSdfNestedModel(
      const Identity &_parentID,
      const ::sdf::Model &_sdfModel) override;

  public: bool AddSdfCollision(
      const Identity &_linkID,
      const ::sdf::Collision &_collision,
      bool isStatic);

  public: Identity ConstructSdfJoint(
      const Identity &_modelID,
      const ::sdf::Joint &_sdfJoint) override;

  private: Identity ConstructSdfCollision(
      const Identity &_linkID,
      const ::sdf::Collision &_collision) override;

  /// \brief Construct a bullet entity from a sdf::Model
  /// \param[in] _parentID Parent identity
  /// \param[in] _sdfModel sdf::Model to construct entity from
  /// \return The entity identity if constructed otherwise an invalid identity
  private: Identity ConstructSdfModelImpl(std::size_t _parentID,
                                          const ::sdf::Model &_sdfModel);

  /// \brief Create and initialze the link collider in link info
  /// \param[in] _linkID ID of link to create the collider for
  /// \param[in] _isStatic True if the link is static
  /// \param[in] _shape Collision shape to attach to link
  /// \param[in] _shapeTF Collision shape local transform in link collider frame
  private: void CreateLinkCollider(const Identity &_linkID,
      bool _isStatic, btCollisionShape *_shape = nullptr,
      const btTransform &_shapeTF = btTransform::getIdentity());
};

}  // namespace bullet_featherstone
}  // namespace physics
}  // namespace gz

#endif
