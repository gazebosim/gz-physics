/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

#ifndef IGNITION_PHYSICS_DARTSIM_SRC_SDFFEATURES_HH_
#define IGNITION_PHYSICS_DARTSIM_SRC_SDFFEATURES_HH_

#include <string>

#include <ignition/physics/sdf/ConstructCollision.hh>
#include <ignition/physics/sdf/ConstructJoint.hh>
#include <ignition/physics/sdf/ConstructLink.hh>
#include <ignition/physics/sdf/ConstructModel.hh>
#include <ignition/physics/sdf/ConstructVisual.hh>
#include <ignition/physics/sdf/ConstructWorld.hh>

#include <ignition/physics/Implements.hh>

#include "Base.hh"
#include "EntityManagementFeatures.hh"

namespace ignition {
namespace physics {
namespace dartsim {

using SDFFeatureList = FeatureList<
  sdf::ConstructSdfWorld,
  sdf::ConstructSdfModel,
  sdf::ConstructSdfLink,
  sdf::ConstructSdfJoint,
  sdf::ConstructSdfCollision,
  sdf::ConstructSdfVisual
>;

class SDFFeatures :
    public virtual EntityManagementFeatures,
    public virtual Implements3d<SDFFeatureList>
{
  public: Identity ConstructSdfWorld(
      const std::size_t /*_engine*/,
      const ::sdf::World &_sdfWorld) override;

  public: Identity ConstructSdfModel(
      const std::size_t _worldID,
      const ::sdf::Model &_sdfModel) override;

  public: Identity ConstructSdfLink(
      const std::size_t _modelID,
      const ::sdf::Link &_sdfLink) override;

  public: Identity ConstructSdfJoint(
      const std::size_t _modelID,
      const ::sdf::Joint &_sdfJoint) override;

  public: Identity ConstructSdfCollision(
      const std::size_t _linkID,
      const ::sdf::Collision &_collision) override;

  public: Identity ConstructSdfVisual(
      const std::size_t _linkID,
      const ::sdf::Visual &_visual) override;

  public: dart::dynamics::BodyNode *FindOrConstructLink(
      const dart::dynamics::SkeletonPtr &_model,
      const std::size_t _modelID,
      const ::sdf::Model &_sdfModel,
      const std::string &_linkName);

  /// \brief Construct a joint between two input links.
  /// \param[in] _sdfJoint Contains joint parameters
  /// \param[in] _parent Pointer to parent link. If nullptr, the parent is
  /// assumed to be world
  /// \param[in] _child Pointer to child link. If nullptr, the child is assumed
  /// to be world
  private: Identity ConstructSdfJoint(const ModelInfo &_modelInfo,
      const ::sdf::Joint &_sdfJoint,
      dart::dynamics::BodyNode * const _parent,
      dart::dynamics::BodyNode * const _child);

  private: Eigen::Isometry3d ResolveSdfLinkReferenceFrame(
      const std::string &_frame,
      const ModelInfo &_model) const;

  private: Eigen::Isometry3d ResolveSdfJointReferenceFrame(
      const std::string &_frame,
      const dart::dynamics::BodyNode *_child) const;
};

}
}
}


#endif
