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

#ifndef IGNITION_PHYSICS_DARTSIM_SDFFEATURES_HH_
#define IGNITION_PHYSICS_DARTSIM_SDFFEATURES_HH_

#include <ignition/physics/sdf/ConstructWorld.hh>
#include <ignition/physics/sdf/ConstructModel.hh>
#include <ignition/physics/sdf/ConstructLink.hh>
#include <ignition/physics/sdf/ConstructJoint.hh>

#include <ignition/physics/Implements.hh>

#include "Base.hh"

namespace ignition {
namespace physics {
namespace dartsim {

using SDFFeatureList = FeatureList<
  sdf::ConstructSdfWorld,
  sdf::ConstructSdfModel,
  sdf::ConstructSdfLink,
  sdf::ConstructSdfJoint
>;

class SDFFeatures :
    public virtual Base,
    public virtual Implements3d<SDFFeatureList>
{
  public: Identity ConstructSdfWorld(
      const std::size_t /*_engine*/,
      const ::sdf::World &_sdfWorld) override;

  public: dart::dynamics::BodyNode *FindOrConstructLink(
      const dart::dynamics::SkeletonPtr &_model,
      const std::size_t _modelID,
      const ::sdf::Model &_sdfModel,
      const std::string &_linkName,
      const std::string &_jointName);

  public: Identity ConstructSdfModel(
      const std::size_t _worldID,
      const ::sdf::Model &_sdfModel) override;

  public: Identity ConstructSdfLink(
      const std::size_t _modelID,
      const ::sdf::Link &_sdfLink) override;

  public: Identity ConstructSdfJoint(
      const std::size_t _modelID,
      const ::sdf::Joint &_sdfJoint) override;

  public: Identity ConstructSdfJoint(
      const dart::dynamics::SkeletonPtr &_model,
      const ::sdf::Joint &_sdfJoint,
      dart::dynamics::BodyNode * const _parent,
      dart::dynamics::BodyNode * const _child);

  public: Eigen::Isometry3d ResolveSdfLinkPose(
      const std::string &_frame,
      const std::size_t _modelID) const;
};

}
}
}


#endif
