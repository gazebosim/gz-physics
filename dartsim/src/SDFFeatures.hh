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

#ifndef GZ_PHYSICS_DARTSIM_SRC_SDFFEATURES_HH_
#define GZ_PHYSICS_DARTSIM_SRC_SDFFEATURES_HH_

#include <gz/math/Inertial.hh>
#include <string>

#include <gz/physics/sdf/ConstructCollision.hh>
#include <gz/physics/sdf/ConstructJoint.hh>
#include <gz/physics/sdf/ConstructLink.hh>
#include <gz/physics/sdf/ConstructModel.hh>
#include <gz/physics/sdf/ConstructNestedModel.hh>
#include <gz/physics/sdf/ConstructVisual.hh>
#include <gz/physics/sdf/ConstructWorld.hh>

#include <gz/physics/Implements.hh>

#include "Base.hh"
#include "EntityManagementFeatures.hh"

namespace gz {
namespace physics {
namespace dartsim {

struct SDFFeatureList : FeatureList<
  sdf::ConstructSdfWorld,
  sdf::ConstructSdfModel,
  sdf::ConstructSdfNestedModel,
  sdf::ConstructSdfLink,
  sdf::ConstructSdfJoint,
  sdf::ConstructSdfCollision,
  sdf::ConstructSdfVisual
> { };

class SDFFeatures :
    public virtual EntityManagementFeatures,
    public virtual Implements3d<SDFFeatureList>
{
  public: Identity ConstructSdfWorld(
      const Identity &/*_engine*/,
      const ::sdf::World &_sdfWorld) override;

  public: Identity ConstructSdfModel(
      const Identity &_parentID,
      const ::sdf::Model &_sdfModel) override;

  public: Identity ConstructSdfNestedModel(
      const Identity &_parentID,
      const ::sdf::Model &_sdfModel) override;

  public: Identity ConstructSdfLink(
      const Identity &_modelID,
      const ::sdf::Link &_sdfLink) override;

  public: Identity ConstructSdfJoint(
      const Identity &_modelID,
      const ::sdf::Joint &_sdfJoint) override;

  public: Identity ConstructSdfCollision(
      const Identity &_linkID,
      const ::sdf::Collision &_collision) override;

  public: Identity ConstructSdfVisual(
      const Identity &_linkID,
      const ::sdf::Visual &_visual) override;

  private: dart::dynamics::BodyNode *FindOrConstructLink(
      const dart::dynamics::SkeletonPtr &_model,
      const Identity &_modelID,
      const ::sdf::Model &_sdfModel,
      const std::string &_linkName);

  /// \brief Construct a joint between two input links.
  /// \param[in] _modelInfo Contains the joint's parent model
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

  /// \brief Construct a dartsim entity from a sdf::model
  /// \param[in] _parentID Id of parent
  /// \param[in] _sdfModel sdf::Model to construct entity from
  /// \return The entity identity if constructed otherwise an invalid identity
  private: Identity ConstructSdfModelImpl(std::size_t _parentID,
                                          const ::sdf::Model &_sdfModel);

  /// \brief Find the dartsim BodyNode associated with the link name
  /// \param[in] _worldName Name of world that contains the link
  /// \param[in] _jointModelName The name of the model associated with the joint
  /// \param[in] _linkRelativeName The relative name of the link as specified in
  /// the sdformat description in the scope of the model identified by
  /// _jointModelName
  /// \returns The matched body node if exactly one match is found, otherwise
  /// a nullptr
  private: dart::dynamics::BodyNode *FindBodyNode(
               const std::string &_worldName,
               const std::string &_jointModelName,
               const std::string &_linkRelativeName);
};

}
}
}


#endif
