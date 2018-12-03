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

#ifndef IGNITION_PHYSICS_GETENTITIES_HH_
#define IGNITION_PHYSICS_GETENTITIES_HH_

#include <string>

#include <ignition/physics/FeatureList.hh>

namespace ignition
{
  namespace physics
  {
    class IGNITION_PHYSICS_VISIBLE GetEntities : public virtual Feature
    {
      public: template <typename PolicyT, typename FeaturesT>
      class Engine : public virtual Feature::Engine<PolicyT, FeaturesT>
      {
        // typedefs for the type of World that this engine can return.
        public: using WorldPtrType = WorldPtr<PolicyT, FeaturesT>;
        public: using ConstWorldPtrType = ConstWorldPtr<PolicyT, FeaturesT>;

        /// \brief Get the name of this engine. The meaning of an engine name
        /// is plugin-defined.
        public: const std::string &GetName() const;

        /// \brief Get the index of this engine. The meaning of an engine index
        /// is plugin-defined.
        public: std::size_t GetIndex() const;

        /// \brief Get the number of worlds inside this engine.
        public: std::size_t GetWorldCount() const;

        /// \brief Get a world that is being managed by this engine.
        /// \param[in] _index
        ///   Index of the world within this engine
        /// \return A world reference. If _index is GetWorldCount() or higher,
        /// this will be a nullptr.
        public: WorldPtrType GetWorld(std::size_t _index);

        /// \sa GetWorld(std::size_t)
        public: ConstWorldPtrType GetWorld(std::size_t _index) const;

        /// \brief Get a world that is being managed by this engine.
        /// \param[in] _name
        ///   Name of the world
        /// \return A world reference. If a world named _name does not exist in
        /// this engine, this will be a nullptr.
        public: WorldPtrType GetWorld(const std::string &_name);

        /// \sa GetWorld(const std::string &)
        public: ConstWorldPtrType GetWorld(const std::string &_name) const;
      };

      public: template <typename PolicyT, typename FeaturesT>
      class World : public virtual Feature::World<PolicyT, FeaturesT>
      {
        // typedefs for the type of Engine that this World can return
        public: using EnginePtrType = EnginePtr<PolicyT, FeaturesT>;
        public: using ConstEnginePtrType = ConstEnginePtr<PolicyT, FeaturesT>;

        // typedefs for the type of Model that this World can return
        public: using ModelPtrType = ModelPtr<PolicyT, FeaturesT>;
        public: using ConstModelPtrType = ConstModelPtr<PolicyT, FeaturesT>;

        /// \brief Get the name of this World.
        public: const std::string &GetName() const;

        /// \brief Get the index of this World within its engine.
        public: std::size_t GetIndex() const;

        /// \brief Get the Engine that is managing this World
        /// \return A reference to the Engine that is managing this World
        public: EnginePtrType GetEngine();

        /// \sa GetEngine()
        public: ConstEnginePtrType GetEngine() const;

        /// \brief Get the number of Models inside this World.
        public: std::size_t GetModelCount() const;

        /// \brief Get a Model that exists within this World.
        /// \param[in] _index
        ///   Index of the model within this world.
        /// \return A model reference. If _index is GetModelCount() or higher,
        /// this will be a nullptr.
        public: ModelPtrType GetModel(std::size_t _index);

        /// \sa GetModel(std::size_t)
        public: ConstModelPtrType GetModel(std::size_t _index) const;

        /// \brief Get a Model that exists within this World.
        /// \param[in] _name
        ///   Name of the model within this world.
        /// \return A model reference. If a model named _name does not exist in
        /// this world, this will be a nullptr.
        public: ModelPtrType GetModel(const std::string &_name);

        /// \sa GetModel(const std::string &)
        public: ConstModelPtrType GetModel(const std::string &_name) const;
      };

      public: template <typename PolicyT, typename FeaturesT>
      class Model : public virtual Feature::Model<PolicyT, FeaturesT>
      {
        // typedefs for the type of World that this Model can return
        public: using WorldPtrType = WorldPtr<PolicyT, FeaturesT>;
        public: using ConstWorldPtrType = ConstWorldPtr<PolicyT, FeaturesT>;

        // typedefs for the type of Link that this Model can return
        public: using LinkPtrType = LinkPtr<PolicyT, FeaturesT>;
        public: using ConstLinkPtrType = ConstLinkPtr<PolicyT, FeaturesT>;

        // typedefs for the type of Joint that this Model can return
        public: using JointPtrType = JointPtr<PolicyT, FeaturesT>;
        public: using ConstJointPtrType = ConstJointPtr<PolicyT, FeaturesT>;

        /// \brief Get the name of this Model
        public: const std::string &GetName() const;

        /// \brief Get the index of this Model within its World
        public: std::size_t GetIndex() const;

        /// \brief Get the World that contains this Model.
        /// \return A reference to the World containing this Model.
        public: WorldPtrType GetWorld();

        /// \sa GetWorld()
        public: ConstWorldPtrType GetWorld() const;

        /// \brief Get the number of Links within this Model.
        public: std::size_t GetLinkCount() const;

        /// \brief Get a Link that exists within this Model.
        /// \param[in] _index
        ///   Index of the Link within this Model.
        /// \return A Link reference. If _index is GetLinkCount() or higher,
        /// this will be a nullptr.
        public: LinkPtrType GetLink(std::size_t _index);

        /// \sa GetLink(std::size_t)
        public: ConstLinkPtrType GetLink(std::size_t _index) const;

        /// \brief Get a Link that exists within this Model.
        /// \param[in] _name
        ///   Name of the Link within this Model.
        /// \return A Link reference. If a Link named _name does not exist in
        /// this Model, this will be a nullptr.
        public: LinkPtrType GetLink(const std::string &_name);

        /// \sa GetLink(const std::string &)
        public: ConstLinkPtrType GetLink(const std::string &_name) const;

        /// \brief Get the number of Joints within this Model.
        public: std::size_t GetJointCount() const;

        /// \brief Get a Joint that exists within this Model.
        /// \param[in] _index
        ///   Index of the Joint within this Model.
        /// \return A Joint reference. If _index is GetJointCount() or higher,
        /// this will be a nullptr.
        public: JointPtrType GetJoint(std::size_t _index);

        /// \sa GetJoint(std::size_t)
        public: ConstJointPtrType GetJoint(std::size_t _index) const;

        /// \brief Get a Joint that exists within this Model.
        /// \param[in] _name
        ///   Name of the Joint within this Model.
        /// \return A Joint reference. If a Joint named _name does not exist in
        /// this Model, this will be a nullptr.
        public: JointPtrType GetJoint(const std::string &_name);

        /// \sa GetJoint(const std::string &)
        public: ConstJointPtrType GetJoint(const std::string &_name) const;
      };

      public: template <typename PolicyT, typename FeaturesT>
      class Link : public virtual Feature::Link<PolicyT, FeaturesT>
      {
        // typedefs for the type of Model that this Link can return
        public: using ModelPtrType = ModelPtr<PolicyT, FeaturesT>;
        public: using ConstModelPtrType = ConstModelPtr<PolicyT, FeaturesT>;

        // typedefs for the type of Shape that this Link can return
        public: using ShapePtrType = ShapePtr<PolicyT, FeaturesT>;
        public: using ConstShapePtrType = ConstShapePtr<PolicyT, FeaturesT>;

        /// \brief Get the name of this Link
        public: const std::string &GetName() const;

        /// \brief Get the index of this Link within its Model.
        public: std::size_t GetIndex() const;

        /// \brief Get the model that contains this Link.
        /// \return A Model reference to the Model that contains this Link.
        public: ModelPtrType GetModel();

        /// \sa GetModel()
        public: ConstModelPtrType GetModel() const;

        /// \brief Get the number of Shapes within this Link.
        public: std::size_t GetShapeCount() const;

        /// \brief Get a Shape that exists within this Link.
        /// \param[in] _index
        ///   Index of the Shape within this Link
        /// \return A Shape reference. If _index is GetShapeCount() or higher,
        /// this will be a nullptr.
        public: ShapePtrType GetShape(std::size_t _index);

        /// \sa GetShape(std::size_t)
        public: ConstShapePtrType GetShape(std::size_t _index) const;

        /// \brief Get a Shape that exists within this Link.
        /// \param[in] _name
        ///   Name of the Shape within this Link
        /// \return A Shape reference. If a Shape named _name does not exist in
        /// this Shape, this will be a nullptr.
        public: ShapePtrType GetShape(const std::string &_name);

        /// \sa GetShape(const std::string&)
        public: ConstShapePtrType GetShape(const std::string &_name) const;
      };

      public: template <typename PolicyT, typename FeaturesT>
      class Joint : public virtual Feature::Joint<PolicyT, FeaturesT>
      {
        // typedefs for the type of Model that this Link can return
        public: using ModelPtrType = ModelPtr<PolicyT, FeaturesT>;
        public: using ConstModelPtrType = ConstModelPtr<PolicyT, FeaturesT>;

        /// \brief Get the name of this Joint.
        public: const std::string &GetName() const;

        /// \brief Get the index of this Joint within its Model.
        public: std::size_t GetIndex() const;

        /// \brief Get the model that contains this Link.
        /// \return A Model reference to the Model that contains this Link.
        public: ModelPtrType GetModel();

        /// \sa GetModel()
        public: ConstModelPtrType GetModel() const;
      };

      public: template <typename PolicyT, typename FeaturesT>
      class Shape : public virtual Feature::Shape<PolicyT, FeaturesT>
      {
        // typedefs for the type of Link that this Shape can return
        public: using LinkPtrType = LinkPtr<PolicyT, FeaturesT>;
        public: using ConstLinkPtrType = ConstLinkPtr<PolicyT, FeaturesT>;

        /// \brief Get the name of this Shape.
        public: const std::string &GetName() const;

        /// \brief Get the index of this Shape within its Link.
        public: std::size_t GetIndex() const;

        /// \brief Get the Link that contains this Shape.
        /// \return a Link reference to the Link that contains this Shape.
        public: LinkPtrType GetLink();

        /// \sa GetLink()
        public: ConstLinkPtrType GetLink() const;
      };

      public: template <typename PolicyT>
      class Implementation : public virtual Feature::Implementation<PolicyT>
      {
        public: virtual const std::string &GetEngineName(
            std::size_t _engineID) const = 0;

        public: virtual std::size_t GetEngineIndex(
            std::size_t _engineID) const = 0;

        public: virtual std::size_t GetWorldCount(
            std::size_t _engineID) const = 0;

        public: virtual Identity GetWorld(
            std::size_t _engineID, std::size_t _worldIndex) const = 0;

        public: virtual Identity GetWorld(
            std::size_t _engineID, const std::string &_worldName) const = 0;

        public: virtual const std::string &GetWorldName(
            std::size_t _worldID) const = 0;

        public: virtual std::size_t GetWorldIndex(
            std::size_t _worldID) const = 0;

        public: virtual Identity GetEngineOfWorld(
            std::size_t _worldID) const = 0;

        public: virtual std::size_t GetModelCount(
            std::size_t _worldID) const = 0;

        public: virtual Identity GetModel(
            std::size_t _worldID, std::size_t _modelIndex) const = 0;

        public: virtual Identity GetModel(
            std::size_t _worldID, const std::string &_modelName) const = 0;

        public: virtual const std::string &GetModelName(
            std::size_t _modelID) const = 0;

        public: virtual std::size_t GetModelIndex(
            std::size_t _modelID) const = 0;

        public: virtual Identity GetWorldOfModel(
            std::size_t _modelID) const = 0;

        public: virtual std::size_t GetLinkCount(
            std::size_t _modelID) const = 0;

        public: virtual Identity GetLink(
            std::size_t _modelID, std::size_t _linkIndex) const = 0;

        public: virtual Identity GetLink(
            std::size_t _modelID, const std::string &_linkName) const = 0;

        public: virtual std::size_t GetJointCount(
            std::size_t _modelID) const = 0;

        public: virtual Identity GetJoint(
            std::size_t _modelID, std::size_t _jointIndex) const = 0;

        public: virtual Identity GetJoint(
            std::size_t _modelID, const std::string &_jointName) const = 0;

        public: virtual const std::string &GetLinkName(
            std::size_t _linkID) const = 0;

        public: virtual std::size_t GetLinkIndex(
            std::size_t _linkID) const = 0;

        public: virtual Identity GetModelOfLink(
            std::size_t _linkID) const = 0;

        public: virtual std::size_t GetShapeCount(
            std::size_t _linkID) const = 0;

        public: virtual Identity GetShape(
            std::size_t _linkID, std::size_t _shapeIndex) const = 0;

        public: virtual Identity GetShape(
            std::size_t _linkID, const std::string &_shapeName) const = 0;

        public: virtual const std::string &GetJointName(
            std::size_t _jointID) const = 0;

        public: virtual std::size_t GetJointIndex(
            std::size_t _jointID) const = 0;

        public: virtual Identity GetModelOfJoint(
            std::size_t _jointID) const = 0;

        public: virtual const std::string &GetShapeName(
            std::size_t _shapeID) const = 0;

        public: virtual std::size_t GetShapeIndex(
            std::size_t _shapeID) const = 0;

        public: virtual Identity GetLinkOfShape(
            std::size_t _shapeID) const = 0;
      };
    };

    using GetEntitiesList = FeatureList<
      GetEntities
    >;
  }
}

#include <ignition/physics/detail/GetEntities.hh>

#endif
