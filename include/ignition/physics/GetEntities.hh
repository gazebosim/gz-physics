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
    /////////////////////////////////////////////////
    /// \brief This feature retrieves the physics engine name in use.
    class IGNITION_PHYSICS_VISIBLE GetEngineInfo : public virtual Feature
    {
      public: template <typename PolicyT, typename FeaturesT>
      class Engine : public virtual Feature::Engine<PolicyT, FeaturesT>
      {
        /// \brief Get the name of this engine. The meaning of an engine name
        /// is plugin-defined.
        public: const std::string &GetName() const;

        /// \brief Get the index of this engine. The meaning of an engine index
        /// is plugin-defined.
        public: std::size_t GetIndex() const;
      };

      public: template <typename PolicyT>
      class Implementation : public virtual Feature::Implementation<PolicyT>
      {
        public: virtual const std::string &GetEngineName(
            const Identity &_engineID) const = 0;

        public: virtual std::size_t GetEngineIndex(
            const Identity &_engineID) const = 0;
      };
    };

    /////////////////////////////////////////////////
    /// \brief This feature retrieves the world pointer using index or name
    /// from the physics engine in use.
    class IGNITION_PHYSICS_VISIBLE GetWorldFromEngine : public virtual Feature
    {
      public: template <typename PolicyT, typename FeaturesT>
      class Engine : public virtual Feature::Engine<PolicyT, FeaturesT>
      {
        // typedefs for the type of World that this engine can return.
        public: using WorldPtrType = WorldPtr<PolicyT, FeaturesT>;
        public: using ConstWorldPtrType = ConstWorldPtr<PolicyT, FeaturesT>;

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

        /// \brief Get the name of this World.
        public: const std::string &GetName() const;

        /// \brief Get the index of this World within its engine.
        public: std::size_t GetIndex() const;

        /// \brief Get the Engine that is managing this World
        /// \return A reference to the Engine that is managing this World
        public: EnginePtrType GetEngine();

        /// \sa GetEngine()
        public: ConstEnginePtrType GetEngine() const;
      };

      public: template <typename PolicyT>
      class Implementation : public virtual Feature::Implementation<PolicyT>
      {
        public: virtual std::size_t GetWorldCount(
            const Identity &_engineID) const = 0;

        public: virtual Identity GetWorld(
            const Identity &_engineID, std::size_t _worldIndex) const = 0;

        public: virtual Identity GetWorld(
            const Identity &_engineID, const std::string &_worldName) const = 0;

        public: virtual const std::string &GetWorldName(
            const Identity &_worldID) const = 0;

        public: virtual std::size_t GetWorldIndex(
            const Identity &_worldID) const = 0;

        public: virtual Identity GetEngineOfWorld(
            const Identity &_worldID) const = 0;
      };
    };

    /////////////////////////////////////////////////
    /// \brief This feature retrieves the model pointer from the simulation
    /// world by specifying world index and model index/name.
    class IGNITION_PHYSICS_VISIBLE GetModelFromWorld : public virtual Feature
    {
      public: template <typename PolicyT, typename FeaturesT>
      class World : public virtual Feature::World<PolicyT, FeaturesT>
      {
        // typedefs for the type of Model that this World can return
        public: using ModelPtrType = ModelPtr<PolicyT, FeaturesT>;
        public: using ConstModelPtrType = ConstModelPtr<PolicyT, FeaturesT>;

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

        /// \brief Get the name of this Model
        public: const std::string &GetName() const;

        /// \brief Get the index of this Model within its World
        public: std::size_t GetIndex() const;

        /// \brief Get the World that contains this Model.
        /// \return A reference to the World containing this Model.
        public: WorldPtrType GetWorld();

        /// \sa GetWorld()
        public: ConstWorldPtrType GetWorld() const;
      };

      public: template <typename PolicyT>
      class Implementation : public virtual Feature::Implementation<PolicyT>
      {
        public: virtual std::size_t GetModelCount(
            const Identity &_worldID) const = 0;

        public: virtual Identity GetModel(
            const Identity &_worldID, std::size_t _modelIndex) const = 0;

        public: virtual Identity GetModel(
            const Identity &_worldID, const std::string &_modelName) const = 0;

        public: virtual const std::string &GetModelName(
            const Identity &_modelID) const = 0;

        public: virtual std::size_t GetModelIndex(
            const Identity &_modelID) const = 0;

        public: virtual Identity GetWorldOfModel(
            const Identity &_modelID) const = 0;
      };
    };

    /////////////////////////////////////////////////
    /// \brief This feature retrieves the nested model pointer from the parent
    /// model by specifying the name or index of the nested model.
    class IGNITION_PHYSICS_VISIBLE GetNestedModelFromModel
        : public virtual FeatureWithRequirements<GetModelFromWorld>
    {
      public: template <typename PolicyT, typename FeaturesT>
      class Model : public virtual Feature::Model<PolicyT, FeaturesT>
      {
        // typedefs for the type of Model that this Model can return
        public: using ModelPtrType = ModelPtr<PolicyT, FeaturesT>;
        public: using ConstModelPtrType = ConstModelPtr<PolicyT, FeaturesT>;

        /// \brief Get the number of nested Models inside this Model.
        public: std::size_t GetNestedModelCount() const;

        /// \brief Get a nested Model that exists within this Model.
        /// \param[in] _index
        ///   Index of the model within this model.
        /// \return A model reference. If _index is GetNestedModelCount() or
        /// higher, this will be a nullptr.
        public: ModelPtrType GetNestedModel(std::size_t _index);

        /// \sa GetNestedModel(std::size_t)
        public: ConstModelPtrType GetNestedModel(std::size_t _index) const;

        /// \brief Get a nested Model that exists within this Model.
        /// \param[in] _name
        ///   Name of the nested model within this model.
        /// \return A model reference. If a nested model named _name does not
        /// exist in this model, this will be a nullptr.
        public: ModelPtrType GetNestedModel(const std::string &_name);

        /// \sa GetNestedModel(const std::string &)
        public: ConstModelPtrType GetNestedModel(
                    const std::string &_name) const;
      };

      public: template <typename PolicyT>
      class Implementation : public virtual Feature::Implementation<PolicyT>
      {
        public: virtual std::size_t GetNestedModelCount(
            const Identity &_modelID) const = 0;

        public: virtual Identity GetNestedModel(
            const Identity &_modelID, std::size_t _modelIndex) const = 0;

        public: virtual Identity GetNestedModel(
            const Identity &_modelID, const std::string &_modelName) const = 0;
      };
    };

    /////////////////////////////////////////////////
    /// \brief This feature retrieves the link pointer from the model
    /// by specifying model index and link index/name.
    class IGNITION_PHYSICS_VISIBLE GetLinkFromModel : public virtual Feature
    {
      public: template <typename PolicyT, typename FeaturesT>
      class Model : public virtual Feature::Model<PolicyT, FeaturesT>
      {
        // typedefs for the type of Link that this Model can return
        public: using LinkPtrType = LinkPtr<PolicyT, FeaturesT>;
        public: using ConstLinkPtrType = ConstLinkPtr<PolicyT, FeaturesT>;

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
      };

      public: template <typename PolicyT, typename FeaturesT>
      class Link : public virtual Feature::Link<PolicyT, FeaturesT>
      {
        // typedefs for the type of Model that this Link can return
        public: using ModelPtrType = ModelPtr<PolicyT, FeaturesT>;
        public: using ConstModelPtrType = ConstModelPtr<PolicyT, FeaturesT>;

        /// \brief Get the name of this Link
        public: const std::string &GetName() const;

        /// \brief Get the index of this Link within its Model.
        public: std::size_t GetIndex() const;

        /// \brief Get the model that contains this Link.
        /// \return A Model reference to the Model that contains this Link.
        public: ModelPtrType GetModel();

        /// \sa GetModel()
        public: ConstModelPtrType GetModel() const;
      };

      public: template <typename PolicyT>
      class Implementation : public virtual Feature::Implementation<PolicyT>
      {
        public: virtual std::size_t GetLinkCount(
            const Identity &_modelID) const = 0;

        public: virtual Identity GetLink(
            const Identity &_modelID, std::size_t _linkIndex) const = 0;

        public: virtual Identity GetLink(
            const Identity &_modelID, const std::string &_linkName) const = 0;

        public: virtual const std::string &GetLinkName(
            const Identity &_linkID) const = 0;

        public: virtual std::size_t GetLinkIndex(
            const Identity &_linkID) const = 0;

        public: virtual Identity GetModelOfLink(
            const Identity &_linkID) const = 0;
      };
    };

    /////////////////////////////////////////////////
    /// \brief This feature retrieves the joint pointer from the model
    /// by specifying model index and joint index/name.
    class IGNITION_PHYSICS_VISIBLE GetJointFromModel : public virtual Feature
    {
      public: template <typename PolicyT, typename FeaturesT>
      class Model : public virtual Feature::Model<PolicyT, FeaturesT>
      {
        // typedefs for the type of Joint that this Model can return
        public: using JointPtrType = JointPtr<PolicyT, FeaturesT>;
        public: using ConstJointPtrType = ConstJointPtr<PolicyT, FeaturesT>;

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

      public: template <typename PolicyT>
      class Implementation : public virtual Feature::Implementation<PolicyT>
      {
        public: virtual std::size_t GetJointCount(
            const Identity &_modelID) const = 0;

        public: virtual Identity GetJoint(
            const Identity &_modelID, std::size_t _jointIndex) const = 0;

        public: virtual Identity GetJoint(
            const Identity &_modelID, const std::string &_jointName) const = 0;

        public: virtual const std::string &GetJointName(
            const Identity &_jointID) const = 0;

        public: virtual std::size_t GetJointIndex(
            const Identity &_jointID) const = 0;

        public: virtual Identity GetModelOfJoint(
            const Identity &_jointID) const = 0;
      };
    };

    /////////////////////////////////////////////////
    /// \brief This feature retrieves the shape pointer from the link
    /// by specifying link index and shape index/name.
    class IGNITION_PHYSICS_VISIBLE GetShapeFromLink : public virtual Feature
    {
      public: template <typename PolicyT, typename FeaturesT>
      class Link : public virtual Feature::Link<PolicyT, FeaturesT>
      {
        // typedefs for the type of Shape that this Link can return
        public: using ShapePtrType = ShapePtr<PolicyT, FeaturesT>;
        public: using ConstShapePtrType = ConstShapePtr<PolicyT, FeaturesT>;

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
        public: virtual std::size_t GetShapeCount(
            const Identity &_linkID) const = 0;

        public: virtual Identity GetShape(
            const Identity &_linkID, std::size_t _shapeIndex) const = 0;

        public: virtual Identity GetShape(
            const Identity &_linkID, const std::string &_shapeName) const = 0;

        public: virtual const std::string &GetShapeName(
            const Identity &_shapeID) const = 0;

        public: virtual std::size_t GetShapeIndex(
            const Identity &_shapeID) const = 0;

        public: virtual Identity GetLinkOfShape(
            const Identity &_shapeID) const = 0;
      };
    };

    struct GetEntities : FeatureList<
      GetEngineInfo,
      GetWorldFromEngine,
      GetModelFromWorld,
      GetNestedModelFromModel,
      GetLinkFromModel,
      GetJointFromModel,
      GetShapeFromLink
    > { };
  }
}

#include <ignition/physics/detail/GetEntities.hh>

#endif
