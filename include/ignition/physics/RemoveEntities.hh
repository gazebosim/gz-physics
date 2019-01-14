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

#ifndef IGNITION_PHYSICS_REMOVEENTITIES_HH_
#define IGNITION_PHYSICS_REMOVEENTITIES_HH_

#include <string>

#include <ignition/physics/FeatureList.hh>

namespace ignition
{
  namespace physics
  {
    // class IGNITION_PHYSICS_VISIBLE RemoveWorldFromEngine : public virtual Feature
    // {
    //   public: template <typename PolicyT, typename FeaturesT>
    //   class Engine : public virtual Feature::Engine<PolicyT, FeaturesT>
    //   {
    //     /// \brief Remove a world that is being managed by this engine.
    //     /// \param[in] _index
    //     ///   Index of the world within this engine
    //     /// \return True if world was found and removed.
    //     public: bool RemoveWorld(std::size_t _index);

    //     /// \brief Remove a world that is being managed by this engine.
    //     /// \param[in] _name
    //     ///   Name of the world within this engine
    //     /// \return True if world was found and removed.
    //     public: bool RemoveWorld(const std::string &_name);
    //   };

    //   public: template <typename PolicyT, typename FeaturesT>
    //   class World : public virtual Feature::World<PolicyT, FeaturesT>
    //   {
    //     /// \brief Remove this model
    //     public: void Remove();
    //   };

    //   public: template <typename PolicyT>
    //   class Implementation : public virtual Feature::Implementation<PolicyT>
    //   {
    //     // Engine functions
    //     public: virtual void RemoveWorldByIndex(
    //         std::size_t _engineID, std::size_t _worldIndex) = 0;

    //     public: virtual bool RemoveWorldByName(
    //         std::size_t _engineID, const std::string &_worldName) = 0;

    //     // World functions
    //     public: virtual bool RemoveWorld(
    //         std::size_t _engineID, std::size_t _worldID) = 0;
    //   };
    // };

    class IGNITION_PHYSICS_VISIBLE RemoveModelFromWorld : public virtual Feature
    {
      public: template <typename PolicyT, typename FeaturesT>
      class World : public virtual Feature::World<PolicyT, FeaturesT>
      {
        /// \brief Remove a Model that exists within this World.
        /// \param[in] _index
        ///   Index of the model within this world.
        /// \return True if the model was found and removed.
        public: bool RemoveModel(std::size_t _index);

        /// \brief Remove a Model that exists within this World.
        /// \param[in] _name
        ///   Name of the model within this world.
        /// \return True if the model was found and removed.
        public: bool RemoveModel(const std::string &_name);
      };

      public: template <typename PolicyT, typename FeaturesT>
      class Model : public virtual Feature::Model<PolicyT, FeaturesT>
      {
        /// \brief Remove this model
        public: void Remove();
      };

      public: template <typename PolicyT>
      class Implementation : public virtual Feature::Implementation<PolicyT>
      {
        // World functions
        public: virtual bool RemoveModelByIndex(
            const Identity &_worldID, std::size_t _modelIndex) = 0;

        public: virtual bool RemoveModelByName(
            const Identity &_worldID, const std::string &_modelName) = 0;

        //Model functions
        public: virtual void RemoveModel(
            const Identity &_modelID) = 0;
      };
    };

    // class IGNITION_PHYSICS_VISIBLE RemoveLinkFromModel : public virtual Feature
    // {
    //   public: template <typename PolicyT, typename FeaturesT>
    //   class Model : public virtual Feature::Model<PolicyT, FeaturesT>
    //   {
    //     // typedefs for the type of Link that this Model can return
    //     public: using LinkPtrType = LinkPtr<PolicyT, FeaturesT>;
    //     public: using ConstLinkPtrType = ConstLinkPtr<PolicyT, FeaturesT>;

    //     /// \brief Get the number of Links within this Model.
    //     public: std::size_t RemoveLinkCount() const;

    //     /// \brief Get a Link that exists within this Model.
    //     /// \param[in] _index
    //     ///   Index of the Link within this Model.
    //     /// \return A Link reference. If _index is RemoveLinkCount() or higher,
    //     /// this will be a nullptr.
    //     public: LinkPtrType RemoveLink(std::size_t _index);

    //     /// \sa RemoveLink(std::size_t)
    //     public: ConstLinkPtrType RemoveLink(std::size_t _index) const;

    //     /// \brief Get a Link that exists within this Model.
    //     /// \param[in] _name
    //     ///   Name of the Link within this Model.
    //     /// \return A Link reference. If a Link named _name does not exist in
    //     /// this Model, this will be a nullptr.
    //     public: LinkPtrType RemoveLink(const std::string &_name);

    //     /// \sa RemoveLink(const std::string &)
    //     public: ConstLinkPtrType RemoveLink(const std::string &_name) const;
    //   };

    //   public: template <typename PolicyT, typename FeaturesT>
    //   class Link : public virtual Feature::Link<PolicyT, FeaturesT>
    //   {
    //     // typedefs for the type of Model that this Link can return
    //     public: using ModelPtrType = ModelPtr<PolicyT, FeaturesT>;
    //     public: using ConstModelPtrType = ConstModelPtr<PolicyT, FeaturesT>;

    //     /// \brief Get the name of this Link
    //     public: const std::string &GetName() const;

    //     /// \brief Get the index of this Link within its Model.
    //     public: std::size_t GetIndex() const;

    //     /// \brief Get the model that contains this Link.
    //     /// \return A Model reference to the Model that contains this Link.
    //     public: ModelPtrType RemoveModel();

    //     /// \sa RemoveModel()
    //     public: ConstModelPtrType RemoveModel() const;
    //   };

    //   public: template <typename PolicyT>
    //   class Implementation : public virtual Feature::Implementation<PolicyT>
    //   {
    //     public: virtual std::size_t RemoveLinkCount(
    //         std::size_t _modelID) const = 0;

    //     public: virtual Identity RemoveLink(
    //         std::size_t _modelID, std::size_t _linkIndex) const = 0;

    //     public: virtual Identity RemoveLink(
    //         std::size_t _modelID, const std::string &_linkName) const = 0;

    //     public: virtual const std::string &RemoveLinkName(
    //         std::size_t _linkID) const = 0;

    //     public: virtual std::size_t RemoveLinkIndex(
    //         std::size_t _linkID) const = 0;

    //     public: virtual Identity RemoveModelOfLink(
    //         std::size_t _linkID) const = 0;
    //   };
    // };

    // class IGNITION_PHYSICS_VISIBLE RemoveJointFromModel : public virtual Feature
    // {
    //   public: template <typename PolicyT, typename FeaturesT>
    //   class Model : public virtual Feature::Model<PolicyT, FeaturesT>
    //   {
    //     // typedefs for the type of Joint that this Model can return
    //     public: using JointPtrType = JointPtr<PolicyT, FeaturesT>;
    //     public: using ConstJointPtrType = ConstJointPtr<PolicyT, FeaturesT>;

    //     /// \brief Get the number of Joints within this Model.
    //     public: std::size_t RemoveJointCount() const;

    //     /// \brief Get a Joint that exists within this Model.
    //     /// \param[in] _index
    //     ///   Index of the Joint within this Model.
    //     /// \return A Joint reference. If _index is RemoveJointCount() or higher,
    //     /// this will be a nullptr.
    //     public: JointPtrType RemoveJoint(std::size_t _index);

    //     /// \sa RemoveJoint(std::size_t)
    //     public: ConstJointPtrType RemoveJoint(std::size_t _index) const;

    //     /// \brief Get a Joint that exists within this Model.
    //     /// \param[in] _name
    //     ///   Name of the Joint within this Model.
    //     /// \return A Joint reference. If a Joint named _name does not exist in
    //     /// this Model, this will be a nullptr.
    //     public: JointPtrType RemoveJoint(const std::string &_name);

    //     /// \sa RemoveJoint(const std::string &)
    //     public: ConstJointPtrType RemoveJoint(const std::string &_name) const;
    //   };

    //   public: template <typename PolicyT, typename FeaturesT>
    //   class Joint : public virtual Feature::Joint<PolicyT, FeaturesT>
    //   {
    //     // typedefs for the type of Model that this Link can return
    //     public: using ModelPtrType = ModelPtr<PolicyT, FeaturesT>;
    //     public: using ConstModelPtrType = ConstModelPtr<PolicyT, FeaturesT>;

    //     /// \brief Get the name of this Joint.
    //     public: const std::string &GetName() const;

    //     /// \brief Get the index of this Joint within its Model.
    //     public: std::size_t GetIndex() const;

    //     /// \brief Get the model that contains this Link.
    //     /// \return A Model reference to the Model that contains this Link.
    //     public: ModelPtrType RemoveModel();

    //     /// \sa RemoveModel()
    //     public: ConstModelPtrType RemoveModel() const;
    //   };

    //   public: template <typename PolicyT>
    //   class Implementation : public virtual Feature::Implementation<PolicyT>
    //   {
    //     public: virtual std::size_t RemoveJointCount(
    //         std::size_t _modelID) const = 0;

    //     public: virtual Identity RemoveJoint(
    //         std::size_t _modelID, std::size_t _jointIndex) const = 0;

    //     public: virtual Identity RemoveJoint(
    //         std::size_t _modelID, const std::string &_jointName) const = 0;

    //     public: virtual const std::string &RemoveJointName(
    //         std::size_t _jointID) const = 0;

    //     public: virtual std::size_t RemoveJointIndex(
    //         std::size_t _jointID) const = 0;

    //     public: virtual Identity RemoveModelOfJoint(
    //         std::size_t _jointID) const = 0;
    //   };
    // };

    // class IGNITION_PHYSICS_VISIBLE RemoveShapeFromLink : public virtual Feature
    // {
    //   public: template <typename PolicyT, typename FeaturesT>
    //   class Link : public virtual Feature::Link<PolicyT, FeaturesT>
    //   {
    //     // typedefs for the type of Shape that this Link can return
    //     public: using ShapePtrType = ShapePtr<PolicyT, FeaturesT>;
    //     public: using ConstShapePtrType = ConstShapePtr<PolicyT, FeaturesT>;

    //     /// \brief Get the number of Shapes within this Link.
    //     public: std::size_t RemoveShapeCount() const;

    //     /// \brief Get a Shape that exists within this Link.
    //     /// \param[in] _index
    //     ///   Index of the Shape within this Link
    //     /// \return A Shape reference. If _index is RemoveShapeCount() or higher,
    //     /// this will be a nullptr.
    //     public: ShapePtrType RemoveShape(std::size_t _index);

    //     /// \sa RemoveShape(std::size_t)
    //     public: ConstShapePtrType RemoveShape(std::size_t _index) const;

    //     /// \brief Get a Shape that exists within this Link.
    //     /// \param[in] _name
    //     ///   Name of the Shape within this Link
    //     /// \return A Shape reference. If a Shape named _name does not exist in
    //     /// this Shape, this will be a nullptr.
    //     public: ShapePtrType RemoveShape(const std::string &_name);

    //     /// \sa RemoveShape(const std::string&)
    //     public: ConstShapePtrType RemoveShape(const std::string &_name) const;
    //   };

    //   public: template <typename PolicyT, typename FeaturesT>
    //   class Shape : public virtual Feature::Shape<PolicyT, FeaturesT>
    //   {
    //     // typedefs for the type of Link that this Shape can return
    //     public: using LinkPtrType = LinkPtr<PolicyT, FeaturesT>;
    //     public: using ConstLinkPtrType = ConstLinkPtr<PolicyT, FeaturesT>;

    //     /// \brief Get the name of this Shape.
    //     public: const std::string &GetName() const;

    //     /// \brief Get the index of this Shape within its Link.
    //     public: std::size_t GetIndex() const;

    //     /// \brief Get the Link that contains this Shape.
    //     /// \return a Link reference to the Link that contains this Shape.
    //     public: LinkPtrType RemoveLink();

    //     /// \sa RemoveLink()
    //     public: ConstLinkPtrType RemoveLink() const;
    //   };

    //   public: template <typename PolicyT>
    //   class Implementation : public virtual Feature::Implementation<PolicyT>
    //   {
    //     public: virtual std::size_t RemoveShapeCount(
    //         std::size_t _linkID) const = 0;

    //     public: virtual Identity RemoveShape(
    //         std::size_t _linkID, std::size_t _shapeIndex) const = 0;

    //     public: virtual Identity RemoveShape(
    //         std::size_t _linkID, const std::string &_shapeName) const = 0;

    //     public: virtual const std::string &RemoveShapeName(
    //         std::size_t _shapeID) const = 0;

    //     public: virtual std::size_t RemoveShapeIndex(
    //         std::size_t _shapeID) const = 0;

    //     public: virtual Identity RemoveLinkOfShape(
    //         std::size_t _shapeID) const = 0;
    //   };
    // };

    // using RemoveEntities = FeatureList<
      // RemoveWorldFromEngine,
      // RemoveModelFromWorld,
      // RemoveLinkFromModel,
      // RemoveJointFromModel,
      // RemoveShapeFromLink
    // >;
    using RemoveEntities = FeatureList<
      RemoveModelFromWorld
    >;
  }
}

#include <ignition/physics/detail/RemoveEntities.hh>

#endif
