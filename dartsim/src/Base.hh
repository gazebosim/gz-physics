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

#ifndef GZ_PHYSICS_DARTSIM_BASE_HH_
#define GZ_PHYSICS_DARTSIM_BASE_HH_

#include <dart/constraint/ConstraintSolver.hpp>
#include <dart/constraint/WeldJointConstraint.hpp>
#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/SimpleFrame.hpp>
#include <dart/dynamics/Skeleton.hpp>
#include <dart/simulation/World.hpp>

#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/math/eigen3/Conversions.hh>
#include <gz/math/Inertial.hh>
#include <gz/physics/Implements.hh>

#include <sdf/Types.hh>

#if DART_VERSION_AT_LEAST(6, 13, 0)
// The BodyNode::getShapeNodes method was deprecated in dart 6.13.0
// in favor of an iterator approach with BodyNode::eachShapeNode
// See https://github.com/dartsim/dart/pull/1644 for more info
#define DART_HAS_EACH_SHAPE_NODE_API
#endif

namespace gz {
namespace physics {
namespace dartsim {

/// \brief The structs ModelInfo, LinkInfo, JointInfo, and ShapeInfo are used
/// for two reasons:
/// 1) Holding extra information such as the name or offset
///    that will be different from the underlying engine
/// 2) Wrap shared pointers to DART entities. Since these shared pointers (eg.
///    dart::dynamics::BodyNodePtr) are different from std::shared_ptr, we
///    cannot use them directly as parameters to GenerateIdentity. Instead we
///    create a std::shared_ptr of the struct that wraps the corresponding DART
///    shared pointer.


struct LinkInfo
{
  dart::dynamics::BodyNodePtr link;
  /// \brief It may be necessary for dartsim to rename a BodyNode (eg. when
  /// moving the BodyNode to a new skeleton), so we store the Gazebo-specified
  /// name of the Link here.
  std::string name;
  /// \brief To close kinematic loops, a body node may be divided into separate
  /// nodes that are welded together using a WeldJointConstraint. The inertia
  /// is divided evenly between these body nodes. This matches the approach
  /// used in gazebo-classic.
  std::vector< std::pair<
      dart::dynamics::BodyNode *,
      dart::constraint::WeldJointConstraintPtr> > weldedNodes;
  /// \brief The total link inertia, which may be split between the `link` and
  /// `weldedNodes` body nodes.
  std::optional<math::Inertiald> inertial;
};

struct JointInfo
{
  dart::dynamics::JointPtr joint;
  dart::dynamics::SimpleFramePtr frame;
};

struct ModelInfo
{
  dart::dynamics::SkeletonPtr model;
  std::string localName;
  dart::dynamics::SimpleFramePtr frame;
  std::string canonicalLinkName;
  std::vector<std::shared_ptr<LinkInfo>> links {};
  std::vector<std::shared_ptr<JointInfo>> joints {};
  std::vector<std::size_t> nestedModels = {};
};

struct ShapeInfo
{
  dart::dynamics::ShapeNodePtr node;

  /// \brief dartsim has more strict name uniqueness rules tha Gazebo, so we
  /// store the Gazebo-specified name of the Shape here.
  std::string name;

  /// \brief This is added because sometimes the relative transform of a shape
  /// according to Gazebo specifications may be different than the relative
  /// transform of a shape within the dartsim specifications. This is the offset
  /// from the Gazebo specs to the dartsim specs, i.e. T_g * tf_offset = T_d
  /// where T_g is the relative transform according to Gazebo and T_d is the
  /// relative transform according to dartsim.
  Eigen::Isometry3d tf_offset = Eigen::Isometry3d::Identity();
};

template <typename Value1, typename Key2 = Value1>
struct EntityStorage
{
  /// \brief Map from an entity ID to its corresponding object
  std::unordered_map<std::size_t, Value1> idToObject;

  /// \brief Map from an object pointer (or other unique key) to its entity ID
  std::unordered_map<Key2, std::size_t> objectToID;

  using IndexMap = std::unordered_map<std::size_t, std::vector<std::size_t>>;
  /// \brief The key represents the parent ID. The value represents a vector of
  /// the objects' IDs. The key of the vector is the object's index within its
  /// container. This is used by World and Model objects, which don't know their
  /// own indices within their containers as well as Links, whose indices might
  /// change when constructing joints.
  ///
  /// The container type for World is Engine.
  /// The container type for Model is World.
  /// The container type for Link is Model.
  ///
  /// Joints are contained in Models, but they know their own indices within
  /// their Models, so we do not need to use this field for Joints
  IndexMap indexInContainerToID;

  /// \brief Map from an entity ID to its index within its container
  std::unordered_map<std::size_t, std::size_t> idToIndexInContainer;

  /// \brief Map from an entity ID to the ID of its container
  std::unordered_map<std::size_t, std::size_t> idToContainerID;

  Value1 &operator[](const std::size_t _id)
  {
    return idToObject[_id];
  }

  Value1 &at(const std::size_t _id)
  {
    return idToObject.at(_id);
  }

  const Value1 &at(const std::size_t _id) const
  {
    return idToObject.at(_id);
  }

  Value1 &at(const Key2 &_key)
  {
    return idToObject.at(objectToID.at(_key));
  }

  const Value1 &at(const Key2 &_key) const
  {
    return idToObject.at(objectToID.at(_key));
  }

  std::size_t size() const
  {
    return idToObject.size();
  }

  std::size_t IdentityOf(const Key2 &_key) const
  {
    return objectToID.at(_key);
  }

  bool HasEntity(const Key2 &_key) const
  {
    return objectToID.find(_key) != objectToID.end();
  }

  bool HasEntity(const std::size_t _id) const
  {
    return idToObject.find(_id) != idToObject.end();
  }

  bool RemoveEntity(const Key2 &_key)
  {
    auto entIter = this->objectToID.find(_key);
    if (entIter!= this->objectToID.end())
    {
      std::size_t entId = entIter->second;

      // Check if we are keeping track of the index of this entity in its
      // container
      auto contIter = this->idToContainerID.find(entId);
      if (contIter != this->idToContainerID.end())
      {
        std::size_t contId = contIter->second;
        std::size_t entIndex = this->idToIndexInContainer.at(entId);

        // house keeping
        // The key in indexInContainerToID is the index of the vector so erasing
        // the element automatically decrements the index of the rest of the
        // elements of the vector. The indices in idToIndexInContainer, however,
        // are stored as numbers (as values in the map). We need to decrement
        // all the indices greater than the index of the model we are removing.
        for (auto indIter =
                 this->indexInContainerToID[contId].begin() + entIndex + 1;
             indIter != this->indexInContainerToID[contId].end(); ++indIter)
        {
          // decrement the index (the value of the map)
          --this->idToIndexInContainer[*indIter];
        }

        this->idToIndexInContainer.erase(entId);
        this->indexInContainerToID[contId].erase(
            this->indexInContainerToID[contId].begin() + entIndex);
        this->idToContainerID.erase(entId);
      }

      this->objectToID.erase(entIter);
      this->idToObject.erase(entId);
      return true;
    }
    return false;
  }
};

class Base : public Implements3d<FeatureList<Feature>>
{
  public: using DartWorld = dart::simulation::World;
  public: using DartWorldPtr = dart::simulation::WorldPtr;
  public: using DartSkeletonPtr = dart::dynamics::SkeletonPtr;
  public: using DartConstSkeletonPtr = dart::dynamics::ConstSkeletonPtr;
  public: using DartSkeleton = dart::dynamics::Skeleton;
  public: using DartBodyNode = dart::dynamics::BodyNode;
  public: using DartBodyNodePtr = dart::dynamics::BodyNodePtr;
  public: using DartJoint = dart::dynamics::Joint;
  public: using DartJointPtr = dart::dynamics::JointPtr;
  public: using DartShapeNode = dart::dynamics::ShapeNode;
  public: using DartShapeNodePtr = std::shared_ptr<DartShapeNode>;
  public: using ModelInfoPtr = std::shared_ptr<ModelInfo>;
  public: using LinkInfoPtr = std::shared_ptr<LinkInfo>;
  public: using JointInfoPtr = std::shared_ptr<JointInfo>;
  public: using ShapeInfoPtr = std::shared_ptr<ShapeInfo>;

  public: inline Identity InitiateEngine(std::size_t /*_engineID*/) override
  {
    this->GetNextEntity();

    // Create a 0th entry in this map
    this->worlds.indexInContainerToID.insert(
          std::make_pair(0u, std::vector<std::size_t>()));

    // dartsim does not have multiple "engines"
    return this->GenerateIdentity(0);
  }

  public: inline std::size_t GetNextEntity()
  {
    return entityCount++;
  }

  public: std::size_t entityCount = 0;

  public: inline std::size_t AddWorld(
      const DartWorldPtr &_world, const std::string &_name)
  {
    const std::size_t id = this->GetNextEntity();

    this->worlds.idToObject[id] = _world;
    this->worlds.objectToID[_name] = id;

    std::vector<std::size_t> &indexInContainerToID =
        this->worlds.indexInContainerToID.at(0);

    this->worlds.idToIndexInContainer[id] = indexInContainerToID.size();
    indexInContainerToID.push_back(id);

    this->worlds.idToContainerID[id] = 0;

    _world->setName(_name);
    this->frames[id] = dart::dynamics::Frame::World();

    return id;
  }

  public: inline std::tuple<std::size_t, ModelInfo&> AddModel(
      const ModelInfo &_info, const std::size_t _worldID)
  {
    const std::size_t id = this->GetNextEntity();
    this->models.idToObject[id] = std::make_shared<ModelInfo>(_info);
    ModelInfo &entry = *this->models.idToObject[id];
    this->models.objectToID[_info.model] = id;

    const dart::simulation::WorldPtr &world = worlds[_worldID];

    std::vector<std::size_t> &indexInContainerToID =
        this->models.indexInContainerToID[_worldID];
    const std::size_t indexInWorld = indexInContainerToID.size();
    this->models.idToIndexInContainer[id] = indexInWorld;
    indexInContainerToID.push_back(id);
    world->addSkeleton(entry.model);

    this->models.idToContainerID[id] = _worldID;
    this->frames[id] = _info.frame.get();

    return std::forward_as_tuple(id, entry);
  }

  public: inline std::tuple<std::size_t, ModelInfo &> AddNestedModel(
              const ModelInfo &_info, const std::size_t _parentID,
              const std::size_t _worldID)
  {
    const std::size_t id = this->GetNextEntity();
    this->models.idToObject[id] = std::make_shared<ModelInfo>(_info);
    ModelInfo &entry = *this->models.idToObject[id];
    this->models.objectToID[_info.model] = id;

    const dart::simulation::WorldPtr &world = worlds[_worldID];

    auto parentModelInfo = this->models.at(_parentID);
    const std::size_t indexInModel =
        parentModelInfo->nestedModels.size();
    this->models.idToIndexInContainer[id] = indexInModel;
    std::vector<std::size_t> &indexInContainerToID =
        this->models.indexInContainerToID[_parentID];
    indexInContainerToID.push_back(id);
    world->addSkeleton(entry.model);

    this->models.idToContainerID[id] = _parentID;
    this->frames[id] = _info.frame.get();
    parentModelInfo->nestedModels.push_back(id);
    return {id, entry};
  }

  public: inline std::size_t AddLink(DartBodyNode *_bn,
        const std::string &_fullName, std::size_t _modelID,
        std::optional<math::Inertiald> _inertial = std::nullopt)
  {
    const std::size_t id = this->GetNextEntity();
    auto linkInfo = std::make_shared<LinkInfo>();
    this->links.idToObject[id] = linkInfo;
    linkInfo->link = _bn;
    // The name of the BodyNode during creation is assumed to be the
    // Gazebo-specified name.
    linkInfo->name = _bn->getName();
    // Inertial properties (if available) used when splitting nodes to close
    // kinematic loops.
    linkInfo->inertial = _inertial;
    this->links.objectToID[_bn] = id;
    this->frames[id] = _bn;

    this->linksByName[_fullName] = _bn;
    this->models.at(_modelID)->links.push_back(linkInfo);

    // Even though DART keeps track of the index of this BodyNode in the
    // skeleton, the BodyNode may be moved to another skeleton when a joint is
    // constructed. Thus, we store the original index here.
    this->links.idToIndexInContainer[id] = _bn->getIndexInSkeleton();
    std::vector<std::size_t> &indexInContainerToID =
        this->links.indexInContainerToID[_modelID];
    indexInContainerToID.push_back(id);

    this->links.idToContainerID[id] = _modelID;

    return id;
  }

  private: static math::Inertiald DivideInertial(
               const math::Inertiald &_wholeInertial, std::size_t _count)
  {
    if (_count == 1)
    {
      return _wholeInertial;
    }
    math::Inertiald dividedInertial;
    math::MassMatrix3d dividedMassMatrix;
    dividedMassMatrix.SetMass(_wholeInertial.MassMatrix().Mass() /
                              static_cast<double>(_count));
    dividedMassMatrix.SetMoi(_wholeInertial.MassMatrix().Moi() *
                             (1. / static_cast<double>(_count)));
    dividedInertial.SetMassMatrix(dividedMassMatrix);
    dividedInertial.SetPose(_wholeInertial.Pose());
    return dividedInertial;
  }

  private: static void AssignInertialToBody(
               const math::Inertiald &_inertial, DartBodyNode * _body)
  {
    const math::Matrix3d &moi = _inertial.Moi();
    const math::Vector3d &com = _inertial.Pose().Pos();
    _body->setMass(_inertial.MassMatrix().Mass());
    _body->setMomentOfInertia(moi(0, 0), moi(1, 1), moi(2, 2), moi(0, 1),
                              moi(0, 2), moi(1, 2));
    _body->setLocalCOM(math::eigen3::convert(com));
  }

  public: inline DartBodyNode* SplitAndWeldLink(LinkInfo *_link)
  {
    // First create a new body node with FreeJoint and a unique name based
    // on the number of welded miror nodes.
    dart::dynamics::BodyNode::Properties weldedBodyProperties;
    weldedBodyProperties.mIsCollidable = false;
    {
      std::size_t weldedBodyCount = _link->weldedNodes.size();
      weldedBodyProperties.mName =
          _link->name + "_welded_mirror_" + std::to_string(weldedBodyCount);
    }
    dart::dynamics::FreeJoint::Properties jointProperties;
    jointProperties.mName = weldedBodyProperties.mName + "_FreeJoint";
    DartSkeletonPtr skeleton = _link->link->getSkeleton();
    auto pairJointBodyNode =
      skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>(
        nullptr, jointProperties, weldedBodyProperties);

    // Weld the new body node to the original
    auto weld = std::make_shared<dart::constraint::WeldJointConstraint>(
        _link->link, pairJointBodyNode.second);
    _link->weldedNodes.emplace_back(pairJointBodyNode.second, weld);
    auto worldId = this->GetWorldOfModelImpl(models.objectToID[skeleton]);
    auto dartWorld = this->worlds.at(worldId);
    dartWorld->getConstraintSolver()->addConstraint(weld);

    // Rebalance the link inertia between the original body node and its
    // welded mirror nodes if inertial data is available.
    if (_link->inertial)
    {
      std::size_t nodeCount = 1 + _link->weldedNodes.size();
      const auto dividedInertial = DivideInertial(*_link->inertial, nodeCount);
      AssignInertialToBody(dividedInertial, _link->link);
      for (const auto &weldedNodePair : _link->weldedNodes)
      {
        AssignInertialToBody(dividedInertial, weldedNodePair.first);
      }
    }

    this->linkByWeldedNode[pairJointBodyNode.second] = _link;
    return pairJointBodyNode.second;
  }

  public: void MergeLinkAndWeldedBody(LinkInfo *_link, DartBodyNode *child)
  {
    // Break the existing joint first.
    child->moveTo<dart::dynamics::FreeJoint>(nullptr);
    auto it = _link->weldedNodes.begin();
    bool foundWeld = false;
    for (; it != _link->weldedNodes.end(); ++it)
    {
      if (it->first == child)
      {
        auto worldId = this->GetWorldOfModelImpl(
            this->models.objectToID[child->getSkeleton()]);
        auto dartWorld = this->worlds.at(worldId);
        dartWorld->getConstraintSolver()->removeConstraint(it->second);
        // Okay to erase since we break afterward.
        _link->weldedNodes.erase(it);
        foundWeld = true;
        break;
      }
    }

    if (!foundWeld)
    {
      // We have not found a welded node associated with _link. This shouldn't
      // happen.
      ignerr << "Could not find welded body node for link " << _link->name
             << ". Merging of link and welded body failed.";
      return;
    }

    if (_link->inertial)
    {
      std::size_t nodeCount = 1 + _link->weldedNodes.size();
      const auto dividedInertial = DivideInertial(*_link->inertial, nodeCount);
      AssignInertialToBody(dividedInertial, _link->link);
      for (const auto &weldedNodePair : _link->weldedNodes)
      {
        AssignInertialToBody(dividedInertial, weldedNodePair.first);
      }
    }
  }

  public: inline std::size_t AddJoint(DartJoint *_joint,
      const std::string &_fullName, std::size_t _modelID)
  {
    const std::size_t id = this->GetNextEntity();
    auto jointInfo = std::make_shared<JointInfo>();
    this->joints.idToObject[id] = jointInfo;
    jointInfo->joint = _joint;

    this->joints.idToObject[id]->joint = _joint;
    this->joints.objectToID[_joint] = id;
    dart::dynamics::SimpleFramePtr jointFrame =
        dart::dynamics::SimpleFrame::createShared(
            _joint->getChildBodyNode(), _joint->getName() + "_frame",
            _joint->getTransformFromChildBodyNode());

    this->jointsByName[_fullName] = _joint;
    this->models.at(_modelID)->joints.push_back(jointInfo);

    // Even though DART keeps track of the index of this joint in the
    // skeleton, the joint may be moved to another skeleton when a joint is
    // constructed. Thus, we store the original index here.
    this->joints.idToIndexInContainer[id] = _joint->getJointIndexInSkeleton();
    std::vector<std::size_t> &indexInContainerToID =
        this->joints.indexInContainerToID[_modelID];
    indexInContainerToID.push_back(id);

    this->joints.idToContainerID[id] = _modelID;

    this->joints.idToObject[id]->frame = jointFrame;
    this->frames[id] = this->joints.idToObject[id]->frame.get();

    return id;
  }

  public: inline std::size_t AddShape(
      const ShapeInfo &_info)
  {
    const std::size_t id = this->GetNextEntity();
    this->shapes.idToObject[id] = std::make_shared<ShapeInfo>(_info);
    this->shapes.objectToID[_info.node] = id;
    this->frames[id] = _info.node.get();

    return id;
  }

  public: bool RemoveModelImpl(const std::size_t _worldID,
                               const std::size_t _modelID)
  {
    const auto &world = this->worlds.at(_worldID);
    auto modelInfo = this->models.at(_modelID);
    auto skel = modelInfo->model;
    // Remove the contents of the skeleton from local entity storage containers
    for (auto &nestedModel : modelInfo->nestedModels)
    {
      this->RemoveModelImpl(_worldID, nestedModel);
    }
    modelInfo->nestedModels.clear();

    for (auto &jt : skel->getJoints())
    {
      this->joints.RemoveEntity(jt);
      this->jointsByName.erase(::sdf::JoinName(
          world->getName(), ::sdf::JoinName(skel->getName(), jt->getName())));
    }
    for (auto &bn : skel->getBodyNodes())
    {
#ifdef DART_HAS_EACH_SHAPE_NODE_API
      bn->eachShapeNode([this](dart::dynamics::ShapeNode *_sn)
      {
        this->shapes.RemoveEntity(_sn);
      });
#else
      for (auto &sn : bn->getShapeNodes())
      {
        this->shapes.RemoveEntity(sn);
      }
#endif
      this->links.RemoveEntity(bn);
      this->linksByName.erase(::sdf::JoinName(
          world->getName(), ::sdf::JoinName(skel->getName(), bn->getName())));
    }

    // If this is a nested model, remove an entry from the parent models
    // "nestedModels" vector
    auto parentID = this->models.idToContainerID.at(_modelID);
    if (parentID != _worldID)
    {
      auto parentModelInfo = this->models.at(parentID);
      const std::size_t modelIndex =
          this->models.idToIndexInContainer.at(_modelID);
      if (modelIndex >= parentModelInfo->nestedModels.size())
        return false;
      parentModelInfo->nestedModels.erase(
          parentModelInfo->nestedModels.begin() + modelIndex);
    }
    this->models.RemoveEntity(skel);
    world->removeSkeleton(skel);
    return true;
  }

  public: inline std::size_t GetWorldOfModelImpl(
              const std::size_t &_modelID) const
  {
    if (this->models.HasEntity(_modelID))
    {
      auto parentIt = this->models.idToContainerID.find(_modelID);
      if (parentIt != this->models.idToContainerID.end())
      {
        if (this->worlds.HasEntity(parentIt->second))
        {
          return parentIt->second;
        }
        return this->GetWorldOfModelImpl(parentIt->second);
      }
    }
    return this->GenerateInvalidId();
  }

  public: inline Identity GetModelOfLinkImpl(const Identity &_linkID) const
  {
    const std::size_t modelID = this->links.idToContainerID.at(_linkID);
    if (this->models.HasEntity(modelID))
    {
      return this->GenerateIdentity(modelID, this->models.at(modelID));
    }
    else
    {
      return this->GenerateInvalidId();
    }
  };

  /// \brief Create a fully (world) scoped joint name.
  /// \param _modelID Identity of the parent model of the joint's child link.
  /// \param _childID Identity of the joint's child link.
  /// \param _name The unscoped joint name.
  /// \return The fully (world) scoped joint name, or an empty string
  /// if a world cannot be resolved.
  public: inline std::string FullyScopedJointName(
    const Identity &_modelID,
    const Identity &_childID,
    const std::string &_name) const
  {
    const auto modelInfo = this->ReferenceInterface<ModelInfo>(_modelID);

    auto worldID = this->GetWorldOfModelImpl(_modelID);
    if (worldID == INVALID_ENTITY_ID)
    {
      gzerr << "World of model [" << modelInfo->model->getName()
            << "] could not be found when creating joint [" << _name
            << "]\n";
      return "";
    }

    auto world = this->worlds.at(worldID);
    const std::string fullJointName = ::sdf::JoinName(
        world->getName(),
        ::sdf::JoinName(modelInfo->model->getName(), _name));

    return fullJointName;
  }

  public: EntityStorage<DartWorldPtr, std::string> worlds;
  public: EntityStorage<ModelInfoPtr, DartConstSkeletonPtr> models;
  public: EntityStorage<LinkInfoPtr, const DartBodyNode*> links;
  public: EntityStorage<JointInfoPtr, const DartJoint*> joints;
  public: EntityStorage<ShapeInfoPtr, const DartShapeNode*> shapes;
  public: std::unordered_map<std::size_t, dart::dynamics::Frame*> frames;

  /// \brief Map from the fully qualified link name (including the world name)
  /// to the BodyNode object. This is useful for keeping track of BodyNodes even
  /// as they move to other skeletons.
  public: std::unordered_map<std::string, DartBodyNode*> linksByName;

  /// \brief Map from the fully qualified joint name (including the world name)
  /// to the dart Joint object. This is useful for keeping track of
  /// dart Joints even as they move to other skeletons.
  public: std::unordered_map<std::string, DartJoint*> jointsByName;

  /// \brief Map from welded body nodes to the LinkInfo for the original link
  /// they are welded to. This is useful when detaching joints.
  public: std::unordered_map<DartBodyNode*, LinkInfo*> linkByWeldedNode;

  /// \brief A debug function to list the models and their immediate
  /// nested models, links and joints.
  /// \return A string containing the list of model information.
  public: std::string DebugModels() const
  {
    std::stringstream ss;
    ss << "*** Models ***\n";
    for (size_t id = 0, i = 0; i < models.size(); ++id)
    {
      if (models.HasEntity(id))
      {
        ++i;
        auto modelInfo = models.at(id);
        ss << "ModelID:     " << id << "\n"
           << "LocalName:   " << modelInfo->localName << "\n"
           << "NodeName:    " << modelInfo->model->getName() << "\n"
           << "NumModels:   " << modelInfo->nestedModels.size() << "\n"
           << "NumLinks:    " << modelInfo->links.size() << "\n"
           << "NumJoints:   " << modelInfo->model->getNumJoints() << "\n";
        for (auto& joint :  modelInfo->model->getJoints())
        {
          ss  << "  Joint:     " << joint->getName() << "\n";
        }
      }
    }
    return ss.str();
  }

  /// \brief A debug function to list the links and their names.
  /// \return A string containing the list of link information.
  public: std::string DebugLinks() const
  {
    std::stringstream ss;
    ss << "*** Links ***\n";
    for (size_t id = 0, i = 0; i < links.size(); ++id)
    {
      if (links.HasEntity(id))
      {
        ++i;
        auto linkInfo = links.at(id);
        ss << "LinkID       " << id << "\n"
           << "Name:        " << linkInfo->name << "\n"
           << "NodeName:    " << linkInfo->link->getName() << "\n";
      }
    }
    return ss.str();
  }

  /// \brief A debug function to list the joints and their names.
  /// \return A string containing the list of joint information.
  public: std::string DebugJoints() const
  {
    std::stringstream ss;
    ss << "*** Joints ***\n";
    for (size_t id = 0, i = 0; i < joints.size(); ++id)
    {
      if (joints.HasEntity(id))
      {
        ++i;
        auto jointInfo = joints.at(id);
        ss << "JointID      " << id << "\n"
           << "NodeName:    " << jointInfo->joint->getName() << "\n";
      }
    }
    return ss.str();
  }

};

}
}
}

#endif
