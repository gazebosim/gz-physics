/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include "SDFFeatures.hh"

#include <BulletCollision/CollisionDispatch/btCollisionObject.h>
#include <BulletCollision/CollisionShapes/btCapsuleShape.h>
#include <BulletDynamics/Dynamics/btRigidBody.h>
#include <BulletDynamics/Featherstone/btMultiBodyLinkCollider.h>
#include <LinearMath/btQuaternion.h>
#include <LinearMath/btTransform.h>

#include <ignition/math/Helpers.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/eigen3/Conversions.hh>
#include <memory>
#include <sdf/Box.hh>
#include <sdf/Capsule.hh>
#include <sdf/Cylinder.hh>
#include <sdf/Ellipsoid.hh>
#include <sdf/Geometry.hh>
#include <sdf/JointAxis.hh>
#include <sdf/Plane.hh>
#include <sdf/Sphere.hh>
#include <utility>

#include "Base.hh"

namespace ignition {
namespace physics {
namespace bullet {

/////////////////////////////////////////////////
/// \brief Resolve the pose of an SDF DOM object with respect to its relative_to
/// frame. If that fails, return the raw pose
static math::Pose3d ResolveSdfPose(const ::sdf::SemanticPose& _semPose) {
  math::Pose3d pose;
  ::sdf::Errors errors = _semPose.Resolve(pose);
  if (!errors.empty()) {
    if (!_semPose.RelativeTo().empty()) {
      ignerr << "There was an error in SemanticPose::Resolve\n";
      for (const auto& err : errors) {
        ignerr << err.Message() << std::endl;
      }
      ignerr << "There is no optimal fallback since the relative_to attribute["
             << _semPose.RelativeTo() << "] of the pose is not empty. "
             << "Falling back to using the raw Pose.\n";
    }
    pose = _semPose.RawPose();
  }

  return pose;
}

/////////////////////////////////////////////////
Identity SDFFeatures::ConstructSdfWorld(const Identity& _engine,
                                        const ::sdf::World& _sdfWorld) {
  auto worldIdentity = this->ConstructEmptyWorld(_engine, _sdfWorld.Name());

  auto g = _sdfWorld.Gravity();
  this->world->btWorld->setGravity(btVector3(g[0], g[1], g[2]));

  return worldIdentity;
}

/////////////////////////////////////////////////
Identity SDFFeatures::ConstructSdfModel(const Identity& /*_worldID */,
                                        const ::sdf::Model& _sdfModel) {
  auto name = std::string("world") + "::" + _sdfModel.Name();
  bool isStatic = _sdfModel.Static();
  auto sdfPose = ResolveSdfPose(_sdfModel.SemanticPose());

  auto rootModel = std::make_unique<RootModel>(name, this->world->btWorld.get(),
                                               isStatic, sdfPose);
  auto model = std::make_unique<Model>(name, sdfPose, rootModel.get());

  // Links within a model will collide unless these are chained with a joint
  // const bool selfCollide = _sdfModel.SelfCollide();

  // Add the new model to our Base object, which will
  // manage its lifecycle.
  auto modelIdentity = this->AddModel(std::move(rootModel), std::move(model));

  return modelIdentity;
}

/////////////////////////////////////////////////
Identity SDFFeatures::ConstructSdfNestedModel(const Identity& _parentID,
                                              const ::sdf::Model& _sdfModel) {
  auto parentModel = std::get<Model*>(this->entities.at(_parentID));
  std::string name = parentModel->name + "::" + _sdfModel.Name();
  auto sdfPose = ResolveSdfPose(_sdfModel.SemanticPose());
  auto rootModel = parentModel->rootModel;

  // Create the nested model
  auto model = std::make_unique<Model>(name, sdfPose, rootModel);

  auto modelIdentity = this->AddNestedModel(_parentID, std::move(model));

  return modelIdentity;
}

/////////////////////////////////////////////////
Identity SDFFeatures::ConstructSdfLink(const Identity& _modelID,
                                       const ::sdf::Link& _sdfLink) {
  auto model = std::get<Model*>(this->entities.at(_modelID));
  auto rootModel = model->rootModel;
  auto name = model->name + "::" + _sdfLink.Name();
  auto v = rootModel->skeleton.AddVertex(name, _sdfLink);
  auto link = std::make_unique<Link>(name, rootModel, v.Id());
  return AddLink(_modelID, std::move(link));
}

/////////////////////////////////////////////////
Identity SDFFeatures::ConstructSdfJoint(const Identity& _modelID,
                                        const ::sdf::Joint& _sdfJoint) {
  auto model = std::get<Model*>(this->entities.at(_modelID));
  auto& skeleton = model->rootModel->skeleton;
  auto parentLinkName = model->name + "::" + _sdfJoint.ParentLinkName();
  auto childLinkName = model->name + "::" + _sdfJoint.ChildLinkName();

  // Get parent link vertex id
  auto vp = skeleton.Vertices(parentLinkName);
  if (vp.empty()) {
    ignerr << "Parent link with name: " << _sdfJoint.ParentLinkName()
           << " not found for joint " << _sdfJoint.Name() << " of model "
           << model->name;
  }
  auto parentLinkVertexId = vp.begin()->first;

  // Get child link vertex id
  auto vc = skeleton.Vertices(childLinkName);
  if (vc.empty()) {
    ignerr << "Child link with name: " << _sdfJoint.ChildLinkName()
           << " not found for joint " << _sdfJoint.Name() << " of model "
           << model->name;
  }
  auto childLinkVertexId = vc.begin()->first;

  auto edge = model->rootModel->skeleton.AddEdge(
      {parentLinkVertexId, childLinkVertexId}, _sdfJoint);
  auto name = _sdfJoint.Name();
  auto type = _sdfJoint.Type();
  auto joint = std::make_unique<Joint>(name, type, model->rootModel, edge.Id());
  return AddJoint(_modelID, std::move(joint));
}

/////////////////////////////////////////////////
static math::graph::VertexId FindBaseVertexId(const RootModel* _rootModel) {
  bool found = false;
  math::graph::VertexId base;
  auto vertices = _rootModel->skeleton.Vertices();
  for (auto& v : vertices) {
    auto id = v.first;
    auto edges_to = _rootModel->skeleton.AdjacentsTo(id);
    if (edges_to.empty()) {
      if (!found) {
        base = id;
        found = true;
      } else {
        ignerr << "Multiple base links in model: " << _rootModel->name << "\n";
        return math::graph::kNullId;
      }
    }
  }

  if (!found) {
    ignerr << "Base link not found for model: " << _rootModel->name << "\n";
    return math::graph::kNullId;
  }

  return base;
}

/////////////////////////////////////////////////
static double LinkMass(const ::sdf::Link& _link) {
  return _link.Inertial().MassMatrix().Mass();
}

/////////////////////////////////////////////////
static btVector3 LinkInertia(const ::sdf::Link& _link) {
  auto inertial = _link.Inertial();
  auto inertialPose = inertial.Pose();
  inertialPose.Rot() *= inertial.MassMatrix().PrincipalAxesOffset();
  auto diagonalMoments = inertial.MassMatrix().PrincipalMoments();
  btVector3 linkInertiaDiag =
      convertVec(ignition::math::eigen3::convert(diagonalMoments));
  return linkInertiaDiag;
}

/////////////////////////////////////////////////
static btTransform LinkPose(const RootModel* _rootModel,
                            math::graph::VertexId _vertexId) {
  auto vertices = _rootModel->skeleton.Vertices();
  auto linkSdf = vertices.at(_vertexId).get().Data();
  auto linkSdfPose = ResolveSdfPose(linkSdf.SemanticPose());
  auto inertialPose = linkSdf.Inertial().Pose();
  auto poseIsometry = ignition::math::eigen3::convert(
      _rootModel->sdfPose * linkSdfPose * inertialPose);
  auto poseTranslation = poseIsometry.translation();
  auto poseLinear = poseIsometry.linear();
  btTransform baseTransform;
  baseTransform.setOrigin(convertVec(poseTranslation));
  baseTransform.setBasis(convertMat(poseLinear));
  return baseTransform;
}

/////////////////////////////////////////////////
static void CreateJoint(RootModel* _rootModel,
                        math::graph::VertexId _parentId,
                        math::graph::VertexId _childId) {
  auto& skeleton = _rootModel->skeleton;
  auto vertices = skeleton.Vertices();
  auto edge = skeleton.EdgeFromVertices(_parentId, _childId);
  auto jointSdf = edge.Data();
  auto type = jointSdf.Type();

  auto parentLinkIndex = _rootModel->vertexIdToLinkIndex[_parentId];
  auto childLinkIndex = _rootModel->vertexIdToLinkIndex[_childId];
  // The joint number (btMultibody context) is the same as the child's link
  // number
  _rootModel->edgeIdToJointIndex[edge.Id()] = childLinkIndex;

  // Child link
  auto childSdf = vertices.at(_childId).get().Data();
  auto childMass = LinkMass(childSdf);
  auto childInertia = LinkInertia(childSdf);
  auto childSdfPose = ResolveSdfPose(childSdf.SemanticPose());
  // TODO(joxoby): take into account inertial poses
  // auto childInertial = childSdf.Inertial().Pose();
  auto childPose = _rootModel->sdfPose * childSdfPose;
  // auto childCOMPose = childPose * childInertial;

  // Parent link
  auto parentSdf = vertices.at(_parentId).get().Data();
  auto parentSdfPose = ResolveSdfPose(parentSdf.SemanticPose());
  // auto parentInertial = parentSdf.Inertial().Pose();
  auto parentPose = _rootModel->sdfPose * parentSdfPose;
  // auto parentCOMPose = parentPose * parentInertial;

  // Pivot pose
  auto jointSdfPose = ResolveSdfPose(jointSdf.SemanticPose());
  auto pivotPose = childPose * jointSdfPose;
  auto pivotPoseInParentFrame = parentPose.Inverse() * pivotPose;
  auto parentComToThisPivotOffset =
      convertVec(ignition::math::eigen3::convert(pivotPoseInParentFrame.Pos()));
  auto rotParentToThis = convertQuat(pivotPoseInParentFrame.Rot().Inverse());

  /*
    X_WP,
    p_Pp = X_PW * p_Wp = (X_WP)-1 * p_Wp;
    p_pC = p_pW * X_WC
    p_Wp = X_WC * p_Cp
    p_pa = p_pC * p_Ca
  */

  auto childCOMPoseFromPivot = pivotPose.Inverse() * childPose;
  _rootModel->vertexIdToLinkPoseFromPivot[_childId] =
      pivotPose.Inverse() * childPose;
  auto thisPivotToThisComOffset =
      convertVec(ignition::math::eigen3::convert(childCOMPoseFromPivot.Pos()));

  auto& multibody = std::get<btMultiBody>(_rootModel->body);

  if (type == ::sdf::JointType::REVOLUTE) {
    auto axisVec = childCOMPoseFromPivot.Rot().RotateVector(
        jointSdfPose.Rot().RotateVector(jointSdf.Axis(0)->Xyz()));
    auto axis = convertVec(math::eigen3::convert(axisVec));
    multibody.setupRevolute(
        childLinkIndex, childMass, childInertia, parentLinkIndex,
        rotParentToThis, axis, parentComToThisPivotOffset,
        thisPivotToThisComOffset, true);
  } else if (type == ::sdf::JointType::FIXED) {
    multibody.setupFixed(childLinkIndex, childMass, childInertia,
                                      parentLinkIndex, rotParentToThis,
                                      parentComToThisPivotOffset,
                                      thisPivotToThisComOffset, true);
  } else {
    ignerr << "Joint type not supported\n";
  }
  multibody.finalizeMultiDof();
}

/////////////////////////////////////////////////
static void CreateCollision(RootModel* _rootModel,
                            math::graph::VertexId _vertexId) {
  auto linkIndex = _rootModel->vertexIdToLinkIndex.at(_vertexId);

  auto iter = _rootModel->vertexIdToSdfCollisions.find(_vertexId);
  if (iter != _rootModel->vertexIdToSdfCollisions.end()) {
    auto& sdfCollisions = iter->second;

    for (auto& sdfCollision : sdfCollisions) {
      // TODO(joxoby): handle multiple collitions
      auto* geom = sdfCollision.Geom();

      std::unique_ptr<btCollisionShape> shape;

      if (geom->BoxShape()) {
        // Box
        auto box = geom->BoxShape();
        auto size = math::eigen3::convert(box->Size());
        auto halfExtents = convertVec(size) * 0.5;
        shape = std::make_unique<btBoxShape>(halfExtents);
      } else if (geom->SphereShape()) {
        // Sphere
        auto sphere = geom->SphereShape();
        auto radius = sphere->Radius();
        shape = std::make_unique<btSphereShape>(radius);
      } else if (geom->CylinderShape()) {
        // Cylinder
        auto cylinder = geom->CylinderShape();
        auto radius = cylinder->Radius();
        auto halfLength = cylinder->Length() * 0.5;
        shape = std::make_unique<btCylinderShapeZ>(
            btVector3(radius, radius, halfLength));
      } else if (geom->PlaneShape()) {
        // Plane
        auto plane = geom->PlaneShape();
        auto normal = convertVec(math::eigen3::convert(plane->Normal()));
        shape = std::make_unique<btStaticPlaneShape>(normal, 0);
      } else if (geom->CapsuleShape()) {
        // Capsule
        auto capsule = geom->CapsuleShape();
        auto radius = capsule->Radius();
        auto length = capsule->Length();
        // TODO(joxoby): What's the definition of Length() ?
        shape = std::make_unique<btCapsuleShape>(radius, length);
      } else if (geom->EllipsoidShape()) {
        // Ellipsoid
        auto ellipsoid = geom->EllipsoidShape();
        auto radii = ellipsoid->Radii();
        // TODO(joxoby): Replace with a real ellipsoid
        shape = std::make_unique<btSphereShape>(radii.Max());
      } else {
        // Should not reach here
        // TODO(joxoby): add link name in error msg
        ignerr << "Invalid collision shape for collision name "
               << sdfCollision.Name();
        continue;
      }

      auto shapePtr = shape.get();
      _rootModel->collisionShapes.push_back(std::move(shape));

      if (std::holds_alternative<btMultiBody>(_rootModel->body)) {
        auto& multibody = std::get<btMultiBody>(_rootModel->body);
        // TODO(joxoby): store this a as unique ptr
        auto btCollision =
            new btMultiBodyLinkCollider(&multibody, linkIndex);
        btCollision->setCollisionShape(shapePtr);

        // Collison pose
        // TODO(joxoby): Use the collision pose from the SDF
        auto localPose = _rootModel->vertexIdToLinkPoseFromPivot.at(_vertexId);
        auto localPos =
            convertVec(ignition::math::eigen3::convert(localPose.Pos()));
        auto localRot = convertQuat(localPose.Rot());
        auto pos = multibody.localPosToWorld(linkIndex, localPos);
        auto mat = multibody.localFrameToWorld(
            linkIndex, btMatrix3x3(localRot));
        btTransform tr(mat, pos);
        btCollision->setWorldTransform(tr);

        if (linkIndex == -1) {
          multibody.setBaseCollider(btCollision);
        } else {
          multibody.getLink(linkIndex).m_collider = btCollision;
        }
        // TODO(joxoby): More than one collision shape
        // TODO(joxoby): 2, 1 + 2 ??
        _rootModel->world->addCollisionObject(btCollision, 1, 1 + 2);
      } else {
        // Model with just one link
        auto& body = std::get<btRigidBody>(_rootModel->body);
        body.setCollisionShape(shapePtr);
        // _rootModel->body->forceActivationState(DISABLE_DEACTIVATION);
        // _rootModel->body->setCollisionFlags(btCollisionObject::CF_KINEMATIC_OBJECT | btCollisionObject::CF_STATIC_OBJECT);
      }
    }
  }

  auto iter2 = _rootModel->vertexIdToMeshes.find(_vertexId);

  if (iter2 != _rootModel->vertexIdToMeshes.end()) {
    for (auto& meshWithPose : iter2->second) {

      auto& mesh = meshWithPose.mesh;
      auto& collisionPose = meshWithPose.pose;

      auto gImpactMeshShape = new btGImpactMeshShape(mesh.get());
      gImpactMeshShape->setMargin(0.001); // To avoid zero vector normalization
      gImpactMeshShape->updateBound();

      if (std::holds_alternative<btMultiBody>(_rootModel->body)) {
        auto& multibody = std::get<btMultiBody>(_rootModel->body);
        // TODO(joxoby): store this a unique ptr
        auto btCollision =
            new btMultiBodyLinkCollider(&multibody, linkIndex);
        btCollision->setCollisionShape(gImpactMeshShape);

        // Collison pose
        auto localPose = _rootModel->vertexIdToLinkPoseFromPivot.at(_vertexId);
        auto shapePose = ignition::math::eigen3::convert(localPose) * collisionPose;
        auto localPos = convertVec(shapePose.translation());
        auto localRot = convertMat(shapePose.linear());
        auto pos = multibody.localPosToWorld(linkIndex, localPos);
        auto mat = multibody.localFrameToWorld(linkIndex, btMatrix3x3(localRot));
        btTransform tr(mat, pos);
        btCollision->setWorldTransform(tr);

        if (linkIndex == -1) {
          multibody.setBaseCollider(btCollision);
        } else {
          multibody.getLink(linkIndex).m_collider = btCollision;
        }
        // TODO(joxoby): More than one collision shape
        _rootModel->world->addCollisionObject(btCollision, 1, 1 + 2);
      }
      else {
        auto& body = std::get<btRigidBody>(_rootModel->body);
        body.setCollisionShape(gImpactMeshShape);
      }
    }
  }
}

/////////////////////////////////////////////////
void SDFFeatures::ConstructSdfMultibody(const Identity& _modelID) {
  auto model = std::get<Model*>(this->entities.at(_modelID));
  auto rootModel = model->rootModel;

  // Return if the root model was already constructed for this model
  if (!std::holds_alternative<std::monostate>(rootModel->body)) {
    return;
  }

  // Vertices are links
  auto vertices = rootModel->skeleton.Vertices();
  auto nLinks = vertices.size();

  // Find the base link
  auto baseId = FindBaseVertexId(rootModel);
  if (baseId == math::graph::kNullId)
    return;

  auto baseLinkSdf = vertices.at(baseId).get().Data();
  auto baseMass = LinkMass(baseLinkSdf);
  auto baseInertia = LinkInertia(baseLinkSdf);
  auto baseLinkPose = LinkPose(rootModel, baseId);
  bool canSleep = true;
  bool isStatic = rootModel->isStatic;
  rootModel->vertexIdToLinkPoseFromPivot[baseId] = math::Pose3d();
  rootModel->vertexIdToLinkIndex[baseId] = -1;

  if (nLinks > 1) {
    // Enumerate all links. This is needed since btMultiBody references
    // each link with a number from -1 (base) to n-2.
    int count = 0;
    for (auto& elem : vertices) {
      auto vertexId = elem.first;
      if (vertexId == baseId)
        continue;
      rootModel->vertexIdToLinkIndex[vertexId] = count++;
    }

    // Construct btMultiBody using base link
    auto& multibody = rootModel->body.emplace<btMultiBody>(
        nLinks - 1, baseMass, baseInertia, isStatic, canSleep);

    // Set the pose
    multibody.setBaseWorldTransform(baseLinkPose);

    // Setup all joints and child links
    auto edges = rootModel->skeleton.Edges();
    for (auto& elem : edges) {
      auto [id, edge] = elem;
      auto [parentId, childId] = edge.get().Vertices();
      CreateJoint(rootModel, parentId, childId);
    }

    // Setup all collisions
    for (auto& elem : vertices) {
      auto [vertexId, _] = elem;
      CreateCollision(rootModel, vertexId);
    }

    multibody.finalizeMultiDof();
    rootModel->world->addMultiBody(&multibody);
  } else {
    auto motionState = new btDefaultMotionState(baseLinkPose);
    auto mass = isStatic ? .0 : baseMass;
    btRigidBody::btRigidBodyConstructionInfo rbInfo(
        mass, motionState, nullptr, baseInertia);
    auto& body = rootModel->body.emplace<btRigidBody>(rbInfo);
    CreateCollision(rootModel, baseId);
    rootModel->world->addRigidBody(&body);
  }
}

/////////////////////////////////////////////////
Identity SDFFeatures::ConstructSdfCollision(
    const Identity& _linkID,
    const ::sdf::Collision& _collisionSdf) {
  // Check if valid collision
  auto* geom = _collisionSdf.Geom();
  if (geom == nullptr) {
    ignerr << "Null geometry element for collision [" << _collisionSdf.Name()
           << "]\n";
    return this->GenerateInvalidId();
  }
  auto link = std::get<Link*>(this->entities.at(_linkID));
  auto rootModel = link->rootModel;
  rootModel->vertexIdToSdfCollisions[link->vertexId].push_back(_collisionSdf);
  auto collision = std::make_unique<Collision>(_collisionSdf.Name(), rootModel,
                                               link->vertexId);
  auto identity = AddCollision(_linkID, std::move(collision));
  return identity;
}
}  // namespace bullet
}  // namespace physics
}  // namespace ignition
