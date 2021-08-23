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
#include "Base.hh"

#include <utility>

namespace ignition {
namespace physics {
namespace bullet {

/////////////////////////////////////////////////
World::World(std::string _name) : name(std::move(_name)) {
  collisionConfiguration = std::make_unique<btDefaultCollisionConfiguration>();
  dispatcher =
      std::make_unique<btCollisionDispatcher>(collisionConfiguration.get());
  broadphase = std::make_unique<btDbvtBroadphase>();
  solver = std::make_unique<btSequentialImpulseConstraintSolver>();
  btWorld = std::make_unique<btDiscreteDynamicsWorld>(
      dispatcher.get(), broadphase.get(), solver.get(),
      collisionConfiguration.get());
  // TO-DO(Lobotuerk): figure out what this line does
  btWorld->getSolverInfo().m_globalCfm = 0;
  btGImpactCollisionAlgorithm::registerAlgorithm(dispatcher.get());
}

/////////////////////////////////////////////////
Model::Model(std::string _name,
             const math::Pose3d& _sdfPose,
             const math::Pose3d& _worldPose,
             bool _static,
             btDiscreteDynamicsWorld* _btWorld)
    : name(std::move(_name)),
      sdfPose(_sdfPose),
      worldPose(_worldPose),
      fixed(_static),
      world(_btWorld) {}

/////////////////////////////////////////////////
Link::Link(std::string _name,
           const math::Pose3d& _sdfPose,
           const math::Pose3d& _sdfInertialPose,
           btDiscreteDynamicsWorld* _btWorld,
           const btTransform& _baseTransform,
           double _mass,
           const btVector3& _linkInertiaDiag)
    : name(std::move(_name)),
      sdfPose(_sdfPose),
      sdfInertialPose(_sdfInertialPose),
      world(_btWorld) {
  motionState = std::make_unique<btDefaultMotionState>(_baseTransform);
  collisionShape = std::make_unique<btCompoundShape>();
  btRigidBody::btRigidBodyConstructionInfo rbInfo(
      _mass, motionState.get(), collisionShape.get(), _linkInertiaDiag);
  body = std::make_unique<btRigidBody>(rbInfo);
  body->setActivationState(DISABLE_DEACTIVATION);
  world->addRigidBody(body.get());
}

/////////////////////////////////////////////////
Link::~Link() {
  world->removeRigidBody(body.get());
}

/////////////////////////////////////////////////
Collision::Collision(std::string _name,
                     btRigidBody* _body,
                     bool _isMesh,
                     std::unique_ptr<btCollisionShape>&& _shape,
                     std::unique_ptr<btTriangleMesh>&& _mesh,
                     const btTransform& _localPose,
                     double _mu,
                     double _mu2)
    : name(std::move(_name)),
      body(_body),
      localPose(_localPose),
      isMesh(_isMesh),
      shape(std::move(_shape)),
      mesh(std::move(_mesh)) {
  // shape->setMargin(btScalar(0.0001));
  // body->setRollingFriction(0.25);
  body->setFriction(1);
  body->setAnisotropicFriction(btVector3(_mu, _mu2, 1),
                               btCollisionObject::CF_ANISOTROPIC_FRICTION);

  btCollisionShape* collisionShapePtr;

  if (isMesh) {
    gImpactMeshShape = std::make_unique<btGImpactMeshShape>(mesh.get());
    gImpactMeshShape->updateBound();
    collisionShapePtr = gImpactMeshShape.get();
  } else {
    collisionShapePtr = shape.get();
  }

  dynamic_cast<btCompoundShape*>(body->getCollisionShape())
      ->addChildShape(_localPose, collisionShapePtr);
}

/////////////////////////////////////////////////
Collision::~Collision() { /* TODO(juan): destructor? */
}

/////////////////////////////////////////////////
Joint::Joint(std::string _name,
             btDiscreteDynamicsWorld* _world,
             btRigidBody* _childBody,
             btRigidBody* _parentBody,
             const btVector3& _pivotChild,
             const btVector3& _pivotParent,
             const btVector3& _axisChild,
             const btVector3& _axisParent,
             ::sdf::JointType _type)
    : name(std::move(_name)),
      world(_world),
      childBody(_childBody),
      parentBody(_parentBody),
      type(_type) {
  if (_parentBody != nullptr) {
    constraint = std::make_unique<btHingeAccumulatedAngleConstraint>(
        *_childBody, *_parentBody, _pivotChild, _pivotParent, _axisChild,
        _axisParent);
  } else {
    constraint = std::make_unique<btHingeAccumulatedAngleConstraint>(
        *_childBody, _pivotChild, _axisChild);
  }

  auto* hinge =
        static_cast<btHingeAccumulatedAngleConstraint*>(constraint.get());

  if (type == ::sdf::JointType::FIXED) {

    auto offset = hinge->getHingeAngle();
    hinge->setLimit(offset, offset);
  }

  if (type == ::sdf::JointType::REVOLUTE) {
    // TODO(juan): Set the limits correctly
    // hingeConstraint->setLimit(-10, 10);
  }

  constraint->enableFeedback(true);
  hinge->setLimit(0.2, 0.2);
  world->addConstraint(constraint.get(), true);
}

/////////////////////////////////////////////////
void Joint::SetPosition(double _value) {
  auto hinge = dynamic_cast<btHingeAccumulatedAngleConstraint*>(constraint.get());
  hinge->setAccumulatedHingeAngle(_value);
}

// /////////////////////////////////////////////////
double Joint::GetPosition() {
  auto hinge = dynamic_cast<btHingeAccumulatedAngleConstraint*>(constraint.get());
  auto angle = hinge->getAccumulatedHingeAngle();
  return angle;
}

/////////////////////////////////////////////////
Joint::~Joint() {
  world->removeConstraint(constraint.get());
}

}  // namespace bullet
}  // namespace physics
}  // namespace ignition
