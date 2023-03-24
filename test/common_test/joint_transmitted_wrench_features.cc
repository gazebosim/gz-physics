/*
 * Copyright (C) 2023 Open Source Robotics Foundation
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
#include <gtest/gtest.h>

#include <chrono>

#include <gz/common/Console.hh>
#include <gz/plugin/Loader.hh>

#include <gz/math/Helpers.hh>
#include <gz/math/eigen3/Conversions.hh>

#include "TestLibLoader.hh"
#include "../Utils.hh"

#include "gz/physics/FrameSemantics.hh"
#include <gz/physics/FindFeatures.hh>
#include <gz/physics/FixedJoint.hh>
#include <gz/physics/ForwardStep.hh>
#include <gz/physics/FreeGroup.hh>
#include <gz/physics/FreeJoint.hh>
#include <gz/physics/GetEntities.hh>
#include <gz/physics/Joint.hh>
#include <gz/physics/RemoveEntities.hh>
#include <gz/physics/RequestEngine.hh>
#include <gz/physics/RevoluteJoint.hh>
#include <gz/physics/Shape.hh>
#include <gz/physics/sdf/ConstructModel.hh>
#include <gz/physics/sdf/ConstructWorld.hh>

#include <sdf/Root.hh>

template <class T>
class JointTransmittedWrenchFeaturesTest:
  public testing::Test, public gz::physics::TestLibLoader
{
  // Documentation inherited
  public: void SetUp() override
  {
    gz::common::Console::SetVerbosity(4);

    std::cerr << "JointTransmittedWrenchFeaturesTest::GetLibToTest() "
      << JointTransmittedWrenchFeaturesTest::GetLibToTest() << '\n';

    loader.LoadLib(JointTransmittedWrenchFeaturesTest::GetLibToTest());

    // TODO(ahcorde): We should also run the 3f, 2d, and 2f variants of
    // FindFeatures
    pluginNames = gz::physics::FindFeatures3d<T>::From(loader);
    if (pluginNames.empty())
    {
      std::cerr << "No plugins with required features found in "
                << GetLibToTest() << std::endl;
      GTEST_SKIP();
    }
    for (const std::string &name : this->pluginNames)
    {
      if(this->PhysicsEngineName(name) == "tpe")
      {
        GTEST_SKIP();
      }
    }
  }

  public: std::set<std::string> pluginNames;
  public: gz::plugin::Loader loader;
};

template <class T>
class JointTransmittedWrenchFixture :
  public JointTransmittedWrenchFeaturesTest<T>
{
  public: using WorldPtr = gz::physics::World3dPtr<T>;
  public: using ModelPtr = gz::physics::Model3dPtr<T>;
  public: using JointPtr = gz::physics::Joint3dPtr<T>;
  public: using LinkPtr = gz::physics::Link3dPtr<T>;

  protected: void SetUp() override
  {
    JointTransmittedWrenchFeaturesTest<T>::SetUp();
    for (const std::string &name : this->pluginNames)
    {
      std::cout << "Testing plugin: " << name << std::endl;
      gz::plugin::PluginPtr plugin = this->loader.Instantiate(name);

      auto engine = gz::physics::RequestEngine3d<T>::From(plugin);
      ASSERT_NE(nullptr, engine);

      sdf::Root root;
      const sdf::Errors errors = root.Load(gz::common::joinPaths(TEST_WORLD_DIR, "pendulum_joint_wrench.sdf"));
      ASSERT_TRUE(errors.empty()) << errors.front();

      this->world = engine->ConstructWorld(*root.WorldByIndex(0));
      ASSERT_NE(nullptr, this->world);

      this->model = this->world->GetModel("pendulum");
      ASSERT_NE(nullptr, this->model);
      this->motorJoint = this->model->GetJoint("motor_joint");
      ASSERT_NE(nullptr, this->motorJoint);
      this->sensorJoint = this->model->GetJoint("sensor_joint");
      ASSERT_NE(nullptr, this->sensorJoint);
      this->armLink = this->model->GetLink("arm");
      ASSERT_NE(nullptr, this->armLink);
    }
  }

  public: void Step(int _iters)
  {
    for (int i = 0; i < _iters; ++i)
    {
      this->world->Step(this->output, this->state, this->input);
    }
  }

  public: gz::physics::ForwardStep::Output output;
  public: gz::physics::ForwardStep::State state;
  public: gz::physics::ForwardStep::Input input;
  public: WorldPtr world;
  public: ModelPtr model;
  public: JointPtr motorJoint;
  public: JointPtr sensorJoint;
  public: LinkPtr armLink;

  // From SDFormat file
  static constexpr double kGravity = 9.8;
  static constexpr double kArmLinkMass = 6.0;
  static constexpr double kSensorLinkMass = 0.4;
  // MOI in the z-axis
  static constexpr double kSensorLinkMOI = 0.02;
  static constexpr double kArmLength = 1.0;
};

struct JointTransmittedWrenchFeatureList : gz::physics::FeatureList<
    gz::physics::ForwardStep,
    gz::physics::FreeGroupFrameSemantics,
    gz::physics::GetBasicJointState,
    gz::physics::GetEngineInfo,
    gz::physics::GetJointFromModel,
    gz::physics::GetJointTransmittedWrench,
    gz::physics::GetLinkFromModel,
    gz::physics::GetModelFromWorld,
    gz::physics::LinkFrameSemantics,
    gz::physics::SetBasicJointState,
    gz::physics::SetFreeGroupWorldPose,
    gz::physics::sdf::ConstructSdfWorld
> { };

using JointTransmittedWrenchFeaturesTestTypes =
  ::testing::Types<JointTransmittedWrenchFeatureList>;

TYPED_TEST_SUITE(JointTransmittedWrenchFixture,
                 JointTransmittedWrenchFeaturesTestTypes,);


void printWrench(const gz::physics::Wrench3d &wrench)
{
    std::cout << "force: ["
      << wrench.force.x() << " "
      << wrench.force.y() << " "
      << wrench.force.z() <<
      "] torque: ["
      << wrench.torque.x() << " "
      << wrench.torque.y() << " "
      << wrench.torque.z() << "]\n";
}

TYPED_TEST(JointTransmittedWrenchFixture, PendulumAtZeroAngle)
{
  // Run a few steps for the constraint forces to stabilize
  this->Step(100);

  // Test wrench expressed in different frames
  {
    auto wrenchAtMotorJoint = this->motorJoint->GetTransmittedWrench();
    gz::physics::Wrench3d expectedWrenchAtMotorJoint{
        gz::physics::Vector3d::Zero(),
        {-this->kGravity * (this->kArmLinkMass + this->kSensorLinkMass), 0, 0}};

    EXPECT_TRUE(
       gz::physics::test::Equal(expectedWrenchAtMotorJoint, wrenchAtMotorJoint, 1e-4));
  }
  {
    auto wrenchAtMotorJointInWorld = this->motorJoint->GetTransmittedWrench(
        this->motorJoint->GetFrameID(), gz::physics::FrameID::World());
    gz::physics::Wrench3d expectedWrenchAtMotorJointInWorld{
        gz::physics::Vector3d::Zero(),
        {0, 0, this->kGravity * (this->kArmLinkMass + this->kSensorLinkMass)}};

    EXPECT_TRUE(gz::physics::test::Equal(expectedWrenchAtMotorJointInWorld,
                                         wrenchAtMotorJointInWorld, 1e-4));
  }
  {
    auto wrenchAtMotorJointInArm = this->motorJoint->GetTransmittedWrench(
        this->armLink->GetFrameID(), this->armLink->GetFrameID());
    // The arm frame is rotated by 90° in the Y-axis of the joint frame.
    gz::physics::Wrench3d expectedWrenchAtMotorJointInArm{
        gz::physics::Vector3d::Zero(),
        {0, 0, this->kGravity * (this->kArmLinkMass + this->kSensorLinkMass)}};

    printWrench(expectedWrenchAtMotorJointInArm);
    printWrench(wrenchAtMotorJointInArm);

    EXPECT_TRUE(gz::physics::test::Equal(expectedWrenchAtMotorJointInArm,
                                         wrenchAtMotorJointInArm, 1e-4));
  }
}

TYPED_TEST(JointTransmittedWrenchFixture, PendulumInMotion)
{
  // Start pendulum at 90° (parallel to the ground) and stop at about 40°
  // so that we have non-trivial test expectations.
  this->motorJoint->SetPosition(0, GZ_DTOR(90.0));
  this->Step(350);

  // Given the position (θ), velocity (ω), and acceleration (α) of the joint
  // and distance from the joint to the COM (r), the reaction forces in
  // the tangent direction (Ft) and normal direction (Fn) are given by:
  //
  // Ft =  m * α * r + (m * g * sin(θ)) = m * (α * r + g * sin(θ))
  // Fn = -m * ω² * r - (m * g * cos(θ)) = -m * (ω² * r +  g * cos(θ))
  {
    const double theta = this->motorJoint->GetPosition(0);
    const double omega = this->motorJoint->GetVelocity(0);

    // In order to get the math to work out, we need to use the joint
    // acceleration and transmitted wrench from the current time step with the
    // joint position and velocity from the previous time step. That is, we need
    // the position and velocity before they are integrated.
    this->Step(1);

    const double omega1 = this->motorJoint->GetVelocity(0);
    const double alpha = (omega1 - omega)/1e-3;

    auto wrenchAtMotorJointInJoint = this->motorJoint->GetTransmittedWrench();

    const double armTangentForce =
        this->kArmLinkMass * ((alpha * this->kArmLength / 2.0) + (this->kGravity * sin(theta)));

    const double motorLinkTangentForce =
        this->kSensorLinkMass * this->kGravity * sin(theta);

    const double armNormalForce =
        -this->kArmLinkMass *
        ((std::pow(omega, 2) * this->kArmLength / 2.0) + (this->kGravity * cos(theta)));

    const double motorLinkNormalForce =
        -this->kSensorLinkMass * this->kGravity * cos(theta);

    const double tangentForce = armTangentForce + motorLinkTangentForce;
    const double normalForce = armNormalForce + motorLinkNormalForce;

    // The orientation of the joint frame is such that the normal force is
    // parallel to the x-axis and the tangent force is parallel to the y-axis.
    gz::physics::Wrench3d expectedWrenchAtMotorJointInJoint{
        gz::physics::Vector3d::Zero(), {normalForce, tangentForce, 0}};

    EXPECT_TRUE(gz::physics::test::Equal(expectedWrenchAtMotorJointInJoint,
                                         wrenchAtMotorJointInJoint, 1e-4));
  }

  // Test Wrench expressed in different frames
  {
    auto wrenchAtMotorJointInJoint = this->motorJoint->GetTransmittedWrench();
    // This is just a rotation of the wrench to be expressed in the world's
    // coordinate frame
    auto wrenchAtMotorJointInWorld = this->motorJoint->GetTransmittedWrench(
        this->motorJoint->GetFrameID(), gz::physics::FrameID::World());
    // The joint frame is rotated by 90° along the world's y-axis
    Eigen::Quaterniond R_WJ =
        Eigen::AngleAxisd(GZ_PI_2, Eigen::Vector3d(0, 1, 0)) *
        Eigen::AngleAxisd(this->motorJoint->GetPosition(0),
                          Eigen::Vector3d(0, 0, 1));

    gz::physics::Wrench3d expectedWrenchAtMotorJointInWorld{
        gz::physics::Vector3d::Zero(), R_WJ * wrenchAtMotorJointInJoint.force};
    EXPECT_TRUE(gz::physics::test::Equal(expectedWrenchAtMotorJointInWorld,
                            wrenchAtMotorJointInWorld, 1e-4));

    // This moves the point of application and changes the coordinate frame
    gz::physics::Wrench3d wrenchAtArmInArm = this->motorJoint->GetTransmittedWrench(
        this->armLink->GetFrameID(), this->armLink->GetFrameID());

    // Notation: arm link (A), joint (J)
    Eigen::Isometry3d X_AJ;
    // Pose of joint (J) in arm link (A) as specified in the SDFormat file.
    X_AJ = Eigen::AngleAxisd(GZ_PI_2, Eigen::Vector3d(0, 1, 0));
    X_AJ.translation() = gz::physics::Vector3d(0, 0, this->kArmLength / 2.0);
    gz::physics::Wrench3d expectedWrenchAtArmInArm;

    expectedWrenchAtArmInArm.force =
        X_AJ.linear() * wrenchAtMotorJointInJoint.force;

    expectedWrenchAtArmInArm.torque =
        X_AJ.linear() * wrenchAtMotorJointInJoint.torque +
        X_AJ.translation().cross(expectedWrenchAtArmInArm.force);

    EXPECT_TRUE(gz::physics::test::Equal(expectedWrenchAtArmInArm, wrenchAtArmInArm, 1e-4));
  }
}

TYPED_TEST(JointTransmittedWrenchFixture, ValidateWrenchWithSecondaryJoint)
{
  // Start pendulum at 90° (parallel to the ground) and stop at about 40°
  // so that we have non-trivial test expectations.
  this->motorJoint->SetPosition(0, GZ_DTOR(90.0));
  this->Step(350);
  const double theta = this->motorJoint->GetPosition(0);
  const double omega = this->motorJoint->GetVelocity(0);
  // In order to get the math to work out, we need to use the joint
  // acceleration and transmitted wrench from the current time step with the
  // joint position and velocity from the previous time step. That is, we need
  // the position and velocity before they are integrated.
  this->Step(1);
  const double omega1 = this->motorJoint->GetVelocity(0);
  const double alpha = (omega1 - omega)/1e-3;

  auto wrenchAtMotorJointInJoint = this->motorJoint->GetTransmittedWrench();
  auto wrenchAtSensorInSensor = this->sensorJoint->GetTransmittedWrench();

  // Since sensor_link has moment of inertia, the fixed joint will transmit a
  // torque necessary to rotate the sensor. This is not detected by the motor
  // joint because no force is transmitted along the revolute axis. On the
  // other hand, the mass of sensor_link will contribute to the constraint
  // forces on the motor joint, but these won't be detected by the sensor
  // joint.
  gz::physics::Vector3d expectedTorqueDiff{0, 0, this->kSensorLinkMOI * alpha};
  gz::physics::Vector3d expectedForceDiff{-this->kSensorLinkMass * this->kGravity * cos(theta),
                             this->kSensorLinkMass * this->kGravity * sin(theta), 0};

  gz::physics::Vector3d torqueDiff =
      wrenchAtMotorJointInJoint.torque - wrenchAtSensorInSensor.torque;
  gz::physics::Vector3d forceDiff =
      wrenchAtMotorJointInJoint.force - wrenchAtSensorInSensor.force;
  EXPECT_TRUE(gz::physics::test::Equal(expectedTorqueDiff, torqueDiff, 1e-4));
  EXPECT_TRUE(gz::physics::test::Equal(expectedForceDiff, forceDiff, 1e-4));
}

TYPED_TEST(JointTransmittedWrenchFixture, JointLosses)
{
  // // Get DART joint pointer to set joint friction, damping, etc.
  // auto dartWorld = this->world->GetDartsimWorld();
  // ASSERT_NE(nullptr, dartWorld);
  // auto dartModel = dartWorld->getSkeleton(this->model->GetIndex());
  // ASSERT_NE(nullptr, dartModel);
  // auto dartJoint = dartModel->getJoint(this->motorJoint->GetIndex());
  // ASSERT_NE(nullptr, dartJoint);
  //
  // // Joint friction
  // {
  //   this->motorJoint->SetPosition(0, GZ_DTOR(90.0));
  //   this->motorJoint->SetVelocity(0, 0);
  //   const double kFrictionCoef = 0.5;
  //   dartJoint->setCoulombFriction(0, kFrictionCoef);
  //   this->Step(10);
  //   auto wrenchAtMotorJointInJoint = this->motorJoint->GetTransmittedWrench();
  //   EXPECT_NEAR(kFrictionCoef, wrenchAtMotorJointInJoint.torque.z(), 1e-4);
  //   dartJoint->setCoulombFriction(0, 0.0);
  // }
  //
  // // Joint damping
  // {
  //   this->motorJoint->SetPosition(0, GZ_DTOR(90.0));
  //   this->motorJoint->SetVelocity(0, 0);
  //   const double kDampingCoef = 0.2;
  //   dartJoint->setDampingCoefficient(0, kDampingCoef);
  //   this->Step(100);
  //   const double omega = this->motorJoint->GetVelocity(0);
  //   this->Step(1);
  //   auto wrenchAtMotorJointInJoint = this->motorJoint->GetTransmittedWrench();
  //   EXPECT_NEAR(-omega * kDampingCoef, wrenchAtMotorJointInJoint.torque.z(),
  //               1e-3);
  //   dartJoint->setDampingCoefficient(0, 0.0);
  // }
  //
  // // Joint stiffness
  // {
  //   // Note: By default, the spring reference position is 0.
  //   this->motorJoint->SetPosition(0, GZ_DTOR(30.0));
  //   this->motorJoint->SetVelocity(0, 0);
  //   const double kSpringStiffness = 0.7;
  //   dartJoint->setSpringStiffness(0, kSpringStiffness);
  //   this->Step(1);
  //   const double theta = this->motorJoint->GetPosition(0);
  //   this->Step(1);
  //   auto wrenchAtMotorJointInJoint = this->motorJoint->GetTransmittedWrench();
  //   EXPECT_NEAR(-theta * kSpringStiffness, wrenchAtMotorJointInJoint.torque.z(),
  //               1e-3);
  //   dartJoint->setSpringStiffness(0, 0.0);
  // }
}

// Check that the transmitted wrench is affected by contact forces
TYPED_TEST(JointTransmittedWrenchFixture, ContactForces)
{
  auto box = this->world->GetModel("box");
  ASSERT_NE(nullptr, box);
  auto boxFreeGroup = box->FindFreeGroup();
  ASSERT_NE(nullptr, boxFreeGroup);
  gz::physics::Pose3d X_WB(Eigen::Translation3d(0, 1, 1));
  boxFreeGroup->SetWorldPose(X_WB);

  this->motorJoint->SetPosition(0, GZ_DTOR(90.0));
  // After this many steps, the pendulum is in contact with the box
  this->Step(1000);
  const double theta = this->motorJoint->GetPosition(0);
  // Sanity check that the pendulum is at rest
  EXPECT_NEAR(0.0, this->motorJoint->GetVelocity(0), 1e-3);

  auto wrenchAtMotorJointInJoint = this->motorJoint->GetTransmittedWrench();

  // To compute the reaction forces, we consider the pivot on the contact point
  // between the pendulum and the box and the fact that the sum of moments about
  // the pivot is zero. We also note that all forces, including the reaction
  // forces, are in the vertical (world's z-axis) direction.
  //
  // Notation:
  // Fp_z: Reaction force at pendulum joint (pin) in the world's z-axis
  // M_b: Moment about the contact point between box and pendulum
  //
  // Fp_z = √(Fn² + Ft²) // Since all of the reaction force is in the world's
  // z-axis
  //
  // ∑M_b = 0 = -Fp_z * sin(θ) * (2*r) + m₁*g*sin(θ)*r + m₂*g*sin(θ)*(2*r)
  //
  // Fp_z = 0.5 * g * (m₁ + 2*m₂)
  //
  // We can then compute the tangential (Ft) and normal (Fn) components as
  //
  // Ft =  Fp_z * sin(θ)
  // Fn = -Fp_z * cos(θ)

  const double reactionForceAtP =
      0.5 * this->kGravity * (this->kArmLinkMass + 2 * this->kSensorLinkMass);

  gz::physics::Wrench3d expectedWrenchAtMotorJointInJoint{
      gz::physics::Vector3d::Zero(),
      {-reactionForceAtP * cos(theta), reactionForceAtP * sin(theta), 0}};

  EXPECT_TRUE(gz::physics::test::Equal(expectedWrenchAtMotorJointInJoint,
                                       wrenchAtMotorJointInJoint, 1e-4));
}

int main(int argc, char *argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  if (!JointTransmittedWrenchFeaturesTest<
      JointTransmittedWrenchFeatureList>::init(argc, argv))
    return -1;
  return RUN_ALL_TESTS();
}
