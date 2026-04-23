/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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
#include <cmath>
#include <cstddef>
#include <string>

#include <gz/common/Console.hh>
#include <gz/math/Helpers.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/eigen3/Conversions.hh>
#include <gz/physics/FindFeatures.hh>
#include <gz/physics/FixedJoint.hh>
#include <gz/physics/ForwardStep.hh>
#include <gz/physics/GetEntities.hh>
#include <gz/physics/Joint.hh>
#include <gz/physics/RequestEngine.hh>
#include <gz/physics/RevoluteJoint.hh>
#include <gz/physics/World.hh>
#include <gz/physics/sdf/ConstructModel.hh>
#include <gz/physics/sdf/ConstructWorld.hh>
#include <gz/plugin/Loader.hh>
#include <sdf/Link.hh>
#include <sdf/Model.hh>
#include <sdf/Root.hh>
#include <sdf/World.hh>

#include "Worlds.hh"
#include "test/TestLibLoader.hh"
#include "test/Utils.hh"

using namespace gz;

template <class T>
class JointFeaturesTest:
  public testing::Test, public physics::TestLibLoader
{
  public: using EnginePtr = physics::Engine3dPtr<T>;
  public: using WorldPtr = physics::World3dPtr<T>;
  public: using ModelPtr = physics::Model3dPtr<T>;
  public: using LinkPtr = physics::Link3dPtr<T>;
  public: using JointPtr = physics::Joint3dPtr<T>;

  // Documentation inherited
  public: void SetUp() override
  {
    common::Console::SetVerbosity(4);

    std::cerr << "TestLibLoader::GetLibToTest() "
              << TestLibLoader::GetLibToTest() << '\n';

    loader.LoadLib(TestLibLoader::GetLibToTest());
    pluginNames = physics::FindFeatures3d<T>::From(loader);
    if (pluginNames.empty())
    {
      std::cerr << "No plugins with required features found in "
                << GetLibToTest() << std::endl;
      GTEST_SKIP();
    }
  }

  public: std::set<std::string> pluginNames;
  public: plugin::Loader loader;
};


struct BasicJointFeatureList : physics::FeatureList<
    physics::ForwardStep,
    physics::GetBasicJointProperties,
    physics::GetBasicJointState,
    physics::GetJointFromModel,
    physics::GetModelFromWorld,
    physics::SetBasicJointState,
    physics::sdf::ConstructSdfWorld
> { };

class BasicJointFeaturesTest : public JointFeaturesTest<BasicJointFeatureList>
{
  public: void SetUp() override
  {
    JointFeaturesTest<BasicJointFeatureList>::SetUp();
    const sdf::Errors errors =
        this->root.Load(common_test::worlds::kSinglePendulumWithBase);
    ASSERT_TRUE(errors.empty()) << errors.front();

    this->sdfWorld = this->root.WorldByIndex(0);
    ASSERT_NE(nullptr, this->sdfWorld);
    this->sdfPendulumModel = this->sdfWorld->ModelByName(this->modelName);
    ASSERT_NE(nullptr, this->sdfPendulumModel);
    this->armLink = this->sdfPendulumModel->LinkByName("arm");
    ASSERT_NE(nullptr, this->armLink);

    auto dur = std::chrono::duration<double>(dt);

    this->input.Get<std::chrono::steady_clock::duration>() =
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(dur);
  }

  void InitPluginAndWorld(const std::string &_pluginName)
  {
    this->plugin = this->loader.Instantiate(_pluginName);

    this->engine =
        physics::RequestEngine3d<BasicJointFeatureList>::From(this->plugin);
    ASSERT_NE(nullptr, engine);

    this->world = this->engine->ConstructWorld(*this->sdfWorld);
    ASSERT_NE(nullptr, this->world);
    this->model = this->world->GetModel(this->modelName);
    ASSERT_NE(nullptr, model);
    this->joint = this->model->GetJoint(this->jointName);
    ASSERT_NE(nullptr, this->joint);
  }

  void Step()
  {
    this->world->Step(this->output, this->state, this->input);
  }

  const std::string modelName{"pendulum_with_base"};
  const std::string jointName{"upper_joint"};
  // Hardcode these to test proper SDFormat parsing
  const double kGravity = 9.81;
  const double kArmLength = 1.0;
  const double kRadius = 0.1;
  const double kMass = 6.0;
  const double moiCom =
      1.0 / 12 * kMass * (3 * std::pow(kRadius, 2) + std::pow(kArmLength, 2));
  // Parallel axis theoerem
  const double moiPivot = moiCom + kMass * std::pow(kArmLength / 2.0, 2);

  const double dt = 0.001;
  physics::ForwardStep::Output output;
  physics::ForwardStep::State state;
  physics::ForwardStep::Input input;

  sdf::Root root;
  sdf::World *sdfWorld;
  sdf::Model *sdfPendulumModel;
  sdf::Link *armLink;
  plugin::PluginPtr plugin;
  EnginePtr engine;
  WorldPtr world;
  ModelPtr model;
  JointPtr joint;
};

TEST_F(BasicJointFeaturesTest, GetSetBasicState)
{
  const double kTol = 1e-6;
  for (const std::string &name : this->pluginNames)
  {
    // Some of the basic expectations, such as the initial joint position, are
    // different in Bullet. Since the implementation is likely to be deprecated,
    // we will skip it in this test rather than trying to fix the underlying
    // issue.
    CHECK_UNSUPPORTED_ENGINE(name, "bullet")

    this->InitPluginAndWorld(name);

    EXPECT_NEAR(0.0, this->joint->GetPosition(0), kTol);
    EXPECT_NEAR(0.0, this->joint->GetVelocity(0), kTol);

    const double startPos = 0.01;
    this->joint->SetPosition(0, startPos);
    EXPECT_NEAR(startPos, this->joint->GetPosition(0), kTol);

    // Expect negative joint velocity after 1 step without joint command
    // 1 ms time step
    this->Step();
    // After step, the position should still be very close to the start
    // position.
    EXPECT_NEAR(startPos, this->joint->GetPosition(0), 1e-3);

    // Reset the initial conditions.
    this->joint->SetPosition(0, startPos);
    this->joint->SetVelocity(0, 0);
    EXPECT_NEAR(startPos, this->joint->GetPosition(0), kTol);
    EXPECT_NEAR(0, this->joint->GetVelocity(0), kTol);
    const int numSteps = 100;
    for (int i = 0; i < numSteps; ++i)
    {
      this->Step();
    }
    const double timeElapsed = numSteps * this->dt;
    EXPECT_LT(this->joint->GetVelocity(0), 0.0);
    EXPECT_LT(this->joint->GetPosition(0), startPos);

    // Using small angle approximation, the joint position is given by:
    // θ = A * cos(ω₀ * t) + B * sin (ω₀ * t)
    // where A = θ₀,
    // B = 0 since starting angular velocity is 0
    // ω₀  = √(m * g *l/I)
    // θ = θ₀ * cos(√(m * g *l/I) * t)
    // See
    // https://ocw.mit.edu/courses/8-01sc-classical-mechanics-fall-2016/mit8_01scs22_chapter24.pdf
    const double angFreq = std::sqrt(this->kMass * this->kGravity *
                                     (this->kArmLength / 2) / this->moiPivot);
    const double expectedPosition = startPos * std::cos(angFreq * timeElapsed);
    const double expectedAngVel =
        -angFreq * startPos * std::sin(angFreq * timeElapsed);
    // Relax tolerence since we are using small angle approximation.
    EXPECT_NEAR(expectedPosition, this->joint->GetPosition(0), kTol * 10);
    EXPECT_NEAR(expectedAngVel, this->joint->GetVelocity(0), kTol);
  }
}

TEST_F(BasicJointFeaturesTest, GetSetForceAccel)
{
  const double kTol = 1e-6;

  for (const std::string &name : this->pluginNames)
  {
    // Some of the basic expectations, such as the initial joint position, are
    // different in Bullet. Since the implementation is likely to be deprecated,
    // we will skip it in this test rather than trying to fix the underlying
    // issue.
    // We're also skipping bullet-featherstone because the expected joint
    // position and velocity values do not match what's reported by the physics
    // engine with a small enough tolerance.
    CHECK_UNSUPPORTED_ENGINE(name, "bullet")
    CHECK_UNSUPPORTED_ENGINE(name, "bullet-featherstone")

    this->sdfWorld->SetGravity(math::Vector3d::Zero);
    this->InitPluginAndWorld(name);
    const double kForceCmd = 0.5;

    // Setting the force should cause acceleration. We'll check the actual
    // valuelater.
    this->joint->SetForce(0, kForceCmd);
    this->Step();

    // dartsim currently clears all joint forces after a step, so GetForce
    // always return 0.
    if (this->PhysicsEngineName(name) != "dartsim")
    {
      EXPECT_DOUBLE_EQ(kForceCmd, this->joint->GetForce(0));
    }
    EXPECT_GT(this->joint->GetAcceleration(0), 0);

    // Stepping without setting force should produce no joint acceleration.
    this->Step();
    EXPECT_NEAR(this->joint->GetAcceleration(0), 0, kTol);
    EXPECT_DOUBLE_EQ(0.0, this->joint->GetForce(0));

    const int numSteps = 100;
    for (int i = 0; i < numSteps; ++i)
    {
      this->joint->SetForce(0, kForceCmd);
      this->Step();
    }

    EXPECT_NEAR(kForceCmd, this->joint->GetAcceleration(0) * this->moiPivot,
                kTol);

    // Getting the force immediately after setting it without stepping should
    // return the set force value.
    this->joint->SetForce(0, kForceCmd * 10);
    if (this->PhysicsEngineName(name) != "dartsim")
    {
      EXPECT_DOUBLE_EQ(kForceCmd * 10, this->joint->GetForce(0));
    }
  }
}

TEST_F(BasicJointFeaturesTest, GetProperties)
{
  const double kTol = 1e-6;
  for (const std::string &name : this->pluginNames)
  {
    CHECK_UNSUPPORTED_ENGINE(name, "bullet")

    this->InitPluginAndWorld(name);

    EXPECT_EQ(2u, this->model->GetJointCount());
    auto fixedJoint = this->model->GetJoint(0);
    EXPECT_EQ(0u, fixedJoint->GetDegreesOfFreedom());
    EXPECT_EQ(1u, this->joint->GetDegreesOfFreedom());

    // TODO(azeey): There seems to be a bug in bullet-featherstone
    CHECK_UNSUPPORTED_ENGINE(name, "bullet-featherstone")
    // Fixed joint
    {
      Eigen::Isometry3d expectedFromParent =
          Eigen::Translation3d(1, 1, 1) *
          Eigen::AngleAxisd(GZ_PI, Eigen::Vector3d::UnitZ());
      EXPECT_TRUE(physics::test::Equal(
          expectedFromParent, fixedJoint->GetTransformFromParent(), kTol))
          << expectedFromParent.matrix() << "\n\n"
          << fixedJoint->GetTransformFromParent().matrix();

      Eigen::Isometry3d expectedToChild =
          Eigen::Translation3d(0, 0, -1) *
          Eigen::AngleAxisd(-GZ_PI, Eigen::Vector3d::UnitZ());
      EXPECT_TRUE(physics::test::Equal(expectedToChild,
                                       fixedJoint->GetTransformToChild(), kTol))
          << expectedToChild.matrix() << "\n\n"
          << fixedJoint->GetTransformToChild().matrix();

      Eigen::Isometry3d expectedFullTransform =
          expectedFromParent * expectedToChild;
      EXPECT_TRUE(physics::test::Equal(expectedFullTransform,
                                       fixedJoint->GetTransform(), kTol))
          << expectedFullTransform.matrix() << "\n\n"
          << fixedJoint->GetTransform().matrix();
    }

    // Revolute joint initial
    {
      Eigen::Isometry3d expectedFromParent(Eigen::Translation3d(0, -1.0, 2.1));
      EXPECT_TRUE(physics::test::Equal(
          expectedFromParent, this->joint->GetTransformFromParent(), kTol))
          << expectedFromParent.matrix() << "\n\n"
          << this->joint->GetTransformFromParent().matrix();

      Eigen::Isometry3d expectedFromChild = Eigen::Isometry3d::Identity();
      EXPECT_TRUE(physics::test::Equal(
          expectedFromChild, this->joint->GetTransformToChild(), kTol))
          << expectedFromChild.matrix() << "\n\n"
          << this->joint->GetTransformToChild().matrix();

      Eigen::Isometry3d expectedFullTransform =
          expectedFromParent * expectedFromChild;
      EXPECT_TRUE(physics::test::Equal(expectedFullTransform,
                                       this->joint->GetTransform(), kTol))
          << expectedFullTransform.matrix() << "\n\n"
          << this->joint->GetTransform().matrix();
    }

    // Revolute joint with a set joint position
    {
      const double testPos = GZ_PI_2;
      this->joint->SetPosition(0, testPos);
      Eigen::Isometry3d expectedFullTransform =
          Eigen::Translation3d(0, -1.0, 2.1) *
          Eigen::AngleAxisd(testPos, Eigen::Vector3d::UnitX());

      // Test before and after step. The values should be about the same.
      EXPECT_TRUE(physics::test::Equal(expectedFullTransform,
                                       this->joint->GetTransform(), kTol))
          << expectedFullTransform.matrix() << "\n\n"
          << this->joint->GetTransform().matrix();

      this->Step();

      EXPECT_TRUE(physics::test::Equal(expectedFullTransform,
                                       this->joint->GetTransform(), 1e-4))
          << expectedFullTransform.matrix() << "\n\n"
          << this->joint->GetTransform().matrix();
    }
  }
}

int main(int argc, char *argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  if (!physics::TestLibLoader::init(argc, argv))
    return -1;
  return RUN_ALL_TESTS();
}
