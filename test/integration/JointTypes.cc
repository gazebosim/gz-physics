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

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <ignition/plugin/Loader.hh>
#include <ignition/plugin/PluginPtr.hh>
#include <ignition/physics/RequestEngine.hh>

#include <ignition/physics/RevoluteJoint.hh>
#include <ignition/physics/FrameSemantics.hh>

#include "../MockJoints.hh"
#include "../Utils.hh"

using namespace ignition::physics;
using namespace ignition::physics::test;

/////////////////////////////////////////////////
ignition::plugin::PluginPtr LoadMockJointTypesPlugin(
    const std::string &_suffix)
{
  ignition::plugin::Loader pl;
  auto plugins = pl.LoadLibrary(MockJoints_LIB);

  ignition::plugin::PluginPtr plugin =
      pl.Instantiate("mock::JointPlugin"+_suffix);
  EXPECT_FALSE(plugin.IsEmpty());

  return plugin;
}

/////////////////////////////////////////////////
template <typename PolicyT, typename FeatureListT>
void TestRevoluteJointFK(
    const std::vector<typename PolicyT::Scalar> &_testJointPositions,
    const EnginePtr<PolicyT, FeatureListT> &_engine,
    const double _tolerance)
{
  using LinearVector =
      typename FromPolicy<PolicyT>::template Use<LinearVector>;

  using AngularVector =
      typename FromPolicy<PolicyT>::template Use<AngularVector>;

  using Pose =
      typename FromPolicy<PolicyT>::template Use<Pose>;

  using Scalar = typename PolicyT::Scalar;

  // This is the initial transform of each parent->joint and joint->child in the
  // test engine.
  Pose T = Pose::Identity();
  T.translation() = LinearVector::UnitY();
  const Pose T_initial = T;
  T = Pose::Identity();

  for (std::size_t i=0; i < _testJointPositions.size(); ++i)
  {
    JointPtr<PolicyT, FeatureListT> joint = _engine->GetJoint(i);

    const Scalar q_desired = _testJointPositions[i];
    joint->SetPosition(0, q_desired);
    EXPECT_NEAR(q_desired, joint->GetPosition(0), _tolerance);
  }

  for (std::size_t i=0; i < _testJointPositions.size(); ++i)
  {
    RevoluteJointPtr<PolicyT, FeatureListT> joint =
        _engine->GetJoint(i)->CastToRevoluteJoint();

    const Scalar q = joint->GetPosition(0);

    const AngularVector &axis = joint->GetAxis();
    const Pose R{Rotate(q, axis)};

    T = T * T_initial * R;

    const Pose pose = joint->FrameDataRelativeTo(FrameID::World()).pose;
    EXPECT_TRUE(Equal(T, pose, _tolerance));

    T = T * T_initial;
  }
}

/////////////////////////////////////////////////
template <typename PolicyT>
void TestRevoluteJoint(const double _tolerance, const std::string &_suffix)
{
  using AngularVector =
      typename FromPolicy<PolicyT>::template Use<AngularVector>;

  auto engine = RequestEngine<PolicyT, mock::MockJointList>::From(
        LoadMockJointTypesPlugin(_suffix));

  {
    // Do a quick test of joint casting
    auto joint = engine->GetJoint(0);
    ASSERT_NE(nullptr, joint);
    auto revolute = joint->CastToRevoluteJoint();
    ASSERT_NE(nullptr, joint);

    AngularVector testAxis = AngularVector::Zero();
    testAxis[0] = 1.0;

    EXPECT_TRUE(Equal(testAxis, revolute->GetAxis(), _tolerance, "axes"));
  }

  // Try various sets of joint position values
  TestRevoluteJointFK<PolicyT>({0.0, 0.0, 0.0}, engine, _tolerance);
  TestRevoluteJointFK<PolicyT>({0.1, -1.57, 2.56}, engine, _tolerance);
  TestRevoluteJointFK<PolicyT>({-2.4, -7.0, -0.18}, engine, _tolerance);
}

/////////////////////////////////////////////////
template <typename PolicyT>
void TestJointTypeCasts(const std::string &_suffix)
{
  using Pose = typename FromPolicy<PolicyT>::template Use<Pose>;

  auto engine = RequestEngine<PolicyT, mock::MockJointList>::From(
        LoadMockJointTypesPlugin(_suffix));

  auto joint = engine->GetJoint(0);

  // For now, we just have this test to see that these different joint types can
  // compile, and that their APIs are available. We can add ASSERT_ and EXPECT_
  // statements later, once the mock::JointPlugin is more developed.
  if (joint)
  {
    auto free = joint->CastToFreeJoint();
    if (free)
    {
      free->SetTransform(Pose::Identity());
    }

    auto prismatic = joint->CastToPrismaticJoint();
    if (prismatic)
    {
      auto axis = prismatic->GetAxis();
      prismatic->SetAxis(axis);
    }
  }
}

/////////////////////////////////////////////////
TEST(JointTypes_TEST, RevoluteJoint3d)
{
  TestRevoluteJoint<FeaturePolicy3d>(1e-16, "3d");
}

/////////////////////////////////////////////////
TEST(JointTypes_TEST, RevoluteJoint2d)
{
  TestRevoluteJoint<FeaturePolicy2d>(1e-16, "2d");
}

/////////////////////////////////////////////////
TEST(JointTypes_TEST, RevoluteJoint3f)
{
  TestRevoluteJoint<FeaturePolicy3f>(1e-16, "3f");
}

/////////////////////////////////////////////////
TEST(JointTypes_TEST, RevoluteJoint2f)
{
  TestRevoluteJoint<FeaturePolicy2f>(1e-16, "2f");
}


int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
