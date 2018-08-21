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

#include <ignition/plugin/Loader.hh>
#include <ignition/plugin/PluginPtr.hh>
#include <ignition/physics/RequestFeatures.hh>

#include "../Utils.hh"
#include "../MockFrameSemantics.hh"


using ignition::physics::FrameData;
using ignition::physics::FrameID;
using ignition::physics::RelativeFrameData;
using ignition::physics::Pose;
using ignition::physics::Vector;
using ignition::physics::LinearVector;
using ignition::physics::AngularVector;

using ignition::math::Rand;
using namespace ignition::physics::test;

/////////////////////////////////////////////////
ignition::plugin::PluginPtr LoadMockFrameSemanticsPlugin(
    const std::string &_suffix)
{
  ignition::plugin::Loader pl;
  auto plugins = pl.LoadLibrary(MockFrames_LIB);
  EXPECT_EQ(4u, plugins.size());

  ignition::plugin::PluginPtr plugin =
      pl.Instantiate("mock::MockFrameSemanticsPlugin"+_suffix);
  EXPECT_FALSE(plugin.IsEmpty());

  return plugin;
}

/////////////////////////////////////////////////
template <typename PolicyT>
void TestRelativeFrames(const double _tolerance, const std::string &_suffix)
{
  using Scalar = typename PolicyT::Scalar;
  constexpr std::size_t Dim = PolicyT::Dim;

  // Instantiate an engine that provides Frame Semantics.
  auto fs =
      ignition::physics::RequestFeatures<PolicyT, mock::MockFrameSemanticsList>
        ::From(LoadMockFrameSemanticsPlugin(_suffix));

  using FrameData = FrameData<Scalar, Dim>;
  using RelativeFrameData = RelativeFrameData<Scalar, Dim>;

  // Note: The World Frame is often designated by the letter O

  // Create Frame A
  const FrameData T_A = RandomFrameData<Scalar, Dim>();
  const FrameID A = fs->CreateLink("A", T_A)->GetFrameID();

  // Create Frame B
  const FrameData T_B = RandomFrameData<Scalar, Dim>();
  const FrameID B = fs->CreateLink("B", T_B)->GetFrameID();

  RelativeFrameData B_T_B = RelativeFrameData(B);
  EXPECT_TRUE(Equal(T_B, fs->Resolve(B_T_B, FrameID::World()), _tolerance));

  // Create a frame relative to A which is equivalent to B
  const RelativeFrameData A_T_B =
      RelativeFrameData(A, fs->GetLink("B")->FrameDataRelativeTo(A));

  // When A_T_B is expressed with respect to the world, it should be equivalent
  // to Frame B
  EXPECT_TRUE(Equal(T_B, fs->Resolve(A_T_B, FrameID::World()), _tolerance));

  const RelativeFrameData O_T_B = RelativeFrameData(FrameID::World(), T_B);

  // When O_T_B is expressed with respect to A, it should be equivalent to
  // A_T_B
  EXPECT_TRUE(Equal(A_T_B.RelativeToParent(),
                    fs->Resolve(O_T_B, A), _tolerance));

  // Define a new frame (C), relative to B
  const RelativeFrameData B_T_C =
      RelativeFrameData(B, RandomFrameData<Scalar, Dim>());

  // Reframe C with respect to the world
  const RelativeFrameData O_T_C = fs->Reframe(B_T_C, FrameID::World());

  // Also, compute its raw transform with respect to the world
  const FrameData T_C = fs->Resolve(B_T_C, FrameID::World());

  EXPECT_TRUE(Equal(T_C, O_T_C.RelativeToParent(), _tolerance));

  const RelativeFrameData O_T_A = RelativeFrameData(FrameID::World(), T_A);
  EXPECT_TRUE(Equal(T_C.pose,
                      O_T_A.RelativeToParent().pose
                    * A_T_B.RelativeToParent().pose
                    * B_T_C.RelativeToParent().pose, _tolerance));
}

/////////////////////////////////////////////////
TEST(FrameSemantics_TEST, RelativeFrames3d)
{
  TestRelativeFrames<ignition::physics::FeaturePolicy3d>(1e-11, "3d");
}

/////////////////////////////////////////////////
TEST(FrameSemantics_TEST, RelativeFrames2d)
{
  TestRelativeFrames<ignition::physics::FeaturePolicy2d>(1e-14, "2d");
}

/////////////////////////////////////////////////
TEST(FrameSemantics_TEST, RelativeFrames3f)
{
  TestRelativeFrames<ignition::physics::FeaturePolicy3f>(1e-3, "3f");
}

/////////////////////////////////////////////////
TEST(FrameSemantics_TEST, RelativeFrames2f)
{
  TestRelativeFrames<ignition::physics::FeaturePolicy2f>(1e-3, "2f");
}

/////////////////////////////////////////////////
template <typename PolicyT>
void TestFrameID(const double _tolerance, const std::string &_suffix)
{
  using Scalar = typename PolicyT::Scalar;
  constexpr std::size_t Dim = PolicyT::Dim;

  // Instantiate an engine that provides Frame Semantics.
  auto fs =
      ignition::physics::RequestFeatures<PolicyT, mock::MockFrameSemanticsList>
        ::From(LoadMockFrameSemanticsPlugin(_suffix));

  using FrameData = FrameData<Scalar, Dim>;
  using RelativeFrameData = RelativeFrameData<Scalar, Dim>;

  using Link = ignition::physics::Link<PolicyT, mock::MockFrameSemanticsList>;
  using LinkPtr = std::unique_ptr<Link>;

  using Joint = ignition::physics::Joint<PolicyT, mock::MockFrameSemanticsList>;
  using ConstJointPtr = std::unique_ptr<const Joint>;

  // We test FrameID in this unit test, because the FrameSemantics interface is
  // needed in order to produce FrameIDs.
  const FrameID world = FrameID::World();

  // The world FrameID is always considered to be "reference counted", because
  // it must always be treated as a valid ID.
  EXPECT_TRUE(world.IsReferenceCounted());

  const FrameData dataA = RandomFrameData<Scalar, Dim>();
  const LinkPtr linkA = fs->CreateLink("A", dataA);

  EXPECT_TRUE(Equal(dataA, linkA->FrameDataRelativeTo(world), _tolerance));

  const FrameID A = linkA->GetFrameID();
  EXPECT_TRUE(A.IsReferenceCounted());
  EXPECT_EQ(A, fs->GetLink("A")->GetFrameID());
  EXPECT_EQ(A, linkA->GetFrameID());

  // improve coverage of FrameID operators
  EXPECT_EQ(world, world);
  EXPECT_GE(world, world);
  EXPECT_LE(world, world);
  EXPECT_NE(world, A);
  EXPECT_LE(world, A);
  EXPECT_LT(world, A);
  EXPECT_NE(A, world);
  EXPECT_GE(A, world);
  EXPECT_GT(A, world);

  // This is the implicit conversion operator which can implicitly turn a
  // FrameSemantics::Object reference into a FrameID.
  const FrameID otherA = *linkA;
  EXPECT_EQ(otherA, A);

  const FrameData dataJ1 = RandomFrameData<Scalar, Dim>();
  const ConstJointPtr joint1 = fs->CreateJoint("B", dataJ1);
  EXPECT_NE(nullptr, joint1);

  const FrameID J1 = joint1->GetFrameID();
  EXPECT_FALSE(J1.IsReferenceCounted());
  EXPECT_EQ(J1, fs->GetJoint("B")->GetFrameID());
  EXPECT_EQ(J1, joint1->GetFrameID());

  // Create relative frame data for J1 with respect to the world frame
  const RelativeFrameData O_T_J1(FrameID::World(), dataJ1);
  // Create a version which is with respect to frame A
  const RelativeFrameData A_T_J1 = fs->Reframe(O_T_J1, A);

  // When we dereference linkA, the implicit conversion operator should be able
  // to automatically convert it to a FrameID that can be used by the Frame
  // Semantics API.
  EXPECT_TRUE(Equal(A_T_J1.RelativeToParent(),
                    fs->Resolve(O_T_J1, *linkA), _tolerance));

  const RelativeFrameData J1_T_J1 = fs->Reframe(A_T_J1, *joint1);
  EXPECT_TRUE(Equal(J1_T_J1.RelativeToParent(),
                    fs->Resolve(O_T_J1, J1), _tolerance));
}

/////////////////////////////////////////////////
TEST(FrameSemantics_TEST, FrameID3d)
{
  TestFrameID<ignition::physics::FeaturePolicy3d>(1e-11, "3d");
}

/////////////////////////////////////////////////
TEST(FrameSemantics_TEST, FrameID2d)
{
  TestFrameID<ignition::physics::FeaturePolicy2d>(1e-12, "2d");
}

/////////////////////////////////////////////////
TEST(FrameSemantics_TEST, FrameID3f)
{
  TestFrameID<ignition::physics::FeaturePolicy3f>(1e-2, "3f");
}

/////////////////////////////////////////////////
TEST(FrameSemantics_TEST, FrameID2f)
{
  TestFrameID<ignition::physics::FeaturePolicy2f>(1e-4, "2f");
}

/////////////////////////////////////////////////
template <typename PolicyT>
void TestFramedQuantities(const double _tolerance, const std::string &_suffix)
{
  using Scalar = typename PolicyT::Scalar;
  constexpr std::size_t Dim = PolicyT::Dim;

  // Instantiate an engine that provides Frame Semantics.
  auto fs =
      ignition::physics::RequestFeatures<PolicyT, mock::MockFrameSemanticsList>
        ::From(LoadMockFrameSemanticsPlugin(_suffix));

  using RelativeFrameData = RelativeFrameData<Scalar, Dim>;
  using Pose = Pose<Scalar, Dim>;
  using LinearVector = LinearVector<Scalar, Dim>;
  using AngularVector = AngularVector<Scalar, Dim>;
  using Rotation = Rotation<Scalar, Dim>;
  using FramedPose = ignition::physics::FramedPose<Scalar, Dim>;
  using FramedPosition = ignition::physics::FramedPosition<Scalar, Dim>;
  using FramedForce = ignition::physics::FramedForce<Scalar, Dim>;
  using FramedTorque = ignition::physics::FramedTorque<Scalar, Dim>;

  const FrameID World = FrameID::World();

  // Create a transform from the world to Frame A
  const RelativeFrameData O_T_A(World, RandomFrameData<Scalar, Dim>());
  // Instantiate Frame A
  const FrameID A = *fs->CreateLink("A", fs->Resolve(O_T_A, World));

  // Create a transform from Frame A to Frame B
  const RelativeFrameData A_T_B(A, RandomFrameData<Scalar, Dim>());
  // Instantiate Frame B using A_T_B. Note that CreateLink(~) expects to receive
  // the link's transform with respect to the world, so we use Resolve(~) before
  // passing along the FrameData
  const FrameID B = *fs->CreateLink("B", fs->Resolve(A_T_B, World));

  // Create a transform from Frame B to Frame C
  const RelativeFrameData B_T_C(B, RandomFrameData<Scalar, Dim>());
  // Instantiate Frame C using B_T_C
  const FrameID C = *fs->CreateLink("C", fs->Resolve(B_T_C, World));

  // Create a transform from Frame A to Frame D
  const RelativeFrameData A_T_D(A, RandomFrameData<Scalar, Dim>());
  // Instantiate Frame D using A_T_D
  const FrameID D = *fs->CreateLink("D", fs->Resolve(A_T_D, World));

  // Create FramedPose for B with respect to A
  const FramedPose A_T_B_pose(A, A_T_B.RelativeToParent().pose);
  EXPECT_TRUE(Equal(A_T_B_pose.RelativeToParent(),
                    fs->Resolve(A_T_B_pose, A), _tolerance));

  const Pose O_T_B_pose =
        O_T_A.RelativeToParent().pose
      * A_T_B_pose.RelativeToParent();
  EXPECT_TRUE(Equal(O_T_B_pose,
                    fs->Resolve(A_T_B_pose, World), _tolerance));

  const Pose A_T_B_pose_inCoordsOfWorld =
        O_T_A.RelativeToParent().pose.linear()
      * A_T_B_pose.RelativeToParent();
  EXPECT_TRUE(Equal(A_T_B_pose_inCoordsOfWorld,
                    fs->Resolve(A_T_B_pose, A, World), _tolerance));

  const Pose A_T_B_pose_inCoordsOfD =
        A_T_D.RelativeToParent().pose.linear().inverse()
      * A_T_B_pose.RelativeToParent();
  EXPECT_TRUE(Equal(A_T_B_pose_inCoordsOfD,
                    fs->Resolve(A_T_B_pose, A, D), _tolerance));

  const Pose D_T_B_pose =
        A_T_D.RelativeToParent().pose.inverse()
      * A_T_B_pose.RelativeToParent();
  EXPECT_TRUE(Equal(D_T_B_pose,
                    fs->Resolve(A_T_B_pose, D), _tolerance));

  // Create point "1" in Frame C
  const FramedPosition C_p1(C, RandomVector<LinearVector>(10.0));
  EXPECT_TRUE(Equal(C_p1.RelativeToParent(), fs->Resolve(C_p1, C), _tolerance));

  const LinearVector C_p1_inCoordsOfWorld =
        O_T_A.RelativeToParent().pose.linear()
      * A_T_B.RelativeToParent().pose.linear()
      * B_T_C.RelativeToParent().pose.linear()
      *  C_p1.RelativeToParent();
  EXPECT_TRUE(Equal(C_p1_inCoordsOfWorld,
                    fs->Resolve(C_p1, C, World), _tolerance));

  const LinearVector C_p1_inCoordsOfD =
      A_T_D.RelativeToParent().pose.linear().transpose()
    * A_T_B.RelativeToParent().pose.linear()
    * B_T_C.RelativeToParent().pose.linear()
    *  C_p1.RelativeToParent();
  EXPECT_TRUE(Equal(C_p1_inCoordsOfD, fs->Resolve(C_p1, C, D), _tolerance));

  const LinearVector O_p1 =
        O_T_A.RelativeToParent().pose
      * A_T_B.RelativeToParent().pose
      * B_T_C.RelativeToParent().pose
      *  C_p1.RelativeToParent();
  EXPECT_TRUE(Equal(O_p1, fs->Resolve(C_p1, World), _tolerance));

  const LinearVector O_p1_inCoordsOfC =
        B_T_C.RelativeToParent().pose.linear().transpose()
      * A_T_B.RelativeToParent().pose.linear().transpose()
      * O_T_A.RelativeToParent().pose.linear().transpose()
      *  O_p1;
  EXPECT_TRUE(Equal(O_p1_inCoordsOfC, fs->Resolve(C_p1, World, C), _tolerance));

  const LinearVector O_p1_inCoordsOfD =
        A_T_D.RelativeToParent().pose.linear().transpose()
      * O_T_A.RelativeToParent().pose.linear().transpose()
      *   O_p1;
  EXPECT_TRUE(Equal(O_p1_inCoordsOfD, fs->Resolve(C_p1, World, D), _tolerance));

  const LinearVector D_p1 =
        A_T_D.RelativeToParent().pose.inverse()
      * A_T_B.RelativeToParent().pose
      * B_T_C.RelativeToParent().pose
      *  C_p1.RelativeToParent();
  EXPECT_TRUE(Equal(D_p1, fs->Resolve(C_p1, D), _tolerance));

  const LinearVector D_p1_inCoordsOfWorld =
        O_T_A.RelativeToParent().pose.linear()
      * A_T_D.RelativeToParent().pose.linear()
      *  D_p1;
  EXPECT_TRUE(Equal(D_p1_inCoordsOfWorld,
                    fs->Resolve(C_p1, D, World), _tolerance));

  const LinearVector D_p1_inCoordsOfC =
        B_T_C.RelativeToParent().pose.linear().transpose()
      * A_T_B.RelativeToParent().pose.linear().transpose()
      * A_T_D.RelativeToParent().pose.linear()
      *  D_p1;
  EXPECT_TRUE(Equal(D_p1_inCoordsOfC, fs->Resolve(C_p1, D, C), _tolerance));

  // Create point "2" in Frame D
  const FramedPosition D_p2(D, RandomVector<LinearVector>(10.0));
  EXPECT_TRUE(Equal(D_p2.RelativeToParent(), fs->Resolve(D_p2, D), _tolerance));

  const LinearVector O_p2 =
        O_T_A.RelativeToParent().pose
      * A_T_D.RelativeToParent().pose
      *  D_p2.RelativeToParent();
  EXPECT_TRUE(Equal(O_p2, fs->Resolve(D_p2, World), _tolerance));

  const LinearVector C_p2 =
        B_T_C.RelativeToParent().pose.inverse()
      * A_T_B.RelativeToParent().pose.inverse()
      * A_T_D.RelativeToParent().pose
      *  D_p2.RelativeToParent();
  EXPECT_TRUE(Equal(C_p2, fs->Resolve(D_p2, C), _tolerance));

  // Create point "3" in the World Frame
  const FramedPosition O_p3(World, RandomVector<LinearVector>(10.0));
  EXPECT_TRUE(Equal(O_p3.RelativeToParent(),
                    fs->Resolve(O_p3, World), _tolerance));

  const LinearVector O_p3_inCoordsOfC =
        B_T_C.RelativeToParent().pose.linear().transpose()
      * A_T_B.RelativeToParent().pose.linear().transpose()
      * O_T_A.RelativeToParent().pose.linear().transpose()
      *  O_p3.RelativeToParent();
  EXPECT_TRUE(Equal(O_p3_inCoordsOfC, fs->Resolve(O_p3, World, C), _tolerance));

  const LinearVector C_p3 =
        B_T_C.RelativeToParent().pose.inverse()
      * A_T_B.RelativeToParent().pose.inverse()
      * O_T_A.RelativeToParent().pose.inverse()
      *  O_p3.RelativeToParent();
  EXPECT_TRUE(Equal(C_p3, fs->Resolve(O_p3, C), _tolerance));

  const LinearVector C_p3_inCoordsOfWorld =
        O_T_A.RelativeToParent().pose.linear()
      * A_T_B.RelativeToParent().pose.linear()
      * B_T_C.RelativeToParent().pose.linear()
      *  C_p3;
  EXPECT_TRUE(Equal(C_p3_inCoordsOfWorld,
                    fs->Resolve(O_p3, C, World), _tolerance));

  // Create force "1" in Frame C
  const FramedForce C_f1(C, RandomVector<LinearVector>(10.0));
  EXPECT_TRUE(Equal(C_f1.RelativeToParent(), fs->Resolve(C_f1, C), _tolerance));

  const LinearVector O_f1 =
        O_T_A.RelativeToParent().pose.linear()
      * A_T_B.RelativeToParent().pose.linear()
      * B_T_C.RelativeToParent().pose.linear()
      *  C_f1.RelativeToParent();
  EXPECT_TRUE(Equal(O_f1, fs->Resolve(C_f1, World), _tolerance));

  const LinearVector D_f1 =
        A_T_D.RelativeToParent().pose.linear().transpose()
      * A_T_B.RelativeToParent().pose.linear()
      * B_T_C.RelativeToParent().pose.linear()
      *  C_f1.RelativeToParent();
  EXPECT_TRUE(Equal(D_f1, fs->Resolve(C_f1, D), _tolerance));

  // Create force "2" in Frame D
  const FramedForce D_f2(D, RandomVector<LinearVector>(10.0));
  EXPECT_TRUE(Equal(D_f2.RelativeToParent(), fs->Resolve(D_f2, D), _tolerance));

  const LinearVector O_f2 =
        O_T_A.RelativeToParent().pose.linear()
      * A_T_D.RelativeToParent().pose.linear()
      *  D_f2.RelativeToParent();
  EXPECT_TRUE(Equal(O_f2, fs->Resolve(D_f2, World), _tolerance));

  const LinearVector C_f2 =
        B_T_C.RelativeToParent().pose.linear().transpose()
      * A_T_B.RelativeToParent().pose.linear().transpose()
      * A_T_D.RelativeToParent().pose.linear()
      *  D_f2.RelativeToParent();
  EXPECT_TRUE(Equal(C_f2, fs->Resolve(D_f2, C), _tolerance));

  // Create force "3" in the World Frame
  const FramedForce O_f3(World, RandomVector<LinearVector>(10.0));
  EXPECT_TRUE(Equal(O_f3.RelativeToParent(),
                    fs->Resolve(O_f3, World), _tolerance));

  const LinearVector C_f3 =
        B_T_C.RelativeToParent().pose.linear().transpose()
      * A_T_B.RelativeToParent().pose.linear().transpose()
      * O_T_A.RelativeToParent().pose.linear().transpose()
      *  O_f3.RelativeToParent();
  EXPECT_TRUE(Equal(C_f3, fs->Resolve(O_f3, C), _tolerance));

  // Create torque "1" in Frame C
  const FramedTorque C_t1(C, RandomVector<AngularVector>(10.0));
  EXPECT_TRUE(Equal(C_t1.RelativeToParent(), fs->Resolve(C_t1, C), _tolerance));

  const AngularVector O_t1 =
      Rotation::Apply(
          O_T_A.RelativeToParent().pose.linear()
        * A_T_B.RelativeToParent().pose.linear()
        * B_T_C.RelativeToParent().pose.linear(),
           C_t1.RelativeToParent());
  EXPECT_TRUE(Equal(O_t1, fs->Resolve(C_t1, World), _tolerance));

  const AngularVector O_t1_inCoordsOfC =
      Rotation::Apply(
          B_T_C.RelativeToParent().pose.linear().transpose()
        * A_T_B.RelativeToParent().pose.linear().transpose()
        * O_T_A.RelativeToParent().pose.linear().transpose(),
           O_t1);
  EXPECT_TRUE(Equal(O_t1_inCoordsOfC, fs->Resolve(C_t1, World, C), _tolerance));

  const AngularVector D_t1 =
      Rotation::Apply(
          A_T_D.RelativeToParent().pose.linear().transpose()
        * A_T_B.RelativeToParent().pose.linear()
        * B_T_C.RelativeToParent().pose.linear(),
           C_t1.RelativeToParent());
  EXPECT_TRUE(Equal(D_t1, fs->Resolve(C_t1, D), _tolerance));

  const AngularVector D_t1_inCoordsOfWorld =
      Rotation::Apply(
          O_T_A.RelativeToParent().pose.linear()
        * A_T_D.RelativeToParent().pose.linear(),
           D_t1);
  EXPECT_TRUE(Equal(D_t1_inCoordsOfWorld,
                    fs->Resolve(C_t1, D, World), _tolerance));

  const AngularVector D_t1_inCoordsOfC =
      Rotation::Apply(
          B_T_C.RelativeToParent().pose.linear().transpose()
        * A_T_B.RelativeToParent().pose.linear().transpose()
        * A_T_D.RelativeToParent().pose.linear(),
           D_t1);
  EXPECT_TRUE(Equal(D_t1_inCoordsOfC, fs->Resolve(C_t1, D, C), _tolerance));

  // Create torque "2" in Frame D
  const FramedTorque D_t2(D, RandomVector<AngularVector>(10.0));
  EXPECT_TRUE(Equal(D_f2.RelativeToParent(), fs->Resolve(D_f2, D), _tolerance));

  const AngularVector O_t2 =
      Rotation::Apply(
          O_T_A.RelativeToParent().pose.linear()
        * A_T_D.RelativeToParent().pose.linear(),
           D_t2.RelativeToParent());
  EXPECT_TRUE(Equal(O_t2, fs->Resolve(D_t2, World), _tolerance));

  const AngularVector C_t2 =
      Rotation::Apply(
          B_T_C.RelativeToParent().pose.linear().transpose()
        * A_T_B.RelativeToParent().pose.linear().transpose()
        * A_T_D.RelativeToParent().pose.linear(),
           D_t2.RelativeToParent());
  EXPECT_TRUE(Equal(C_t2, fs->Resolve(D_t2, C), _tolerance));

  // Create torque "3" in the World Frame
  const FramedTorque O_t3(World, RandomVector<AngularVector>(10.0));
  EXPECT_TRUE(Equal(O_t3.RelativeToParent(),
                    fs->Resolve(O_t3, World), _tolerance));

  const AngularVector C_t3 =
      Rotation::Apply(
          B_T_C.RelativeToParent().pose.linear().transpose()
        * A_T_B.RelativeToParent().pose.linear().transpose()
        * O_T_A.RelativeToParent().pose.linear().transpose(),
           O_t3.RelativeToParent());
  EXPECT_TRUE(Equal(C_t3, fs->Resolve(O_t3, C), _tolerance));
}

/////////////////////////////////////////////////
TEST(FrameSemantics_TEST, FramedQuantities3d)
{
  TestFramedQuantities<ignition::physics::FeaturePolicy3d>(1e-11, "3d");
}

/////////////////////////////////////////////////
TEST(FrameSemantics_TEST, FramedQuantities2d)
{
  TestFramedQuantities<ignition::physics::FeaturePolicy2d>(1e-11, "2d");
}

/////////////////////////////////////////////////
TEST(FrameSemantics_TEST, FramedQuantities3f)
{
  TestFramedQuantities<ignition::physics::FeaturePolicy3f>(1e-2, "3f");
}

/////////////////////////////////////////////////
TEST(FrameSemantics_TEST, FramedQuantities2f)
{
  TestFramedQuantities<ignition::physics::FeaturePolicy2f>(1e-4, "2f");
}

/////////////////////////////////////////////////
template <typename PolicyT>
void TestRelativeFrameData(const double _tolerance, const std::string &_suffix)
{
  using Scalar = typename PolicyT::Scalar;
  constexpr std::size_t Dim = PolicyT::Dim;
  ASSERT_EQ(3u, Dim);

  // Instantiate an engine that provides Frame Semantics.
  auto fs =
      ignition::physics::RequestFeatures<PolicyT, mock::MockFrameSemanticsList>
        ::From(LoadMockFrameSemanticsPlugin(_suffix));

  using FrameData = FrameData<Scalar, Dim>;
  using RelativeFrameData = RelativeFrameData<Scalar, Dim>;
  using LinearVector = LinearVector<Scalar, Dim>;
  using AngularVector = AngularVector<Scalar, Dim>;

  const FrameID World = FrameID::World();

  // Consider a pivot irrigation system with following points:
  // A: pivot
  // B: attachment of pipe to rolling support
  // C: wheel center
  // D: wheel-ground contact point
  const Scalar pivotAngle = 30 * IGN_PI / 180;
  const Scalar sinPivot = sin(pivotAngle);
  const Scalar cosPivot = cos(pivotAngle);
  const Scalar pivotHeight = 1;
  const Scalar pivotLength = 5;
  const Scalar pivotRotationRate = 0.1;
  const Scalar wheelRadius = 0.25;
  const Scalar wheelRotationRate = pivotRotationRate
                                 * pivotLength / wheelRadius;

  // Create a transform from the world to Frame Base. We do this to have better
  // coverage of FramedQuantities
  FrameData T_Base;
  const RelativeFrameData O_T_Base(World, T_Base);
  const FrameID Base = *fs->CreateLink("Base", fs->Resolve(O_T_Base, World));

  // Create a transform from the world to Frame A
  const LinearVector axisZ = {0, 0, 1};
  FrameData T_A;
  T_A.pose.translation() = LinearVector(0, 0, pivotHeight);
  T_A.pose.rotate(Eigen::AngleAxis<Scalar>(pivotAngle, axisZ));
  T_A.angularVelocity = AngularVector(0, 0, pivotRotationRate);
  const RelativeFrameData Base_T_A(Base, T_A);

  // Instantiate Frame A as a Link
  const FrameID A = *fs->CreateLink("A", fs->Resolve(Base_T_A, World));

  // Create a transform from Frame A to Frame B
  FrameData T_B;
  T_B.pose.translation() = LinearVector(pivotLength, 0, 0);
  const RelativeFrameData A_T_B(A, T_B);

  // Instantiate Frame B as a Joint
  const FrameID B = *fs->CreateJoint("B", fs->Resolve(A_T_B, World));

  // Create a transform from Frame B to Frame C
  FrameData T_C;
  T_C.pose.translation() = LinearVector(0, 0, wheelRadius - pivotHeight);
  T_C.angularVelocity = AngularVector(-wheelRotationRate, 0, 0);
  const RelativeFrameData B_T_C(B, T_C);

  // Instantiate Frame C as a Link
  const FrameID C = *fs->CreateLink("C", fs->Resolve(B_T_C, World));

  // Create a transform from Frame C to Frame D
  FrameData T_D;
  T_D.pose.translation() = LinearVector(0, 0, -wheelRadius);
  const RelativeFrameData C_T_D(C, T_D);

  // Instantiate Frame D as a Joint
  const FrameID D = *fs->CreateJoint("D", fs->Resolve(C_T_D, World));

  // Get ABCD FrameData relative to world in coordinates of frame A
  const FrameData A_A = fs->Resolve(Base_T_A, Base, A);
  const FrameData B_A = fs->Resolve(A_T_B, Base, A);
  const FrameData C_A = fs->Resolve(B_T_C, Base, A);
  const FrameData D_A = fs->Resolve(C_T_D, Base, A);

  // position:
  //  xy: A = [0, 0]
  EXPECT_NEAR(A_A.pose.translation()[0], 0.0, _tolerance);
  EXPECT_NEAR(A_A.pose.translation()[1], 0.0, _tolerance);
  //  xy: B = [pivotLength, 0]
  EXPECT_NEAR(B_A.pose.translation()[0], pivotLength, _tolerance);
  EXPECT_NEAR(B_A.pose.translation()[1], 0.0, _tolerance);
  //  xy: B==C==D
  EXPECT_NEAR(B_A.pose.translation()[0],
              C_A.pose.translation()[0], _tolerance);
  EXPECT_NEAR(B_A.pose.translation()[1],
              C_A.pose.translation()[1], _tolerance);
  EXPECT_NEAR(B_A.pose.translation()[0],
              D_A.pose.translation()[0], _tolerance);
  EXPECT_NEAR(B_A.pose.translation()[1],
              D_A.pose.translation()[1], _tolerance);
  //  z: A==B
  EXPECT_NEAR(A_A.pose.translation()[2], pivotHeight, _tolerance);
  EXPECT_NEAR(A_A.pose.translation()[2],
              B_A.pose.translation()[2], _tolerance);
  //  z: C == wheelRadius
  EXPECT_NEAR(C_A.pose.translation()[2], wheelRadius, _tolerance);
  //  z: D == 0.0
  EXPECT_NEAR(D_A.pose.translation()[2], 0.0, _tolerance);

  //  A==D==0: no relative linear velocity at pivot or wheel contact point
  EXPECT_EQ(A_A.linearVelocity, LinearVector::Zero());
  EXPECT_TRUE(Equal(A_A.linearVelocity, D_A.linearVelocity, _tolerance));
  //  B==C
  EXPECT_TRUE(Equal(B_A.linearVelocity, C_A.linearVelocity, _tolerance));
  EXPECT_NEAR(B_A.linearVelocity[0], 0.0, _tolerance);
  EXPECT_NEAR(B_A.linearVelocity[1], pivotLength*pivotRotationRate, _tolerance);
  EXPECT_NEAR(B_A.linearVelocity[2], 0.0, _tolerance);

  // angular velocity: A==B, C==D
  EXPECT_TRUE(Equal(A_A.angularVelocity, B_A.angularVelocity, _tolerance));
  EXPECT_NEAR(A_A.angularVelocity[0], 0.0, _tolerance);
  EXPECT_NEAR(A_A.angularVelocity[1], 0.0, _tolerance);
  EXPECT_NEAR(A_A.angularVelocity[2], pivotRotationRate, _tolerance);
  EXPECT_TRUE(Equal(C_A.angularVelocity, D_A.angularVelocity, _tolerance));
  EXPECT_NEAR(C_A.angularVelocity[0], -wheelRotationRate, _tolerance);
  EXPECT_NEAR(C_A.angularVelocity[1], 0.0, _tolerance);
  EXPECT_NEAR(C_A.angularVelocity[2], pivotRotationRate, _tolerance);

  // linear acceleration:
  //  A==0
  EXPECT_EQ(A_A.linearAcceleration, LinearVector::Zero());
  //  B centripetal acceleration torward pivot
  const Scalar accelX = -pivotLength * std::pow(pivotRotationRate, 2);
  EXPECT_NEAR(B_A.linearAcceleration[0], accelX, _tolerance);
  EXPECT_NEAR(B_A.linearAcceleration[1], 0.0, _tolerance);
  EXPECT_NEAR(B_A.linearAcceleration[2], 0.0, _tolerance);
  //  C == B
  EXPECT_TRUE(Equal(B_A.linearAcceleration,
                    C_A.linearAcceleration, _tolerance));
  // See ipython notebook deriving the following nonintuitive condition:
  //  D[0] == -B[0]
  //  D[1] == -B[1]
  EXPECT_NEAR(D_A.linearAcceleration[0],
             -B_A.linearAcceleration[0], _tolerance);
  EXPECT_NEAR(D_A.linearAcceleration[1],
             -B_A.linearAcceleration[1], _tolerance);
  //  D[2] centripetal acceleration toward wheel center
  const Scalar accelZ = wheelRadius * std::pow(wheelRotationRate, 2);
  EXPECT_NEAR(D_A.linearAcceleration[2], accelZ, _tolerance);

  // angular acceleration
  //  A==B==0
  EXPECT_EQ(A_A.angularAcceleration, LinearVector::Zero());
  EXPECT_EQ(B_A.angularAcceleration, LinearVector::Zero());
  //  C==D
  EXPECT_TRUE(Equal(C_A.angularAcceleration,
                    D_A.angularAcceleration, _tolerance));
  //  based on cross product of angular velocity vectors
  const Scalar accelY = pivotRotationRate * (-wheelRotationRate);
  EXPECT_NEAR(C_A.angularAcceleration[0], 0.0, _tolerance);
  EXPECT_NEAR(C_A.angularAcceleration[1], accelY, _tolerance);
  EXPECT_NEAR(C_A.angularAcceleration[2], 0.0, _tolerance);

  const FrameData A_O = fs->Resolve(Base_T_A, Base, World);
  const FrameData B_O = fs->Resolve(A_T_B, Base, World);
  const FrameData C_O = fs->Resolve(B_T_C, Base, World);
  const FrameData D_O = fs->Resolve(C_T_D, Base, World);

  EXPECT_NEAR(A_O.pose.translation()[0], 0.0, _tolerance);
  EXPECT_NEAR(A_O.pose.translation()[1], 0.0, _tolerance);
  //  xy: B = [pivotLength, 0]
  EXPECT_NEAR(B_O.pose.translation()[0], pivotLength * cosPivot, _tolerance);
  EXPECT_NEAR(B_O.pose.translation()[1], pivotLength * sinPivot, _tolerance);
  //  xy: B==C==D
  EXPECT_NEAR(B_O.pose.translation()[0],
              C_O.pose.translation()[0], _tolerance);
  EXPECT_NEAR(B_O.pose.translation()[1],
              C_O.pose.translation()[1], _tolerance);
  EXPECT_NEAR(B_O.pose.translation()[0],
              D_O.pose.translation()[0], _tolerance);
  EXPECT_NEAR(B_O.pose.translation()[1],
              D_O.pose.translation()[1], _tolerance);
  // //  z: A==B
  EXPECT_NEAR(A_O.pose.translation()[2], pivotHeight, _tolerance);
  EXPECT_NEAR(A_O.pose.translation()[2],
              B_O.pose.translation()[2], _tolerance);
  //  z: C == wheelRadius
  EXPECT_NEAR(C_O.pose.translation()[2], wheelRadius, _tolerance);
  //  z: D == 0.0
  EXPECT_NEAR(D_O.pose.translation()[2], 0.0, _tolerance);

  // linear velocity:
  //  A==D==0: no relative linear velocity at pivot or wheel contact point
  EXPECT_EQ(A_O.linearVelocity, LinearVector::Zero());
  EXPECT_TRUE(Equal(A_O.linearVelocity, D_O.linearVelocity, _tolerance));
  //  B==C
  EXPECT_TRUE(Equal(B_O.linearVelocity, C_O.linearVelocity, _tolerance));
  EXPECT_NEAR(B_O.linearVelocity[0],
      -sinPivot * pivotLength * pivotRotationRate, _tolerance);
  EXPECT_NEAR(B_O.linearVelocity[1],
      cosPivot * pivotLength * pivotRotationRate, _tolerance);
  EXPECT_NEAR(B_O.linearVelocity[2], 0.0, _tolerance);

  // // angular velocity: A==B, C==D
  EXPECT_TRUE(Equal(A_O.angularVelocity, B_O.angularVelocity, _tolerance));
  EXPECT_NEAR(A_O.angularVelocity[0], 0.0, _tolerance);
  EXPECT_NEAR(A_O.angularVelocity[1], 0.0, _tolerance);
  EXPECT_NEAR(A_O.angularVelocity[2], pivotRotationRate, _tolerance);
  EXPECT_TRUE(Equal(C_O.angularVelocity, D_O.angularVelocity, _tolerance));
  EXPECT_NEAR(C_O.angularVelocity[0], -cosPivot*wheelRotationRate, _tolerance);
  EXPECT_NEAR(C_O.angularVelocity[1], -sinPivot*wheelRotationRate, _tolerance);
  EXPECT_NEAR(C_O.angularVelocity[2], pivotRotationRate, _tolerance);

  // linear acceleration:
  //  A==0
  EXPECT_EQ(A_O.linearAcceleration, LinearVector::Zero());
  //  B centripetal acceleration torward pivot
  EXPECT_NEAR(B_O.linearAcceleration[0], cosPivot * accelX, _tolerance);
  EXPECT_NEAR(B_O.linearAcceleration[1], sinPivot * accelX, _tolerance);
  EXPECT_NEAR(B_O.linearAcceleration[2], 0.0, _tolerance);
  //  C == B
  EXPECT_TRUE(Equal(B_O.linearAcceleration,
                    C_O.linearAcceleration, _tolerance));
  // See ipython notebook deriving the following nonintuitive condition:
  //  D[0] == -B[0]
  //  D[1] == -B[1]
  EXPECT_NEAR(D_O.linearAcceleration[0],
             -B_O.linearAcceleration[0], _tolerance);
  EXPECT_NEAR(D_O.linearAcceleration[1],
             -B_O.linearAcceleration[1], _tolerance);
  //  D[2] centripetal acceleration toward wheel center
  EXPECT_NEAR(D_O.linearAcceleration[2], accelZ, _tolerance);

  // angular acceleration
  //  A==B==0
  EXPECT_EQ(A_O.angularAcceleration, LinearVector::Zero());
  EXPECT_EQ(B_O.angularAcceleration, LinearVector::Zero());
  //  C==D
  EXPECT_TRUE(Equal(C_O.angularAcceleration,
                    D_O.angularAcceleration, _tolerance));
  EXPECT_NEAR(C_O.angularAcceleration[0], -sinPivot * accelY, _tolerance);
  EXPECT_NEAR(C_O.angularAcceleration[1], cosPivot * accelY, _tolerance);
  EXPECT_NEAR(C_O.angularAcceleration[2], 0.0, _tolerance);
}

/////////////////////////////////////////////////
TEST(FrameSemantics_TEST, RelativeFrameData3d)
{
  TestRelativeFrameData<ignition::physics::FeaturePolicy3d>(1e-11, "3d");
}

/////////////////////////////////////////////////
TEST(FrameSemantics_TEST, RelativeFrameData3f)
{
  TestRelativeFrameData<ignition::physics::FeaturePolicy3f>(1e-2, "3f");
}

int main(int argc, char **argv)
{
  // This seed is arbitrary, but we always use the same seed value to ensure
  // that results are reproduceable between runs. You may change this number,
  // but understand that the values generated in these tests will be different
  // each time that you change it. The expected tolerances might need to be
  // adjusted if the seed number is changed.
  ignition::math::Rand::Seed(416);

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
