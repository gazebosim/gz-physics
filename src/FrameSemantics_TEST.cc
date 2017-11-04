/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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

#include <string>
#include <map>
#include <unordered_map>
#include <unordered_set>

#include <gtest/gtest.h>

#include <ignition/physics/FrameSemantics.hh>

#include <ignition/math/Rand.hh>

using ignition::physics::CompleteFrameSemantics;
using ignition::physics::FrameData;
using ignition::physics::FrameID;
using ignition::physics::RelativeFrameData;
using ignition::physics::Pose;
using ignition::physics::Vector;
using ignition::physics::LinearVector;
using ignition::physics::AngularVector;

using ignition::math::Rand;

/////////////////////////////////////////////////
template <typename Scalar, std::size_t Dim>
class EngineLink
{
  public: EngineLink(
    const std::string &_name,
    const FrameData<Scalar, Dim> &_data)
    : name(_name),
      data(_data)
  {
    // Do nothing
  }

  public: std::string name;
  public: FrameData<Scalar, Dim> data;
};

/////////////////////////////////////////////////
template <typename Scalar, std::size_t Dim>
class EngineJoint
{
  public: EngineJoint(
    const std::string &_name,
    const FrameData<Scalar, Dim> &_data)
    : name(_name),
      data(_data)
  {
    // Do nothing
  }

  public: std::string name;
  public: FrameData<Scalar, Dim> data;
};

/////////////////////////////////////////////////
template <typename Scalar, std::size_t Dim>
class TestFrameSemantics final
    : public CompleteFrameSemantics::Engine<Scalar, Dim>
{
  public: using EngineLink = ::EngineLink<Scalar, Dim>;
  public: using EngineJoint = ::EngineLink<Scalar, Dim>;
  public: using FrameData = ::FrameData<Scalar, Dim>;

  /////////////////////////////////////////////////
  public: class Link
    : public virtual CompleteFrameSemantics::Link<Scalar, Dim>
  {
    public: Link(
        ignition::physics::Feature::Engine *const _engine,
        const std::size_t _id,
        const std::shared_ptr<const void> &_ref)
      : Link::BasicObject(_engine, _id, _ref)
    {
      // Do nothing
    }
  };

  /////////////////////////////////////////////////
  public: class Joint
    : public virtual CompleteFrameSemantics::Joint<Scalar, Dim>
  {
    public: Joint(
        ignition::physics::Feature::Engine *const _engine,
        const std::size_t _id,
        const std::shared_ptr<const void> &_ref)
      : Joint::BasicObject(_engine, _id, _ref)
    {
      // Do nothing
    }
  };

  /////////////////////////////////////////////////
  public: TestFrameSemantics()
  {
    // Initialize this with a nullptr object which will act as a stand-in for
    // the world frame (which must have an object ID of 0).
    this->idToEngineLink.push_back(std::weak_ptr<EngineLink>());

    // Set the "last" ID to 1 so that the first object which gets created is
    // given an ID of 1.
    nextID = 1;
  }

  /////////////////////////////////////////////////
  public: FrameData FrameDataRelativeToWorld(
    const FrameID &_id) const override
  {
    const std::size_t id = _id.ID();

    const std::weak_ptr<EngineLink> &weakLink = this->idToEngineLink[id];
    const std::shared_ptr<EngineLink> &link = weakLink.lock();

    return link->data;
  }

  /////////////////////////////////////////////////
  public: Link *CreateLink(
    const std::string &_linkName,
    const FrameData &_frameData)
  {
    typename LinkNameMap::iterator it; bool inserted;
    std::tie(it, inserted) = this->nameToLink.insert(
          std::make_pair(_linkName, nullptr));

    if (inserted)
    {
      std::shared_ptr<EngineLink> newEngineLink =
          std::make_shared<EngineLink>(_linkName, _frameData);

      if(this->idToEngineLink.size() < nextID+1)
        this->idToEngineLink.resize(nextID+1);

      const std::size_t newID = nextID;
      ++nextID;
      Link *newProxyLink = new Link(this, newID, newEngineLink);
      it->second = std::unique_ptr<Link>(newProxyLink);

      this->idToEngineLink[newID] = newEngineLink;
      this->activeEngineLinks.insert(std::make_pair(newID, newEngineLink));
    }

    return it->second.get();
  }

  /////////////////////////////////////////////////
  public: Link *GetLink(const std::string &_linkName)
  {
    typename LinkNameMap::iterator linkEntry = this->nameToLink.find(_linkName);
    if (linkEntry == this->nameToLink.end())
      return nullptr;

    return linkEntry->second.get();
  }

  /////////////////////////////////////////////////
  public: const Link *GetLink(const std::string &_linkName) const
  {
    return const_cast<TestFrameSemantics*>(this)->GetLink(_linkName);
  }

  /////////////////////////////////////////////////
  public: void SetLinkData(const FrameID &_id, const FrameData &_data)
  {
    std::shared_ptr<EngineLink> elink = this->idToEngineLink[_id.ID()].lock();
    if(elink)
      elink->data = _data;
  }

  /////////////////////////////////////////////////
  public: void DeactivateLink(const std::string &_linkName)
  {
    Link *link = this->GetLink(_linkName);

    if (link)
      activeEngineLinks.erase(link->GetFrameID().ID());
  }

  /////////////////////////////////////////////////
  public: Joint *CreateJoint(
    const std::string &_jointName,
    const FrameData &_frameData)
  {
    typename JointNameMap::iterator it; bool inserted;
    std::tie(it, inserted) = this->nameToJoint.insert(
          std::make_pair(_jointName, nullptr));

    if (inserted)
    {
      EngineJoint *newEngineJoint = new EngineJoint(_jointName, _frameData);

      const std::size_t newID = nextID;
      ++nextID;
      Joint *newProxyJoint = new Joint(this, newID, nullptr);
      it->second = std::unique_ptr<Joint>(newProxyJoint);

      this->idToEngineJoint.insert(
            std::make_pair(newID,
                           std::unique_ptr<EngineJoint>(newEngineJoint)));
    }

    return it->second.get();
  }

  /////////////////////////////////////////////////
  public: Joint *GetJoint(const std::string &_jointName)
  {
    typename JointNameMap::iterator jointEntry =
      this->nameToJoint.find(_jointName);
    if (jointEntry == this->nameToJoint.end())
      return nullptr;

    return jointEntry->second.get();
  }

  public: const Joint *GetJoint(const std::string &_jointName) const
  {
    return const_cast<TestFrameSemantics*>(this)->GetJoint(_jointName);
  }

  private: using LinkNameMap = std::map<std::string, std::unique_ptr<Link>>;
  /// \brief A map from a name to a proxy Link object. This allows us to do
  /// lookups based on the link name.
  private: LinkNameMap nameToLink;

  private: using JointNameMap = std::map<std::string, std::unique_ptr<Joint>>;
  /// \brief A map from a name to a proxy Joint object. This allows us to do
  /// lookups based on the joint name.
  private: JointNameMap nameToJoint;

  /// \brief A map from a unique object ID to the corresponding engine link (if
  /// it still exists).
  private: std::vector<std::weak_ptr<EngineLink>> idToEngineLink;

  /// \brief A map from a unique object ID to the corresponding engine joint,
  /// if it still exists.
  ///
  /// Note: We do reference-counting for Links but not for Joints. That way, we
  /// can test both the reference-counted case and the uncounted case.
  private: std::map<std::size_t, std::unique_ptr<EngineJoint>> idToEngineJoint;

  private: using ActiveEngineLinkMap =
      std::unordered_map<std::size_t, std::shared_ptr<EngineLink>>;
  /// \brief A map from a unique ID to the corresponding engine link. All engine
  /// links inside of this map are guaranteed to still exist.
  private: ActiveEngineLinkMap activeEngineLinks;

  private: std::size_t nextID;

};

/////////////////////////////////////////////////
template <typename VectorType>
VectorType RandomVector(const double range)
{
  VectorType v;
  for (std::size_t i = 0; i < VectorType::RowsAtCompileTime; ++i)
    v[i] = Rand::DblUniform(-range, range);

  return v;
}

/////////////////////////////////////////////////
template <typename Scalar, std::size_t Dim>
struct Rotation
{
  /// \brief Randomize the orientation of a 3D pose
  static void Randomize(ignition::physics::Pose<Scalar, Dim> &_pose)
  {
    for (std::size_t i = 0; i < 3; ++i)
    {
      Vector<Scalar, Dim> axis = Vector<Scalar, Dim>::Zero();
      axis[i] = 1.0;
      _pose.rotate(Eigen::AngleAxis<Scalar>(
                     static_cast<Scalar>(Rand::DblUniform(0, 2*M_PI)), axis));
    }
  }

  static bool Equal(
      const Eigen::Matrix<Scalar, Dim, Dim> &_R1,
      const Eigen::Matrix<Scalar, Dim, Dim> &_R2,
      const double _tolerance)
  {
    Eigen::AngleAxis<Scalar> R(_R1.transpose() * _R2);
    if (std::abs(R.angle()) > _tolerance)
    {
      std::cout << "Difference in angle: " << R.angle() << std::endl;
      return false;
    }

    return true;
  }
};

/////////////////////////////////////////////////
template <typename Scalar>
struct Rotation<Scalar, 2>
{
  /// \brief Randomize the orientation of a 2D pose
  static void Randomize(ignition::physics::Pose<Scalar, 2> &_pose)
  {
    _pose.rotate(Eigen::Rotation2D<Scalar>(Rand::DblUniform(0, 2*M_PI)));
  }

  static bool Equal(
      const Eigen::Matrix<Scalar, 2, 2> &_R1,
      const Eigen::Matrix<Scalar, 2, 2> &_R2,
      const double _tolerance)
  {
    // Choose the largest of either 1.0 or the size of the larger angle.
    const double scale =
        std::max(
          static_cast<Scalar>(1.0),
          std::max(
            Eigen::Rotation2D<Scalar>(_R1).angle(),
            Eigen::Rotation2D<Scalar>(_R2).angle()));

    const Eigen::Rotation2D<Scalar> R(_R1.transpose() * _R2);
    if (std::abs(R.angle()/scale) > _tolerance)
    {
      std::cout << "Scaled difference in angle: "
                << R.angle()/scale << " | Difference: " << R.angle()
                << " | Scale: " << scale
                << " | (Tolerance: " << _tolerance << ")" << std::endl;
      return false;
    }

    return true;
  }
};

/////////////////////////////////////////////////
template <typename Scalar, std::size_t Dim>
FrameData<Scalar, Dim> RandomFrameData()
{
  using LinearVector = ::LinearVector<Scalar, Dim>;
  using AngularVector = ::AngularVector<Scalar, Dim>;

  FrameData<Scalar, Dim> data;
  data.pose.translation() = RandomVector<LinearVector>(100.0);
  Rotation<Scalar, Dim>::Randomize(data.pose);
  data.linearVelocity = RandomVector<LinearVector>(10.0);
  data.angularVelocity = RandomVector<AngularVector>(10.0);
  data.linearAcceleration = RandomVector<LinearVector>(1.0);
  data.angularAcceleration = RandomVector<AngularVector>(1.0);

  return data;
}

/////////////////////////////////////////////////
template <typename Scalar, int Dim>
bool Equal(const Vector<Scalar, Dim> &_vec1,
           const Vector<Scalar, Dim> &_vec2,
           const double _tolerance,
           const std::string &_label = "")
{
  // Choose the largest of either 1.0 or the lenght of the longer vector.
  const double scale = std::max(static_cast<Scalar>(1.0),
                                std::max(_vec1.norm(), _vec2.norm()));
  const double diff = (_vec1 - _vec2).norm();
  if (diff/scale <= _tolerance)
    return true;

  std::cout << "Scaled difference in vectors of " << _label << ": "
            << diff/scale << " | Difference: " << diff
            << " | Scale: " << scale
            << " | (Tolerance: " << _tolerance << ")" << std::endl;

  return false;
}

/////////////////////////////////////////////////
template <typename Scalar, int Dim>
bool Equal(const Pose<Scalar, Dim> &_T1,
           const Pose<Scalar, Dim> &_T2,
           const double _tolerance)
{
  if (!Equal(Vector<Scalar, Dim>(_T1.translation()),
             Vector<Scalar, Dim>(_T2.translation()),
             _tolerance, "position"))
    return false;

  if (!Rotation<Scalar, Dim>::Equal(
        _T1.linear(), _T2.linear(), _tolerance))
    return false;

  return true;
}

/////////////////////////////////////////////////
template <typename Scalar, std::size_t Dim>
bool Equal(const FrameData<Scalar, Dim> &_data1,
           const FrameData<Scalar, Dim> &_data2,
           const double _tolerance)
{
  if (!Equal(_data1.pose, _data2.pose, _tolerance))
    return false;

  if (!Equal(_data1.linearVelocity, _data2.linearVelocity,
             _tolerance, "linear velocity"))
    return false;

  if (!Equal(_data1.angularVelocity, _data2.angularVelocity,
             _tolerance, "angular velocity"))
    return false;

  if (!Equal(_data1.linearAcceleration, _data2.linearAcceleration,
             _tolerance, "linear acceleration"))
    return false;

  if (!Equal(_data1.angularAcceleration,
             _data2.angularAcceleration,
             _tolerance))
    return false;

  return true;
}

/////////////////////////////////////////////////
template <typename Scalar, std::size_t Dim>
void TestRelativeFrames(const double _tolerance)
{
  using FrameData = ::FrameData<Scalar, Dim>;
  using RelativeFrameData = ::RelativeFrameData<Scalar, Dim>;

  // Instantiate a class that provides Frame Semantics
  TestFrameSemantics<Scalar, Dim> fs;

  // Note: The World Frame is often designated by the letter O

  // Create Frame A
  const FrameData T_A = RandomFrameData<Scalar, Dim>();
  const FrameID A = fs.CreateLink("A", T_A)->GetFrameID();

  // Create Frame B
  const FrameData T_B = RandomFrameData<Scalar, Dim>();
  const FrameID B = fs.CreateLink("B", T_B)->GetFrameID();

  RelativeFrameData B_T_B = RelativeFrameData(B);
  EXPECT_TRUE(Equal(T_B, fs.Resolve(B_T_B, FrameID::World()), _tolerance));

  // Create a frame relative to A which is equivalent to B
  const RelativeFrameData A_T_B =
      RelativeFrameData(A, fs.GetLink("B")->FrameDataRelativeTo(A));

  // When A_T_B is expressed with respect to the world, it should be equivalent
  // to Frame B
  EXPECT_TRUE(Equal(T_B, fs.Resolve(A_T_B, FrameID::World()), _tolerance));

  const RelativeFrameData O_T_B = RelativeFrameData(FrameID::World(), T_B);

  // When O_T_B is expressed with respect to A, it should be equivalent to
  // A_T_B
  EXPECT_TRUE(Equal(A_T_B.RelativeToParent(),
                    fs.Resolve(O_T_B, A), _tolerance));

  // Define a new frame (C), relative to B
  const RelativeFrameData B_T_C =
      RelativeFrameData(B, RandomFrameData<Scalar, Dim>());

  // Reframe C with respect to the world
  const RelativeFrameData O_T_C = fs.Reframe(B_T_C, FrameID::World());

  // Also, compute its raw transform with respect to the world
  const FrameData T_C = fs.Resolve(B_T_C, FrameID::World());

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
  TestRelativeFrames<double, 3>(1e-11);
}

/////////////////////////////////////////////////
TEST(FrameSemantics_TEST, RelativeFrames2d)
{
  TestRelativeFrames<double, 2>(1e-14);
}

/////////////////////////////////////////////////
TEST(FrameSemantics_TEST, RelativeFrames3f)
{
  TestRelativeFrames<float, 3>(1e-3);
}

/////////////////////////////////////////////////
TEST(FrameSemantics_TEST, RelativeFrames2f)
{
  TestRelativeFrames<float, 2>(1e-3);
}

/////////////////////////////////////////////////
template <typename Scalar, std::size_t Dim>
void TestFrameID(const double _tolerance)
{
  using Link = typename TestFrameSemantics<Scalar, Dim>::Link;
  using Joint = typename TestFrameSemantics<Scalar, Dim>::Joint;
  using FrameData = ::FrameData<Scalar, Dim>;

  // We test FrameID in this unit test, because the FrameSemantics interface is
  // needed in order to produce FrameIDs.
  FrameID world = FrameID::World();

  // The world FrameID is always considered to be "reference counted", because
  // it must always be treated as a valid ID.
  EXPECT_TRUE(world.IsReferenceCounted());

  TestFrameSemantics<Scalar, Dim> fs;

  FrameData dataA = RandomFrameData<Scalar, Dim>();
  Link *linkA = fs.CreateLink("A", dataA);

  EXPECT_TRUE(Equal(dataA, linkA->FrameDataRelativeTo(world), _tolerance));

  FrameID A = linkA->GetFrameID();
  EXPECT_TRUE(A.IsReferenceCounted());
  EXPECT_EQ(A, fs.GetLink("A")->GetFrameID());
  EXPECT_EQ(A, linkA->GetFrameID());

  FrameData dataJ1 = RandomFrameData<Scalar, Dim>();
  Joint *joint1 = fs.CreateJoint("J1", dataJ1);
  EXPECT_TRUE(joint1);

  FrameID J1 = joint1->GetFrameID();
  EXPECT_FALSE(J1.IsReferenceCounted());
  EXPECT_EQ(J1, fs.GetJoint("J1")->GetFrameID());
  EXPECT_EQ(J1, joint1->GetFrameID());
}

/////////////////////////////////////////////////
TEST(FrameSemantics_TEST, FrameID3d)
{
  TestFrameID<double, 3>(1e-16);
}

/////////////////////////////////////////////////
TEST(FrameSemantics_TEST, FrameID2d)
{
  TestFrameID<double, 2>(1e-16);
}

/////////////////////////////////////////////////
TEST(FrameSemantics_TEST, FrameID3f)
{
  TestFrameID<float, 3>(1e-16);
}

/////////////////////////////////////////////////
TEST(FrameSemantics_TEST, FrameID2f)
{
  TestFrameID<float, 2>(1e-16);
}

int main(int argc, char **argv)
{
  // This seed is arbitrary, but we always use the same seed value to ensure
  // that results are reproduceable between runs. You may change this number,
  // but understand that the values generated in these tests will be different
  // each time that you change it. The acceptable tolerances might need to be
  // adjusted if the seed number is changed.
  ignition::math::Rand::Seed(416);

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
