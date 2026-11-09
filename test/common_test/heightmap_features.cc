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

#include <cmath>
#include <cstddef>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/common/geospatial/Dem.hh>
#include <gz/common/geospatial/HeightmapData.hh>
#include <gz/common/geospatial/ImageHeightmap.hh>
#include <gz/math/Helpers.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/eigen3/Conversions.hh>
#include <gz/plugin/Loader.hh>

#include "test/Resources.hh"
#include "test/TestLibLoader.hh"
#include "Worlds.hh"

#include <gz/physics/BoxShape.hh>
#include <gz/physics/FindFeatures.hh>
#include <gz/physics/FixedJoint.hh>
#include <gz/physics/ForwardStep.hh>
#include <gz/physics/FrameSemantics.hh>
#include <gz/physics/GetBoundingBox.hh>
#include <gz/physics/GetContacts.hh>
#include <gz/physics/GetEntities.hh>
#include <gz/physics/Joint.hh>
#include <gz/physics/RequestEngine.hh>
#include <gz/physics/heightmap/HeightmapShape.hh>
#include <gz/physics/sdf/ConstructWorld.hh>

#include <sdf/Root.hh>

template <class T>
class HeightmapFeaturesTest:
  public testing::Test, public gz::physics::TestLibLoader
{
  // Documentation inherited
  public: void SetUp() override
  {
    gz::common::Console::SetVerbosity(4);

    loader.LoadLib(HeightmapFeaturesTest::GetLibToTest());

    // TODO(ahcorde): We should also run the 3f, 2d, and 2f variants of
    // FindFeatures
    pluginNames = gz::physics::FindFeatures3d<T>::From(loader);
    if (pluginNames.empty())
    {
      std::cerr << "No plugins with required features found in "
                << GetLibToTest() << std::endl;
      GTEST_SKIP();
    }
  }

  public: std::set<std::string> pluginNames;
  public: gz::plugin::Loader loader;
};

struct HeightmapFeaturesList : gz::physics::FeatureList<
  gz::physics::sdf::ConstructSdfWorld,
  gz::physics::GetModelFromWorld,
  gz::physics::GetLinkFromModel,
  gz::physics::GetShapeFromLink,
  gz::physics::LinkFrameSemantics,
  gz::physics::ForwardStep,
  gz::physics::AttachFixedJointFeature,
  gz::physics::heightmap::GetHeightmapShapeProperties,
  gz::physics::heightmap::AttachHeightmapShapeFeature
> { };

/// \brief Casting shapes to boxes is optional for engines with heightmaps, so
/// it is not part of HeightmapFeaturesList, which would make the whole suite
/// skip on engines that don't have it.
using HeightmapBoxCastFeatures = gz::physics::FeatureList<
  HeightmapFeaturesList,
  gz::physics::BoxShapeCast
>;

using HeightmapFeaturesTestTypes =
  ::testing::Types<HeightmapFeaturesList>;
TYPED_TEST_SUITE(HeightmapFeaturesTest,
                 HeightmapFeaturesTestTypes);

/////////////////////////////////////////////////
/// \brief Construct a world from SDF with an engine of a plugin.
/// \param[in] _plugin Physics engine plugin.
/// \param[in] _worldSdf SDF of the world, or empty for
/// common_test::worlds::kHeightmapSdf.
/// \return The world, or nullptr if the engine doesn't have the features of
/// FeatureListT, or, which fails the test, if the world can't be constructed.
template <class FeatureListT = HeightmapFeaturesList>
gz::physics::World3dPtr<FeatureListT> ConstructWorld(
    const gz::plugin::PluginPtr &_plugin, const std::string &_worldSdf = "")
{
  auto engine = gz::physics::RequestEngine3d<FeatureListT>::From(_plugin);
  if (nullptr == engine)
    return nullptr;

  sdf::Root root;
  const sdf::Errors errors = _worldSdf.empty() ?
      root.Load(common_test::worlds::kHeightmapSdf) :
      root.LoadSdfString(_worldSdf);
  if (!errors.empty())
  {
    ADD_FAILURE() << errors.front();
    return nullptr;
  }
  auto world = engine->ConstructWorld(*root.WorldByIndex(0));
  EXPECT_NE(nullptr, world);
  return world;
}

/////////////////////////////////////////////////
/// \brief Step a world.
template <class WorldPtrT>
void StepWorld(const WorldPtrT &_world, std::size_t _steps)
{
  gz::physics::ForwardStep::Output output;
  gz::physics::ForwardStep::State state;
  gz::physics::ForwardStep::Input input;
  for (std::size_t i = 0; i < _steps; ++i)
  {
    _world->Step(output, state, input);
  }
}

/////////////////////////////////////////////////
/// \brief Check where the link of a model rests.
/// \param[in] _xyTolerance Tolerance of the x and y coordinates. The z
/// coordinate is checked more strictly, as it is what the heightmap sets.
template <class WorldPtrT>
void ExpectRestingAt(const WorldPtrT &_world, const std::string &_model,
    const Eigen::Vector3d &_expected, double _xyTolerance)
{
  auto link = _world->GetModel(_model)->GetLink("link");
  ASSERT_NE(nullptr, link);
  const Eigen::Vector3d pos =
      link->FrameDataRelativeToWorld().pose.translation();
  EXPECT_NEAR(_expected.x(), pos.x(), _xyTolerance) << _model;
  EXPECT_NEAR(_expected.y(), pos.y(), _xyTolerance) << _model;
  EXPECT_NEAR(_expected.z(), pos.z(), 5e-3) << _model;
}

/////////////////////////////////////////////////
/// \brief Check that the bounding box of a heightmap shape is as large as the
/// size it reports and centered on its x and y.
template <class ShapePtrT, class HeightmapPtrT>
void ExpectBoundingBoxExtent(const ShapePtrT &_shape,
    const HeightmapPtrT &_heightmap, double _tolerance)
{
  const auto box = _shape->GetAxisAlignedBoundingBox();
  const Eigen::Vector3d size = _heightmap->GetSize();
  EXPECT_NEAR(size.x(), box.max().x() - box.min().x(), _tolerance);
  EXPECT_NEAR(size.y(), box.max().y() - box.min().y(), _tolerance);
  EXPECT_NEAR(size.z(), box.max().z() - box.min().z(), _tolerance);
  EXPECT_NEAR(0, box.min().x() + box.max().x(), _tolerance);
  EXPECT_NEAR(0, box.min().y() + box.max().y(), _tolerance);
}

/////////////////////////////////////////////////
/// \brief Attach an image heightmap and a DEM to a link and check their
/// sizes.
template <class LinkPtrT>
void AttachImageAndDem(const LinkPtrT &link)
{
  const gz::math::Pose3d pose(0, 0, 0.2, 0, 0, 0);

  gz::common::ImageHeightmap data;
  EXPECT_EQ(0, data.Load(gz::physics::test::resources::kHeightmapBowlPng));

  const gz::math::Vector3d size({129, 129, 10});
  auto heightmapShape = link->AttachHeightmapShape("heightmap", data,
      gz::math::eigen3::convert(pose),
      gz::math::eigen3::convert(size));
  ASSERT_NE(nullptr, heightmapShape);

  EXPECT_NEAR(size.X(), heightmapShape->GetSize()[0], 1e-6);
  EXPECT_NEAR(size.Y(), heightmapShape->GetSize()[1], 1e-6);
  EXPECT_NEAR(size.Z(), heightmapShape->GetSize()[2], 1e-6);

  auto heightmapShapeGeneric = link->GetShape("heightmap");
  ASSERT_NE(nullptr, heightmapShapeGeneric);
  auto heightmapShapeRecast = heightmapShapeGeneric->CastToHeightmapShape();
  ASSERT_NE(nullptr, heightmapShapeRecast);
  EXPECT_NEAR(size.X(), heightmapShapeRecast->GetSize()[0], 1e-6);
  EXPECT_NEAR(size.Y(), heightmapShapeRecast->GetSize()[1], 1e-6);
  EXPECT_NEAR(size.Z(), heightmapShapeRecast->GetSize()[2], 1e-6);

  gz::common::Dem dem;
  EXPECT_EQ(0, dem.Load(gz::physics::test::resources::kVolcanoTif));

  gz::math::Vector3d sizeDem;
  sizeDem.X(dem.WorldWidth());
  sizeDem.Y(dem.WorldHeight());
  sizeDem.Z(dem.MaxElevation() - dem.MinElevation());

  auto demShape = link->AttachHeightmapShape("dem", dem,
      gz::math::eigen3::convert(pose),
      gz::math::eigen3::convert(sizeDem));
  ASSERT_NE(nullptr, demShape);

  // there is a loss in precision with large dems since heightmaps use floats
  EXPECT_NEAR(sizeDem.X(), demShape->GetSize()[0], 1e-3);
  EXPECT_NEAR(sizeDem.Y(), demShape->GetSize()[1], 1e-3);
  EXPECT_NEAR(sizeDem.Z(), demShape->GetSize()[2], 1e-6);

  auto demShapeGeneric = link->GetShape("dem");
  ASSERT_NE(nullptr, demShapeGeneric);
  auto demShapeRecast = demShapeGeneric->CastToHeightmapShape();
  ASSERT_NE(nullptr, demShapeRecast);
  EXPECT_NEAR(sizeDem.X(), demShapeRecast->GetSize()[0], 1e-3);
  EXPECT_NEAR(sizeDem.Y(), demShapeRecast->GetSize()[1], 1e-3);
  EXPECT_NEAR(sizeDem.Z(), demShapeRecast->GetSize()[2], 1e-6);
}

/////////////////////////////////////////////////
TYPED_TEST(HeightmapFeaturesTest, AttachHeightmap)
{
  for (const std::string &name : this->pluginNames)
  {
    std::cout << "Testing plugin: " << name << std::endl;
    auto world = ConstructWorld(this->loader.Instantiate(name));
    ASSERT_NE(nullptr, world);
    auto link = world->GetModel("terrain")->GetLink("link");
    ASSERT_NE(nullptr, link);

    AttachImageAndDem(link);

    // A shape that is not a heightmap can't be cast to one
    auto boxShape = world->GetModel("box_on_slope")->GetLink("link")->GetShape(
        "collision");
    ASSERT_NE(nullptr, boxShape);
    EXPECT_EQ(nullptr, boxShape->CastToHeightmapShape());
  }
}

/////////////////////////////////////////////////
// Heightmaps can't be cast to boxes. Skipped for engines that can't cast
// shapes to boxes.
TYPED_TEST(HeightmapFeaturesTest, HeightmapIsNotBoxShape)
{
  for (const std::string &name : this->pluginNames)
  {
    std::cout << "Testing plugin: " << name << std::endl;
    auto world = ConstructWorld<HeightmapBoxCastFeatures>(
        this->loader.Instantiate(name));
    if (nullptr == world)
    {
      GTEST_SKIP() << name << " does not support BoxShapeCast";
    }
    auto link = world->GetModel("terrain")->GetLink("link");
    ASSERT_NE(nullptr, link);

    // Attaches the shapes "heightmap" and "dem"
    AttachImageAndDem(link);
    EXPECT_EQ(nullptr, link->GetShape("heightmap")->CastToBoxShape());
    EXPECT_EQ(nullptr, link->GetShape("dem")->CastToBoxShape());
  }
}

/////////////////////////////////////////////////
/// \brief Bounding boxes are optional for engines with heightmaps, like
/// HeightmapBoxCastFeatures.
using HeightmapBoundingBoxFeatures = gz::physics::FeatureList<
  HeightmapFeaturesList,
  gz::physics::GetShapeBoundingBox
>;

/////////////////////////////////////////////////
// The bounding box of a heightmap is its extent. Skipped for engines without
// bounding boxes.
TYPED_TEST(HeightmapFeaturesTest, HeightmapBoundingBox)
{
  for (const std::string &name : this->pluginNames)
  {
    std::cout << "Testing plugin: " << name << std::endl;
    auto world = ConstructWorld<HeightmapBoundingBoxFeatures>(
        this->loader.Instantiate(name));
    if (nullptr == world)
    {
      GTEST_SKIP() << name << " does not support GetShapeBoundingBox";
    }
    auto link = world->GetModel("terrain")->GetLink("link");
    ASSERT_NE(nullptr, link);

    // Attaches the shapes "heightmap" and "dem"
    AttachImageAndDem(link);
    auto heightmap = link->GetShape("heightmap");
    auto dem = link->GetShape("dem");
    ExpectBoundingBoxExtent(heightmap, heightmap->CastToHeightmapShape(),
        1e-6);
    // there is a loss in precision with large dems since heightmaps use floats
    ExpectBoundingBoxExtent(dem, dem->CastToHeightmapShape(), 1e-3);
  }
}

/////////////////////////////////////////////////
/// \brief Height of the ramp heightmap attached by AttachRamp.
constexpr double kRampHeight = 2.0;
/// \brief Size of the ramp heightmap along x, and by default along y.
constexpr double kRampSize = 34.0;
/// \brief Height of the ramp heightmap's origin above the link.
constexpr double kRampZ = 0.3;

/////////////////////////////////////////////////
/// \brief Attach the ramp heightmap to a link.
/// The 18x18 image has a ramp along +x in its top half (+y), going from
/// pixel value 0.2 at -x to 1.0 at +x. Its bottom half (-y) has a plateau
/// at 1.0 for -x and a plateau at 0.2 for +x.
/// \param[in] _sizeY Size of the heightmap along y.
template <class LinkPtrT>
auto AttachRamp(const LinkPtrT &_link, double _sizeY = kRampSize)
{
  gz::common::ImageHeightmap data;
  EXPECT_EQ(0, data.Load(gz::physics::test::resources::kHeightmapRampPng));
  return _link->AttachHeightmapShape("heightmap", data,
      Eigen::Isometry3d(Eigen::Translation3d(0, 0, kRampZ)),
      Eigen::Vector3d(kRampSize, _sizeY, kRampHeight), 2);
}

/////////////////////////////////////////////////
// Drop objects on a heightmap and check that they come to rest at the
// expected heights. This guards the orientation, scaling and offset of the
// heightmap.
TYPED_TEST(HeightmapFeaturesTest, ObjectsRestOnHeightmap)
{
  for (const std::string &name : this->pluginNames)
  {
    std::cout << "Testing plugin: " << name << std::endl;
    auto world = ConstructWorld(this->loader.Instantiate(name));
    ASSERT_NE(nullptr, world);
    auto terrainLink = world->GetModel("terrain")->GetLink("link");
    ASSERT_NE(nullptr, terrainLink);

    auto heightmap = AttachRamp(terrainLink);
    ASSERT_NE(nullptr, heightmap);
    // The size along z is the height range of the terrain, which is less
    // than kRampHeight because the darkest pixel is not 0.
    EXPECT_NEAR(kRampSize, heightmap->GetSize()[0], 1e-6);
    EXPECT_NEAR(kRampSize, heightmap->GetSize()[1], 1e-6);
    EXPECT_NEAR(0.8 * kRampHeight, heightmap->GetSize()[2], 1e-5);

    StepWorld(world, 2000);

    const double halfBox = 0.25;
    const double radius = 0.25;
    // Slope of the ramp along x
    const double slope = 0.8 * kRampHeight / kRampSize;

    // Low plateau at +x, -y
    ExpectRestingAt(world, "sphere_low",
        {8.5, -8.5, kRampZ + 0.2 * kRampHeight + radius}, 0.05);
    // High plateau at -x, -y
    ExpectRestingAt(world, "sphere_high",
        {-8.5, -8.5, kRampZ + 1.0 * kRampHeight + radius}, 0.05);
    // Middle of the ramp at +y
    const double boxZ =
        kRampZ + 0.6 * kRampHeight + halfBox / std::cos(std::atan(slope));
    ExpectRestingAt(world, "box_on_slope", {0, 8.5, boxZ}, 0.05);
  }
}

/////////////////////////////////////////////////
/// \brief Construct a world from SDF and attach an image heightmap and a DEM
/// to a link of its model "model".
/// \param[in] _plugin Physics engine plugin.
/// \param[in] _worldSdf SDF of the world.
/// \param[in] _link Name of the link that gets the heightmaps.
/// \param[in] _parent If not empty, name of the link of the model that _link
/// is attached to with a fixed joint, before attaching the heightmaps.
/// \param[in] _supported False if the engine is expected to refuse the
/// heightmaps, which it does by returning a null shape without breaking the
/// world.
void AttachToLinkOfSdfWorld(const gz::plugin::PluginPtr &_plugin,
    const std::string &_worldSdf, const std::string &_link,
    const std::string &_parent = "", bool _supported = true)
{
  auto world = ConstructWorld(_plugin, _worldSdf);
  ASSERT_NE(nullptr, world);
  auto model = world->GetModel("model");
  ASSERT_NE(nullptr, model);
  auto link = model->GetLink(_link);
  ASSERT_NE(nullptr, link);

  if (!_parent.empty())
  {
    auto parent = model->GetLink(_parent);
    ASSERT_NE(nullptr, parent);
    ASSERT_NE(nullptr, link->AttachFixedJoint(parent, "heightmap_joint"));
  }

  if (_supported)
  {
    AttachImageAndDem(link);
    return;
  }

  gz::common::ImageHeightmap data;
  ASSERT_EQ(0, data.Load(gz::physics::test::resources::kHeightmapBowlPng));
  EXPECT_EQ(nullptr, link->AttachHeightmapShape("heightmap", data,
      Eigen::Isometry3d::Identity(), Eigen::Vector3d(129, 129, 10)));
  EXPECT_EQ(nullptr, link->GetShape("heightmap"));

  // The world still steps: gravity pulls the link down.
  StepWorld(world, 100);
  EXPECT_GT(0.0, link->FrameDataRelativeToWorld().pose.translation().z());
}

/////////////////////////////////////////////////
// Attach heightmaps to a link that is not static but is held to the world by
// a fixed joint. The link also has a heightmap collision in SDF, which is
// skipped while the world is built, since the heightmaps are attached
// afterwards, the way gz-sim does it.
TYPED_TEST(HeightmapFeaturesTest, AttachHeightmapToFixedLink)
{
  const std::string worldSdf = R"(<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="default">
    <model name="model">
      <link name="link">
        <inertial>
          <mass>1.0</mass>
          <inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia>
        </inertial>
        <collision name="sdf_heightmap">
          <geometry>
            <heightmap>
              <uri>)" + std::string(
      gz::physics::test::resources::kHeightmapBowlPng) + R"(</uri>
              <size>129 129 10</size>
            </heightmap>
          </geometry>
        </collision>
      </link>
      <joint name="world_joint" type="fixed">
        <parent>world</parent>
        <child>link</child>
      </joint>
    </model>
  </world>
</sdf>)";

  for (const std::string &name : this->pluginNames)
  {
    std::cout << "Testing plugin: " << name << std::endl;
    AttachToLinkOfSdfWorld(this->loader.Instantiate(name), worldSdf, "link");
  }
}

/////////////////////////////////////////////////
// Attach heightmaps to a link that moves: it is fixed to a link with a
// prismatic joint. Only dartsim supports it: the other engines refuse the
// heightmaps and keep stepping.
TYPED_TEST(HeightmapFeaturesTest, AttachHeightmapToMovingLink)
{
  const std::string worldSdf = R"(<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="default">
    <model name="model">
      <link name="link">
        <inertial>
          <mass>1.0</mass>
          <inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia>
        </inertial>
      </link>
      <link name="child">
        <inertial>
          <mass>1.0</mass>
          <inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia>
        </inertial>
      </link>
      <link name="heightmap_link">
        <inertial>
          <mass>1.0</mass>
          <inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia>
        </inertial>
      </link>
      <joint name="revolute" type="revolute">
        <parent>world</parent>
        <child>link</child>
        <axis><xyz>0 0 1</xyz></axis>
      </joint>
      <joint name="prismatic" type="prismatic">
        <parent>link</parent>
        <child>child</child>
        <axis><xyz>0 0 1</xyz></axis>
      </joint>
    </model>
  </world>
</sdf>)";

  for (const std::string &name : this->pluginNames)
  {
    std::cout << "Testing plugin: " << name << std::endl;
    AttachToLinkOfSdfWorld(this->loader.Instantiate(name), worldSdf,
        "heightmap_link", "child",
        gz::physics::TestLibLoader::EngineInList(name, {"dartsim"}));
  }
}

/////////////////////////////////////////////////
/// \brief Heightmap with four flat quadrants, high in +x and +y, so a wrong
/// orientation shows up as a sphere resting at the wrong height.
class QuadrantHeightmap : public gz::common::HeightmapData
{
  /// \brief Height of the lowest quadrant, without the offset.
  public: static constexpr double kMinHeight = 0.1;

  /// \brief Height of the highest quadrant, without the offset.
  public: static constexpr double kMaxHeight = 0.7;

  /// \param[in] _offset Added to the heights, which are negative if it is
  /// below -kMaxHeight.
  /// \param[in] _width Width and height of the data in pixels, which is the
  /// number of samples per side.
  public: explicit QuadrantHeightmap(double _offset = 0.0,
      unsigned int _width = 17)
      : offset(_offset), width(_width)
  {
  }

  /// \brief Height of the quadrant that contains (x, y).
  public: double HeightAt(double _x, double _y) const
  {
    return this->offset + kMinHeight + (_x > 0 ? 0.2 : 0.0) +
           (_y > 0 ? 0.4 : 0.0);
  }

  public: void FillHeightMap(int, unsigned int _vertSize,
      const gz::math::Vector3d &_size, const gz::math::Vector3d &,
      bool _flipY, std::vector<float> &_heights) const override
  {
    _heights.resize(_vertSize * _vertSize);
    for (unsigned int r = 0; r < _vertSize; ++r)
    {
      // Without flipping, row 0 is the +y edge, like the top row of a
      // heightmap image.
      const unsigned int imageRow = _flipY ? _vertSize - 1 - r : r;
      const double y = _size.Y() / 2 - imageRow * _size.Y() / (_vertSize - 1);
      for (unsigned int c = 0; c < _vertSize; ++c)
      {
        const double x = -_size.X() / 2 + c * _size.X() / (_vertSize - 1);
        _heights[r * _vertSize + c] = static_cast<float>(this->HeightAt(x, y));
      }
    }
  }

  public: unsigned int Height() const override { return this->width; }
  public: unsigned int Width() const override { return this->width; }
  public: float MaxElevation() const override
  {
    return static_cast<float>(kMaxHeight + this->offset);
  }
  public: float MinElevation() const override
  {
    return static_cast<float>(kMinHeight + this->offset);
  }
  public: std::string Filename() const override { return "quadrants"; }

  private: double offset;
  private: unsigned int width;
};

/////////////////////////////////////////////////
/// \brief Heightmap whose samples all have the same height, so that its
/// range of heights is 0.
class FlatHeightmap : public gz::common::HeightmapData
{
  public: explicit FlatHeightmap(double _height) : height(_height) {}

  public: double HeightAt(double, double) const { return this->height; }

  public: void FillHeightMap(int, unsigned int _vertSize,
      const gz::math::Vector3d &, const gz::math::Vector3d &, bool,
      std::vector<float> &_heights) const override
  {
    _heights.assign(_vertSize * _vertSize, static_cast<float>(this->height));
  }

  public: unsigned int Height() const override { return 17; }
  public: unsigned int Width() const override { return 17; }
  public: float MaxElevation() const override
  {
    return static_cast<float>(this->height);
  }
  public: float MinElevation() const override
  {
    return static_cast<float>(this->height);
  }
  public: std::string Filename() const override { return "flat"; }

  private: double height;
};

/////////////////////////////////////////////////
/// \brief SDF of a model with a link of 1 kg that has one collision.
/// \param[in] _geometry SDF of the geometry of the collision.
/// \param[in] _inertia Diagonal of the inertia of the link.
std::string ProbeModel(const std::string &_name,
    const gz::math::Pose3d &_pose, const std::string &_geometry,
    const gz::math::Vector3d &_inertia)
{
  std::ostringstream sdf;
  sdf << R"(
    <model name=")" << _name << R"(">
      <pose>)" << _pose << R"(</pose>
      <link name="link">
        <inertial>
          <mass>1.0</mass>
          <inertia><ixx>)" << _inertia.X() << "</ixx><iyy>" << _inertia.Y()
      << "</iyy><izz>" << _inertia.Z() << R"(</izz></inertia>
        </inertial>
        <collision name="collision">
          <geometry>)" << _geometry << R"(</geometry>
        </collision>
      </link>
    </model>)";
  return sdf.str();
}

/////////////////////////////////////////////////
/// \brief SDF of a sphere with a radius of 0.1 m.
std::string SphereModel(const std::string &_name, double _x, double _y,
    double _z = 1.0)
{
  return ProbeModel(_name, gz::math::Pose3d(_x, _y, _z, 0, 0, 0),
      "<sphere><radius>0.1</radius></sphere>",
      gz::math::Vector3d(0.004, 0.004, 0.004));
}

/////////////////////////////////////////////////
/// \brief SDF of a box.
std::string BoxModel(const std::string &_name, const gz::math::Pose3d &_pose,
    const gz::math::Vector3d &_size)
{
  std::ostringstream geometry;
  geometry << "<box><size>" << _size << "</size></box>";
  const gz::math::Vector3d squared = _size * _size;
  return ProbeModel(_name, _pose, geometry.str(), gz::math::Vector3d(
      squared.Y() + squared.Z(), squared.X() + squared.Z(),
      squared.X() + squared.Y()) / 12);
}

/////////////////////////////////////////////////
/// \brief Beginning of the SDF of a world, up to its models.
const char kWorldBegin[] = R"(<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="default">)";

/////////////////////////////////////////////////
/// \brief End of the SDF of a world, after its models.
const char kWorldEnd[] = R"(
  </world>
</sdf>)";

/////////////////////////////////////////////////
/// \brief SDF of a static model "ground" whose link "link" has no
/// collisions, for heightmaps to be attached to.
const char kStaticGround[] = R"(
    <model name="ground">
      <static>true</static>
      <link name="link"/>
    </model>)";

/////////////////////////////////////////////////
/// \brief Drop a sphere near each of the four corners of a heightmap and
/// check that it rests on the height that the heightmap has there.
/// \param[in] _plugin Physics engine plugin.
/// \param[in] _data Heightmap, which has a HeightAt(x, y) function.
/// \param[in] _size Size of the heightmap along x and y.
/// \param[in] _xy The spheres are dropped at (+-_xy, +-_xy) in the frame of
/// the heightmap. The default is off the vertices of the grids of these
/// tests, where the cells around a sphere make it roll on some engines and
/// platforms.
/// \param[in] _pose Pose of the heightmap in its link frame. It may only
/// rotate about z, for the spheres to rest.
/// \param[in] _ground SDF of the model "ground", whose link "link" gets the
/// heightmap and whose link frame is the world frame.
template <class HeightmapT>
void ExpectSpheresRestOnHeightmap(const gz::plugin::PluginPtr &_plugin,
    const HeightmapT &_data, double _size = 4.0, double _xy = 1.1,
    const Eigen::Isometry3d &_pose = Eigen::Isometry3d::Identity(),
    const std::string &_ground = kStaticGround)
{
  const double radius = 0.1;
  // Where the spheres come to rest
  std::vector<Eigen::Vector3d> spheres;
  for (const auto &[x, y] : std::vector<std::pair<double, double>>{
           {_xy, _xy}, {-_xy, _xy}, {_xy, -_xy}, {-_xy, -_xy}})
  {
    spheres.push_back(_pose * Eigen::Vector3d(x, y, _data.HeightAt(x, y)) +
        Eigen::Vector3d(0, 0, radius));
  }

  std::string worldSdf = kWorldBegin + _ground;
  for (std::size_t i = 0; i < spheres.size(); ++i)
  {
    worldSdf += SphereModel("sphere" + std::to_string(i), spheres[i].x(),
        spheres[i].y(), spheres[i].z() + 0.05);
  }
  worldSdf += R"(
  </world>
</sdf>)";

  auto world = ConstructWorld(_plugin, worldSdf);
  ASSERT_NE(nullptr, world);

  auto ground = world->GetModel("ground")->GetLink("link");
  ASSERT_NE(nullptr, ground);
  // The fake heightmaps ignore the size along z.
  ASSERT_NE(nullptr, ground->AttachHeightmapShape("heightmap", _data,
      _pose, Eigen::Vector3d(_size, _size, 1.0)));

  StepWorld(world, 2000);

  // The spheres may roll a little, which doesn't change their height.
  for (std::size_t i = 0; i < spheres.size(); ++i)
    ExpectRestingAt(world, "sphere" + std::to_string(i), spheres[i], 0.05);
}

/////////////////////////////////////////////////
// Spheres dropped on each quadrant of a heightmap that is not an image come
// to rest on that quadrant's height, which checks the heightmap's
// orientation and its height.
TYPED_TEST(HeightmapFeaturesTest, SpheresRestOnQuadrants)
{
  for (const std::string &name : this->pluginNames)
  {
    std::cout << "Testing plugin: " << name << std::endl;
    ExpectSpheresRestOnHeightmap(
        this->loader.Instantiate(name), QuadrantHeightmap());
  }
}

/////////////////////////////////////////////////
// The samples of a heightmap can be below z = 0, where the engines offset
// the heightmap by its lowest sample.
TYPED_TEST(HeightmapFeaturesTest, SpheresRestOnNegativeHeightmap)
{
  for (const std::string &name : this->pluginNames)
  {
    std::cout << "Testing plugin: " << name << std::endl;
    // Samples from -0.9 to -0.3
    ExpectSpheresRestOnHeightmap(this->loader.Instantiate(name),
        QuadrantHeightmap(-1.0));
  }
}

/////////////////////////////////////////////////
// A heightmap without any relief, above and below z = 0.
TYPED_TEST(HeightmapFeaturesTest, SpheresRestOnFlatHeightmap)
{
  for (const std::string &name : this->pluginNames)
  {
    std::cout << "Testing plugin: " << name << std::endl;
    ExpectSpheresRestOnHeightmap(
        this->loader.Instantiate(name), FlatHeightmap(0.3));
    ExpectSpheresRestOnHeightmap(
        this->loader.Instantiate(name), FlatHeightmap(-0.3));
  }
}

/////////////////////////////////////////////////
// A heightmap with an even number of samples per side, which has no sample
// in the middle.
TYPED_TEST(HeightmapFeaturesTest, SpheresRestOnEvenGrid)
{
  for (const std::string &name : this->pluginNames)
  {
    std::cout << "Testing plugin: " << name << std::endl;
    ExpectSpheresRestOnHeightmap(
        this->loader.Instantiate(name), QuadrantHeightmap(0.0, 16));
  }
}

/////////////////////////////////////////////////
/// \brief Pose of a heightmap in its link frame, rotated about z by a
/// quarter turn, which moves each quadrant of a QuadrantHeightmap to the next
/// one, and offset.
const Eigen::Isometry3d kRotatedPose = Eigen::Translation3d(0.5, -0.3, 0.2) *
    Eigen::AngleAxisd(GZ_PI / 2, Eigen::Vector3d::UnitZ());

/////////////////////////////////////////////////
// A heightmap rotated and offset in its link frame, below z = 0, where the
// engines offset it by its lowest sample.
TYPED_TEST(HeightmapFeaturesTest, SpheresRestOnRotatedHeightmap)
{
  for (const std::string &name : this->pluginNames)
  {
    std::cout << "Testing plugin: " << name << std::endl;
    ExpectSpheresRestOnHeightmap(this->loader.Instantiate(name),
        QuadrantHeightmap(-1.0), 4.0, 1.1, kRotatedPose);
  }
}

/////////////////////////////////////////////////
// The link of a heightmap has an inertial pose, which some engines use as
// the frame of the link's body. The heightmap is in the link frame. The
// link is static, or not static but fixed to the world.
TYPED_TEST(HeightmapFeaturesTest, SpheresRestOnHeightmapOfLinkWithInertialPose)
{
  const std::string link = R"(
      <link name="link">
        <inertial>
          <pose>0.4 -0.3 0.2 0.1 0.2 0.3</pose>
          <mass>1.0</mass>
          <inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia>
        </inertial>
      </link>)";
  const std::string staticGround = R"(
    <model name="ground">
      <static>true</static>)" + link + R"(
    </model>)";
  const std::string fixedGround = R"(
    <model name="ground">)" + link + R"(
      <joint name="world_joint" type="fixed">
        <parent>world</parent>
        <child>link</child>
      </joint>
    </model>)";

  for (const std::string &name : this->pluginNames)
  {
    std::cout << "Testing plugin: " << name << std::endl;
    for (const std::string &ground : {staticGround, fixedGround})
    {
      ExpectSpheresRestOnHeightmap(this->loader.Instantiate(name),
          QuadrantHeightmap(-1.0), 4.0, 1.1, kRotatedPose, ground);
    }
  }
}

/////////////////////////////////////////////////
// A box rests on a flat heightmap that is tilted, whose surface is a plane.
// The plane is in the wrong place if an engine offsets the heightmap by its
// height along the z axis of the link instead of that of the heightmap.
TYPED_TEST(HeightmapFeaturesTest, BoxRestsOnTiltedHeightmap)
{
  const double height = 1.5;
  const Eigen::Isometry3d pose = Eigen::Translation3d(0.3, 0.2, 0.1) *
      Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitX());
  // The surface of the heightmap
  const Eigen::Vector3d normal = pose.linear() * Eigen::Vector3d::UnitZ();
  const Eigen::Vector3d surface = pose * Eigen::Vector3d(0, 0, height);

  // The box spans a few cells and is dropped flat on the surface, away from
  // the center of the heightmap.
  const gz::math::Vector3d boxSize(1.0, 1.0, 0.2);
  const Eigen::Isometry3d boxPose = pose *
      Eigen::Translation3d(1.3, -0.9, height + boxSize.Z() / 2 + 0.05);
  const std::string worldSdf = kWorldBegin + std::string(kStaticGround) +
      BoxModel("box", gz::math::eigen3::convert(boxPose), boxSize) +
      kWorldEnd;

  for (const std::string &name : this->pluginNames)
  {
    std::cout << "Testing plugin: " << name << std::endl;
    auto world = ConstructWorld(this->loader.Instantiate(name), worldSdf);
    ASSERT_NE(nullptr, world);
    ASSERT_NE(nullptr, world->GetModel("ground")->GetLink("link")
        ->AttachHeightmapShape("heightmap", FlatHeightmap(height), pose,
            Eigen::Vector3d(10, 10, 1)));

    StepWorld(world, 2000);

    // Distance from the surface to the center of the box
    const Eigen::Vector3d pos = world->GetModel("box")->GetLink("link")
        ->FrameDataRelativeToWorld().pose.translation();
    EXPECT_NEAR(boxSize.Z() / 2, normal.dot(pos - surface), 5e-3) << name;
  }
}

/////////////////////////////////////////////////
// Objects dropped on a ramp that is longer along x than along y, away from
// its center and near its edges. They rest at the wrong height or fall off
// if the samples do not span the whole size, or if x and y are swapped.
TYPED_TEST(HeightmapFeaturesTest, ObjectsRestNearRampEdges)
{
  const double sizeY = 20.0;
  const double radius = 0.1;
  const double slope = 0.8 * kRampHeight / kRampSize;
  // The boxes span a few of the ramp's cells, which are 1 m x 0.59 m. On a
  // single cell, MuJoCo can rest a box on one contact, tilted.
  const gz::math::Vector3d boxSize(2.5, 1.5, 0.2);
  const double halfBoxOnSlope =
      boxSize.Z() / 2 / std::cos(std::atan(slope));
  // Height of the ramp, which is at +y
  auto rampZ = [](double _x)
  {
    return kRampZ + kRampHeight * (0.2 + 0.8 * (_x / kRampSize + 0.5));
  };
  const double x = kRampSize / 2 - 0.2;
  const double y = sizeY / 2 - 0.2;
  // Where the objects come to rest
  const std::vector<std::pair<std::string, Eigen::Vector3d>> spheres{
      {"sphere_low", {x, -y, kRampZ + 0.2 * kRampHeight + radius}},
      {"sphere_high", {-x, -y, kRampZ + kRampHeight + radius}}};
  const std::vector<std::pair<std::string, Eigen::Vector3d>> boxes{
      {"box_up", {x / 2, y / 2, rampZ(x / 2) + halfBoxOnSlope}},
      {"box_down", {-x / 2, y / 2, rampZ(-x / 2) + halfBoxOnSlope}}};

  std::string worldSdf = R"(<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="default">
    <model name="terrain">
      <static>true</static>
      <link name="link"/>
    </model>)";
  for (const auto &[model, pos] : spheres)
    worldSdf += SphereModel(model, pos.x(), pos.y(), pos.z() + 0.05);
  // The boxes are dropped parallel to the ramp.
  for (const auto &[model, pos] : boxes)
  {
    worldSdf += BoxModel(model, gz::math::Pose3d(pos.x(), pos.y(),
        pos.z() + 0.05, 0, -std::atan(slope), 0), boxSize);
  }
  worldSdf += R"(
  </world>
</sdf>)";

  for (const std::string &name : this->pluginNames)
  {
    // dartsim spaces the samples size / vertSize apart, so its heightmaps
    // are one cell short of their size.
    CHECK_UNSUPPORTED_ENGINE(name, "dartsim")

    std::cout << "Testing plugin: " << name << std::endl;
    auto world = ConstructWorld(this->loader.Instantiate(name), worldSdf);
    ASSERT_NE(nullptr, world);
    ASSERT_NE(nullptr,
        AttachRamp(world->GetModel("terrain")->GetLink("link"), sizeY));

    StepWorld(world, 2000);

    for (const auto &[model, pos] : spheres)
      ExpectRestingAt(world, model, pos, 0.05);
    for (const auto &[model, pos] : boxes)
      ExpectRestingAt(world, model, pos, 0.05);
  }
}

/////////////////////////////////////////////////
/// \brief Features to check the contacts with a heightmap. They are optional
/// for engines with heightmaps, like HeightmapBoxCastFeatures.
using HeightmapContactFeatures = gz::physics::FeatureList<
  HeightmapFeaturesList,
  gz::physics::GetContactsFromLastStepFeature
>;

/////////////////////////////////////////////////
// A contact with a link that has a box and a heightmap is with the collision
// that is under the other object. Skipped for engines that don't report
// contacts.
TYPED_TEST(HeightmapFeaturesTest, ContactsWithBoxAndHeightmapOfOneLink)
{
  const std::string worldSdf = R"(<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="default">
    <model name="ground">
      <static>true</static>
      <link name="link">
        <collision name="box">
          <pose>-3 0 0 0 0 0</pose>
          <geometry><box><size>2 2 0.2</size></box></geometry>
        </collision>
      </link>
    </model>)" + SphereModel("on_box", -3, 0) +
      SphereModel("on_heightmap", 3, 0) + R"(
  </world>
</sdf>)";

  for (const std::string &name : this->pluginNames)
  {
    std::cout << "Testing plugin: " << name << std::endl;
    auto world = ConstructWorld<HeightmapContactFeatures>(
        this->loader.Instantiate(name), worldSdf);
    if (nullptr == world)
    {
      GTEST_SKIP() << name << " does not support GetContactsFromLastStep";
    }

    // The heightmap is attached after the box, and the sphere on it is over
    // the middle of its grid, away from its first rows.
    auto ground = world->GetModel("ground")->GetLink("link");
    ASSERT_NE(nullptr, ground);
    ASSERT_NE(nullptr, ground->AttachHeightmapShape("heightmap",
        FlatHeightmap(0.3), Eigen::Isometry3d(Eigen::Translation3d(3, 0, 0)),
        Eigen::Vector3d(4.0, 4.0, 1.0)));

    StepWorld(world, 1000);

    // Names of the collisions of the ground that each sphere touches
    std::map<std::string, std::set<std::string>> touched;
    for (const auto &contact : world->GetContactsFromLastStep())
    {
      const auto &point = contact.template Get<
          gz::physics::World3d<HeightmapContactFeatures>::ContactPoint>();
      ASSERT_NE(nullptr, point.collision1);
      ASSERT_NE(nullptr, point.collision2);
      auto groundShape = point.collision1;
      auto sphereShape = point.collision2;
      if (groundShape->GetLink()->GetModel()->GetName() != "ground")
        std::swap(groundShape, sphereShape);
      ASSERT_EQ("ground", groundShape->GetLink()->GetModel()->GetName());
      touched[sphereShape->GetLink()->GetModel()->GetName()].insert(
          groundShape->GetName());
    }
    EXPECT_EQ(std::set<std::string>{"box"}, touched["on_box"]);
    EXPECT_EQ(std::set<std::string>{"heightmap"}, touched["on_heightmap"]);
  }
}

/////////////////////////////////////////////////
/// \brief Features to detach a link with a heightmap from a static link.
using HeightmapDetachFeatures = gz::physics::FeatureList<
  HeightmapFeaturesList,
  gz::physics::DetachJointFeature
>;

/////////////////////////////////////////////////
// bullet-featherstone supports heightmaps on a link fixed to a static link,
// but not once the link is detached and dynamic, so it refuses to detach it:
// the heightmap stays in place and a sphere keeps resting on it.
TYPED_TEST(HeightmapFeaturesTest, DetachLinkWithHeightmapIsRefused)
{
  const double height = 0.3;
  const std::string worldSdf = kWorldBegin + std::string(kStaticGround) + R"(
    <model name="terrain">
      <link name="link">
        <inertial>
          <mass>1.0</mass>
          <inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia>
        </inertial>
      </link>
    </model>)" + SphereModel("sphere", 1.1, 1.1, height + 0.15) + kWorldEnd;

  for (const std::string &name : this->pluginNames)
  {
    CHECK_SUPPORTED_ENGINE(name, "bullet-featherstone")

    std::cout << "Testing plugin: " << name << std::endl;
    auto world = ConstructWorld<HeightmapDetachFeatures>(
        this->loader.Instantiate(name), worldSdf);
    ASSERT_NE(nullptr, world);
    auto link = world->GetModel("terrain")->GetLink("link");
    ASSERT_NE(nullptr, link);
    auto joint = link->AttachFixedJoint(
        world->GetModel("ground")->GetLink("link"), "weld");
    ASSERT_NE(nullptr, joint);
    ASSERT_NE(nullptr, link->AttachHeightmapShape("heightmap",
        FlatHeightmap(height), Eigen::Isometry3d::Identity(),
        Eigen::Vector3d(4, 4, 1)));

    StepWorld(world, 10);
    joint->Detach();
    StepWorld(world, 2000);

    ExpectRestingAt(world, "terrain", Eigen::Vector3d::Zero(), 1e-3);
    ExpectRestingAt(world, "sphere", {1.1, 1.1, height + 0.1}, 0.05);
  }
}

int main(int argc, char *argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  if (!HeightmapFeaturesTest<HeightmapFeaturesList>::init(
       argc, argv))
    return -1;
  return RUN_ALL_TESTS();
}
