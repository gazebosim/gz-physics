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

#include <limits>
#include <string>
#include <vector>

#include <gz/physics/heightmap/detail/SampleHeightmap.hh>

using gz::physics::heightmap::detail::SampleHeightmap;

/// \brief Heightmap whose sample at image row r and column c is
/// (100 * r + c) * scale.Z, with row 0 at the top of the image (+y), or 3 if
/// flat. Like common::Dem, it negates the samples if the size is negative
/// along z.
class IndexHeightmap : public gz::common::HeightmapData
{
  public: explicit IndexHeightmap(unsigned int _width) : width(_width) {}

  public: void FillHeightMap(int, unsigned int _vertSize,
      const gz::math::Vector3d &_size, const gz::math::Vector3d &_scale,
      bool _flipY, std::vector<float> &_heights) const override
  {
    this->filled = true;
    this->lastScaleZ = _scale.Z();
    _heights.resize(this->badSize ? 1 : _vertSize * _vertSize);
    if (this->skipWrite)
      return;
    for (unsigned int i = 0; i < _heights.size(); ++i)
    {
      const unsigned int row = i / _vertSize;
      const unsigned int imageRow = _flipY ? _vertSize - 1 - row : row;
      const double h = this->flat ? 3.0 :
          (100 * imageRow + i % _vertSize) * _scale.Z();
      _heights[i] = static_cast<float>(_size.Z() < 0 ? -h : h);
    }
  }

  public: unsigned int Height() const override { return this->width; }
  public: unsigned int Width() const override { return this->width; }
  public: float MaxElevation() const override { return this->maxElevation; }
  public: float MinElevation() const override { return this->minElevation; }
  public: std::string Filename() const override { return "index"; }

  public: unsigned int width;
  public: float minElevation = 0;
  public: float maxElevation = 10;
  public: bool flat = false;
  public: bool badSize = false;
  public: bool skipWrite = false;
  public: mutable bool filled = false;
  public: mutable double lastScaleZ = 0;
};

/////////////////////////////////////////////////
TEST(SampleHeightmap, RowOrderAndRange)
{
  IndexHeightmap data(3);

  // Row 0 is the +y edge, which is the top row of the image.
  auto sampled = SampleHeightmap(data, {4, 4, 5}, 1, false);
  ASSERT_TRUE(sampled.has_value());
  EXPECT_EQ(3u, sampled->vertSize);
  EXPECT_EQ((std::vector<float>{
      0, 0.5, 1,
      50, 50.5, 51,
      100, 100.5, 101}), sampled->heights);
  EXPECT_FLOAT_EQ(0, sampled->minHeight);
  EXPECT_FLOAT_EQ(101, sampled->maxHeight);

  // Row 0 is the -y edge, which is the bottom row of the image.
  sampled = SampleHeightmap(data, {4, 4, 5}, 1, true);
  ASSERT_TRUE(sampled.has_value());
  EXPECT_EQ((std::vector<float>{
      100, 100.5, 101,
      50, 50.5, 51,
      0, 0.5, 1}), sampled->heights);
  EXPECT_FLOAT_EQ(0, sampled->minHeight);
  EXPECT_FLOAT_EQ(101, sampled->maxHeight);
}

/////////////////////////////////////////////////
TEST(SampleHeightmap, SubSampling)
{
  IndexHeightmap data(3);
  const auto sampled = SampleHeightmap(data, {4, 4, 5}, 2, false);
  ASSERT_TRUE(sampled.has_value());
  EXPECT_EQ(5u, sampled->vertSize);
  EXPECT_EQ(25u, sampled->heights.size());

  // These don't make a grid
  EXPECT_FALSE(SampleHeightmap(data, {4, 4, 5}, 0, false).has_value());
  EXPECT_FALSE(SampleHeightmap(data, {4, 4, 5}, -1, false).has_value());
}

/////////////////////////////////////////////////
TEST(SampleHeightmap, Scale)
{
  IndexHeightmap data(3);
  data.minElevation = 2;
  data.maxElevation = 6;
  auto sampled = SampleHeightmap(data, {4, 4, 2}, 1, false);
  ASSERT_TRUE(sampled.has_value());
  EXPECT_DOUBLE_EQ(0.5, data.lastScaleZ);
  EXPECT_FLOAT_EQ(0, sampled->minHeight);
  EXPECT_FLOAT_EQ(101, sampled->maxHeight);

  // A negative size along z scales by its magnitude and is left to the data
  // to apply, which negates the samples.
  sampled = SampleHeightmap(data, {4, 4, -2}, 1, false);
  ASSERT_TRUE(sampled.has_value());
  EXPECT_DOUBLE_EQ(0.5, data.lastScaleZ);
  EXPECT_EQ((std::vector<float>{
      0, -0.5, -1,
      -50, -50.5, -51,
      -100, -100.5, -101}), sampled->heights);
  EXPECT_FLOAT_EQ(-101, sampled->minHeight);
  EXPECT_FLOAT_EQ(0, sampled->maxHeight);

  // A flat heightmap is not scaled
  data.minElevation = data.maxElevation;
  data.flat = true;
  sampled = SampleHeightmap(data, {4, 4, 2}, 1, false);
  ASSERT_TRUE(sampled.has_value());
  EXPECT_DOUBLE_EQ(1.0, data.lastScaleZ);
  EXPECT_EQ(std::vector<float>(9, 3.0f), sampled->heights);
  EXPECT_FLOAT_EQ(3, sampled->minHeight);
  EXPECT_FLOAT_EQ(3, sampled->maxHeight);
}

/////////////////////////////////////////////////
TEST(SampleHeightmap, Invalid)
{
  IndexHeightmap data(3);
  EXPECT_FALSE(SampleHeightmap(data, {0, 4, 5}, 1, false).has_value());
  EXPECT_FALSE(SampleHeightmap(data, {4, -4, 5}, 1, false).has_value());

  const IndexHeightmap tooSmall(1);
  EXPECT_FALSE(SampleHeightmap(tooSmall, {4, 4, 5}, 1, false).has_value());
  EXPECT_FALSE(SampleHeightmap(tooSmall, {4, 4, 5}, 2, false).has_value());

  // These look like 2 samples per side with the product of signed ints
  const IndexHeightmap empty(0);
  EXPECT_FALSE(SampleHeightmap(empty, {4, 4, 5}, -1, false).has_value());
  EXPECT_FALSE(empty.filled);

  data.badSize = true;
  EXPECT_FALSE(SampleHeightmap(data, {4, 4, 5}, 1, false).has_value());
}

/////////////////////////////////////////////////
TEST(SampleHeightmap, TooLarge)
{
  // The largest grid has a number of samples that fits in an int, unlike
  // a grid with one more sample per side.
  EXPECT_EQ(46340, gz::physics::heightmap::detail::kMaxSamplesPerSide);
  EXPECT_LE(46340LL * 46340, std::numeric_limits<int>::max());
  EXPECT_GT(46341LL * 46341, std::numeric_limits<int>::max());

  // 46341 samples per side, rejected without sampling
  const IndexHeightmap twoPixels(2);
  EXPECT_FALSE(
      SampleHeightmap(twoPixels, {4, 4, 5}, 46340, false).has_value());
  EXPECT_FALSE(twoPixels.filled);

  // vertSize * vertSize wraps around in unsigned int past 65535, and so
  // would the size of the heights.
  const IndexHeightmap huge(65537);
  EXPECT_FALSE(SampleHeightmap(huge, {4, 4, 5}, 1, false).has_value());
  EXPECT_FALSE(huge.filled);

  // vertSize is 65537, which wraps around to 131073 samples
  const IndexHeightmap wraps(1025);
  EXPECT_FALSE(SampleHeightmap(wraps, {4, 4, 5}, 64, false).has_value());
  EXPECT_FALSE(wraps.filled);

  // Overflow of a signed int
  const IndexHeightmap few(3);
  EXPECT_FALSE(SampleHeightmap(few, {4, 4, 5},
      std::numeric_limits<int>::max(), false).has_value());
  EXPECT_FALSE(few.filled);
}

/////////////////////////////////////////////////
TEST(SampleHeightmap, NotWritten)
{
  // The heights have the right size but the data didn't write them, as
  // common::ImageHeightmap does with a pixel format it doesn't support.
  IndexHeightmap data(3);
  data.skipWrite = true;
  EXPECT_FALSE(SampleHeightmap(data, {4, 4, 5}, 1, false).has_value());
  EXPECT_TRUE(data.filled);
}
