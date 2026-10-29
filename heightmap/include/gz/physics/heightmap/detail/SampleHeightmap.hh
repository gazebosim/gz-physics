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

#ifndef GZ_PHYSICS_HEIGHTMAP_DETAIL_SAMPLEHEIGHTMAP_HH_
#define GZ_PHYSICS_HEIGHTMAP_DETAIL_SAMPLEHEIGHTMAP_HH_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <vector>

#include <Eigen/Core>

#include <gz/common/geospatial/HeightmapData.hh>
#include <gz/math/Helpers.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/eigen3/Conversions.hh>

namespace gz
{
namespace physics
{
namespace heightmap
{
namespace detail
{
  /// \brief A square grid of heights sampled from a heightmap, for physics
  /// engines to build their height field from.
  struct SampledHeightmap
  {
    /// \brief Number of samples along each side of the grid, which is at
    /// least 2.
    unsigned int vertSize = 0;

    /// \brief vertSize * vertSize heights, row-major, with the row order
    /// chosen by the _flipY argument of SampleHeightmap. Column 0 is the -x
    /// edge of the heightmap and the last column is its +x edge.
    /// The heights are in meters, scaled so that MaxElevation() -
    /// MinElevation() of the data spans |size.z|, like
    /// common::HeightmapData::FillHeightMap returns them; they are not
    /// shifted, so they don't necessarily start at 0.
    std::vector<float> heights;

    /// \brief Range of heights.
    float minHeight = 0;
    float maxHeight = 0;
  };

  /// \brief Largest number of samples per side of a grid. MuJoCo, Bullet's
  /// btHeightfieldTerrainShape and ODE index the samples of a height field
  /// with int, so the number of samples must fit in an int.
  constexpr std::int64_t kMaxSamplesPerSide = 46340;
  static_assert(kMaxSamplesPerSide * kMaxSamplesPerSide <=
                std::numeric_limits<int>::max() &&
                (kMaxSamplesPerSide + 1) * (kMaxSamplesPerSide + 1) >
                std::numeric_limits<int>::max());

  /// \brief Sample a heightmap into a grid of heights. This only computes
  /// heights: the spacing between samples is up to the caller, as engines
  /// differ in whether size spans vertSize or vertSize - 1 intervals.
  /// \param[in] _data Heightmap data.
  /// \param[in] _size Size of the heightmap in meters.
  /// \param[in] _subSampling Number of samples per pixel.
  /// \param[in] _flipY If false, row 0 of the heights is the +y edge of the
  /// heightmap, which is the top of an image, as DART expects. If true, row 0
  /// is the -y edge, as MuJoCo and Bullet expect.
  /// \return The sampled heights, or std::nullopt if the heightmap is less
  /// than 2 pixels wide, _subSampling is below 1, the grid has more than
  /// kMaxSamplesPerSide samples per side, the size is not positive along x
  /// or y, or _data fails to write a finite height to every sample. Grids
  /// that are too large are rejected before they are allocated.
  inline std::optional<SampledHeightmap> SampleHeightmap(
      const common::HeightmapData &_data,
      const Eigen::Vector3d &_size,
      int _subSampling,
      bool _flipY)
  {
    if (!(_size.x() > 0 && _size.y() > 0))
      return std::nullopt;

    if (_data.Width() < 2 || _subSampling < 1)
      return std::nullopt;

    // 64 bits cannot overflow here.
    const std::int64_t vertSize =
        (static_cast<std::int64_t>(_data.Width()) - 1) * _subSampling + 1;
    if (vertSize > kMaxSamplesPerSide)
      return std::nullopt;

    const float heightRange = _data.MaxElevation() - _data.MinElevation();
    const double scaleZ = math::equal(heightRange, 0.0f) ?
        1.0 : std::fabs(_size.z()) / heightRange;

    SampledHeightmap result;
    result.vertSize = static_cast<unsigned int>(vertSize);
    // The samplers resize the heights without clearing them, and one can
    // return without writing: NaN tells which samples were not written.
    const std::size_t count = static_cast<std::size_t>(vertSize) * vertSize;
    result.heights.assign(count, std::numeric_limits<float>::quiet_NaN());
    _data.FillHeightMap(_subSampling, result.vertSize,
        math::eigen3::convert(_size), math::Vector3d(1, 1, scaleZ), _flipY,
        result.heights);
    if (result.heights.size() != count ||
        !std::all_of(result.heights.begin(), result.heights.end(),
            [](float _h) { return std::isfinite(_h); }))
    {
      return std::nullopt;
    }

    const auto [minIt, maxIt] =
        std::minmax_element(result.heights.begin(), result.heights.end());
    result.minHeight = *minIt;
    result.maxHeight = *maxIt;
    return result;
  }
}
}
}
}

#endif
