/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer/mapping/internal/2d/ray_casting.h"

namespace cartographer {
namespace mapping {
namespace {

// Factor for subpixel accuracy of start and end point.
constexpr int kSubpixelScale = 1000;

// We divide each pixel in kSubpixelScale x kSubpixelScale subpixels. 'begin'
// and 'end' are coordinates at subpixel precision. We compute all pixels in
// which some part of the line segment connecting 'begin' and 'end' lies.
void CastRay(const Eigen::Array2i& begin, const Eigen::Array2i& end,
             const std::vector<uint16>& miss_table,
             ProbabilityGrid* const probability_grid) {
  // For simplicity, we order 'begin' and 'end' by their x coordinate.
  if (begin.x() > end.x()) {
    CastRay(end, begin, miss_table, probability_grid);
    return;
  }

  CHECK_GE(begin.x(), 0);
  CHECK_GE(begin.y(), 0);
  CHECK_GE(end.y(), 0);

  // Special case: We have to draw a vertical line in full pixels, as 'begin'
  // and 'end' have the same full pixel x coordinate.
  if (begin.x() / kSubpixelScale == end.x() / kSubpixelScale) {
    Eigen::Array2i current(begin.x() / kSubpixelScale,
                           std::min(begin.y(), end.y()) / kSubpixelScale);
    const int end_y = std::max(begin.y(), end.y()) / kSubpixelScale;
    for (; current.y() <= end_y; ++current.y()) {
      probability_grid->ApplyLookupTable(current, miss_table);
    }
    return;
  }

  const int64 dx = end.x() - begin.x();
  const int64 dy = end.y() - begin.y();
  const int64 denominator = 2 * kSubpixelScale * dx;

  // The current full pixel coordinates. We begin at 'begin'.
  Eigen::Array2i current = begin / kSubpixelScale;

  // To represent subpixel centers, we use a factor of 2 * 'kSubpixelScale' in
  // the denominator.
  // +-+-+-+ -- 1 = (2 * kSubpixelScale) / (2 * kSubpixelScale)
  // | | | |
  // +-+-+-+
  // | | | |
  // +-+-+-+ -- top edge of first subpixel = 2 / (2 * kSubpixelScale)
  // | | | | -- center of first subpixel = 1 / (2 * kSubpixelScale)
  // +-+-+-+ -- 0 = 0 / (2 * kSubpixelScale)

  // The center of the subpixel part of 'begin.y()' assuming the
  // 'denominator', i.e., sub_y / denominator is in (0, 1).
  int64 sub_y = (2 * (begin.y() % kSubpixelScale) + 1) * dx;

  // The distance from the from 'begin' to the right pixel border, to be divided
  // by 2 * 'kSubpixelScale'.
  const int first_pixel =
      2 * kSubpixelScale - 2 * (begin.x() % kSubpixelScale) - 1;
  // The same from the left pixel border to 'end'.
  const int last_pixel = 2 * (end.x() % kSubpixelScale) + 1;

  // The full pixel x coordinate of 'end'.
  const int end_x = std::max(begin.x(), end.x()) / kSubpixelScale;

  // Move from 'begin' to the next pixel border to the right.
  sub_y += dy * first_pixel;
  if (dy > 0) {
    while (true) {
      probability_grid->ApplyLookupTable(current, miss_table);
      while (sub_y > denominator) {
        sub_y -= denominator;
        ++current.y();
        probability_grid->ApplyLookupTable(current, miss_table);
      }
      ++current.x();
      if (sub_y == denominator) {
        sub_y -= denominator;
        ++current.y();
      }
      if (current.x() == end_x) {
        break;
      }
      // Move from one pixel border to the next.
      sub_y += dy * 2 * kSubpixelScale;
    }
    // Move from the pixel border on the right to 'end'.
    sub_y += dy * last_pixel;
    probability_grid->ApplyLookupTable(current, miss_table);
    while (sub_y > denominator) {
      sub_y -= denominator;
      ++current.y();
      probability_grid->ApplyLookupTable(current, miss_table);
    }
    CHECK_NE(sub_y, denominator);
    CHECK_EQ(current.y(), end.y() / kSubpixelScale);
    return;
  }

  // Same for lines non-ascending in y coordinates.
  while (true) {
    probability_grid->ApplyLookupTable(current, miss_table);
    while (sub_y < 0) {
      sub_y += denominator;
      --current.y();
      probability_grid->ApplyLookupTable(current, miss_table);
    }
    ++current.x();
    if (sub_y == 0) {
      sub_y += denominator;
      --current.y();
    }
    if (current.x() == end_x) {
      break;
    }
    sub_y += dy * 2 * kSubpixelScale;
  }
  sub_y += dy * last_pixel;
  probability_grid->ApplyLookupTable(current, miss_table);
  while (sub_y < 0) {
    sub_y += denominator;
    --current.y();
    probability_grid->ApplyLookupTable(current, miss_table);
  }
  CHECK_NE(sub_y, 0);
  CHECK_EQ(current.y(), end.y() / kSubpixelScale);
}

void GrowAsNeeded(const sensor::RangeData& range_data,
                  ProbabilityGrid* const probability_grid) {
  Eigen::AlignedBox2f bounding_box(range_data.origin.head<2>());
  constexpr float kPadding = 1e-6f;
  for (const Eigen::Vector3f& hit : range_data.returns) {
    bounding_box.extend(hit.head<2>());
  }
  for (const Eigen::Vector3f& miss : range_data.misses) {
    bounding_box.extend(miss.head<2>());
  }
  probability_grid->GrowLimits(bounding_box.min() -
                               kPadding * Eigen::Vector2f::Ones());
  probability_grid->GrowLimits(bounding_box.max() +
                               kPadding * Eigen::Vector2f::Ones());
}

}  // namespace

void CastRays(const sensor::RangeData& range_data,
              const std::vector<uint16>& hit_table,
              const std::vector<uint16>& miss_table,
              const bool insert_free_space,
              ProbabilityGrid* const probability_grid) {
  GrowAsNeeded(range_data, probability_grid);

  const MapLimits& limits = probability_grid->limits();  // 获取栅格地图的 limits
  // 定义一个超分辨率像素，把当前的分辨率又划分成了 kSubpixelScale 份，这里 int kSubpixelScale = 1000
  const double superscaled_resolution = limits.resolution() / kSubpixelScale;
  // 根据超分辨率像素又生成了一个新的 MapLimits
  const MapLimits superscaled_limits(
      superscaled_resolution, limits.max(),
      CellLimits(limits.cell_limits().num_x_cells * kSubpixelScale,  // 划分格数变成了 kSubpixelScale 倍
                 limits.cell_limits().num_y_cells * kSubpixelScale));
  // 根据 RangeData 原点的前两项，获取其对应的栅格化坐标。该坐标是我们所求的射线的原点。
  const Eigen::Array2i begin =
      superscaled_limits.GetCellIndex(range_data.origin.head<2>());
  // Compute and add the end points.
  // 定义一个向量集合，该集合存储 RangeData 中的 hits 的点。
  std::vector<Eigen::Array2i> ends;
  // reserver 函数用来给 vector 预分配存储区大小，即 capacity 的值 ，但是没有给这段内存进行初始化。
  // reserve 的参数 n 是推荐预分配内存的大小，实际分配的可能等于或大于这个值，即 n 大于 capacity 的值，
  // 就会 reallocate 内存 capacity 的值会大于或者等于 n 。这样，当 vector 调用push_back函数使得 size 
  // 超过原来的默认分配的 capacity 值时 避免了内存重分配开销。
  // 这里就是根据 returns 集合的大小，给 ends 预分配一块存储区。
  ends.reserve(range_data.returns.size());
  for (const Eigen::Vector3f& hit : range_data.returns) {
    // 遍历 returns 这个集合，把每个点先压入 ends 中，这里获取的是栅格化坐标
    ends.push_back(superscaled_limits.GetCellIndex(hit.head<2>()));
    // ens.back() 返回的是 vector 中的最末尾项，也就是我们刚刚压入 vector 中的那一项；
    // 这里我猜测，hit_table 就是预先计算好的。如果一个 cell，原先的值是 value，那么在检测到 hit 后应该更新为多少。
    probability_grid->ApplyLookupTable(ends.back() / kSubpixelScale, hit_table);
  }

  if (!insert_free_space) {
  // 如果配置项里设置是不考虑 free space。那么函数到这里结束，只处理完 hit 后返回即可
  // 否则的话，需要计算那条射线，射线中间的点都是 free space，同时，没有检测到 hit 的 misses 集里也都是 free
    return;
  }

  // Now add the misses.
  // 处理 origin 跟 hit 之间的射线中间的点
  for (const Eigen::Array2i& end : ends) {
    CastRay(begin, end, miss_table, probability_grid);
  }

  // Finally, compute and add empty rays based on misses in the range data.
  // 同样处理 miss 集合中的点
  for (const Eigen::Vector3f& missing_echo : range_data.misses) {
    CastRay(begin, superscaled_limits.GetCellIndex(missing_echo.head<2>()),
            miss_table, probability_grid);
  }
}

}  // namespace mapping
}  // namespace cartographer
