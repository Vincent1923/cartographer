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
#include "cartographer/mapping/2d/probability_grid.h"

#include <limits>

#include "cartographer/common/make_unique.h"
#include "cartographer/mapping/probability_values.h"
#include "cartographer/mapping/submaps.h"

namespace cartographer {
namespace mapping {

/**
 * 1. 构造函数。
 * 2. 该函数有一个 MapLimits 类型的参数 limits，它用于描述地图的边界。
 * 3. 在初始构造列表中，可以看到参数 limits 和另外两个常数被用来构造父类 Grid2D。
 * 4. kMinCorrespondenceCost = 0.1 描述了栅格元素的最小值，kMaxCorrespondenceCost = 0.9 描述了栅格元素的最大值。
 *    它们被定义在文件 "probability_values.h" 中。
 *    所以各个栅格的最小值是0.1，最大值是0.9。
 */
ProbabilityGrid::ProbabilityGrid(const MapLimits& limits)
    : Grid2D(limits, kMinCorrespondenceCost, kMaxCorrespondenceCost) {}

ProbabilityGrid::ProbabilityGrid(const proto::Grid2D& proto) : Grid2D(proto) {
  CHECK(proto.has_probability_grid_2d());
}

// Sets the probability of the cell at 'cell_index' to the given
// 'probability'. Only allowed if the cell was unknown before.
// 赋予单元索引 cell_index 概率值 probability，在调用该函数之前需要保证对应的栅格单元的概率是未知的。
void ProbabilityGrid::SetProbability(const Eigen::Array2i& cell_index,
                                     const float probability) {
  // 根据索引 cell_index 获取目标栅格单元。
  // 这里调用的两个函数 mutable_correspondence_cost_cells() 和 ToFlatIndex() 都是在父类中定义的。
  uint16& cell =
      (*mutable_correspondence_cost_cells())[ToFlatIndex(cell_index)];
  // 检查栅格单元是否未知
  // 常数 kUnknownProbabilityValue = 0，也是在文件 "probability_values.h" 中定义的。
  CHECK_EQ(cell, kUnknownProbabilityValue);
  /**
   * 1. 为栅格单元赋予新值。
   *    调用了两个在文件 "probability_values.h" 中定义的函数。
   * 2. 函数 CorrespondenceCostToValue() 是将 float 类型的数据转换为 uint16 类型，
   *    并将输入从区间 [kMinCorrespondenceCost,kMaxCorrespondenceCost] 映射到 [1,32767]。
   * 3. 函数 ProbabilityToCorrespondenceCost() 实际上是对输入概率值取反，这意味着
   *    如果我们的输入描述的是栅格单元的占用(occupancy)概率，那么实际存储的则是栅格单元空闲(free)的概率。
   */
  cell =
      CorrespondenceCostToValue(ProbabilityToCorrespondenceCost(probability));
  // 最后通过父类 Grid2D 的接口记录下栅格单元索引
  mutable_known_cells_box()->extend(cell_index.matrix());
}

// Applies the 'odds' specified when calling ComputeLookupTableToApplyOdds()
// to the probability of the cell at 'cell_index' if the cell has not already
// been updated. Multiple updates of the same cell will be ignored until
// FinishUpdate() is called. Returns true if the cell was updated.
//
// If this is the first call to ApplyOdds() for the specified cell, its value
// will be set to probability corresponding to 'odds'.
// 通过查表来更新栅格单元的占用概率。
// 该函数有两个输入参数，其中 cell_index 是将要更新的栅格单元索引，table 是更新过程中将要查的表。
bool ProbabilityGrid::ApplyLookupTable(const Eigen::Array2i& cell_index,
                                       const std::vector<uint16>& table) {
  // 在函数的一开始的时候，先检查一下查找表的大小。
  // kUpdateMarker = 2 ^ 15，所以查找表的大小等于 32768
  DCHECK_EQ(table.size(), kUpdateMarker);
  // 然后通过 cell_index 计算栅格单元的存储索引，获取对应的\(p_{free}\)存储值，并确保该值不会超出查找表的数组边界。
  const int flat_index = ToFlatIndex(cell_index);  // 函数 ToFlatIndex() 把二维离散的像素坐标转化为一维索引
  // 根据索引 flat_index 获取该栅格单元的地址 cell，单元存储的是空闲概率。
  // mutable_correspondence_cost_cells() 是 Grid2D 的成员函数，返回存放概率值的一维向量。
  uint16* cell = &(*mutable_correspondence_cost_cells())[flat_index];  // cell 是栅格单元的地址
  // 确保该值不会超出查找表的数组边界
  if (*cell >= kUpdateMarker) {
    return false;
  }
  // 接着通过父类的接口记录下当前更新的栅格单元的存储索引 flat_index。
  // 已更新的信息都存储在容器 update_indices_ 中。
  mutable_update_indices()->push_back(flat_index);
  // 通过查表更新栅格单元。
  // 根据栅格单元 cell 的存储值 *cell 来查表，获取更新后应该是什么值。然后把这个值放入到 cell 原先的地址中，实际就是更新该值。
  *cell = table[*cell];
  DCHECK_GE(*cell, kUpdateMarker);
  // 最后通过父类标记 cell_index 所对应的栅格的占用概率已知。
  // mutable_known_cells_box() 是 Grid2D 的成员函数，返回存放已知概率值的一个子区域的盒子，
  // 现在就是把该栅格单元 cell 的索引 cell_index 放入已知概率值的盒子中。
  mutable_known_cells_box()->extend(cell_index.matrix());
  return true;
}

// Returns the probability of the cell with 'cell_index'.
// 获取栅格单元的占用概率
float ProbabilityGrid::GetProbability(const Eigen::Array2i& cell_index) const {
  // 通过父类的接口判定一下输入的单元索引是否在子图的范围内，如果不在就直接返回一个最小的概率。
  if (!limits().Contains(cell_index)) return kMinProbability;
  /**
   * 1. 获取栅格对应的占用概率并返回。
   *    调用了两个在文件 "probability_values.h" 中定义的函数。
   * 2. 函数 ValueToCorrespondenceCost() 是将 uint16 类型的数据转换为 float 类型，
   *    并将输入从区间 [1,32767] 映射到 [kMinCorrespondenceCost,kMaxCorrespondenceCost]。
   * 3. 函数 CorrespondenceCostToProbability() 是对输入概率值取反，由于栅格单元存储的是空闲(free)的概率，
   *    所以这里实际返回的是栅格单元的占用(occupancy)概率。
   */
  return CorrespondenceCostToProbability(ValueToCorrespondenceCost(
      correspondence_cost_cells()[ToFlatIndex(cell_index)]));
}

proto::Grid2D ProbabilityGrid::ToProto() const {
  proto::Grid2D result;
  result = Grid2D::ToProto();
  result.mutable_probability_grid_2d();
  return result;
}

// 函数 ComputeCroppedGrid() 用于裁剪子图。
// 因为在更新子图的过程中，并不能保证更新的数据能够完整覆盖整个子图的所有栅格。该函数就是以最小的矩形框出已经更新的栅格。
std::unique_ptr<Grid2D> ProbabilityGrid::ComputeCroppedGrid() const {
  // 在该函数的一开始，就先通过父类的接口获取包含所有已知栅格的子区域，
  // 变量 offset 记录了子区域的起点，cell_limits 描述了从 offset 开始的矩形框。
  Eigen::Array2i offset;
  CellLimits cell_limits;
  ComputeCroppedLimits(&offset, &cell_limits);
  // 接着获取子图的分辨率和最大的xy索引，并构建一个新的 ProbabilityGrid 对象 cropped_grid。
  const double resolution = limits().resolution();
  const Eigen::Vector2d max =
      limits().max() - resolution * Eigen::Vector2d(offset.y(), offset.x());
  std::unique_ptr<ProbabilityGrid> cropped_grid =
      common::make_unique<ProbabilityGrid>(
          MapLimits(resolution, max, cell_limits));
  // 最后在遍历了对象 cropped_grid 中的所有栅格，拷贝占用概率之后，返回对象退出。
  for (const Eigen::Array2i& xy_index : XYIndexRangeIterator(cell_limits)) {
    if (!IsKnown(xy_index + offset)) continue;
    cropped_grid->SetProbability(xy_index, GetProbability(xy_index + offset));
  }

  return std::unique_ptr<Grid2D>(cropped_grid.release());
}

bool ProbabilityGrid::DrawToSubmapTexture(
    proto::SubmapQuery::Response::SubmapTexture* const texture,
    transform::Rigid3d local_pose) const {
  Eigen::Array2i offset;
  CellLimits cell_limits;
  ComputeCroppedLimits(&offset, &cell_limits);

  std::string cells;
  for (const Eigen::Array2i& xy_index : XYIndexRangeIterator(cell_limits)) {
    if (!IsKnown(xy_index + offset)) {
      cells.push_back(0 /* unknown log odds value */);
      cells.push_back(0 /* alpha */);
      continue;
    }
    // We would like to add 'delta' but this is not possible using a value and
    // alpha. We use premultiplied alpha, so when 'delta' is positive we can
    // add it by setting 'alpha' to zero. If it is negative, we set 'value' to
    // zero, and use 'alpha' to subtract. This is only correct when the pixel
    // is currently white, so walls will look too gray. This should be hard to
    // detect visually for the user, though.
    const int delta =
        128 - ProbabilityToLogOddsInteger(GetProbability(xy_index + offset));
    const uint8 alpha = delta > 0 ? 0 : -delta;
    const uint8 value = delta > 0 ? delta : 0;
    cells.push_back(value);
    cells.push_back((value || alpha) ? alpha : 1);
  }

  common::FastGzipString(cells, texture->mutable_cells());
  texture->set_width(cell_limits.num_x_cells);
  texture->set_height(cell_limits.num_y_cells);
  const double resolution = limits().resolution();
  texture->set_resolution(resolution);
  const double max_x = limits().max().x() - resolution * offset.y();
  const double max_y = limits().max().y() - resolution * offset.x();
  *texture->mutable_slice_pose() = transform::ToProto(
      local_pose.inverse() *
      transform::Rigid3d::Translation(Eigen::Vector3d(max_x, max_y, 0.)));

  return true;
}

}  // namespace mapping
}  // namespace cartographer
