/*
 * Copyright 2018 The Cartographer Authors
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
#include "cartographer/mapping/2d/grid_2d.h"

#include "cartographer/mapping/probability_values.h"

namespace cartographer {
namespace mapping {

proto::GridOptions2D CreateGridOptions2D(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::GridOptions2D options;
  const std::string grid_type_string =
      parameter_dictionary->GetString("grid_type");
  proto::GridOptions2D_GridType grid_type;
  CHECK(proto::GridOptions2D_GridType_Parse(grid_type_string, &grid_type))
      << "Unknown GridOptions2D_GridType kind: " << grid_type_string;
  options.set_grid_type(grid_type);
  options.set_resolution(parameter_dictionary->GetDouble("resolution"));
  return options;
}

// 构造函数
Grid2D::Grid2D(const MapLimits& limits, float min_correspondence_cost,
               float max_correspondence_cost)
    : limits_(limits),  // 初始化地图范围 limits_
      correspondence_cost_cells_(  // 初始化地图的概率值，根据 cells 的大小生成一维向量
          limits_.cell_limits().num_x_cells * limits_.cell_limits().num_y_cells,  // 地图的 size 大小，根据 limits_ 的成员 cell_limits_ 计算
          kUnknownCorrespondenceValue),                                           // 每个单元格的概率值为 kUnknownCorrespondenceValue，默认为0
      min_correspondence_cost_(min_correspondence_cost),   // 初始化 min_correspondence_cost_
      max_correspondence_cost_(max_correspondence_cost) {  // 初始化 max_correspondence_cost_
      // 检查 min_correspondence_cost_ 是否小于 max_correspondence_cost_，即检查最小值是否小于最大值
  CHECK_LT(min_correspondence_cost_, max_correspondence_cost_);
}

// 从 proto 流中恢复 Grid2D
Grid2D::Grid2D(const proto::Grid2D& proto)
    : limits_(proto.limits()), correspondence_cost_cells_() {
  if (proto.has_known_cells_box()) {
    const auto& box = proto.known_cells_box();
    known_cells_box_ =
        Eigen::AlignedBox2i(Eigen::Vector2i(box.min_x(), box.min_y()),
                            Eigen::Vector2i(box.max_x(), box.max_y()));
  }
  correspondence_cost_cells_.reserve(proto.cells_size());
  for (const auto& cell : proto.cells()) {
    CHECK_LE(cell, std::numeric_limits<uint16>::max());
    correspondence_cost_cells_.push_back(cell);
  }
  if (proto.min_correspondence_cost() == 0.f &&
      proto.max_correspondence_cost() == 0.f) {
    LOG(WARNING)
        << "proto::Grid2D: max_correspondence_cost and min_correspondence_cost "
           "are initialized with 0 indicating an older version of the "
           "protobuf format. Loading default values.";
    min_correspondence_cost_ = kMinCorrespondenceCost;
    max_correspondence_cost_ = kMaxCorrespondenceCost;
  } else {
    min_correspondence_cost_ = proto.min_correspondence_cost();
    max_correspondence_cost_ = proto.max_correspondence_cost();
  }
  CHECK_LT(min_correspondence_cost_, max_correspondence_cost_);
}

// Finishes the update sequence.
// 停止更新
void Grid2D::FinishUpdate() {
  while (!update_indices_.empty()) {  // 如果索引值列表不为空
    // 检查该索引值中的 Correspondence 是否大于等于 kUpdateMarker
    // kUpdateMarker 等值都定义在 /mapping/probability_values.h 中
    DCHECK_GE(correspondence_cost_cells_[update_indices_.back()],
              kUpdateMarker);
    correspondence_cost_cells_[update_indices_.back()] -= kUpdateMarker;
    update_indices_.pop_back();
  }
}

// Returns the correspondence cost of the cell with 'cell_index'.
// 返回指定 pixel 坐标的 CorrespondenceCost 值
float Grid2D::GetCorrespondenceCost(const Eigen::Array2i& cell_index) const {
  if (!limits().Contains(cell_index)) return kMaxCorrespondenceCost;
  // 这些函数都定义在 /mapping/probability_values.h 中
  return ValueToCorrespondenceCost(
      correspondence_cost_cells()[ToFlatIndex(cell_index)]);  // ToFlatIndex 将一个二维坐标转化为一维的索引值
}

// Returns true if the correspondence cost at the specified index is known.
// 判断一个指定坐标是否已经有概率值
bool Grid2D::IsKnown(const Eigen::Array2i& cell_index) const {
  // kUnknownCorrespondenceValue = 0
  return limits_.Contains(cell_index) &&
         correspondence_cost_cells_[ToFlatIndex(cell_index)] !=
             kUnknownCorrespondenceValue;
}

// Fills in 'offset' and 'limits' to define a subregion of that contains all
// known cells.
// 圈出来一个子区域
void Grid2D::ComputeCroppedLimits(Eigen::Array2i* const offset,
                                  CellLimits* const limits) const {
  if (known_cells_box_.isEmpty()) {
    *offset = Eigen::Array2i::Zero();
    *limits = CellLimits(1, 1);
    return;
  }
  *offset = known_cells_box_.min().array();
  *limits = CellLimits(known_cells_box_.sizes().x() + 1,
                       known_cells_box_.sizes().y() + 1);
}

// Grows the map as necessary to include 'point'. This changes the meaning of
// these coordinates going forward. This method must be called immediately
// after 'FinishUpdate', before any calls to 'ApplyLookupTable'.
// 增长子图
void Grid2D::GrowLimits(const Eigen::Vector2f& point) {
  CHECK(update_indices_.empty());
  while (!limits_.Contains(limits_.GetCellIndex(point))) {
    const int x_offset = limits_.cell_limits().num_x_cells / 2;
    const int y_offset = limits_.cell_limits().num_y_cells / 2;
    const MapLimits new_limits(
        limits_.resolution(),
        limits_.max() +
            limits_.resolution() * Eigen::Vector2d(y_offset, x_offset),
        CellLimits(2 * limits_.cell_limits().num_x_cells,
                   2 * limits_.cell_limits().num_y_cells));
    const int stride = new_limits.cell_limits().num_x_cells;
    const int offset = x_offset + stride * y_offset;
    const int new_size = new_limits.cell_limits().num_x_cells *
                         new_limits.cell_limits().num_y_cells;
    std::vector<uint16> new_cells(new_size, kUnknownCorrespondenceValue);
    for (int i = 0; i < limits_.cell_limits().num_y_cells; ++i) {
      for (int j = 0; j < limits_.cell_limits().num_x_cells; ++j) {
        new_cells[offset + j + i * stride] =
            correspondence_cost_cells_[j +
                                       i * limits_.cell_limits().num_x_cells];
      }
    }
    correspondence_cost_cells_ = new_cells;
    limits_ = new_limits;
    if (!known_cells_box_.isEmpty()) {
      known_cells_box_.translate(Eigen::Vector2i(x_offset, y_offset));
    }
  }
}

proto::Grid2D Grid2D::ToProto() const {
  proto::Grid2D result;
  *result.mutable_limits() = mapping::ToProto(limits_);
  result.mutable_cells()->Reserve(correspondence_cost_cells_.size());
  for (const auto& cell : correspondence_cost_cells_) {
    result.mutable_cells()->Add(cell);
  }
  CHECK(update_indices().empty()) << "Serializing a grid during an update is "
                                     "not supported. Finish the update first.";
  if (!known_cells_box().isEmpty()) {
    auto* const box = result.mutable_known_cells_box();
    box->set_max_x(known_cells_box().max().x());
    box->set_max_y(known_cells_box().max().y());
    box->set_min_x(known_cells_box().min().x());
    box->set_min_y(known_cells_box().min().y());
  }
  result.set_min_correspondence_cost(min_correspondence_cost_);
  result.set_max_correspondence_cost(max_correspondence_cost_);
  return result;
}

// 将一个二维坐标转化为一维的索引值
int Grid2D::ToFlatIndex(const Eigen::Array2i& cell_index) const {
  CHECK(limits_.Contains(cell_index)) << cell_index;
  return limits_.cell_limits().num_x_cells * cell_index.y() + cell_index.x();
}

}  // namespace mapping
}  // namespace cartographer
