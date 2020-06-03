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

#ifndef CARTOGRAPHER_MAPPING_2D_MAP_LIMITS_H_
#define CARTOGRAPHER_MAPPING_2D_MAP_LIMITS_H_

#include <utility>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/common/math.h"
#include "cartographer/mapping/2d/xy_index.h"
#include "cartographer/mapping/proto/2d/map_limits.pb.h"
#include "cartographer/mapping/trajectory_node.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/sensor/range_data.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

// Defines the limits of a grid map. This class must remain inlined for
// performance reasons.
// 定义栅格地图（grid map）的限制，出于性能原因，该类必须保持内联
class MapLimits {
 public:
  /**
   * @brief MapLimits    构造函数，主要是初始化成员变量
   * @param resolution   地图分辨率，程序中设置是0.05m，也就是5cm
   * @param max          这是一个浮点型二维向量，max_.x()和.y()分别表示x、y方向的最大值
   * @param cell_limits  栅格化后的 x 和 y 方向的最大范围，以 pixel 为单位
   * @return
   */
  MapLimits(const double resolution, const Eigen::Vector2d& max,
            const CellLimits& cell_limits)
      : resolution_(resolution), max_(max), cell_limits_(cell_limits) {
    CHECK_GT(resolution_, 0.);              // 检查 resolution_ 是否大于0
    CHECK_GT(cell_limits.num_x_cells, 0.);  // 检查 cell_limits.num_x_cells 是否大于0
    CHECK_GT(cell_limits.num_y_cells, 0.);  // 检查 cell_limits.num_y_cells 是否大于0
  }

  /**
   * @brief MapLimits   构造函数，从 proto 流中构造地图范围，主要是初始化成员变量
   * @param map_limits  地图范围，数据类型为 proto::MapLimits，这是一个 ProtocolBuffer 消息类型，消息类型定义在
   *                    “src/cartographer/cartographer/mapping/proto/2d/map_limits.proto”文件中。
   * @return
   */
  explicit MapLimits(const proto::MapLimits& map_limits)
      : resolution_(map_limits.resolution()),
        max_(transform::ToEigen(map_limits.max())),
        cell_limits_(map_limits.cell_limits()) {}

  // Returns the cell size in meters. All cells are square and the resolution is
  // the length of one side.
  // 获取分辨率
  double resolution() const { return resolution_; }

  // Returns the corner of the limits, i.e., all pixels have positions with
  // smaller coordinates.
  // 获取最大范围值
  const Eigen::Vector2d& max() const { return max_; }

  // Returns the limits of the grid in number of cells.
  // 获取pixel坐标的最大范围
  const CellLimits& cell_limits() const { return cell_limits_; }

  // Returns the index of the cell containing the 'point' which may be outside
  // the map, i.e., negative or too large indices that will return false for
  // Contains().
  // 返回“point”的单元格的索引，包含可能位于地图之外的“point”，例如，对于Contains()如果是负数或太大的索引，那么将返回false。
  // 给出一个point在Submap中的坐标，求其pixel坐标。
  Eigen::Array2i GetCellIndex(const Eigen::Vector2f& point) const {
    // Index values are row major and the top left has Eigen::Array2i::Zero()
    // and contains (centered_max_x, centered_max_y). We need to flip and
    // rotate.
    return Eigen::Array2i(
        common::RoundToInt((max_.y() - point.y()) / resolution_ - 0.5),
        common::RoundToInt((max_.x() - point.x()) / resolution_ - 0.5));
  }

  // Returns true if the ProbabilityGrid contains 'cell_index'.
  // 返回布尔型，判断所给pixel坐标是否大于0，小于等于最大值
  bool Contains(const Eigen::Array2i& cell_index) const {
    return (Eigen::Array2i(0, 0) <= cell_index).all() &&
           (cell_index <
            Eigen::Array2i(cell_limits_.num_x_cells, cell_limits_.num_y_cells))
               .all();
  }

 private:
  double resolution_;    // 地图的分辨率，即一个栅格单元对应的地图尺寸。程序中设置是0.05m，也就是5cm。
  Eigen::Vector2d max_;  // 这是一个浮点型二维向量，max_.x() 和 max_.y() 分别记录了地图的 x,y 方向的最大值。
  /**
   * 1. x 和 y 方向上的栅格数量。
   *    在 MapLimits 中根据最大范围 max_ 和分辨率 resolution 就可以创建 cell_limits_。
   * 2. 数据类型 CellLimits 定义在 "/mapping/2d/xy_index.h" 中，它是一个结构体，包括两个 int 型成员变量：
   *    num_x_cells 是 x 轴上的栅格数量，num_y_cells 是 y 轴上的栅格数量。
   */
  CellLimits cell_limits_;
};

inline proto::MapLimits ToProto(const MapLimits& map_limits) {
  proto::MapLimits result;
  result.set_resolution(map_limits.resolution());
  *result.mutable_max() = transform::ToProto(map_limits.max());
  *result.mutable_cell_limits() = ToProto(map_limits.cell_limits());
  return result;
}

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_MAP_LIMITS_H_
