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

#ifndef CARTOGRAPHER_MAPPING_2D_GRID_2D_H_
#define CARTOGRAPHER_MAPPING_2D_GRID_2D_H_

#include <vector>

#include "cartographer/mapping/2d/map_limits.h"
#include "cartographer/mapping/grid_interface.h"
#include "cartographer/mapping/proto/2d/grid_2d.pb.h"
#include "cartographer/mapping/proto/2d/submaps_options_2d.pb.h"
#include "cartographer/mapping/proto/submap_visualization.pb.h"

namespace cartographer {
namespace mapping {

proto::GridOptions2D CreateGridOptions2D(
    common::LuaParameterDictionary* const parameter_dictionary);

class Grid2D : public GridInterface {
 public:
  /**
   * @brief Grid2D                   构造函数
   * @param limits                   地图的范围
   * @param min_correspondence_cost  栅格空闲概率 p(free) 的最小值
   * @param max_correspondence_cost  栅格空闲概率 p(free) 的最大值
   *                                 probability 表示栅格单元被占用的概率；
   *                                 probability + correspondence_cost = 1
   */
  explicit Grid2D(const MapLimits& limits, float min_correspondence_cost,
                  float max_correspondence_cost);
  explicit Grid2D(const proto::Grid2D& proto);

  // Returns the limits of this Grid2D.
  // 返回该栅格地图的范围
  const MapLimits& limits() const { return limits_; }

  // Finishes the update sequence.
  // 停止更新
  void FinishUpdate();
  // Returns the correspondence cost of the cell with 'cell_index'.
  // 返回索引为 "cell_index" 的栅格单元的空闲概率值。这里的索引是像素坐标。
  float GetCorrespondenceCost(const Eigen::Array2i& cell_index) const;

  // Returns the minimum possible correspondence cost.
  // 返回栅格空闲概率 p(free) 的最小值。
  float GetMinCorrespondenceCost() const { return min_correspondence_cost_; }

  // Returns the maximum possible correspondence cost.
  // 返回栅格空闲概率 p(free) 的最大值。
  float GetMaxCorrespondenceCost() const { return max_correspondence_cost_; }

  // Returns true if the probability at the specified index is known.
  // 判断指定索引处的栅格单元是否有已知的空闲概率值。这里的索引是像素坐标。
  bool IsKnown(const Eigen::Array2i& cell_index) const;

  // Fills in 'offset' and 'limits' to define a subregion of that contains all
  // known cells.
  // 填充 "offset" 和 "limits"，以定义一个包含所有已知栅格单元的子区域。
  /**
   * @brief ComputeCroppedLimits  获取包含所有已知栅格的子区域
   * @param offset                记录了子区域的起点
   * @param limits                描述了从 offset 开始的矩形框
   */
  void ComputeCroppedLimits(Eigen::Array2i* const offset,
                            CellLimits* const limits) const;

  // Grows the map as necessary to include 'point'. This changes the meaning of
  // these coordinates going forward. This method must be called immediately
  // after 'FinishUpdate', before any calls to 'ApplyLookupTable'.
  // 必要时grow我们的submap。这是一个虚函数。
  virtual void GrowLimits(const Eigen::Vector2f& point);

  // 用于裁剪子图
  virtual std::unique_ptr<Grid2D> ComputeCroppedGrid() const = 0;

  // 写入proto流
  virtual proto::Grid2D ToProto() const;

  virtual bool DrawToSubmapTexture(
      proto::SubmapQuery::Response::SubmapTexture* const texture,
      transform::Rigid3d local_pose) const = 0;

 protected:
  // 返回记录栅格地图概率值的向量
  const std::vector<uint16>& correspondence_cost_cells() const {
    return correspondence_cost_cells_;
  }
  // 更新索引
  const std::vector<int>& update_indices() const { return update_indices_; }
  // 返回一个已知概率值的区域
  const Eigen::AlignedBox2i& known_cells_box() const {
    return known_cells_box_;
  }

  std::vector<uint16>* mutable_correspondence_cost_cells() {
    return &correspondence_cost_cells_;
  }
  std::vector<int>* mutable_update_indices() { return &update_indices_; }
  Eigen::AlignedBox2i* mutable_known_cells_box() { return &known_cells_box_; }

  // Converts a 'cell_index' into an index into 'cells_'.
  // 将pixel坐标再转化为局部坐标系中的点的坐标
  int ToFlatIndex(const Eigen::Array2i& cell_index) const;

 private:
  MapLimits limits_;                               // 地图的范围
  // 记录各个栅格单元的空闲概率 p(free)，0表示对应栅格概率未知，[1, 32767] 表示空闲概率。
  // 可以通过一对儿函数 CorrespondenceCostToValue() 和 ValueToCorrespondenceCost() 相互转换。
  // Cartographer 通过查表的方式更新栅格单元的占用概率。
  std::vector<uint16> correspondence_cost_cells_;
  float min_correspondence_cost_;                  // 栅格空闲概率 p(free) 的最小值
  float max_correspondence_cost_;                  // 栅格空闲概率 p(free) 最大值
  std::vector<int> update_indices_;                // 记录更新过的栅格单元的存储索引

  // Bounding box of known cells to efficiently compute cropping limits.
  // 维护一个已知概率值的所有栅格单元的盒子。方便计算裁剪范围。
  Eigen::AlignedBox2i known_cells_box_;            // 一个用于记录哪些栅格单元中有值的数据结构
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_GRID_2D_H_
