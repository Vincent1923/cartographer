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

#ifndef CARTOGRAPHER_MAPPING_2D_PROBABILITY_GRID_H_
#define CARTOGRAPHER_MAPPING_2D_PROBABILITY_GRID_H_

#include <vector>

#include "cartographer/common/port.h"
#include "cartographer/mapping/2d/grid_2d.h"
#include "cartographer/mapping/2d/map_limits.h"
#include "cartographer/mapping/2d/xy_index.h"

namespace cartographer {
namespace mapping {

// Represents a 2D grid of probabilities.
// 表示概率的二维栅格。
// ProbabilityGrid 是从 Grid2D 派生而来的，在它的定义中没有额外的添加成员变量，它只是通过一些函数来解释栅格单元的占用概率。
class ProbabilityGrid : public Grid2D {
 public:
  /**
   * @brief ProbabilityGrid  构造函数
   * @param limits           MapLimits 类型的参数，它用于描述地图的边界
   */
  explicit ProbabilityGrid(const MapLimits& limits);
  explicit ProbabilityGrid(const proto::Grid2D& proto);

  // Sets the probability of the cell at 'cell_index' to the given
  // 'probability'. Only allowed if the cell was unknown before.
  // 将单元格在 "cell_index" 处的概率设置为给定的 "probability"。只有在单元格之前未知的情况下才允许。
  /**
   * @brief SetProbability  赋予单元索引 cell_index 概率值 probability，在调用该函数之前需要保证对应的栅格单元的概率是未知的。
   *                        输入的是占用(occupancy)概率，栅格单元实际存储的则是空闲(free)的概率。
   * @param cell_index      单元索引
   * @param probability     栅格单元的概率值，这里是占用(occupancy)概率
   */
  void SetProbability(const Eigen::Array2i& cell_index,
                      const float probability);

  // Applies the 'odds' specified when calling ComputeLookupTableToApplyOdds()
  // to the probability of the cell at 'cell_index' if the cell has not already
  // been updated. Multiple updates of the same cell will be ignored until
  // FinishUpdate() is called. Returns true if the cell was updated.
  //
  // If this is the first call to ApplyOdds() for the specified cell, its value
  // will be set to probability corresponding to 'odds'.
  bool ApplyLookupTable(const Eigen::Array2i& cell_index,
                        const std::vector<uint16>& table);

  // Returns the probability of the cell with 'cell_index'.
  // 返回索引为 "cell_index" 的栅格单元的概率值。
  /**
   * @brief GetProbability  获取栅格单元的占用概率
   * @param cell_index      单元索引
   * @return                栅格单元的占用概率
   */
  float GetProbability(const Eigen::Array2i& cell_index) const;

  virtual proto::Grid2D ToProto() const override;
  /**
   * @brief ComputeCroppedGrid  用于裁剪子图。
   *                            因为在更新子图的过程中，并不能保证更新的数据能够完整覆盖整个子图的所有栅格。
   *                            该函数就是以最小的矩形框出已经更新的栅格。
   * @return                    裁剪后的子图
   */
  virtual std::unique_ptr<Grid2D> ComputeCroppedGrid() const override;
  virtual bool DrawToSubmapTexture(
      proto::SubmapQuery::Response::SubmapTexture* const texture,
      transform::Rigid3d local_pose) const override;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_PROBABILITY_GRID_H_
