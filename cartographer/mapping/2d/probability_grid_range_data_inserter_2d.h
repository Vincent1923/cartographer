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

#ifndef CARTOGRAPHER_MAPPING_2D_RANGE_DATA_INSERTER_2D_PROBABILITY_GRID_H_
#define CARTOGRAPHER_MAPPING_2D_RANGE_DATA_INSERTER_2D_PROBABILITY_GRID_H_

#include <utility>
#include <vector>

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/port.h"
#include "cartographer/mapping/2d/probability_grid.h"
#include "cartographer/mapping/2d/xy_index.h"
#include "cartographer/mapping/proto/2d/probability_grid_range_data_inserter_options_2d.pb.h"
#include "cartographer/mapping/range_data_inserter_interface.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/sensor/range_data.h"

namespace cartographer {
namespace mapping {

// 加载配置项，设置了 hit_probability(0.55) 和 miss_probability(0.49) 的具体数值
proto::ProbabilityGridRangeDataInserterOptions2D
CreateProbabilityGridRangeDataInserterOptions2D(
    common::LuaParameterDictionary* parameter_dictionary);

// 插入器
class ProbabilityGridRangeDataInserter2D : public RangeDataInserterInterface {
 public:
  /**
   * @brief ProbabilityGridRangeDataInserter2D  构造函数。它通过定义在 "probability_values.h" 文件中的函数
   *                                            ComputeLookupTableToApplyCorrespondenceCostOdds() 完成查找表
   *                                            hit_table_ 和 miss_table_ 的初始化工作。
   * @param options                             插入器的配置
   */
  explicit ProbabilityGridRangeDataInserter2D(
      const proto::ProbabilityGridRangeDataInserterOptions2D& options);

  // 屏蔽了拷贝构造和拷贝赋值
  ProbabilityGridRangeDataInserter2D(
      const ProbabilityGridRangeDataInserter2D&) = delete;
  ProbabilityGridRangeDataInserter2D& operator=(
      const ProbabilityGridRangeDataInserter2D&) = delete;

  // Inserts 'range_data' into 'probability_grid'.
  // 将 "range_data" 插入 "probability_grid"。
  /**
   * @brief Insert      将激光扫描数据插入到栅格地图中
   * @param range_data  将要插入的扫描数据
   * @param grid        栅格地图
   */
  virtual void Insert(const sensor::RangeData& range_data,
                      GridInterface* grid) const override;

 private:
  const proto::ProbabilityGridRangeDataInserterOptions2D options_;  // 记录了插入器的配置
  const std::vector<uint16> hit_table_;                             // 用于更新栅格单元的占用概率的查找表
  const std::vector<uint16> miss_table_;                            // 用于更新栅格单元的占用概率的查找表
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_RANGE_DATA_INSERTER_2D_PROBABILITY_GRID_H_
