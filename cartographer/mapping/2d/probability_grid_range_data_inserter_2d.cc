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

#include "cartographer/mapping/2d/probability_grid_range_data_inserter_2d.h"

#include <cstdlib>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/mapping/2d/xy_index.h"
#include "cartographer/mapping/internal/2d/ray_casting.h"
#include "cartographer/mapping/probability_values.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

// 加载配置项，设置了 hit_probability(0.55) 和 miss_probability(0.49) 的具体数值
proto::ProbabilityGridRangeDataInserterOptions2D
CreateProbabilityGridRangeDataInserterOptions2D(
    common::LuaParameterDictionary* parameter_dictionary) {
  proto::ProbabilityGridRangeDataInserterOptions2D options;
  options.set_hit_probability(
      parameter_dictionary->GetDouble("hit_probability"));
  options.set_miss_probability(
      parameter_dictionary->GetDouble("miss_probability"));
  options.set_insert_free_space(
      parameter_dictionary->HasKey("insert_free_space")
          ? parameter_dictionary->GetBool("insert_free_space")
          : true);
  CHECK_GT(options.hit_probability(), 0.5);
  CHECK_LT(options.miss_probability(), 0.5);
  return options;
}

ProbabilityGridRangeDataInserter2D::ProbabilityGridRangeDataInserter2D(
    const proto::ProbabilityGridRangeDataInserterOptions2D& options)
    : options_(options),
      // 把 value~[1,32767] 之间的所有 value 都预先计算出来，存成两个表 hit_table_ 和 miss_table_，
      // 那么这样在使用时就可以以当前 cell 的 value 为索引值直接查找到更新后的结果 value。
      hit_table_(ComputeLookupTableToApplyCorrespondenceCostOdds(
          Odds(options.hit_probability()))),
      miss_table_(ComputeLookupTableToApplyCorrespondenceCostOdds(
          Odds(options.miss_probability()))) {}

// 向栅格化地图中插入传感器数据
void ProbabilityGridRangeDataInserter2D::Insert(
    const sensor::RangeData& range_data, GridInterface* const grid) const {
  // 将 Grid 类型强制转化为 ProbabilityGrid 类型
  ProbabilityGrid* const probability_grid = static_cast<ProbabilityGrid*>(grid);
  // By not finishing the update after hits are inserted, we give hits priority
  // (i.e. no hits will be ignored because of a miss in the same cell).
  // 调用 CastRays 函数更新 Grid
  // CastRay 中就是把 RangeData 中包含的一系列点，计算出一条从原点到这些点的射线，
  // 射线端点处的点是 Hit，射线中间的点是 Free。把所有这些点要在地图上把相应的 cell 进行更新。
  CastRays(range_data, hit_table_, miss_table_, options_.insert_free_space(),
           CHECK_NOTNULL(probability_grid));
  probability_grid->FinishUpdate();
}

}  // namespace mapping
}  // namespace cartographer
