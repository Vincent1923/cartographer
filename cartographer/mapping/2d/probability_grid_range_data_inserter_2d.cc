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

// 构造函数。
// 它通过定义在 "probability_values.h" 文件中的函数 ComputeLookupTableToApplyCorrespondenceCostOdds()
// 完成查找表 hit_table_ 和 miss_table_ 的初始化工作。
ProbabilityGridRangeDataInserter2D::ProbabilityGridRangeDataInserter2D(
    const proto::ProbabilityGridRangeDataInserterOptions2D& options)
    : options_(options), 
      /**
       * 1. 函数 ComputeLookupTableToApplyCorrespondenceCostOdds() 用于构建查找表。
       * 2. 在分析该函数之前，我们先来看一下传参。
       *    配置项 hit_probability 和 miss_probability 是定义在文件 "probability_grid_range_data_inserter_options_2d.proto"
       *    中的两个字段，通过函数 Odds() 转换为更新系数 C(hit) 和 C(miss)。
       *    这两个配置项在文件 "cartographer/configuration_files/trajectory_builder_2d.lua" 中设置。
       *    hit_probability 的设置要大于0.5，miss_probability 的设置要小于0.5。
       * 3. 函数主要是把 value~[1,32767] 之间的所有 value 都预先计算出来，存成两个表 hit_table_ 和 miss_table_，
       *    那么这样在使用时就可以以当前单元格的数值(value)为索引，直接查找到更新后的结果(value)。
       */
      hit_table_(ComputeLookupTableToApplyCorrespondenceCostOdds(
          Odds(options.hit_probability()))),
      miss_table_(ComputeLookupTableToApplyCorrespondenceCostOdds(
          Odds(options.miss_probability()))) {}

// 将激光扫描数据插入到栅格地图中。
// 它有两个参数，其中 range_data 是将要插入的扫描数据，grid 则是栅格地图。
void ProbabilityGridRangeDataInserter2D::Insert(
    const sensor::RangeData& range_data, GridInterface* const grid) const {
  // 在函数的一开始将 grid 强制转换为 ProbabilityGrid 类型的数据，所以这个插入器只适配 ProbabilityGrid 类型的栅格地图。
  ProbabilityGrid* const probability_grid = static_cast<ProbabilityGrid*>(grid);
  // By not finishing the update after hits are inserted, we give hits priority
  // (i.e. no hits will be ignored because of a miss in the same cell).
  //
  // 接着通过文件 "ray_casting.cc" 中实现的函数 CastRays() 完成 RayCasting() 操作，并更新栅格地图。
  // CastRay() 中就是把 range_data 中包含的一系列点，计算出一条从原点到这些点的射线，
  // 射线端点处的点是 Hit，射线中间的点是 Free。把这些点对应的栅格单元进行更新。
  CastRays(range_data, hit_table_, miss_table_, options_.insert_free_space(),
           CHECK_NOTNULL(probability_grid));
  // 最后调用栅格地图的接口结束更新。
  probability_grid->FinishUpdate();
}

}  // namespace mapping
}  // namespace cartographer
