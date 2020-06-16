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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_2D_RAY_CASTING_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_2D_RAY_CASTING_H_

#include <vector>

#include "cartographer/common/port.h"
#include "cartographer/mapping/2d/probability_grid.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/sensor/range_data.h"
#include "cartographer/transform/transform.h"

namespace cartographer {
namespace mapping {

// For each ray in 'range_data', inserts hits and misses into
// 'probability_grid'. Hits are handled before misses.
// 对于 "range_data" 中的每条射线，将 hits 和 misses 插入 "probability_grid" 中。hits 先于 misses。
/**
 * @brief CastRays           一方面根据 RangeData 类型的扫描数据完成 RayCasting 操作，获得一次扫描测量过程中相关栅格的观测事件；
 *                           另一方面，调用占用栅格的接口完成查表更新。
 * @param range_data         将要处理的扫描数据
 * @param hit_table          hit 事件的查找表，用于更新栅格单元的占用概率时需要的查找表
 * @param miss_table         miss 事件的查找表，用于更新栅格单元的占用概率时需要的查找表
 * @param insert_free_space  一个配置项，指是否更新发生 miss 事件的栅格单元的占用概率
 * @param probability_grid   将要更新的占用栅格
 */
void CastRays(const sensor::RangeData& range_data,
              const std::vector<uint16>& hit_table,
              const std::vector<uint16>& miss_table, bool insert_free_space,
              ProbabilityGrid* probability_grid);

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_2D_RAY_CASTING_H_
