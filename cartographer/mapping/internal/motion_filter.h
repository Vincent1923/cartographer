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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_MOTION_FILTER_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_MOTION_FILTER_H_

#include <limits>

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/proto/motion_filter_options.pb.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace mapping {

// 从 Proto 流中读取配置文件，设置配置项，主要包括最大时间间隔、最大角度间隔、最大距离间隔等
proto::MotionFilterOptions CreateMotionFilterOptions(
    common::LuaParameterDictionary* parameter_dictionary);

// Takes poses as input and filters them to get fewer poses.
// 将 poses 作为输入并对其进行过滤以得到更少的 poses
// MotionFilter主要作用是对数据进行一下滤波。
// 当两帧数据的间隔时间，两帧的 Pose 跨过的距离，两帧的 Pose 转过的角度等不超过一定的阈值时，
// 认为新的数据提供的信息很少，这些数据可以直接舍去。
class MotionFilter {
 public:
  explicit MotionFilter(const proto::MotionFilterOptions& options);

  // If the accumulated motion (linear, rotational, or time) is above the
  // threshold, returns false. Otherwise the relative motion is accumulated and
  // true is returned.
  // 根据预先设置的阈值，如果累积运动超过提前预设的阈值，则返回 false；
  // 否则返回 true，然后把该数据累加上。
  // MotionFilter 的作用是判断当前位姿、时间和上一时刻对子图进行更新的位姿、时间的间隔是否满足要求，
  // 当间隔过小时，判断为 IsSimilar = true,就不进行更新。不是实时对地图进行更新，是隔一段时间或距离进行更新。
  bool IsSimilar(common::Time time, const transform::Rigid3d& pose);

 private:
  const proto::MotionFilterOptions options_;  // 配置项，主要包括最大时间间隔、最大角度间隔、最大距离间隔等
  int num_total_ = 0;             // 总共的 pose 数
  int num_different_ = 0;         // 过滤之后剩下的 pose 数
  common::Time last_time_;        // 上一次时间
  transform::Rigid3d last_pose_;  // 上一次的 pose
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_MOTION_FILTER_H_
