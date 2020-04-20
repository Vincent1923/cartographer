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

#include "cartographer/mapping/internal/motion_filter.h"

#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

// 以下三个阈值定义在配置文件 /src/cartographer/configuration_files/trajectory_builder_2d.lua 中
proto::MotionFilterOptions CreateMotionFilterOptions(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::MotionFilterOptions options;
  options.set_max_time_seconds(
      parameter_dictionary->GetDouble("max_time_seconds"));     // max_time_seconds = 5.0
  options.set_max_distance_meters(
      parameter_dictionary->GetDouble("max_distance_meters"));  // max_distance_meters = 0.2
  options.set_max_angle_radians(
      parameter_dictionary->GetDouble("max_angle_radians"));    // max_angle_radians = math.rad(1.)
  return options;
}

MotionFilter::MotionFilter(const proto::MotionFilterOptions& options)
    : options_(options) {}

// 根据预先设置的阈值，如果累积运动超过提前预设的阈值，则返回 false；否则返回 true，然后把该数据累加上。
bool MotionFilter::IsSimilar(const common::Time time,
                             const transform::Rigid3d& pose) {
  // LOG_IF_EVERY_N(INFO, num_total_ >= 500, 500) 表示当 num_total_ >= 500 的条件连续成立 500 次的时候记录日志信息。
  LOG_IF_EVERY_N(INFO, num_total_ >= 500, 500)
      << "Motion filter reduced the number of nodes to "
      << 100. * num_different_ / num_total_ << "%.";
  ++num_total_;
  // 以下三个阈值定义在配置文件 /src/cartographer/configuration_files/trajectory_builder_2d.lua 中
  if (num_total_ > 1 &&  // 总的 pose 数大于1
      time - last_time_ <= common::FromSeconds(options_.max_time_seconds()) &&  // 时间间隔小于给定阈值
      (pose.translation() - last_pose_.translation()).norm() <=  // 移动距离小于给定阈值
          options_.max_distance_meters() &&
      transform::GetAngle(pose.inverse() * last_pose_) <=  // 偏转角度小于给定阈值。已知前后位姿求变换矩阵？
          options_.max_angle_radians()) {
    return true;
  }
  last_time_ = time;
  last_pose_ = pose;
  ++num_different_;
  return false;
}

}  // namespace mapping
}  // namespace cartographer
