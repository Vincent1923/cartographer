/*
 * Copyright 2017 The Cartographer Authors
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

#ifndef CARTOGRAPHER_SENSOR_TIMED_POINT_CLOUD_DATA_H_
#define CARTOGRAPHER_SENSOR_TIMED_POINT_CLOUD_DATA_H_

#include "Eigen/Core"
#include "cartographer/common/time.h"
#include "cartographer/sensor/point_cloud.h"

namespace cartographer {
namespace sensor {

// 带时间信息的扫描数据
struct TimedPointCloudData {
  common::Time time;       // 获取最后一个扫描点的时间
  Eigen::Vector3f origin;  // 当次扫描测量时传感器在机器人坐标系下的位置
  // 扫描数据在机器人坐标系下的空间坐标。
  // 每个元素都是一个4维的向量，其中前三维的数据记录了扫描数据在机器人坐标系下的空间坐标，第四维的数据则记录了数据产生的时间。
  sensor::TimedPointCloud ranges;
};

struct TimedPointCloudOriginData {
  struct RangeMeasurement {
    Eigen::Vector4f point_time;
    size_t origin_index;
  };
  common::Time time;
  std::vector<Eigen::Vector3f> origins;
  std::vector<RangeMeasurement> ranges;
};

// Converts 'timed_point_cloud_data' to a proto::TimedPointCloudData.
proto::TimedPointCloudData ToProto(
    const TimedPointCloudData& timed_point_cloud_data);

// Converts 'proto' to TimedPointCloudData.
TimedPointCloudData FromProto(const proto::TimedPointCloudData& proto);

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_SENSOR_TIMED_POINT_CLOUD_DATA_H_
