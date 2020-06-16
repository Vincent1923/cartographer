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

#ifndef CARTOGRAPHER_SENSOR_RANGE_DATA_H_
#define CARTOGRAPHER_SENSOR_RANGE_DATA_H_

#include "cartographer/common/port.h"
#include "cartographer/sensor/compressed_point_cloud.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/sensor/proto/sensor.pb.h"

namespace cartographer {
namespace sensor {

// Rays begin at 'origin'. 'returns' are the points where obstructions were
// detected. 'misses' are points in the direction of rays for which no return
// was detected, and were inserted at a configured distance. It is assumed that
// between the 'origin' and 'misses' is free space.
// 光线以 "origin" 开始。"returns" 是检测到障碍物的点。"misses" 是在光线方向上未检测到返回的点，并以配置的距离插入。
// 假定 "origin" 和 "misses" 之间是自由空间。
/**
 * 1. RangeData 是 Cartographer 定义的激光雷达传感器测量数据的存储结构。
 * 2. 它有三个字段，其中 origin 用于描述当次扫描测量时激光雷达的位置，字段 returns 和 misses 记录了扫描到的 hit 点与 miss 点。
 *    所谓的 hit 点是指在该点上扫描到了障碍物，该点所在的栅格单元就发生了一次 hit 事件。
 *    miss 点所在的位置上并没有检测到障碍物，只是以传感器的最远有效距离记录下坐标而已。
 *    origin 到 hit 点和 miss 点之间的空间都将被认为是空闲的，所对应的栅格单元则被看作发生了一次 miss 事件。
 * 3. 字段 returns 和 misses 的数据类型是定义在 "point_cloud.h" 中的三维向量。
 *    对于我们的二维地图而言，z轴的数据也就是向量的第三个元素为0。
 */
struct RangeData {
  Eigen::Vector3f origin;  // 描述当次扫描测量时激光雷达的位置
  PointCloud returns;      // 记录了扫描到的 hit 点
  PointCloud misses;       // 记录了扫描到的 miss 点
};

// Like 'RangeData', but with 'TimedPointClouds'.
struct TimedRangeData {
  Eigen::Vector3f origin;
  TimedPointCloud returns;
  TimedPointCloud misses;
};

RangeData TransformRangeData(const RangeData& range_data,
                             const transform::Rigid3f& transform);

TimedRangeData TransformTimedRangeData(const TimedRangeData& range_data,
                                       const transform::Rigid3f& transform);

// Crops 'range_data' according to the region defined by 'min_z' and 'max_z'.
RangeData CropRangeData(const RangeData& range_data, float min_z, float max_z);

// Crops 'range_data' according to the region defined by 'min_z' and 'max_z'.
TimedRangeData CropTimedRangeData(const TimedRangeData& range_data, float min_z,
                                  float max_z);

// Converts 'range_data' to a proto::RangeData.
proto::RangeData ToProto(const RangeData& range_data);

// Converts 'proto' to RangeData.
RangeData FromProto(const proto::RangeData& proto);

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_SENSOR_RANGE_DATA_H_
