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
// 光线以“origin”开始。“returns”是检测到障碍物的点。“misses”是在光线方向上未检测到返回的点，并以配置的距离插入。
// 假定“origin”和“misses”之间是自由空间。
// origin 是一个3维向量，表示的是这一帧数据的原点。
// returns 是那些检测到了 hits 的点，那么被 Hits 的点加入 hits 这个集合，相应地更改 submap 中的信息。
// 而 origin 和 hits 中间的点也是 Free 的，需要归入到 misses 集合中。
// misses 是那些没有检测到 return 的点，原点和 misses 形成的射线上的所有点都是 free Space，都需要归入 misses 集合中。
// 上述所有的点的表示里，如果是 2d 情况的话，第三个元素是0。
struct RangeData {
  Eigen::Vector3f origin;
  PointCloud returns;
  PointCloud misses;
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
