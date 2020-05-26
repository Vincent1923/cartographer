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

#ifndef CARTOGRAPHER_SENSOR_POINT_CLOUD_H_
#define CARTOGRAPHER_SENSOR_POINT_CLOUD_H_

#include <vector>

#include "Eigen/Core"
#include "cartographer/sensor/proto/sensor.pb.h"
#include "cartographer/transform/rigid_transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace sensor {

// Stores 3D positions of points.
// For 2D points, the third entry is 0.f.
// 存储点的3D位置。对于2D点，第三个元素是0.f。
typedef std::vector<Eigen::Vector3f> PointCloud;

// Stores 3D positions of points with their relative measurement time in the
// fourth entry. Time is in seconds, increasing and relative to the moment when
// the last point was acquired. So, the fourth entry for the last point is 0.f.
// If timing is not available, all fourth entries are 0.f. For 2D points, the
// third entry is 0.f (and the fourth entry is time).
// 前三项存储点的3D位置，第四项存储相对测量时间。时间以秒为单位，每一个点和获取最后一点的时间相比较，每一个点的时间逐渐增加。
// 因此，最后一个点的第四项是0.f。如果计时不可用，则所有第四项均为0.f。对于2D点，第三项是0.f（第四项是时间）。
//
// 每个元素都是一个4维的向量，其中前三维的数据记录了扫描数据所对应的空间坐标，第四维的数据则记录了数据产生的时间。
typedef std::vector<Eigen::Vector4f> TimedPointCloud;

// 带相对测量时间和强度的点云数据
struct PointCloudWithIntensities {
  TimedPointCloud points;          // 记录了一次扫描的各个测量值，带相对测量时间的点云数据
  std::vector<float> intensities;  // 描述了 points 中每个元素所对应的数据强度
};

// Transforms 'point_cloud' according to 'transform'.
PointCloud TransformPointCloud(const PointCloud& point_cloud,
                               const transform::Rigid3f& transform);

/**
 * @brief TransformTimedPointCloud  根据变换矩阵 'transform' 对点云数据 'point_cloud' 进行变换
 * @param point_cloud               带有时间信息的点云数据
 * @param transform                 变换矩阵
 * @return                          经过变换后的点云数据
 */
// Transforms 'point_cloud' according to 'transform'.
// 根据变换矩阵 'transform' 对点云数据 'point_cloud' 进行变换
TimedPointCloud TransformTimedPointCloud(const TimedPointCloud& point_cloud,
                                         const transform::Rigid3f& transform);

// Returns a new point cloud without points that fall outside the region defined
// by 'min_z' and 'max_z'.
PointCloud CropPointCloud(const PointCloud& point_cloud, float min_z,
                          float max_z);

// Returns a new point cloud without points that fall outside the region defined
// by 'min_z' and 'max_z'.
TimedPointCloud CropTimedPointCloud(const TimedPointCloud& point_cloud,
                                    float min_z, float max_z);

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_SENSOR_POINT_CLOUD_H_
