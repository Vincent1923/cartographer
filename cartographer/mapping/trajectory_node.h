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

#ifndef CARTOGRAPHER_MAPPING_TRAJECTORY_NODE_H_
#define CARTOGRAPHER_MAPPING_TRAJECTORY_NODE_H_

#include <memory>
#include <vector>

#include "Eigen/Core"
#include "cartographer/common/optional.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/proto/trajectory_node_data.pb.h"
#include "cartographer/sensor/range_data.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace mapping {

struct TrajectoryNodePose {
  struct ConstantPoseData {
    common::Time time;
    transform::Rigid3d local_pose;
  };
  // The node pose in the global SLAM frame.
  transform::Rigid3d global_pose;

  common::optional<ConstantPoseData> constant_pose_data;
};

struct TrajectoryNode {
  // TrajectoryNode::Data 是存储这个节点时的状态，包括时间、传感器数据等信息。
  struct Data {
    // 字段 time 记录了扫描数据被插入子图的时刻，在 TrajectoryNode 中可以通过函数 time() 获取。
    common::Time time;  // 当前帧的时间

    // Transform to approximately gravity align the tracking frame as
    // determined by local SLAM.
    // 一个表示旋转矩阵的四元数。该旋转矩阵将非水平面的传感器数据投射到水平面上，
    // 利用IMU的重力传感器可计算出该旋转矩阵。
    //
    // 字段 gravity_alignment 是当时的重力方向。
    Eigen::Quaterniond gravity_alignment;

    // Used for loop closure in 2D: voxel filtered returns in the
    // 'gravity_alignment' frame.
    // 用于 2D loop closure："gravity_alignment" 坐标系下经过体素滤波后的返回（结果）。
    // 经过水平投射后的点云数据，可用于 2D 情况下做 Loop Closure。
    //
    // 根据重力方向修正之后的点云数据。
    sensor::PointCloud filtered_gravity_aligned_point_cloud;

    // Used for loop closure in 3D.
    sensor::PointCloud high_resolution_point_cloud;     // 高分辨率点云
    sensor::PointCloud low_resolution_point_cloud;      // 低分辨率点云
    Eigen::VectorXf rotational_scan_matcher_histogram;  // 旋转匹配直方图；VectorXf 是一个长度可变的向量。

    // The node pose in the local SLAM frame.
    // 节点在 Local SLAM 中的位姿
    //
    // local_pose 则记录了节点在局部地图坐标系下的位姿。
    transform::Rigid3d local_pose;
  };

  // 返回成员变量 constant_data 的时间
  common::Time time() const { return constant_data->time; }

  // This must be a shared_ptr. If the data is used for visualization while the
  // node is being trimmed, it must survive until all use finishes.
  // 这必须是共享指针 shared_ptr。如果在修剪节点时将数据用于可视化，则数据必须保留到所有使用完成为止。
  //
  // 插入的节点数据，数据类型为内部定义的结构体 Data，记录了点云、重力方向、局部位姿等数据。
  // 共享指针，允许多个指针指向同一个对象。
  std::shared_ptr<const Data> constant_data;

  // The node pose in the global SLAM frame.
  // 节点在世界坐标系下的位姿
  transform::Rigid3d global_pose;
};

proto::TrajectoryNodeData ToProto(const TrajectoryNode::Data& constant_data);
TrajectoryNode::Data FromProto(const proto::TrajectoryNodeData& proto);

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_TRAJECTORY_NODE_H_
