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

#ifndef CARTOGRAPHER_MAPPING_TRAJECTORY_BUILDER_INTERFACE_H_
#define CARTOGRAPHER_MAPPING_TRAJECTORY_BUILDER_INTERFACE_H_

#include <functional>
#include <memory>
#include <string>

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/port.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/proto/trajectory_builder_options.pb.h"
#include "cartographer/mapping/submaps.h"
#include "cartographer/sensor/fixed_frame_pose_data.h"
#include "cartographer/sensor/imu_data.h"
#include "cartographer/sensor/landmark_data.h"
#include "cartographer/sensor/odometry_data.h"
#include "cartographer/sensor/timed_point_cloud_data.h"

namespace cartographer {
namespace mapping {

proto::TrajectoryBuilderOptions CreateTrajectoryBuilderOptions(
    common::LuaParameterDictionary* const parameter_dictionary);

class LocalSlamResultData;

// This interface is used for both 2D and 3D SLAM. Implementations wire up a
// global SLAM stack, i.e. local SLAM for initial pose estimates, scan matching
// to detect loop closure, and a sparse pose graph optimization to compute
// optimized pose estimates.
// 该接口用于 2D 和 3D SLAM。实现连接 global SLAM，例如用于初始位姿估计的 local SLAM，用于闭环检测的 scan matching，
// 以及用于计算优化后的位姿估计的稀疏位姿图优化。
//
// 接口类 TrajectoryBuilderInterface 是为了给 2D 和 3D 建图提供一个统一的访问接口。
// 通过该接口我们可以获得一个完整的 SLAM 技术栈，包括局部建图方法、局部位姿估计、闭环检测方法、以及面向稀疏位姿图的全局优化方法。
class TrajectoryBuilderInterface {
 public:
  // 结构体 InsertionResult 描述前端的一次子图更新操作，将传感器的扫描数据插入子图中。
  struct InsertionResult {
    // NodeId 是一个结构体，它有两个字段分别记录了轨迹的索引(trajectory_id)以及一个从零开始计数的节点编号(node_index)。
    // 根据 NodeId 的定义，我们可以把一条轨迹理解为由若干个节点串联起来的一个数据结构。
    NodeId node_id;
    // constant_data 记录了子图更新时在局部地图中的位姿，以及有传感器原始数据转换之后的点云信息。此外还记录了更新子图的时刻和重力方向。
    std::shared_ptr<const TrajectoryNode::Data> constant_data;
    // insertion_submaps 则记录了被更新的子图对象
    std::vector<std::shared_ptr<const Submap>> insertion_submaps;
  };

  // A callback which is called after local SLAM processes an accumulated
  // 'sensor::RangeData'. If the data was inserted into a submap, reports the
  // assigned 'NodeId', otherwise 'nullptr' if the data was filtered out.
  // 在局部 SLAM 处理累积的 "sensor::RangeData" 之后调用的回调函数。
  // 如果数据插入到子图中，则报告分配的 "NodeId"，否则数据被滤除则报告 "nullptr"。
  //
  // 通过关键字 using 和 std::function 定义了回调函数 LocalSlamResultCallback 的类型。
  // 回调函数有5个参数，分别记录了轨迹索引、更新子图的时间、局部的位姿估计、激光传感器的扫描数据在局部坐标系下的点云信息、子图更新结果。
  // 在我们的 demo 中最终会调用到 cartographer_ros 中定义的函数 OnLocalSlamResult。
  using LocalSlamResultCallback =
      std::function<void(int /* trajectory ID */, common::Time,
                         transform::Rigid3d /* local pose estimate */,
                         sensor::RangeData /* in local frame */,
                         std::unique_ptr<const InsertionResult>)>;

  // 定义传感器类型的结构体
  struct SensorId {
    enum class SensorType {
      RANGE = 0,         // Range 传感器。激光或其他可提供点云数据的传感器，如 Kinect。
      IMU,               // IMU 数据
      ODOMETRY,          // 里程计
      FIXED_FRAME_POSE,
      LANDMARK,
      LOCAL_SLAM_RESULT
    };

    SensorType type;  // 传感器类型
    std::string id;   // 传感器 id

    bool operator==(const SensorId& other) const {
      return std::forward_as_tuple(type, id) ==
             std::forward_as_tuple(other.type, other.id);
    }

    bool operator<(const SensorId& other) const {
      return std::forward_as_tuple(type, id) <
             std::forward_as_tuple(other.type, other.id);
    }
  };

  // 构造函数
  TrajectoryBuilderInterface() {}
  virtual ~TrajectoryBuilderInterface() {}

  // 等号重载
  TrajectoryBuilderInterface(const TrajectoryBuilderInterface&) = delete;
  TrajectoryBuilderInterface& operator=(const TrajectoryBuilderInterface&) =
      delete;

  // 处理传感器数据的5个纯虚函数
  // 针对不同的传感器数据，接口类 TrajectoryBuilderInterface 定义了5个 AddSensorData() 接口。
  virtual void AddSensorData(
      const std::string& sensor_id,
      const sensor::TimedPointCloudData& timed_point_cloud_data) = 0;
  virtual void AddSensorData(const std::string& sensor_id,
                             const sensor::ImuData& imu_data) = 0;
  virtual void AddSensorData(const std::string& sensor_id,
                             const sensor::OdometryData& odometry_data) = 0;
  virtual void AddSensorData(
      const std::string& sensor_id,
      const sensor::FixedFramePoseData& fixed_frame_pose) = 0;
  virtual void AddSensorData(const std::string& sensor_id,
                             const sensor::LandmarkData& landmark_data) = 0;
  // Allows to directly add local SLAM results to the 'PoseGraph'. Note that it
  // is invalid to add local SLAM results for a trajectory that has a
  // 'LocalTrajectoryBuilder2D/3D'.
  // 允许直接将 local SLAM 结果添加到 "PoseGraph" 中。请注意，为具有 "LocalTrajectoryBuilder2D/3D"
  // 的轨迹添加 local SLAM 结果是无效的。
  //
  // 提供一个接口 AddLocalSlamResultData() 来直接将一个 Local SLAM 的结果添加到后端位姿图上。
  // 但是那些使用 LocalTrajectoryBuilder2D/3D 作为前端的对象不能使用该接口。
  virtual void AddLocalSlamResultData(
      std::unique_ptr<mapping::LocalSlamResultData> local_slam_result_data) = 0;
};

proto::SensorId ToProto(const TrajectoryBuilderInterface::SensorId& sensor_id);
TrajectoryBuilderInterface::SensorId FromProto(
    const proto::SensorId& sensor_id_proto);

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_TRAJECTORY_BUILDER_INTERFACE_H_
