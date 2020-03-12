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
// 该接口用于2D和3D SLAM。实现连接global SLAM，例如用于初始位姿估计的local SLAM，用于闭环检测的scan matching，
// 以及用于计算优化后的位姿估计的稀疏位姿图优化。
class TrajectoryBuilderInterface {
 public:
  // InsertionResult就是用来保存插入Local Slam的一个节点的数据结构
  struct InsertionResult {
    /*
     * NodeId是一个结构体，在/mapping/id.h中定义，
     * 包含两个部分：一个int型的trajectory_id和一个int型的node_index，
     * 它使用唯一的trajectory ID和该trajectory内的节点的从零开始的索引的组合唯一地识别一个trajectory node。
     */
    NodeId node_id;
    /*
     * TrajectoryNode是一个结构体，在/mapping/trajectory_node.h中定义，
     * TrajectoryNode::Data包含了经过处理的一帧传感器数据。
     */
    std::shared_ptr<const TrajectoryNode::Data> constant_data;
    // 已经建立起来的子图列表
    std::vector<std::shared_ptr<const Submap>> insertion_submaps;
  };

  // A callback which is called after local SLAM processes an accumulated
  // 'sensor::RangeData'. If the data was inserted into a submap, reports the
  // assigned 'NodeId', otherwise 'nullptr' if the data was filtered out.
  // 这个回调函数是当一个 RangeData 数据进来时被调用，主要有5个参数：trajectory_id，
  // 该帧数据的插入时刻 Time, 插入到 Local Slam 中的 pose，RangeData 数据本身和插入的结果 InsertionResult。
  // InsertionResult 的结构我们进行了详细地分析。
  // 如果该帧数据成功插入了 submap 中则返回节点 id:NodeId，否则返回空指针
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
  // 允许直接将本地 SLAM 结果添加到“PoseGraph”中。请注意，为具有“LocalTrajectoryBuilder2D/3D”
  // 的轨迹添加本地 SLAM 结果是无效的。
  // 直接将 Local Slam 的结果添加到 PoseGraph 的函数
  virtual void AddLocalSlamResultData(
      std::unique_ptr<mapping::LocalSlamResultData> local_slam_result_data) = 0;
};

proto::SensorId ToProto(const TrajectoryBuilderInterface::SensorId& sensor_id);
TrajectoryBuilderInterface::SensorId FromProto(
    const proto::SensorId& sensor_id_proto);

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_TRAJECTORY_BUILDER_INTERFACE_H_
