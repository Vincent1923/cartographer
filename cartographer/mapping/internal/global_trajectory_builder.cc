/*
 * Copyright 2018 The Cartographer Authors
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

#include "cartographer/mapping/internal/global_trajectory_builder.h"

#include <memory>

#include "cartographer/common/make_unique.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/local_slam_result_data.h"
#include "cartographer/metrics/family_factory.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace {

static auto* kLocalSlamMatchingResults = metrics::Counter::Null();
static auto* kLocalSlamInsertionResults = metrics::Counter::Null();

// GlobalTrajectoryBuilder 是连接前端与后端的桥梁，它继承了 TrajectoryBuilderInterface。
// GlobalTrajectoryBuilder 实际上是一个模板类。其模板列表中的 LocalTrajectoryBuilder 和 PoseGraph 分别是前端和后端的两个核心类型。
// LocalTrajectoryBuilder 负责接收来自激光雷达的数据，进行扫描匹配，估计机器人位姿，并将传感器数据插入子图中，更新子图。
// PoseGraph 在后台进行闭环检测全局优化。
template <typename LocalTrajectoryBuilder, typename PoseGraph>
class GlobalTrajectoryBuilder : public mapping::TrajectoryBuilderInterface {
 public:
  // Passing a 'nullptr' for 'local_trajectory_builder' is acceptable, but no
  // 'TimedPointCloudData' may be added in that case.
  // 为 "local_trajectory_builder" 传递 "nullptr" 是可以接受的，但在这种情况下，不能添加 "TimedPointCloudData"。
  /**
   * @brief GlobalTrajectoryBuilder     构造函数，它有四个输入参数，分别记录了轨迹跟踪器、轨迹索引、位姿图、前端回调函数。
   *                                    这些参数在成员构造列表中，被拿来一一构建成员变量。函数体是空的没有任何操作。
   * @param local_trajectory_builder    位姿跟踪器，前端的核心对象，其数据类型是一个模板参数
   * @param trajectory_id               轨迹索引
   * @param pose_graph                  位姿图，后端的核心对象，其数据类型是一个模板参数
   * @param local_slam_result_callback  前端数据更新后的回调函数
   */
  GlobalTrajectoryBuilder(
      std::unique_ptr<LocalTrajectoryBuilder> local_trajectory_builder,
      const int trajectory_id, PoseGraph* const pose_graph,
      const LocalSlamResultCallback& local_slam_result_callback)
      : trajectory_id_(trajectory_id),
        pose_graph_(pose_graph),
        local_trajectory_builder_(std::move(local_trajectory_builder)),
        local_slam_result_callback_(local_slam_result_callback) {}
  // 析构函数，函数体也是空的什么也没做。
  ~GlobalTrajectoryBuilder() override {}

  // 屏蔽了拷贝构造和拷贝赋值两个构造对象的途径。
  GlobalTrajectoryBuilder(const GlobalTrajectoryBuilder&) = delete;
  GlobalTrajectoryBuilder& operator=(const GlobalTrajectoryBuilder&) = delete;

  /**
   * @brief AddSensorData           处理点云数据，可以说该接口控制了整个 Cartographer 系统的运行过程。
   * @param sensor_id               点云数据的传感器主题名称
   * @param timed_point_cloud_data  机器人坐标系下的激光扫描数据。cartographer_ros 使用 SensorBridge将 ROS 系统中的激光扫描数据
   *                                转换成这里的 sensor::TimedPointCloudData 类型。
   *                                TimedPointCloudData 类型的扫描数据包含三个字段，其中 time 是获取最后一个扫描点的时间，
   *                                origin 是当次扫描测量时传感器在机器人坐标系下的位置，而 ranges 则是扫描数据在机器人坐标系下的空间坐标。
   */
  void AddSensorData(
      const std::string& sensor_id,
      const sensor::TimedPointCloudData& timed_point_cloud_data) override {
    // 检查一下前端核心对象是否存在
    CHECK(local_trajectory_builder_)
        << "Cannot add TimedPointCloudData without a LocalTrajectoryBuilder.";
    // 如果存在就通过它的成员函数 AddRangeData() 完成 Local SLAM 的业务主线。
    // 如果一切正常，就会返回扫描匹配的结果，在该结果中同时记录了子图的更新信息。
    // 返回结果通过智能指针 matching_result 记录。
    // 所以，matching_result 记录了前端 local_trajectory_builder_ 进行扫描匹配的结果。
    std::unique_ptr<typename LocalTrajectoryBuilder::MatchingResult>
        matching_result = local_trajectory_builder_->AddRangeData(
            sensor_id, timed_point_cloud_data);
    if (matching_result == nullptr) {
      // The range data has not been fully accumulated yet.
      return;
    }
    // 前端工作完成之后，GlobalTrajectoryBuilder 就要将前端的输出结果喂给后端进行闭环检测和全局优化。
    // 首先控制计数器 kLocalSlamMatchingResults 自增，记录下前端的输出次数。
    kLocalSlamMatchingResults->Increment();
    std::unique_ptr<InsertionResult> insertion_result;
    // 通过查询扫描匹配结果 matching_result 的字段 insertion_result 判定前端是否成功地将传感器的数据插入到子图中
    if (matching_result->insertion_result != nullptr) {
      kLocalSlamInsertionResults->Increment();
      // 若是，则通过后端的位姿图接口 AddNode() 创建一个轨迹节点，并把前端的输出结果喂给后端。
      // 如果一切正常，我们就会得到新建的轨迹节点索引，它被记录在临时对象 node_id 中。
      // 所以，node_id 是后端的位姿图 pose_graph_ 新建的轨迹节点索引。
      auto node_id = pose_graph_->AddNode(
          matching_result->insertion_result->constant_data, trajectory_id_,
          matching_result->insertion_result->insertion_submaps);
      CHECK_EQ(node_id.trajectory_id, trajectory_id_);
      // 最后结合后端输出的节点索引 node_id，以及前端输出的扫描匹配结果 matching_result 实例化对象 insertion_result
      insertion_result = common::make_unique<InsertionResult>(InsertionResult{
          node_id, matching_result->insertion_result->constant_data,
          std::vector<std::shared_ptr<const Submap>>(
              matching_result->insertion_result->insertion_submaps.begin(),
              matching_result->insertion_result->insertion_submaps.end())});
    }
    // 最后，如果我们提供了回调函数，就调用回调函数，并将前端的输出和刚刚构建的 insertion_result 对象传参。
    if (local_slam_result_callback_) {
      local_slam_result_callback_(
          trajectory_id_, matching_result->time, matching_result->local_pose,
          std::move(matching_result->range_data_in_local),
          std::move(insertion_result));
    }
  }

  // 由于 IMU 和里程计的数据都可以拿来通过积分运算进行局部的定位，所以这两个传感器的数据处理方式基本一样。
  /**
   * @brief AddSensorData  处理 IMU 数据
   * @param sensor_id      IMU 数据的传感器主题名称
   * @param imu_data       IMU 数据。cartographer_ros 使用 SensorBridge 将 ROS 系统中的 IMU 消息
   *                       转换成这里的 sensor::ImuData 类型。
   */
  void AddSensorData(const std::string& sensor_id,
                     const sensor::ImuData& imu_data) override {
    // 判断前端核心对象是否存在
    if (local_trajectory_builder_) {
      // 将 IMU 数据喂给前端对象进行局部定位
      local_trajectory_builder_->AddImuData(imu_data);
    }
    // 通过后端的位姿图 pose_graph_ 将传感器的信息添加到全局地图中
    pose_graph_->AddImuData(trajectory_id_, imu_data);
  }

  /**
   * @brief AddSensorData  处理里程计数据
   * @param sensor_id      里程计数据的传感器主题名称
   * @param imu_data       里程计数据。cartographer_ros 使用 SensorBridge 将 ROS 系统中的里程计消息
   *                       转换成这里的 sensor::OdometryData 类型。
   */
  void AddSensorData(const std::string& sensor_id,
                     const sensor::OdometryData& odometry_data) override {
    CHECK(odometry_data.pose.IsValid()) << odometry_data.pose;
    // 判断前端核心对象是否存在
    if (local_trajectory_builder_) {
      // 将里程计数据喂给前端对象进行局部定位
      local_trajectory_builder_->AddOdometryData(odometry_data);
    }
    // 通过后端的位姿图 pose_graph_ 将传感器的信息添加到全局地图中
    pose_graph_->AddOdometryData(trajectory_id_, odometry_data);
  }

  // 在 Cartographer 中将类似于 GPS 这种具有全局定位能力的传感器输出的位姿称为固定坐标系位姿(fixed frame pose)。
  // 由于它们的测量结果是全局的信息，所以没有喂给前端用于局部定位。
  void AddSensorData(
      const std::string& sensor_id,
      const sensor::FixedFramePoseData& fixed_frame_pose) override {
    if (fixed_frame_pose.pose.has_value()) {
      CHECK(fixed_frame_pose.pose.value().IsValid())
          << fixed_frame_pose.pose.value();
    }
    pose_graph_->AddFixedFramePoseData(trajectory_id_, fixed_frame_pose);
  }

  // 路标数据也可以认为是全局的定位信息，也直接喂给了后端
  void AddSensorData(const std::string& sensor_id,
                     const sensor::LandmarkData& landmark_data) override {
    pose_graph_->AddLandmarkData(trajectory_id_, landmark_data);
  }

  // 最后是关于直接给后端添加 Local SLAM 的结果数据的接口。因为我们的前端对象的数据类型是 LocalTrajectoryBuilder2D，
  // 所以如果前端对象存在就不能调用该接口。
  void AddLocalSlamResultData(std::unique_ptr<mapping::LocalSlamResultData>
                                  local_slam_result_data) override {
    CHECK(!local_trajectory_builder_) << "Can't add LocalSlamResultData with "
                                         "local_trajectory_builder_ present.";
    local_slam_result_data->AddToPoseGraph(trajectory_id_, pose_graph_);
  }

 private:
  const int trajectory_id_;
  PoseGraph* const pose_graph_;
  std::unique_ptr<LocalTrajectoryBuilder> local_trajectory_builder_;
  LocalSlamResultCallback local_slam_result_callback_;
};

}  // namespace

// 构建一个 GlobalTrajectoryBuilder 类型的对象。它是连接前端与后端的桥梁，继承了 TrajectoryBuilderInterface。
// 它有四个输入参数。
// local_trajectory_builder: 位姿跟踪器，前端的核心对象，其数据类型是一个模板参数。
// trajectory_id: 轨迹索引。
// pose_graph: 位姿图，后端的核心对象，其数据类型是一个模板参数。
// local_slam_result_callback: 前端数据更新后的回调函数。
std::unique_ptr<TrajectoryBuilderInterface> CreateGlobalTrajectoryBuilder2D(
    std::unique_ptr<LocalTrajectoryBuilder2D> local_trajectory_builder,
    const int trajectory_id, mapping::PoseGraph2D* const pose_graph,
    const TrajectoryBuilderInterface::LocalSlamResultCallback&
        local_slam_result_callback) {
  // 生成一个 GlobalTrajectoryBuilder 的智能指针。它实际上是一个模板类。
  // 这里传入的模板为 LocalTrajectoryBuilder2D 和 PoseGraph2D，表示 2D 构图。
  return common::make_unique<
      GlobalTrajectoryBuilder<LocalTrajectoryBuilder2D, mapping::PoseGraph2D>>(
      std::move(local_trajectory_builder), trajectory_id, pose_graph,
      local_slam_result_callback);
}

std::unique_ptr<TrajectoryBuilderInterface> CreateGlobalTrajectoryBuilder3D(
    std::unique_ptr<LocalTrajectoryBuilder3D> local_trajectory_builder,
    const int trajectory_id, mapping::PoseGraph3D* const pose_graph,
    const TrajectoryBuilderInterface::LocalSlamResultCallback&
        local_slam_result_callback) {
  return common::make_unique<
      GlobalTrajectoryBuilder<LocalTrajectoryBuilder3D, mapping::PoseGraph3D>>(
      std::move(local_trajectory_builder), trajectory_id, pose_graph,
      local_slam_result_callback);
}

void GlobalTrajectoryBuilderRegisterMetrics(metrics::FamilyFactory* factory) {
  auto* results = factory->NewCounterFamily(
      "mapping_internal_global_trajectory_builder_local_slam_results",
      "Local SLAM results");
  kLocalSlamMatchingResults = results->Add({{"type", "MatchingResult"}});
  kLocalSlamInsertionResults = results->Add({{"type", "InsertionResult"}});
}

}  // namespace mapping
}  // namespace cartographer
