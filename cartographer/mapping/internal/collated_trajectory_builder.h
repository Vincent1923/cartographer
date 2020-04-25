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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_COLLATED_TRAJECTORY_BUILDER_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_COLLATED_TRAJECTORY_BUILDER_H_

#include <chrono>
#include <map>
#include <memory>
#include <set>
#include <string>

#include "cartographer/common/port.h"
#include "cartographer/common/rate_timer.h"
#include "cartographer/mapping/local_slam_result_data.h"
#include "cartographer/mapping/submaps.h"
#include "cartographer/mapping/trajectory_builder_interface.h"
#include "cartographer/sensor/collator_interface.h"
#include "cartographer/sensor/internal/dispatchable.h"

namespace cartographer {
namespace mapping {

// Collates sensor data using a sensor::CollatorInterface, then passes it on to
// a mapping::TrajectoryBuilderInterface which is common for 2D and 3D.
// 使用 sensor::CollatorInterface 整理传感器数据，然后将其传递到 2D 和 3D 通用的 mapping::TrajectoryBuilderInterface。
//
// CollatedTrajectoryBuilder 类继承自 TrajectoryBuilderInterface。
// 作用：使用 Collator 处理从传感器收集而来的数据，并传递给 GlobalTrajectoryBuilderInterface。
class CollatedTrajectoryBuilder : public TrajectoryBuilderInterface {
 public:
  using SensorId = TrajectoryBuilderInterface::SensorId;

  // 构造函数
  CollatedTrajectoryBuilder(
      sensor::CollatorInterface* sensor_collator, int trajectory_id,
      const std::set<SensorId>& expected_sensor_ids,
      std::unique_ptr<TrajectoryBuilderInterface> wrapped_trajectory_builder);
  ~CollatedTrajectoryBuilder() override;

  CollatedTrajectoryBuilder(const CollatedTrajectoryBuilder&) = delete;
  CollatedTrajectoryBuilder& operator=(const CollatedTrajectoryBuilder&) =
      delete;

  // AddSensorData() 添加传感器数据
  void AddSensorData(
      const std::string& sensor_id,
      const sensor::TimedPointCloudData& timed_point_cloud_data) override {
    AddData(sensor::MakeDispatchable(sensor_id, timed_point_cloud_data));
  }

  void AddSensorData(const std::string& sensor_id,
                     const sensor::ImuData& imu_data) override {
    AddData(sensor::MakeDispatchable(sensor_id, imu_data));
  }

  void AddSensorData(const std::string& sensor_id,
                     const sensor::OdometryData& odometry_data) override {
    AddData(sensor::MakeDispatchable(sensor_id, odometry_data));
  }

  void AddSensorData(
      const std::string& sensor_id,
      const sensor::FixedFramePoseData& fixed_frame_pose_data) override {
    AddData(sensor::MakeDispatchable(sensor_id, fixed_frame_pose_data));
  }

  void AddSensorData(const std::string& sensor_id,
                     const sensor::LandmarkData& landmark_data) override {
    AddData(sensor::MakeDispatchable(sensor_id, landmark_data));
  }

  void AddLocalSlamResultData(std::unique_ptr<mapping::LocalSlamResultData>
                                  local_slam_result_data) override {
    AddData(std::move(local_slam_result_data));
  }

 private:
  void AddData(std::unique_ptr<sensor::Data> data);

  void HandleCollatedSensorData(const std::string& sensor_id,
                                std::unique_ptr<sensor::Data> data);

  // 成员变量
  sensor::CollatorInterface* const sensor_collator_;  // 传感器收集类实例
  const int trajectory_id_;  // 轨迹 id
  std::unique_ptr<TrajectoryBuilderInterface> wrapped_trajectory_builder_;  // 全局的建图接口

  // Time at which we last logged the rates of incoming sensor data.
  // 我们上次记录传入传感器数据速率的时间。
  std::chrono::steady_clock::time_point last_logging_time_;  // 上一次传递数据时间
  std::map<std::string, common::RateTimer<>> rate_timers_;   // 频率
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_COLLATED_TRAJECTORY_BUILDER_H_
