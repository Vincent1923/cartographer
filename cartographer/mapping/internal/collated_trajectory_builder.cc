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

#include "cartographer/mapping/internal/collated_trajectory_builder.h"

#include "cartographer/common/time.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

namespace {

constexpr double kSensorDataRatesLoggingPeriodSeconds = 15.;

}  // namespace

// 构造函数
// sensor_collator：传感器收集类实例，将多传感器采集的数据归并到轨迹上。
// trajectory_id：轨迹线 id。
// expected_sensor_ids：路径 trajectory_id 上的传感器 id。
// wrapped_trajectory_builder：全局的建图接口。在 map_builder_ 中传入的数据类型为 GlobalTrajectoryBuilder。
//                             GlobalTrajectoryBuilder 继承自 TrajectoryBuilderInterface。
CollatedTrajectoryBuilder::CollatedTrajectoryBuilder(
    sensor::CollatorInterface* const sensor_collator, const int trajectory_id,
    const std::set<SensorId>& expected_sensor_ids,
    std::unique_ptr<TrajectoryBuilderInterface> wrapped_trajectory_builder)
    : sensor_collator_(sensor_collator),
      trajectory_id_(trajectory_id),
      wrapped_trajectory_builder_(std::move(wrapped_trajectory_builder)),
      // last_logging_time_ 赋值为当前时间，std::chrono::steady_clock::now() 代表当前时间的时间点。
      last_logging_time_(std::chrono::steady_clock::now()) {
  // 获取 trajectory_id 路径上的传感器 id
  std::unordered_set<std::string> expected_sensor_id_strings;
  for (const auto& sensor_id : expected_sensor_ids) {
    expected_sensor_id_strings.insert(sensor_id.id);
  }
  // 添加一个轨迹线，接收有序的传感器数据，并使用 callback 回调函数 HandleCollatedSensorData() 处理 data
  sensor_collator_->AddTrajectory(
      trajectory_id, expected_sensor_id_strings,
      [this](const std::string& sensor_id, std::unique_ptr<sensor::Data> data) {
        HandleCollatedSensorData(sensor_id, std::move(data));
      });
}

CollatedTrajectoryBuilder::~CollatedTrajectoryBuilder() {}

void CollatedTrajectoryBuilder::AddData(std::unique_ptr<sensor::Data> data) {
  sensor_collator_->AddSensorData(trajectory_id_, std::move(data));
}

void CollatedTrajectoryBuilder::HandleCollatedSensorData(
    const std::string& sensor_id, std::unique_ptr<sensor::Data> data) {
  auto it = rate_timers_.find(sensor_id);
  if (it == rate_timers_.end()) {
    it = rate_timers_
             .emplace(
                 std::piecewise_construct, std::forward_as_tuple(sensor_id),
                 std::forward_as_tuple(
                     common::FromSeconds(kSensorDataRatesLoggingPeriodSeconds)))
             .first;
  }
  it->second.Pulse(data->GetTime());

  if (std::chrono::steady_clock::now() - last_logging_time_ >
      common::FromSeconds(kSensorDataRatesLoggingPeriodSeconds)) {
    for (const auto& pair : rate_timers_) {
      LOG(INFO) << pair.first << " rate: " << pair.second.DebugString();
    }
    last_logging_time_ = std::chrono::steady_clock::now();
  }

  data->AddToTrajectoryBuilder(wrapped_trajectory_builder_.get());
}

}  // namespace mapping
}  // namespace cartographer
