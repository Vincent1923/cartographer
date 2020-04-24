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

#include "cartographer/sensor/internal/collator.h"

namespace cartographer {
namespace sensor {

// 添加一个轨迹线,接收有序的传感器数据,并使用 callback 回调处理 data。
// 一个轨迹线，多个传感器。
// Callback 在 “/cartographer/sensor/colator_interface.h” 定义：
//   using Callback =
//      std::function<void(const std::string&, std::unique_ptr<Data>)>;
void Collator::AddTrajectory(
    const int trajectory_id,
    const std::unordered_set<std::string>& expected_sensor_ids,
    const Callback& callback) {
  // 遍历 expected_sensor_ids
  for (const auto& sensor_id : expected_sensor_ids) {
    /*
     * 对于每一个轨迹线+传感器，设置一个 key。
     * struct QueueKey {
     *   int trajectory_id;  // 轨线 id
     *   string sensor_id;   // 传感器 id
     * }
     */
    const auto queue_key = QueueKey{trajectory_id, sensor_id};
    // 添加一个名为 queue_key 的队列，并设置回调函数处理 data
    queue_.AddQueue(queue_key,
                    [callback, sensor_id](std::unique_ptr<Data> data) {
                      callback(sensor_id, std::move(data));
                    });
    // map<int,vector<key>>：添加轨迹线对应的 key
    queue_keys_[trajectory_id].push_back(queue_key);
  }
}

// 标记轨迹线 trajectory_id 已经完成采样，队列不再接收数据。
void Collator::FinishTrajectory(const int trajectory_id) {
  for (const auto& queue_key : queue_keys_[trajectory_id]) {
    // 将队列标记为已完成，即无法添加其他数据。一旦从队列中分派了最后一条数据，该队列将被删除。
    // 某一 queue_key 标识的队列 Queue 已经完成入队，因此不能再入队列，并在 map 中移除 queue_key。
    queue_.MarkQueueAsFinished(queue_key);
  }
}

// 添加一个传感器 id 对应的数据 data，数据必须按时间排序。
// 主要的操作，添加传感器数据，数据形式是：key + data
void Collator::AddSensorData(const int trajectory_id,
                             std::unique_ptr<Data> data) {
  QueueKey queue_key{trajectory_id, data->GetSensorId()};
  // 找到 key，再 move(data)
  queue_.Add(std::move(queue_key), std::move(data));
}

// 刷新
void Collator::Flush() { queue_.Flush(); }

common::optional<int> Collator::GetBlockingTrajectoryId() const {
  return common::optional<int>(queue_.GetBlocker().trajectory_id);
}

}  // namespace sensor
}  // namespace cartographer
