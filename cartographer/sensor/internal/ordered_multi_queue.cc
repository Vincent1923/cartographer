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

#include "cartographer/sensor/internal/ordered_multi_queue.h"

#include <algorithm>
#include <sstream>
#include <vector>

#include "cartographer/common/make_unique.h"
#include "glog/logging.h"

namespace cartographer {
namespace sensor {

namespace {

// Number of items that can be queued up before we log which queues are waiting
// for data.
// 在我们记录哪些队列正在等待数据之前可以排队的项目数
const int kMaxQueueSize = 500;

}  // namespace

// 重载 QueueKey 的<<输出运算符，友元函数
inline std::ostream& operator<<(std::ostream& out, const QueueKey& key) {
  return out << '(' << key.trajectory_id << ", " << key.sensor_id << ')';
}

// 构造函数
OrderedMultiQueue::OrderedMultiQueue() {}

// 析构函数
OrderedMultiQueue::~OrderedMultiQueue() {
  // 遍历所有的队列，即根据键值 QueueKey（trajectory_id 和 sensor_id）遍历所有队列，检查队列 Queue 是否为 finished
  for (auto& entry : queues_) {
    CHECK(entry.second.finished);
  }
}

// 添加一个关键词是“queue_key”的队列，该队列必须不存在。并用比较函数 Callback 排序
void OrderedMultiQueue::AddQueue(const QueueKey& queue_key, Callback callback) {
  // 检查 queues_ 中是否已经存在键值为 queue_key（trajectory_id 和 sensor_id）的队列
  CHECK_EQ(queues_.count(queue_key), 0);
  // map.count()相对于[]，调用时，是不添加 key 的。而[]是带有副作用的。c++pr.386
  queues_[queue_key].callback = std::move(callback);  // 调用回调函数？
}

// 将队列标记为已完成，即无法添加其他数据。一旦从队列中分派了最后一条数据，该队列将被删除。
// 某一 queue_key 标识的队列 Queue 已经完成入队，因此不能再入队列，并在 map 中移除 queue_key。
void OrderedMultiQueue::MarkQueueAsFinished(const QueueKey& queue_key) {
  auto it = queues_.find(queue_key);  // 查找键值为 queue_key 的队列
  CHECK(it != queues_.end()) << "Did not find '" << queue_key << "'.";  // 如果没找到
  auto& queue = it->second;  // 取出该队列
  CHECK(!queue.finished);    // 检查状态，检查键值为 queue_key 的队列是否为 finished
  queue.finished = true;     // 标记本队列已完成，别的数据不能再入队
  Dispatch();                // 调用一次 MarkQueueAsFinished() 就要调用一次 Dispatch()
}

// 根据 queue_key 找到队列,并添加 data 元素
void OrderedMultiQueue::Add(const QueueKey& queue_key,
                            std::unique_ptr<Data> data) {
  auto it = queues_.find(queue_key);  // 查找键值为 queue_key 的队列
  // 没有 queue_key 时，警告
  if (it == queues_.end()) {
    LOG_EVERY_N(WARNING, 1000)
        << "Ignored data for queue: '" << queue_key << "'";
    return;
  }
  // 把 data 压入队列中。如果队列已满，则阻塞。
  it->second.queue.Push(std::move(data));  // Queue 的 queue.push()
  Dispatch();  // 调用一次 Add() 就要调用一次 Dispatch()
}

// 按排序顺序分配所有剩余值，并删除基础队列。
// 先找到没有 finished 的队列，然后再对这些队列标记 finished。已完成的则不作任何处理。
void OrderedMultiQueue::Flush() {
  // unfinished_queues 存放还没有 finished 的队列的键值 QueueKey（trajectory_id 和 sensor_id）
  std::vector<QueueKey> unfinished_queues;
  // 遍历所有队列
  for (auto& entry : queues_) {
    if (!entry.second.finished) {
      // 如果队列还没有 finished，则把该队列的键值 QueueKey 添加到 unfinished_queues 中。
      unfinished_queues.push_back(entry.first);
    }
  }
  // 遍历 unfinished_queues，并 finish
  for (auto& unfinished_queue : unfinished_queues) {
    MarkQueueAsFinished(unfinished_queue);  // 一个一个的处理
  }
}

// 仅当存在至少一个未完成的队列时才必须调用。返回在 OrderedMultiQueue 可以分派数据之前需要更多数据的队列的键。
// 返回阻塞的队列（意为该队列对应的 sensor 的 data 未到达）
QueueKey OrderedMultiQueue::GetBlocker() const {
  CHECK(!queues_.empty());
  return blocker_;  // 返回队列的键值 QueueKey
}

/*
 * Dispatch() 函数，不断的处理来自 sensor 的数据。按照 data 采集的时间顺序处理。
 * kFirst: {1,2,3} finised
 * kSecond:{}      finised
 * kThird: {}      finised
 */
void OrderedMultiQueue::Dispatch() {
  while (true) {
    // 首先处理的数据，也即最早采集的数据
    const Data* next_data = nullptr;
    Queue* next_queue = nullptr;
    QueueKey next_queue_key;
    // 遍历队列中的每一个 key(queue_key)：填充上面3个变量值。如果某一 key 对应的 data 为空，则直接 return。
    for (auto it = queues_.begin(); it != queues_.end();) {
      // 获取某一队的队首 data，如果队列为空，则返回 null
      const auto* data = it->second.queue.Peek<Data>();
      if (data == nullptr) {  // 如果队列为空
        if (it->second.finished) {  // 如果队列已经 finished
          queues_.erase(it++);  // it 对应的队列为空且为 finished，故删除 it 对应的 key
          continue;
        }
        // 如果队列还没有 finished
        CannotMakeProgress(it->first);  // 此时什么也不做
        return;
      }
      if (next_data == nullptr || data->GetTime() < next_data->GetTime()) {
        // 找到 next_data 数据：即采集时间最早的数据，理论上应该最先处理它。
        next_data = data;
        next_queue = &it->second;
        next_queue_key = it->first;
      }
      // 检查 last_dispatched_time_ 是否小于等于 next_data 的时间
      CHECK_LE(last_dispatched_time_, next_data->GetTime())
          << "Non-sorted data added to queue: '" << it->first << "'";
      ++it;
    }
    if (next_data == nullptr) {
      CHECK(queues_.empty());  // 只有多队列为空，才可能 next_data == nullptr
      return;
    }

    // If we haven't dispatched any data for this trajectory yet, fast forward
    // all queues of this trajectory until a common start time has been reached.
    // 如果我们尚未为该 trajectory 调度任何数据，则快进该 trajectory 的所有队列，直到到达一个共同的开始时间为止。
    //
    // common_start_time 即所有的 sensor 都开始有 data 的时间点
    const common::Time common_start_time =
        GetCommonStartTime(next_queue_key.trajectory_id);

    if (next_data->GetTime() >= common_start_time) {  // 大多数情况，happy case
      // Happy case, we are beyond the 'common_start_time' already.
      // 很高兴，我们已经超越了“common_start_time”。
      last_dispatched_time_ = next_data->GetTime();
      next_queue->callback(next_queue->queue.Pop());
    } else if (next_queue->queue.Size() < 2) {  // 罕见情况
      if (!next_queue->finished) {
        // We cannot decide whether to drop or dispatch this yet.
        // 我们尚不能决定是删除还是发送此消息
        CannotMakeProgress(next_queue_key);
        return;
      }
      last_dispatched_time_ = next_data->GetTime();
      next_queue->callback(next_queue->queue.Pop());
    } else {
      // We take a peek at the time after next data. If it also is not beyond
      // 'common_start_time' we drop 'next_data', otherwise we just found the
      // first packet to dispatch from this queue.
      // 下次获取数据后，我们来看看。如果它也没有超出“common_start_time”，我们将丢弃“next_data”，
      // 否则我们只是找到了要从该队列分派的第一个数据包。
      std::unique_ptr<Data> next_data_owner = next_queue->queue.Pop();
      if (next_queue->queue.Peek<Data>()->GetTime() > common_start_time) {
        last_dispatched_time_ = next_data->GetTime();
        next_queue->callback(std::move(next_data_owner));
      }
    }
  }
}

void OrderedMultiQueue::CannotMakeProgress(const QueueKey& queue_key) {
  blocker_ = queue_key;  // 标识该队列 Queue 已经阻塞
  for (auto& entry : queues_) {
    if (entry.second.queue.Size() > kMaxQueueSize) {
      LOG_EVERY_N(WARNING, 60) << "Queue waiting for data: " << queue_key;
      return;
    }
  }
}

/*
 * 对同一轨迹 id，求得所有 sensor 的首次采集 data 的最晚时间 maxtime
 * 不同轨迹按不同轨迹算：
 * kFirst: {0,1,2,3} finised
 * kSecond:{2}
 * kThird: {4}
 * {2,2,}
 */
common::Time OrderedMultiQueue::GetCommonStartTime(const int trajectory_id) {
  // map.emplace():Construct and insert element，根据 trajectory_id 构造一个 map
  auto emplace_result = common_start_time_per_trajectory_.emplace(
      trajectory_id, common::Time::min());
  common::Time& common_start_time = emplace_result.first->second;  // 首先赋一个最小值
  if (emplace_result.second) {
    for (auto& entry : queues_) {
      // entry 是 map 的 pair<,>。本循环求得所有传感器中的 maxtime
      // queues_：first 是 QueueKey，second 是 Queue。
      // struct QueueKey
      // {
      //   int trajectory_id;
      //   std::string sensor_id;
      // }
      if (entry.first.trajectory_id == trajectory_id) {
        common_start_time = std::max(
            common_start_time, entry.second.queue.Peek<Data>()->GetTime());
      }
    }
    LOG(INFO) << "All sensor data for trajectory " << trajectory_id
              << " is available starting at '" << common_start_time << "'.";
  }
  return common_start_time;
}

}  // namespace sensor
}  // namespace cartographer
