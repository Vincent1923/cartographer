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

#ifndef CARTOGRAPHER_SENSOR_INTERNAL_ORDERED_MULTI_QUEUE_H_
#define CARTOGRAPHER_SENSOR_INTERNAL_ORDERED_MULTI_QUEUE_H_

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <tuple>

#include "cartographer/common/blocking_queue.h"
#include "cartographer/common/port.h"
#include "cartographer/common/time.h"
#include "cartographer/sensor/internal/dispatchable.h"

namespace cartographer {
namespace sensor {

// QueueKey 是一对 trajectory_id 和 sensor_id
struct QueueKey {
  int trajectory_id;      // 轨迹线 id
  std::string sensor_id;  // 传感器 id

  // 重载小于运算符，forward_as_tuple:完美转发。(以tuple规则比较2者)，tuple定义了<运算符
  bool operator<(const QueueKey& other) const {
    return std::forward_as_tuple(trajectory_id, sensor_id) <
           std::forward_as_tuple(other.trajectory_id, other.sensor_id);
  }
};

/*
 * OrderedMultiQueue，用于管理多个有序的传感器数据，
 * 是有序的多队列类,每个队列有一个 key，并且有一个自定义排序函数。
 * queues_ 的形式为：
 * key1:Queue
 * key2:Queue
 * key3:Queue
 */
// Maintains multiple queues of sorted sensor data and dispatches it in merge
// sorted order. It will wait to see at least one value for each unfinished
// queue before dispatching the next time ordered value across all queues.
// 维护排序后的传感器数据的多个队列，并按合并排序的顺序进行调度。
// 它将等待为每个未完成的队列查看至少一个值，然后在所有队列中分派下一个按时间排序的值。
//
// This class is thread-compatible.
// 此类是线程兼容的。
class OrderedMultiQueue {
 public:
  // Data 只有一个 std::string 类型的成员变量 sensor_id_
  using Callback = std::function<void(std::unique_ptr<Data>)>;  // 回调函数

  // 构造函数
  OrderedMultiQueue();
  ~OrderedMultiQueue();

  // Adds a new queue with key 'queue_key' which must not already exist.
  // 'callback' will be called whenever data from this queue can be dispatched.
  // 用键“queue_key”添加一个新队列，该队列必须不存在。只要可以调度此队列中的数据，就会调用“callback”。
  //
  // 添加一个队列 Queue，名称是 queue_key，以后入队的 data，调用回调函数 callback 处理
  void AddQueue(const QueueKey& queue_key, Callback callback);

  // Marks a queue as finished, i.e. no further data can be added. The queue
  // will be removed once the last piece of data from it has been dispatched.
  // 将队列标记为已完成，即无法添加其他数据。一旦从队列中分派了最后一条数据，该队列将被删除。
  /*
   * 某一 queue_key 标识的队列 Queue 已经完成入队，因此不能再入队列，并在 map 中移除 queue_key。
   */
  void MarkQueueAsFinished(const QueueKey& queue_key);

  // Adds 'data' to a queue with the given 'queue_key'. Data must be added
  // sorted per queue.
  // 使用给定的“queue_key”将“data”添加到队列中。必须添加按队列排序的数据。
  /*
   * 对某一 queue_key 标识的队列 Queue，压入 data，data 按照回调函数处理。
   */
  void Add(const QueueKey& queue_key, std::unique_ptr<Data> data);

  /*
   * 标记全部队列都已经 finished。
   * kFirst: {0,4,5,6}
   * kSecond: {0,1,3,7}
   * kThird: {0,2,8}
   * 之前只处理到6，调用 Flush() 则处理剩余的7,8
   * 如果不调用 Flush()，则析构时会出错
   */
  // Dispatches all remaining values in sorted order and removes the underlying
  // queues.
  // 按排序顺序分配所有剩余值，并删除基础队列。
  void Flush();

  /*
   * 返回阻塞的队列（意为该队列对应的 sensor 的 data 未到达）
   */
  // Must only be called if at least one unfinished queue exists. Returns the
  // key of a queue that needs more data before the OrderedMultiQueue can
  // dispatch data.
  // 仅当存在至少一个未完成的队列时才必须调用。返回在 OrderedMultiQueue 可以分派数据之前需要更多数据的队列的键。
  QueueKey GetBlocker() const;

 private:
  struct Queue {
    common::BlockingQueue<std::unique_ptr<Data>> queue;  // common::BlockingQueue 为线程安全的阻塞队列
    Callback callback;
    bool finished = false;
  };

  void Dispatch();
  void CannotMakeProgress(const QueueKey& queue_key);
  common::Time GetCommonStartTime(int trajectory_id);

  // Used to verify that values are dispatched in sorted order.
  // 用于验证值是否按排序顺序分派。
  common::Time last_dispatched_time_ = common::Time::min();

  std::map<int, common::Time> common_start_time_per_trajectory_;
  // 
  /*
   * 多队列主体,本类最大的内存占用量。QueueKey 是一对 trajectory_id 和 sensor_id。
   * queues_:
   * first 是 QueueKey
   * second 是 Queue
   */
  std::map<QueueKey, Queue> queues_;
  QueueKey blocker_;
};

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_SENSOR_INTERNAL_ORDERED_MULTI_QUEUE_H_
