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

#ifndef CARTOGRAPHER_MAPPING_SUBMAPS_H_
#define CARTOGRAPHER_MAPPING_SUBMAPS_H_

#include <memory>
#include <vector>

#include "Eigen/Geometry"
#include "cartographer/common/math.h"
#include "cartographer/common/port.h"
#include "cartographer/mapping/id.h"
#include "cartographer/mapping/probability_values.h"
#include "cartographer/mapping/proto/serialization.pb.h"
#include "cartographer/mapping/proto/submap_visualization.pb.h"
#include "cartographer/mapping/trajectory_node.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

// Converts the given probability to log odds.
inline float Logit(float probability) {
  return std::log(probability / (1.f - probability));
}

const float kMaxLogOdds = Logit(kMaxProbability);
const float kMinLogOdds = Logit(kMinProbability);

// Converts a probability to a log odds integer. 0 means unknown, [kMinLogOdds,
// kMaxLogOdds] is mapped to [1, 255].
inline uint8 ProbabilityToLogOddsInteger(const float probability) {
  const int value = common::RoundToInt((Logit(probability) - kMinLogOdds) *
                                       254.f / (kMaxLogOdds - kMinLogOdds)) +
                    1;
  CHECK_LE(1, value);
  CHECK_GE(255, value);
  return value;
}

// An individual submap, which has a 'local_pose' in the local map frame, keeps
// track of how many range data were inserted into it, and sets the
// 'finished_probability_grid' to be used for loop closing once the map no
// longer changes.
// 单个 submap，在 local map 坐标系中具有'local_pose'，跟踪有多少测距仪数据（range data）插入其中，
// 并设置'finished_probability_grid'以在地图不再更改时用于 loop closing。
/*
 * （1）一个子图，首先要有一个 local_pose。这个 local_pose 可以看做是没有经过全局优化的
 *     该 submap 相对于世界坐标系的位姿。
 * （2）一张子图要不停地监视有多少 range data（这里的 range data 我理解的就是点云）被插入到这个 submap 中。
 *     当没有 range data 插入时，则设置成员变量 finished_ 为真，然后开始做 Loop Closure。
 */
class Submap {
 public:
  // 构造函数，只包括一个 local_pose
  /**
   * @brief Submap             构造函数，设定子图的位姿
   * @param local_submap_pose  子图的位姿
   */
  Submap(const transform::Rigid3d& local_submap_pose)
      : local_pose_(local_submap_pose) {}
  virtual ~Submap() {}

  // 序列化与反序列化
  // 以下三个是纯虚函数，需要继承它的类去实现它。
  virtual void ToProto(proto::Submap* proto,
                       bool include_probability_grid_data) const = 0;
  virtual void UpdateFromProto(const proto::Submap& proto) = 0;

  // Fills data into the 'response'.
  // 把 submap 的放入到 response 的 proto 流中。方便 service 中查询 submap 讯息。
  virtual void ToResponseProto(
      const transform::Rigid3d& global_submap_pose,
      proto::SubmapQuery::Response* response) const = 0;

  // Pose of this submap in the local map frame.
  // 返回成员变量 local_pose_，即该 submap 的位姿。
  transform::Rigid3d local_pose() const { return local_pose_; }

  // Number of RangeData inserted.
  // 返回成员变量 num_range_data_，即插入到该 Submap 中的 range data 的数量。
  int num_range_data() const { return num_range_data_; }
  // 设置成员变量num_range_data_
  void set_num_range_data(const int num_range_data) {
    num_range_data_ = num_range_data;
  }

  // Whether the submap is finished or not.
  // 查看布尔型成员变量 finished_，即该子图是否还需要更新。
  bool finished() const { return finished_; }
  void set_finished(bool finished) { finished_ = finished; }

 private:
  const transform::Rigid3d local_pose_;  // 子图的局部坐标系原点的位姿
  int num_range_data_ = 0;               // 子图中插入的数据数量
  bool finished_ = false;                // 标志着子图是否已经构建完成，是否需要继续更新该子图
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_SUBMAPS_H_
