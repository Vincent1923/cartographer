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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_2D_LOCAL_TRAJECTORY_BUILDER_2D_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_2D_LOCAL_TRAJECTORY_BUILDER_2D_H_

#include <chrono>
#include <memory>

#include "cartographer/common/time.h"
#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/ceres_scan_matcher_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/real_time_correlative_scan_matcher_2d.h"
#include "cartographer/mapping/internal/motion_filter.h"
#include "cartographer/mapping/internal/range_data_collator.h"
#include "cartographer/mapping/pose_extrapolator.h"
#include "cartographer/mapping/proto/2d/local_trajectory_builder_options_2d.pb.h"
#include "cartographer/metrics/family_factory.h"
#include "cartographer/sensor/imu_data.h"
#include "cartographer/sensor/internal/voxel_filter.h"
#include "cartographer/sensor/odometry_data.h"
#include "cartographer/sensor/range_data.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace mapping {

// Wires up the local SLAM stack (i.e. pose extrapolator, scan matching, etc.)
// without loop closure.
// 连接 local SLAM（即 pose extrapolator，scan matching 等），而没有 loop closure。
// TODO(gaschler): Add test for this class similar to the 3D test.
class LocalTrajectoryBuilder2D {
 public:
  struct InsertionResult {
    std::shared_ptr<const TrajectoryNode::Data> constant_data;       // 节点数据
    std::vector<std::shared_ptr<const Submap2D>> insertion_submaps;  // 插入的 submap 的向量
  };
  // matching 结果。包括时间、匹配的 local_pose、传感器数据、插入结果等
  struct MatchingResult {
    common::Time time;
    transform::Rigid3d local_pose;
    sensor::RangeData range_data_in_local;
    // 'nullptr' if dropped by the motion filter.
    // 如果被 motion filter 滤掉了，那么 insertion_result 返回空指针。
    std::unique_ptr<const InsertionResult> insertion_result;
  };

  // 构造函数，为几个成员变量初始化
  explicit LocalTrajectoryBuilder2D(
      const proto::LocalTrajectoryBuilderOptions2D& options,
      const std::vector<std::string>& expected_range_sensor_ids);
  ~LocalTrajectoryBuilder2D();

  LocalTrajectoryBuilder2D(const LocalTrajectoryBuilder2D&) = delete;
  LocalTrajectoryBuilder2D& operator=(const LocalTrajectoryBuilder2D&) = delete;

  // Returns 'MatchingResult' when range data accumulation completed,
  // otherwise 'nullptr'. Range data must be approximately horizontal
  // for 2D SLAM. `TimedPointCloudData::time` is when the last point in
  // `range_data` was acquired, `TimedPointCloudData::ranges` contains the
  // relative time of point with respect to `TimedPointCloudData::time`.
  //
  // range data 累积完成后，返回“MatchingResult”，否则返回“nullptr”。
  // 对于2D SLAM，range data 必须近似为水平。"TimedPointCloudData::time"是获取"range_data"中最后一个点的时间，
  // "TimedPointCloudData::ranges"包含点相对于“TimedPointCloudData::time”的相对时间。
  std::unique_ptr<MatchingResult> AddRangeData(
      const std::string& sensor_id,
      const sensor::TimedPointCloudData& range_data);
  void AddImuData(const sensor::ImuData& imu_data);
  void AddOdometryData(const sensor::OdometryData& odometry_data);

  static void RegisterMetrics(metrics::FamilyFactory* family_factory);

 private:
  std::unique_ptr<MatchingResult> AddAccumulatedRangeData(
      common::Time time, const sensor::RangeData& gravity_aligned_range_data,
      const transform::Rigid3d& gravity_alignment);
  // 将 RangeData 转化成重力校正后的数据，并经过 VoxelFilter
  sensor::RangeData TransformToGravityAlignedFrameAndFilter(
      const transform::Rigid3f& transform_to_gravity_aligned_frame,
      const sensor::RangeData& range_data) const;

  std::unique_ptr<InsertionResult> InsertIntoSubmap(
      common::Time time, const sensor::RangeData& range_data_in_local,
      const sensor::RangeData& gravity_aligned_range_data,
      const transform::Rigid3d& pose_estimate,
      const Eigen::Quaterniond& gravity_alignment);

  // Scan matches 'gravity_aligned_range_data' and returns the observed pose,
  // or nullptr on failure.
  // 对'gravity_aligned_range_data'扫描匹配并返回观察到的位姿，如果失败则返回 nullptr
  std::unique_ptr<transform::Rigid2d> ScanMatch(
      common::Time time, const transform::Rigid2d& pose_prediction,
      const sensor::RangeData& gravity_aligned_range_data);

  // Lazily constructs a PoseExtrapolator.
  void InitializeExtrapolator(common::Time time);

  const proto::LocalTrajectoryBuilderOptions2D options_;  // 参数配置项
  ActiveSubmaps2D active_submaps_;  // 在 mapping/2d/submap_2d.h 中定义。同时维护着两个 submap

  // MotionFilter 定义在"/mapping/internal/motion_filter.h"中，其主要作用是对数据进行一下滤波。
  // 当两帧数据的间隔时间/两帧的Pose跨过的距离/两帧的Pose转过的角度等不超过一定的阈值时，认为新的数据提供的信息很少，这些数据可以直接舍去。
  MotionFilter motion_filter_;
  scan_matching::RealTimeCorrelativeScanMatcher2D
      real_time_correlative_scan_matcher_;  // 实时的扫描匹配，用的相关分析方法
  scan_matching::CeresScanMatcher2D ceres_scan_matcher_;  // Ceres 方法匹配。所以两者都是 Scan-to-match 的匹配？方法二选一？

  std::unique_ptr<PoseExtrapolator> extrapolator_;  // 轨迹推算器。融合 IMU，里程计数据

  int num_accumulated_ = 0;  // 累积数据的数量
  sensor::RangeData accumulated_range_data_;  // 该轨迹的累积数据
  std::chrono::steady_clock::time_point accumulation_started_;  // 标记该轨迹的开始时刻

  RangeDataCollator range_data_collator_;  // 收集传感器数据
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_2D_LOCAL_TRAJECTORY_BUILDER_2D_H_
