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
    // constant_data 的数据类型 TrajectoryNode::Data，实际上是结构体 TrajectoryNode 内部定义的结构体，包含了处理之后的点云数据。
    std::shared_ptr<const TrajectoryNode::Data> constant_data;       // 插入的节点数据
    std::vector<std::shared_ptr<const Submap2D>> insertion_submaps;  // 被插入的子图
  };
  // matching 结果。包括时间、匹配的 local_pose、传感器数据、插入结果等
  struct MatchingResult {
    common::Time time;                      // 扫描匹配发生的时间
    transform::Rigid3d local_pose;          // 在局部地图坐标系下的位姿
    sensor::RangeData range_data_in_local;  // 局部的扫描数据
    // 'nullptr' if dropped by the motion filter.
    // 如果扫描匹配的结果被运动滤波器过滤了，那么字段 insertion_result 中记录的是一个空指针 "nullptr"。
    std::unique_ptr<const InsertionResult> insertion_result;  // 子图插入结果
  };

  /**
   * @brief LocalTrajectoryBuilder2D   构造函数。函数体是空的，在它的构造列表中通过输入参数 options 完成了对各个成员变量的构造。
   * @param options                    轨迹跟踪器的配置选项
   * @param expected_range_sensor_ids  用于建图的所有传感器主题名称
   */
  explicit LocalTrajectoryBuilder2D(
      const proto::LocalTrajectoryBuilderOptions2D& options,
      const std::vector<std::string>& expected_range_sensor_ids);
  // 析构函数
  ~LocalTrajectoryBuilder2D();

  // 为了防止一些意外的情况发生，该类屏蔽了拷贝构造函数和拷贝赋值运算符。
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

/*************************************** 成员变量 ***************************************/
  const proto::LocalTrajectoryBuilderOptions2D options_;  // 轨迹跟踪器的配置选项
  ActiveSubmaps2D active_submaps_;                        // 当前正在维护的子图

  MotionFilter motion_filter_;                            // 运动滤波器，对位姿相关的数据进行降采样
  // 实时相关性分析的扫描匹配器，算法 "Real-Time Correlative Scan Matching" 的实现
  scan_matching::RealTimeCorrelativeScanMatcher2D
      real_time_correlative_scan_matcher_;
  scan_matching::CeresScanMatcher2D ceres_scan_matcher_;  // 使用 Ceres 库将扫描数据放置到地图中的扫描匹配器

  std::unique_ptr<PoseExtrapolator> extrapolator_;  // 位姿估计器，用一段时间内的位姿数据估计线速度和角速度，进而预测运动

  int num_accumulated_ = 0;                                     // 累积数据的数量
  sensor::RangeData accumulated_range_data_;                    // 累积的扫描数据
  std::chrono::steady_clock::time_point accumulation_started_;  // 开始累积数据的时间，也是开始跟踪轨迹的时间

  RangeDataCollator range_data_collator_;  // 累积数据收集器
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_2D_LOCAL_TRAJECTORY_BUILDER_2D_H_
