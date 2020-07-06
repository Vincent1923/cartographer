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
    transform::Rigid3d local_pose;          // 优化后机器人在局部地图坐标系下的位姿，包含位置和方向信息
    sensor::RangeData range_data_in_local;  // 在局部地图坐标系下的扫描数据
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
  // 完成扫描数据 range data 的积累后，返回匹配结果 "MatchingResult"，否则返回 "nullptr"。
  // 对于 2D SLAM，扫描数据必须近似为水平。
  // "TimedPointCloudData::time" 是获取扫描数据 "range_data" 最后一个扫描点的时间，
  // "TimedPointCloudData::ranges" 包含扫描点相对于 "TimedPointCloudData::time" 的时间。
  /**
   * @brief AddRangeData  添加激光传感器的扫描数据，该函数返回的就是一个 MatchingResult 的对象，这是扫描匹配后的结果，
   *                      实际上该类型中有一个字段 insertion_result 用于描述把扫描数据插入子图的结果。
   *                      该函数基本上完成了整个 Local SLAM 的业务。
   * @param sensor_id     激光雷达的索引
   * @param range_data    机器人坐标系下未经时间同步的扫描数据，它应该是从 ROS 系统中转换过来的。
   *                      类型 TimedPointCloudData 包含三个字段，其中 time 是获取最后一个扫描点的时间，
   *                      origin 是当次扫描测量时传感器在机器人坐标系下的位置，而 ranges 则是扫描数据在机器人坐标系下的空间坐标。
   * @return              扫描匹配的结果，为一个指向数据类型 MatchingResult 的智能指针，有4个字段。
   *                      (1) time 是当前同步时间；
   *                      (2) pose_estimate 是优化后机器人在局部地图坐标系下的位姿，包含位置和方向信息；
   *                      (3) range_data_in_local 是在优化之后的位姿估计下观测到的 hit 点和 miss 点在局部地图坐标系下的点云数据，
   *                          包含三个字段，其中 origin 是当次扫描测量时机器人在局部地图坐标系的位置，
   *                          returns 和 misses 则分别记录了 hit 点和 miss 点在局部地图坐标系下的空间坐标。
   *                      (4) insertion_result 是子图插入结果，它是指向类型 InsertionResult 的智能指针，有两个字段。
   *                          constant_data 是插入的节点数据，类型为 TrajectoryNode::Data，这里包含以下4个更新的字段：
   *                            time 是当前同步时间；
   *                            gravity_alignment 是重力方向，机器人在局部地图坐标系下的方向；
   *                            filtered_gravity_aligned_point_cloud 是经过滤波和重力修正后的扫描数据，从局部地图坐标系平移到机器人坐标系下的扫描数据；
   *                            local_pose 为优化后机器人在局部地图坐标系下的位姿，包含位置和方向信息。
   *                          insertion_submaps 是扫描数据所插入的 active_submaps_ 当前维护的子图对象，一般 size = 2。
   */
  std::unique_ptr<MatchingResult> AddRangeData(
      const std::string& sensor_id,
      const sensor::TimedPointCloudData& range_data);
  /**
   * @brief AddImuData  接收 IMU 数据，并完成位姿估计器 extrapolator_ 的初始化工作
   * @param imu_data    IMU 数据
   */
  void AddImuData(const sensor::ImuData& imu_data);
  /**
   * @brief AddOdometryData  接收里程计数据
   * @param odometry_data    里程计数据
   */
  void AddOdometryData(const sensor::OdometryData& odometry_data);

  static void RegisterMetrics(metrics::FamilyFactory* family_factory);

 private:
  /**
   * @brief AddAccumulatedRangeData     添加累积的传感器数据，完成 Local SLAM 的几项核心任务，主要是进行扫描匹配、把数据插入子图等操作，
   *                                    并返回记录了子图插入结果的扫描匹配结果。
   * @param time                        当前同步时间
   * @param gravity_aligned_range_data  经过重力修正后的传感器数据，从局部地图坐标系平移到机器人坐标系下的扫描数据，但没有经过旋转。
   *                                    包含三个字段，其中 origin 近似为 (0,0,0)，而 returns 和 misses 则分别是
   *                                    局部地图坐标系下的 hit 点和 miss 点经过平移后在机器人坐标系下的空间坐标，但没有经过旋转。
   * @param gravity_alignment           重力方向，translation 近似为 (0,0,0)，rotation 为当前机器人在局部地图坐标系下的方向，
   *                                    所以只包含机器人在局部地图坐标系下的方向信息。
   * @return                            扫描匹配的结果，为一个指向数据类型 MatchingResult 的智能指针，有4个字段。
   *                                    (1) time 是当前同步时间；
   *                                    (2) pose_estimate 是优化后机器人在局部地图坐标系下的位姿，包含位置和方向信息；
   *                                    (3) range_data_in_local 是在优化之后的位姿估计下观测到的 hit 点和 miss 点在局部地图坐标系下的点云数据，
   *                                    包含三个字段，其中 origin 是当次扫描测量时机器人在局部地图坐标系的位置，
   *                                    returns 和 misses 则分别记录了 hit 点和 miss 点在局部地图坐标系下的空间坐标。
   *                                    (4) insertion_result 是子图插入结果，它是指向类型 InsertionResult 的智能指针，有两个字段。
   *                                        constant_data 是插入的节点数据，类型为 TrajectoryNode::Data，这里包含以下4个更新的字段：
   *                                          time 是当前同步时间；
   *                                          gravity_alignment 是重力方向，机器人在局部地图坐标系下的方向；
   *                                          filtered_gravity_aligned_point_cloud 是经过滤波和重力修正后的扫描数据，从局部地图坐标系平移到机器人坐标系下的扫描数据；
   *                                          local_pose 为优化后机器人在局部地图坐标系下的位姿，包含位置和方向信息。
   *                                        insertion_submaps 是扫描数据所插入的 active_submaps_ 当前维护的子图对象，一般 size = 2。
   */
  std::unique_ptr<MatchingResult> AddAccumulatedRangeData(
      common::Time time, const sensor::RangeData& gravity_aligned_range_data,
      const transform::Rigid3d& gravity_alignment);
  /**
   * @brief TransformToGravityAlignedFrameAndFilter  这个函数主要是以重力方向为参考修正传感器数据后进行体素化滤波。
   *                                                 实际上是把局部地图坐标系下的扫描数据平移到机器人坐标系上，但是不旋转。
   * @param transform_to_gravity_aligned_frame       表示局部地图坐标系到机器人坐标系下的重力方向的变换，
   *                                                 其中 translation 为局部地图坐标系到机器人坐标系的平移，
   *                                                 而 rotation 以四元数表示，近似为 (0,0,0,1)，即旋转角度近似为0。
   * @param range_data                               局部地图坐标系下的扫描数据。
   *                                                 包含三个字段，其中 origin 是当次扫描测量时机器人在局部地图坐标系的位置，
   *                                                 而 returns 和 misses 则分别记录了扫描到的 hit 点和 miss 点在局部地图坐标系下的空间坐标。
   * @return                                         从局部地图坐标系平移到机器人坐标系下但没有经过旋转，并经过体素化滤波后的扫描数据。
   *                                                 包含三个字段，其中 origin 近似为 (0,0,0)，而 returns 和 misses 则分别是
   *                                                 局部地图坐标系下的 hit 点和 miss 点经过平移后在机器人坐标系下的空间坐标，但没有经过旋转。
   */
  sensor::RangeData TransformToGravityAlignedFrameAndFilter(
      const transform::Rigid3f& transform_to_gravity_aligned_frame,
      const sensor::RangeData& range_data) const;

  /**
   * @brief InsertIntoSubmap            将传感器数据插入到当前正在维护的子图中
   * @param time                        当前同步时间
   * @param range_data_in_local         在优化之后的位姿估计下观测到的 hit 点和 miss 点在局部地图坐标系下的点云数据，类型为 RangeData，
   *                                    包含三个字段，其中 origin 是当次扫描测量时机器人在局部地图坐标系的位置，
   *                                    returns 和 misses 则分别记录了 hit 点和 miss 点在局部地图坐标系下的空间坐标。
   * @param gravity_aligned_range_data  扫描匹配之前执行了重力修正的扫描数据，从局部地图坐标系平移到机器人坐标系下的扫描数据，但没有经过旋转。
   *                                    包含三个字段，其中 origin 近似为 (0,0,0)，而 returns 和 misses 则分别是
   *                                    局部地图坐标系下的 hit 点和 miss 点经过平移后在机器人坐标系下的空间坐标，但没有经过旋转。
   * @param pose_estimate               优化之后的位姿估计，机器人在局部地图坐标系下的位姿，包含位置和方向信息。
   * @param gravity_alignment           重力方向，表示当前机器人在局部地图坐标系下的方向。
   * @return                            子图插入结果，它是指向类型 InsertionResult 的智能指针，有两个字段。
   *                                    (1) constant_data 是插入的节点数据，类型为 TrajectoryNode::Data，这里包含以下4个更新的字段：
   *                                          time 是当前同步时间；
   *                                          gravity_alignment 是重力方向，机器人在局部地图坐标系下的方向；
   *                                          filtered_gravity_aligned_point_cloud 是经过滤波和重力修正后的扫描数据，从局部地图坐标系平移到机器人坐标系下的扫描数据；
   *                                          local_pose 为优化后机器人在局部地图坐标系下的位姿，包含位置和方向信息。
   *                                    (2) insertion_submaps 是扫描数据所插入的 active_submaps_ 当前维护的子图对象，一般 size = 2。
   */
  std::unique_ptr<InsertionResult> InsertIntoSubmap(
      common::Time time, const sensor::RangeData& range_data_in_local,
      const sensor::RangeData& gravity_aligned_range_data,
      const transform::Rigid3d& pose_estimate,
      const Eigen::Quaterniond& gravity_alignment);

  // Scan matches 'gravity_aligned_range_data' and returns the observed pose,
  // or nullptr on failure.
  // 对 "gravity_aligned_range_data" 进行扫描匹配并返回观察到的位姿，如果失败则返回 nullptr。
  /**
   * @brief ScanMatch                   进行扫描匹配，主要是将当前的传感器数据与当前维护的子图进行匹配，寻找一个位姿估计使得传感器数据能够尽可能的与地图匹配上。
   *                                    这是一个最优化的问题，Cartographer 主要通过 ceres 库求解。
   * @param time                        参考时间
   * @param pose_prediction             位姿估计器预测的位姿。
   *                                    translation 为机器人在局部地图坐标系下的位置，而 rotation 用四元数表示后近似为 (0,0,0,1)。
   *                                    所以它只包含机器人在局部地图坐标系下的位置信息，但不包含方向信息。
   * @param gravity_aligned_range_data  经过重力修正后的传感器数据，从局部地图坐标系平移到机器人坐标系下的扫描数据，但没有经过旋转。
   *                                    包含三个字段，其中 origin 近似为 (0,0,0)，而 returns 和 misses 则分别是
   *                                    局部地图坐标系下的 hit 点和 miss 点经过平移后在机器人坐标系下的空间坐标，但没有经过旋转。
   * @return                            位姿估计，使得传感器数据能够尽可能的与地图匹配上。
   *                                    translation 为机器人在局部地图坐标系下的位置估计，而 rotation 用四元数表示后近似为 (0,0,0,1)。
   *                                    所以返回值也只包含机器人在局部地图坐标系下的位置信息，但不包含方向信息。
   */
  std::unique_ptr<transform::Rigid2d> ScanMatch(
      common::Time time, const transform::Rigid2d& pose_prediction,
      const sensor::RangeData& gravity_aligned_range_data);

  // Lazily constructs a PoseExtrapolator.
  /**
   * @brief InitializeExtrapolator  初始化位姿估计器对象 extrapolator_
   * @param time                    IMU 数据产生的时间
   */
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

  int num_accumulated_ = 0;  // 累积数据的数量
  // 累积的扫描数据，局部地图坐标系下的扫描数据。
  // 类型 RangeData 包含三个字段，其中 origin 是当次扫描测量时机器人在局部地图坐标系的位置，
  // returns 和 misses 则分别记录了扫描到的 hit 点和 miss 点在局部地图坐标系下的空间坐标。
  sensor::RangeData accumulated_range_data_;
  std::chrono::steady_clock::time_point accumulation_started_;  // 开始累积数据的时间，也是开始跟踪轨迹的时间

  RangeDataCollator range_data_collator_;  // 累积数据收集器
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_2D_LOCAL_TRAJECTORY_BUILDER_2D_H_
