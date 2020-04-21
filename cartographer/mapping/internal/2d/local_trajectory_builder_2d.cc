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

#include "cartographer/mapping/internal/2d/local_trajectory_builder_2d.h"

#include <limits>
#include <memory>

#include "cartographer/common/make_unique.h"
#include "cartographer/metrics/family_factory.h"
#include "cartographer/sensor/range_data.h"

namespace cartographer {
namespace mapping {

static auto* kLocalSlamLatencyMetric = metrics::Gauge::Null();
static auto* kFastCorrelativeScanMatcherScoreMetric =
    metrics::Histogram::Null();
static auto* kCeresScanMatcherCostMetric = metrics::Histogram::Null();
static auto* kScanMatcherResidualDistanceMetric = metrics::Histogram::Null();
static auto* kScanMatcherResidualAngleMetric = metrics::Histogram::Null();

// 构造函数，为几个成员变量初始化
LocalTrajectoryBuilder2D::LocalTrajectoryBuilder2D(
    const proto::LocalTrajectoryBuilderOptions2D& options,
    const std::vector<std::string>& expected_range_sensor_ids)
    : options_(options),
      active_submaps_(options.submaps_options()),
      motion_filter_(options_.motion_filter_options()),
      real_time_correlative_scan_matcher_(
          options_.real_time_correlative_scan_matcher_options()),
      ceres_scan_matcher_(options_.ceres_scan_matcher_options()),
      range_data_collator_(expected_range_sensor_ids) {}

LocalTrajectoryBuilder2D::~LocalTrajectoryBuilder2D() {}

// 将 RangeData 转化成重力校正后的数据，并经过 VoxelFilter。
// 关于 VoxelFilter 和 AdaptiveVoxelFilter，可参见如下两个参考链接：
// 源码解读：https://blog.csdn.net/learnmoreonce/article/details/76218136
// VoxelFilter 的原理：https://blog.csdn.net/xiaoma_bk/article/details/81780066
// cartographer 中的 VoxelFilter 是把空间分为很多个立方体的栅格，然后一个栅格内可能有很多点，
// 但只用一个点来代表整个栅格中的所有点。
// 简单说，这些滤波就是对点云数据的预处理：去除不合理的范围的点、合并相同位置的点等，从而减少点云数量，提高点云质量。
sensor::RangeData
LocalTrajectoryBuilder2D::TransformToGravityAlignedFrameAndFilter(
    const transform::Rigid3f& transform_to_gravity_aligned_frame,
    const sensor::RangeData& range_data) const {
  // 裁剪数据，指定一定 z 轴范围内的数据，数据范围在 options_.min_z() 到 options_.max_z() 之间。
  // 在“/src/cartographer/configuration_files/trajectory_builder_2d.lua”中找到。
  const sensor::RangeData cropped =
      sensor::CropRangeData(sensor::TransformRangeData(
                                range_data, transform_to_gravity_aligned_frame),
                            options_.min_z(), options_.max_z());
  // 进行体素化滤波
  // options_.voxel_filter_size() 在“/src/cartographer/configuration_files/trajectory_builder_2d.lua”中设置。
  return sensor::RangeData{
      cropped.origin,
      sensor::VoxelFilter(options_.voxel_filter_size()).Filter(cropped.returns),
      sensor::VoxelFilter(options_.voxel_filter_size()).Filter(cropped.misses)};
}

// 扫描匹配。输入是：
// 1. 时间
// 2. 由 PoseExtrapolator 预测的初始位姿 pose_prediction
// 3. 经过重力 aligned 的 RangeData
// 输出是对该帧传感器数据的最优 pose。采用 Ceres Scan Matcher
std::unique_ptr<transform::Rigid2d> LocalTrajectoryBuilder2D::ScanMatch(
    const common::Time time, const transform::Rigid2d& pose_prediction,
    const sensor::RangeData& gravity_aligned_range_data) {
  // 获取要进行扫描匹配的 Submap 的 index
  std::shared_ptr<const Submap2D> matching_submap =
      active_submaps_.submaps().front();
  // The online correlative scan matcher will refine the initial estimate for
  // the Ceres scan matcher.
  // Online Correlative Scan Matcher 将优化 Ceres scan matcher 的初始估计
  transform::Rigid2d initial_ceres_pose = pose_prediction;
  // 生成一个自适应体素滤波器
  sensor::AdaptiveVoxelFilter adaptive_voxel_filter(
      options_.adaptive_voxel_filter_options());
  // 对数据进行一下自适应体素滤波
  const sensor::PointCloud filtered_gravity_aligned_point_cloud =
      adaptive_voxel_filter.Filter(gravity_aligned_range_data.returns);
  // 如果滤波后结果为空，则返回空指针
  if (filtered_gravity_aligned_point_cloud.empty()) {
    return nullptr;
  }
  // 如果配置项设置使用 OnlineCorrelativeScanMatching。但配置项里默认该项为 false
  if (options_.use_online_correlative_scan_matching()) {
    CHECK_EQ(options_.submaps_options().grid_options_2d().grid_type(),
             proto::GridOptions2D_GridType_PROBABILITY_GRID);  // 检查是否为概率图
    // 调用 RealTimeCorrelativeScanMatcher2D 的方法进行匹配，返回一个得分
    double score = real_time_correlative_scan_matcher_.Match(
        pose_prediction, filtered_gravity_aligned_point_cloud,
        *static_cast<const ProbabilityGrid*>(matching_submap->grid()),
        &initial_ceres_pose);
    kFastCorrelativeScanMatcherScoreMetric->Observe(score);
  }

  // 调用 Ceres 库来实现匹配。匹配结果放到 pose_observation 中
  auto pose_observation = common::make_unique<transform::Rigid2d>();
  ceres::Solver::Summary summary;
  ceres_scan_matcher_.Match(pose_prediction.translation(), initial_ceres_pose,
                            filtered_gravity_aligned_point_cloud,
                            *matching_submap->grid(), pose_observation.get(),
                            &summary);
  // 统计残差
  if (pose_observation) {
    kCeresScanMatcherCostMetric->Observe(summary.final_cost);
    double residual_distance =
        (pose_observation->translation() - pose_prediction.translation())
            .norm();
    kScanMatcherResidualDistanceMetric->Observe(residual_distance);
    double residual_angle = std::abs(pose_observation->rotation().angle() -
                                     pose_prediction.rotation().angle());
    kScanMatcherResidualAngleMetric->Observe(residual_angle);
  }
  // 返回优化后的 pose
  return pose_observation;
}

// 添加 RangeData，返回匹配结果。这是最上层的函数。通过该函数调用其他各种函数。
std::unique_ptr<LocalTrajectoryBuilder2D::MatchingResult>
LocalTrajectoryBuilder2D::AddRangeData(
    const std::string& sensor_id,
    const sensor::TimedPointCloudData& unsynchronized_data) {
  // RangeDataCollator 定义在"/mapping/internal/range_data_collator.h"中。
  // 该类的主要任务是把来自不同传感器的 TimePointCloudData 进行一下同步。
  // TimePointCloudData 的第三个元素为类型 TimePointCloud，是带时间的点云数据。
  // 3D情况下，前3个元素是点的坐标，第4个元素是测量到每个点的相对时间。
  // 时间以s为单位，以最后一个点的捕获时间为0，则前面捕获的点都为负数，并且越早捕获的点时间值的绝对值越大。
  // 对于2D情况，第3个元素始终为0，第4个元素同样表示时间。
  auto synchronized_data =
      range_data_collator_.AddRangeData(sensor_id, unsynchronized_data);
  if (synchronized_data.ranges.empty()) {
    LOG(INFO) << "Range data collator filling buffer.";
    return nullptr;
  }

  // 取点云获取的时间为基准为 PoseExtrapolator 初始化。
  // 这里猜测点云获取的时间 synchronized_data.time 为获取点云集最后一个点的时间
  const common::Time& time = synchronized_data.time;
  // Initialize extrapolator now if we do not ever use an IMU.
  // 初始化 PoseExtrapolator
  if (!options_.use_imu_data()) {
    InitializeExtrapolator(time);
  }

  if (extrapolator_ == nullptr) {
    // Until we've initialized the extrapolator with our first IMU message, we
    // cannot compute the orientation of the rangefinder.
    // 除非我们使用第一个 IMU 消息初始化 extrapolator，否则我们无法计算测距仪的方向
    LOG(INFO) << "Extrapolator not yet initialized.";
    return nullptr;
  }

  // 数据是否为空
  CHECK(!synchronized_data.ranges.empty());
  // TODO(gaschler): Check if this can strictly be 0.
  // 看一下数据点的第4个元素是不是小于等于0。2d情况第3个元素都是0。第四个元素是时间
  CHECK_LE(synchronized_data.ranges.back().point_time[3], 0.f);
  // 第一个点的时间就等于点云集获取的时间加上第一个点记录的相对时间
  const common::Time time_first_point =
      time +
      common::FromSeconds(synchronized_data.ranges.front().point_time[3]);
  // 如果该时间比 PoseExtrapolator 的最新时间还要早，说明在第一个点被捕获时，PoseExtrapolator 还没初始化
  if (time_first_point < extrapolator_->GetLastPoseTime()) {
    LOG(INFO) << "Extrapolator is still initializing.";
    return nullptr;
  }

  // 轨迹开始时刻。只在初始时执行一次。之后 num_accumulated_ 不再等于0，该语句就不执行了
  if (num_accumulated_ == 0) {
    accumulation_started_ = std::chrono::steady_clock::now();
  }

  // 遍历每个点，计算在每个点时 PoseExtrapolator 推算出来的机器人的 Pose，放入 range_data_poses 这个向量中
  std::vector<transform::Rigid3f> range_data_poses;
  // 为该集合预分配内存大小
  range_data_poses.reserve(synchronized_data.ranges.size());
  bool warned = false;
  for (const auto& range : synchronized_data.ranges) {  // 遍历点集
    // 点云中每个点的捕获时刻
    common::Time time_point = time + common::FromSeconds(range.point_time[3]);
    // 如果该时刻早于 PoseExtrapolator 的时间
    if (time_point < extrapolator_->GetLastExtrapolatedTime()) {
      if (!warned) {
        LOG(ERROR)
            << "Timestamp of individual range data point jumps backwards from "
            << extrapolator_->GetLastExtrapolatedTime() << " to " << time_point;
        warned = true;
      }
      // 时间设置为 PoseExtrapolator 的时间
      time_point = extrapolator_->GetLastExtrapolatedTime();
    }
    // 推算 time_point 这个时间点的 Pose
    range_data_poses.push_back(
        extrapolator_->ExtrapolatePose(time_point).cast<float>());
  }

  if (num_accumulated_ == 0) {
    // 'accumulated_range_data_.origin' is uninitialized until the last
    // accumulation.
    accumulated_range_data_ = sensor::RangeData{{}, {}, {}};
  }

  // Drop any returns below the minimum range and convert returns beyond the
  // maximum range into misses.
  // 把任何 returns 降至 minimum range 以下，并将超出 maximum range 的 returns 转换为 misses
  // 遍历每一个点
  for (size_t i = 0; i < synchronized_data.ranges.size(); ++i) {
    // 获取第i个点
    const Eigen::Vector4f& hit = synchronized_data.ranges[i].point_time;
    // 第i个点的原点
    const Eigen::Vector3f origin_in_local =
        range_data_poses[i] *
        synchronized_data.origins.at(synchronized_data.ranges[i].origin_index);
    // 将 hit 集转变成在 Local 坐标系下
    const Eigen::Vector3f hit_in_local = range_data_poses[i] * hit.head<3>();
    // 局部坐标系下由 hit 点到 origin 的射线
    const Eigen::Vector3f delta = hit_in_local - origin_in_local;
    const float range = delta.norm();  // 该向量的模
    // 如果该向量的模在合理范围内，则把它压入 accumulated_range_data_ 的 returns 集合中
    // 参数 min_range() 和 max_range() 主要是设置雷达数据的最小距离和最大距离，
    // 配置参数的文件在“cartographer/configuration_files/trajectory_builder_2d.lua”中。
    if (range >= options_.min_range()) {
      if (range <= options_.max_range()) {
        accumulated_range_data_.returns.push_back(hit_in_local);
      } else {
        // 否则，放入 accumulated_range_data_ 的 misses 集合中
        accumulated_range_data_.misses.push_back(
            origin_in_local +
            options_.missing_data_ray_length() / range * delta);
      }
    }
  }
  ++num_accumulated_;

  // 点云的数量大于1的话
  if (num_accumulated_ >= options_.num_accumulated_range_data()) {
    num_accumulated_ = 0;
    // 估计重力
    const transform::Rigid3d gravity_alignment = transform::Rigid3d::Rotation(
        extrapolator_->EstimateGravityOrientation(time));
    // TODO(gaschler): This assumes that 'range_data_poses.back()' is at time
    // 'time'.
    accumulated_range_data_.origin = range_data_poses.back().translation();
    // 调用 AddAccumulatedRangeData 进行匹配、插入数据等。返回 MatchingResult
    return AddAccumulatedRangeData(
        time,
        TransformToGravityAlignedFrameAndFilter(  // 调用这个函数进行重力 align
            gravity_alignment.cast<float>() * range_data_poses.back().inverse(),
            accumulated_range_data_),
        gravity_alignment);
  }
  return nullptr;
}

// 添加累计的 RangeData，返回匹配结果。调用了 ScanMatch 和 InsertIntoSubmap
std::unique_ptr<LocalTrajectoryBuilder2D::MatchingResult>
LocalTrajectoryBuilder2D::AddAccumulatedRangeData(
    const common::Time time,
    const sensor::RangeData& gravity_aligned_range_data,
    const transform::Rigid3d& gravity_alignment) {
  // 如果数据为空
  if (gravity_aligned_range_data.returns.empty()) {
    LOG(WARNING) << "Dropped empty horizontal range data.";
    return nullptr;
  }

  // Computes a gravity aligned pose prediction.
  // 计算经过重力 aligned 的 Pose
  // 从 PoseExtrapolator 处获得未经重力 aligned 的 Pose
  const transform::Rigid3d non_gravity_aligned_pose_prediction =
      extrapolator_->ExtrapolatePose(time);
  // 重力 align
  const transform::Rigid2d pose_prediction = transform::Project2D(
      non_gravity_aligned_pose_prediction * gravity_alignment.inverse());

  // local map frame <- gravity-aligned frame
  // 调用 ScanMathc 函数进行匹配，求取 Scan 插入 Submap 的最优 Pose
  std::unique_ptr<transform::Rigid2d> pose_estimate_2d =
      ScanMatch(time, pose_prediction, gravity_aligned_range_data);
  // 如果为空，表示匹配为空
  if (pose_estimate_2d == nullptr) {
    LOG(WARNING) << "Scan matching failed.";
    return nullptr;
  }
  // 考虑重力方向，将 2d 的 pose 变成 3d 的 pose
  const transform::Rigid3d pose_estimate =
      transform::Embed3D(*pose_estimate_2d) * gravity_alignment;
  // 将新的 pose 添加到 PoseExtrapolator 的 Pose 队列中
  extrapolator_->AddPose(time, pose_estimate);

  // 把点云数据通过估计出来的 Pose，转化为在局部坐标系中的点云数据
  sensor::RangeData range_data_in_local =
      TransformRangeData(gravity_aligned_range_data,
                         transform::Embed3D(pose_estimate_2d->cast<float>()));
  // 调用 InsertIntoSubmap 函数更新 Submap，同时返回插入结果
  std::unique_ptr<InsertionResult> insertion_result =
      InsertIntoSubmap(time, range_data_in_local, gravity_aligned_range_data,
                       pose_estimate, gravity_alignment.rotation());
  // 统计一下数据累计的时间
  auto duration = std::chrono::steady_clock::now() - accumulation_started_;
  kLocalSlamLatencyMetric->Set(
      std::chrono::duration_cast<std::chrono::seconds>(duration).count());
  // 返回匹配结果
  return common::make_unique<MatchingResult>(
      MatchingResult{time, pose_estimate, std::move(range_data_in_local),
                     std::move(insertion_result)});
}

// 插入 submap, 返回插入结果。这是在都已经完成匹配的情况下，调用该函数更新 submap
std::unique_ptr<LocalTrajectoryBuilder2D::InsertionResult>
LocalTrajectoryBuilder2D::InsertIntoSubmap(
    const common::Time time, const sensor::RangeData& range_data_in_local,
    const sensor::RangeData& gravity_aligned_range_data,
    const transform::Rigid3d& pose_estimate,
    const Eigen::Quaterniond& gravity_alignment) {
  // 如果没有被 MotionFilter 滤掉
  if (motion_filter_.IsSimilar(time, pose_estimate)) {
    return nullptr;
  }

  // Querying the active submaps must be done here before calling
  // InsertRangeData() since the queried values are valid for next insertion.
  // 在调用 InsertRangeData() 之前，必须在此处完成对 active submaps 的查询，因为查询的值对于下一次插入有效。
  // 把 active_submaps_ 中维护的 Submap 列表放到 insertion_submaps 这个向量中
  std::vector<std::shared_ptr<const Submap2D>> insertion_submaps;
  for (const std::shared_ptr<Submap2D>& submap : active_submaps_.submaps()) {
    insertion_submaps.push_back(submap);
  }
  // 调用 submap 的工具函数将传感器数据插入，更新 Submap。
  active_submaps_.InsertRangeData(range_data_in_local);

  // 生成一个体素化滤波器并滤波
  sensor::AdaptiveVoxelFilter adaptive_voxel_filter(
      options_.loop_closure_adaptive_voxel_filter_options());
  const sensor::PointCloud filtered_gravity_aligned_point_cloud =
      adaptive_voxel_filter.Filter(gravity_aligned_range_data.returns);

  // 返回插入结果
  return common::make_unique<InsertionResult>(InsertionResult{
      std::make_shared<const TrajectoryNode::Data>(TrajectoryNode::Data{
          time,
          gravity_alignment,
          filtered_gravity_aligned_point_cloud,
          {},  // 'high_resolution_point_cloud' is only used in 3D.
          {},  // 'low_resolution_point_cloud' is only used in 3D.
          {},  // 'rotational_scan_matcher_histogram' is only used in 3D.
          pose_estimate}),
      std::move(insertion_submaps)});
}

void LocalTrajectoryBuilder2D::AddImuData(const sensor::ImuData& imu_data) {
  CHECK(options_.use_imu_data()) << "An unexpected IMU packet was added.";
  InitializeExtrapolator(imu_data.time);
  extrapolator_->AddImuData(imu_data);
}

void LocalTrajectoryBuilder2D::AddOdometryData(
    const sensor::OdometryData& odometry_data) {
  if (extrapolator_ == nullptr) {
    // Until we've initialized the extrapolator we cannot add odometry data.
    LOG(INFO) << "Extrapolator not yet initialized.";
    return;
  }
  extrapolator_->AddOdometryData(odometry_data);
}

void LocalTrajectoryBuilder2D::InitializeExtrapolator(const common::Time time) {
  if (extrapolator_ != nullptr) {
    return;
  }
  // We derive velocities from poses which are at least 1 ms apart for numerical
  // stability. Usually poses known to the extrapolator will be further apart
  // in time and thus the last two are used.
  constexpr double kExtrapolationEstimationTimeSec = 0.001;
  // TODO(gaschler): Consider using InitializeWithImu as 3D does.
  extrapolator_ = common::make_unique<PoseExtrapolator>(
      ::cartographer::common::FromSeconds(kExtrapolationEstimationTimeSec),
      options_.imu_gravity_time_constant());
  extrapolator_->AddPose(time, transform::Rigid3d::Identity());
}

void LocalTrajectoryBuilder2D::RegisterMetrics(
    metrics::FamilyFactory* family_factory) {
  auto* latency = family_factory->NewGaugeFamily(
      "mapping_internal_2d_local_trajectory_builder_latency",
      "Duration from first incoming point cloud in accumulation to local slam "
      "result");
  kLocalSlamLatencyMetric = latency->Add({});
  auto score_boundaries = metrics::Histogram::FixedWidth(0.05, 20);
  auto* scores = family_factory->NewHistogramFamily(
      "mapping_internal_2d_local_trajectory_builder_scores",
      "Local scan matcher scores", score_boundaries);
  kFastCorrelativeScanMatcherScoreMetric =
      scores->Add({{"scan_matcher", "fast_correlative"}});
  auto cost_boundaries = metrics::Histogram::ScaledPowersOf(2, 0.01, 100);
  auto* costs = family_factory->NewHistogramFamily(
      "mapping_internal_2d_local_trajectory_builder_costs",
      "Local scan matcher costs", cost_boundaries);
  kCeresScanMatcherCostMetric = costs->Add({{"scan_matcher", "ceres"}});
  auto distance_boundaries = metrics::Histogram::ScaledPowersOf(2, 0.01, 10);
  auto* residuals = family_factory->NewHistogramFamily(
      "mapping_internal_2d_local_trajectory_builder_residuals",
      "Local scan matcher residuals", distance_boundaries);
  kScanMatcherResidualDistanceMetric =
      residuals->Add({{"component", "distance"}});
  kScanMatcherResidualAngleMetric = residuals->Add({{"component", "angle"}});
}

}  // namespace mapping
}  // namespace cartographer
