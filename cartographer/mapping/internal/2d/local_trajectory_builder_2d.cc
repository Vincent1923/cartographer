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

// 构造函数。函数体是空的，在它的构造列表中通过输入参数 options 完成了对各个成员变量的构造。
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

/**
 * 1. 这个函数主要是以重力方向为参考修正传感器数据后进行体素化滤波。
 *    实际上是把局部地图坐标系下的扫描数据平移到机器人坐标系上，但是不旋转。
 * 2. 函数有两个输入参数。
 *    transform_to_gravity_aligned_frame 表示局部地图坐标系到机器人坐标系下的重力方向的变换，
 *    其中 translation 为局部地图坐标系到机器人坐标系的平移，而 rotation 以四元数表示，近似为 (0,0,0,1)，即旋转角度近似为0。
 *    range_data 则是局部地图坐标系下的扫描数据。
 * 3. Cartographer 中的体素化滤波是把空间分为很多个立方体的栅格，然后一个栅格内可能有很多点，但只用一个点来代表整个栅格中的所有点。
 *    简单说，这些滤波就是对点云数据的预处理：去除不合理的范围的点、合并相同位置的点等，从而减少点云数量，提高点云质量。
 */
sensor::RangeData
LocalTrajectoryBuilder2D::TransformToGravityAlignedFrameAndFilter(
    const transform::Rigid3f& transform_to_gravity_aligned_frame,
    const sensor::RangeData& range_data) const {
  // std::cout << "transform_to_gravity_aligned_frame translation: " 
  //           << transform_to_gravity_aligned_frame.translation()[0] << " "
  //           << transform_to_gravity_aligned_frame.translation()[1] << " "
  //           << transform_to_gravity_aligned_frame.translation()[2] << std::endl;
  // std::cout << "transform_to_gravity_aligned_frame rotation: " << transform_to_gravity_aligned_frame.rotation().x() << " "
  //                                             << transform_to_gravity_aligned_frame.rotation().y() << " "
  //                                             << transform_to_gravity_aligned_frame.rotation().z() << " "
  //                                             << transform_to_gravity_aligned_frame.rotation().w() << std::endl;
  /**
   * 1. 把局部地图坐标系下的扫描数据平移到机器人坐标系上，但是不旋转，并对数据进行裁剪。
   * 2. 函数 TransformRangeData() 是把局部地图坐标系下的扫描数据平移到机器人坐标系上，但是不旋转。
   * 3. 函数 CropRangeData() 是裁剪数据，指定一定 z 轴范围内的数据，数据范围在 options_.min_z() 到 options_.max_z() 之间。
   *    参数在文件 "/src/cartographer/configuration_files/trajectory_builder_2d.lua" 中配置。
   * 4. 返回的扫描数据 cropped 包含三个字段，其中 origin 近似为 (0,0,0)，而 returns 和 misses
   *    则分别是局部地图坐标系下的 hit 点和 miss 点经过平移后在机器人坐标系下的空间坐标，但没有经过旋转。
   */
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

// 进行扫描匹配，主要是将当前的传感器数据与当前维护的子图进行匹配，寻找一个位姿估计使得传感器数据能够尽可能的与地图匹配上。
// 这是一个最优化的问题，Cartographer 主要通过 ceres 库求解。
// 它有三个参数，
// time 是参考时间，
// pose_prediction 是位姿估计器预测的位姿，只包含机器人在局部地图坐标系下的位置信息，但不包含方向信息。
// gravity_aligned_range_data 经过重力修正后的传感器数据，从局部地图坐标系平移到机器人坐标系下的扫描数据，但没有经过旋转。
std::unique_ptr<transform::Rigid2d> LocalTrajectoryBuilder2D::ScanMatch(
    const common::Time time, const transform::Rigid2d& pose_prediction,
    const sensor::RangeData& gravity_aligned_range_data) {
  // 在函数的一开始先获取对象 active_submaps_ 中维护的用于扫描匹配的旧图。
  std::shared_ptr<const Submap2D> matching_submap =
      active_submaps_.submaps().front();
  // The online correlative scan matcher will refine the initial estimate for
  // the Ceres scan matcher.
  // Online Correlative Scan Matcher 将优化 Ceres scan matcher 的初始估计
  //
  // 以输入的位姿估计器预测的位姿作为扫描匹配器迭代的初始位姿。
  transform::Rigid2d initial_ceres_pose = pose_prediction;
  // 接着构建了一个自适应的体素滤波器，再次对输入的扫描数据进行了一次过滤。
  sensor::AdaptiveVoxelFilter adaptive_voxel_filter(
      options_.adaptive_voxel_filter_options());
  const sensor::PointCloud filtered_gravity_aligned_point_cloud =
      adaptive_voxel_filter.Filter(gravity_aligned_range_data.returns);
  // 如果滤波后结果为空，则返回空指针
  if (filtered_gravity_aligned_point_cloud.empty()) {
    return nullptr;
  }
  // 我们还可以通过配置项要求使用对象 real_time_correlative_scan_matcher_ 所实现的一种在线的相关性扫描匹配算法，
  // 进一步的对位姿估计进行优化以得到一个较好的迭代初值。但配置项里默认该项为 false。
  if (options_.use_online_correlative_scan_matching()) {
    // 检查配置项 grid_type 是否为概率栅格地图
    CHECK_EQ(options_.submaps_options().grid_options_2d().grid_type(),
             proto::GridOptions2D_GridType_PROBABILITY_GRID);
    // 调用 RealTimeCorrelativeScanMatcher2D 的方法进行匹配，返回一个得分
    double score = real_time_correlative_scan_matcher_.Match(
        pose_prediction, filtered_gravity_aligned_point_cloud,
        *static_cast<const ProbabilityGrid*>(matching_submap->grid()),
        &initial_ceres_pose);
    kFastCorrelativeScanMatcherScoreMetric->Observe(score);
  }

  // 给定了迭代初值之后，就可以通过对象 ceres_scan_matcher_ 把扫描匹配问题描述成为一个最小二乘的问题，
  // 使用优化库 ceres 提供的工具寻优，得到一个使观测数据出现的概率最大化的位姿估计。该估计将被当作机器人的实际位姿参与后序的计算。
  // 寻优的结果将保存在对象 pose_observation 中，临时对象 summary 则记录了优化迭代的过程。
  auto pose_observation = common::make_unique<transform::Rigid2d>();
  ceres::Solver::Summary summary;
  ceres_scan_matcher_.Match(pose_prediction.translation(), initial_ceres_pose,
                            filtered_gravity_aligned_point_cloud,
                            *matching_submap->grid(), pose_observation.get(),
                            &summary);
  // 如果我们成功的求解了扫描匹配问题，那么就计算一下残差通过判据对象 kCeresScanMatcherCostMetric，
  // kScanMatcherResidualDistanceMetric 和 kScanMatcherResidualAngleMetric 检查一下计算结果。
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
  // 最后返回优化之后的位姿估计
  return pose_observation;
}

// 添加激光传感器的扫描数据，该函数返回的就是一个 MatchingResult 的对象，这是扫描匹配后的结果，
// 实际上该类型中有一个字段 insertion_result 用于描述把扫描数据插入子图的结果。
// 该函数基本上完成了整个 Local SLAM 的业务。
// 这个函数有两个输入参数，
// sensor_id 是激光雷达的索引，
// unsynchronized_data 是机器人坐标系下未经时间同步的扫描数据，它应该是从 ROS 系统中转换过来的。
std::unique_ptr<LocalTrajectoryBuilder2D::MatchingResult>
LocalTrajectoryBuilder2D::AddRangeData(
    const std::string& sensor_id,
    const sensor::TimedPointCloudData& unsynchronized_data) {
  // 在函数的一开始，先把输入的索引和数据记录在数据收集器 range_data_collator_ 中，
  // 同时我们将得到一个做了时间同步的扫描数据。检查一下数据，如果为空就报错退出。
  // RangeDataCollator 定义在 "/mapping/internal/range_data_collator.h" 中。
  // 该类的主要任务是把来自不同传感器的 TimePointCloudData 进行一下同步。
  auto synchronized_data =
      range_data_collator_.AddRangeData(sensor_id, unsynchronized_data);
  if (synchronized_data.ranges.empty()) {
    LOG(INFO) << "Range data collator filling buffer.";
    return nullptr;
  }

  // 检查通过之后，就用一个临时的对象 time 记录下同步时间。
  // synchronized_data.time 为扫描数据获取的时间，这里猜测为获取最后一个扫描点的时间。
  const common::Time& time = synchronized_data.time;
  // Initialize extrapolator now if we do not ever use an IMU.
  // 如果我们不使用 IMU，现在初始化位姿估计器 extrapolator。
  //
  // 接下来调用函数 InitializeExtrpolator() 构造位姿估计器。如果配置中说明不使用 IMU 估计位姿，
  // 那么就需要根据扫描匹配的结果估计位姿，要求 AddRangeData 完成位姿估计器的构造。
  if (!options_.use_imu_data()) {
    InitializeExtrapolator(time);
  }

  // 如果使用 IMU 进行估计，那么将在接收到 IMU 数据之后开始构造位姿估计器，这点可以通过追踪函数 AddImuData() 看到。
  if (extrapolator_ == nullptr) {
    // Until we've initialized the extrapolator with our first IMU message, we
    // cannot compute the orientation of the rangefinder.
    // 除非我们使用第一个 IMU 消息初始化位姿估计器 extrapolator，否则我们无法计算测距仪的方向。
    LOG(INFO) << "Extrapolator not yet initialized.";
    return nullptr;
  }

  // 检查扫描数据是否为空
  CHECK(!synchronized_data.ranges.empty());
  // TODO(gaschler): Check if this can strictly be 0.
  // 检查扫描数据的时间确保最后一个数据的时间偏移量小于等于0。
  // 然后获取第一个数据点的绝对时间，将它与位姿估计器的时间对比来判定位姿估计器是否完成了初始化操作。
  CHECK_LE(synchronized_data.ranges.back().point_time[3], 0.f);
  // 获取第一个扫描点的绝对时间，等于获取点云集的时间加上第一个扫描点记录的相对时间
  const common::Time time_first_point =  // 获取第一个扫描数据点的绝对时间
      time +
      common::FromSeconds(synchronized_data.ranges.front().point_time[3]);
  // 如果该时间比位姿估计器的最新时间还要早，说明在第一个扫描点被捕获时，位姿估计器还没初始化。
  if (time_first_point < extrapolator_->GetLastPoseTime()) {
    LOG(INFO) << "Extrapolator is still initializing.";
    return nullptr;
  }

  // 如果历史没有累积数据，就用成员变量 accumulation_started_ 记录下当前的时间，作为跟踪轨迹的开始时刻。
  if (num_accumulated_ == 0) {
    // accumulation_started_ 是跟踪轨迹的开始时刻
    accumulation_started_ = std::chrono::steady_clock::now();
  }

  // 接着创建了一个临时的容器 range_data_poses 用于记录各个扫描点所对应的机器人位姿，
  // 并为它申请了与测量点数量相同的内存空间。变量 warned 用于控制警告信息的打印。
  // 这里记录每个扫描点获取的时间所对应的机器人位姿，是为了消除相邻扫描点获取的时间差所引起的误差，
  // 可以更精确地计算扫描点在地图的坐标。
  std::vector<transform::Rigid3f> range_data_poses;
  range_data_poses.reserve(synchronized_data.ranges.size());  // 分配与扫描点数量相同的内存空间
  bool warned = false;
  // 该函数在一个 for 循环中通过位姿估计器获取各个扫描点对应的机器人位姿。
  // 在循环的一开始，先计算测量时间，并与位姿估计器的时间对比。
  // 如果时间滞后，在给出了警告信息后，盲目的调整时间，以防止因果倒置的问题。
  // 如果一切正常，我们就可以通过对象 extrapolator_ 和测量时间获取对应的位姿。
  for (const auto& range : synchronized_data.ranges) {  // 遍历点集
    common::Time time_point = time + common::FromSeconds(range.point_time[3]);  // 获取扫描点的绝对时间
    // 扫描点的绝对时间与位姿估计器 extrapolator_ 的时间对比
    if (time_point < extrapolator_->GetLastExtrapolatedTime()) {
      if (!warned) {
        LOG(ERROR)
            << "Timestamp of individual range data point jumps backwards from "
            << extrapolator_->GetLastExtrapolatedTime() << " to " << time_point;
        warned = true;
      }
      // 盲目的调整时间，时间设置为位姿估计器 extrapolator_ 的时间
      time_point = extrapolator_->GetLastExtrapolatedTime();
    }
    // 通过位姿估计器 extrapolator_ 推算扫描点对应的机器人位姿
    range_data_poses.push_back(
        extrapolator_->ExtrapolatePose(time_point).cast<float>());
  }

  // 如果我们还没有累积过扫描数据，就重置对象 accumulated_range_data_。
  if (num_accumulated_ == 0) {
    // 'accumulated_range_data_.origin' is uninitialized until the last
    // accumulation.
    accumulated_range_data_ = sensor::RangeData{{}, {}, {}};
  }

  // Drop any returns below the minimum range and convert returns beyond the
  // maximum range into misses.
  // 把任何 returns 降至 minimum range 以下，并将超出 maximum range 的 returns 转换为 misses
  //
  // 在接下来的 for 循环中，抛弃掉所有测量距离小于配置 min_range 的 hit 点，并把那些超过配置 max_range 的测量点划归 miss 点。
  for (size_t i = 0; i < synchronized_data.ranges.size(); ++i) {
    // 获取 hit 点在机器人坐标系下的坐标。
    // Eigen::Vector4f 是一个 4 × 1 的 float 向量。
    const Eigen::Vector4f& hit = synchronized_data.ranges[i].point_time;
    // 计算传感器在局部地图坐标系下的坐标。Eigen::Vector3f 是一个 3 × 1 的 float 向量。
    // 通过左乘机器人位姿，将机器人坐标系下的传感器坐标转换到局部地图的坐标系下。
    // range_data_poses[i] 表示机器人位姿，数据类型为 transform::Rigid3f，
    // synchronized_data.origins.at(synchronized_data.ranges[i].origin_index) 表示传感器在机器人坐标系下的坐标，数据类型为 Eigen::Vector3f，
    // 这里利用类型 transform::Rigid3f 的重载运算符 * 来进行计算。
    const Eigen::Vector3f origin_in_local =
        range_data_poses[i] *
        synchronized_data.origins.at(synchronized_data.ranges[i].origin_index);
    // 计算 hit 点在局部坐标系下的位姿。Eigen::Vector3f 是一个 3 × 1 的 float 向量。
    // 通过左乘机器人位姿，将激光测量的 hit 点在机器人坐标系下的坐标转换到局部地图的坐标系下。
    // range_data_poses[i] 表示机器人位姿，数据类型为 transform::Rigid3f，
    // hit 表示 hit 点在机器人坐标系下的坐标，数据类型为 Eigen::Vector4f，是一个 4 × 1 的 float 向量，
    // hit.head<3>() 表示取出向量 hit 的前三项变成类型 Eigen::Vector3f，是一个 3 × 1 的 float 向量，
    // 这里利用类型 transform::Rigid3f 的重载运算符 * 来进行计算。
    const Eigen::Vector3f hit_in_local = range_data_poses[i] * hit.head<3>();
    // 计算测量距离 range。
    // 实际上这个计算有点重复了，因为 ROS 系统中原始的激光数据就是距离信息，这里来回转换了一遍有点多余。
    const Eigen::Vector3f delta = hit_in_local - origin_in_local;  // 局部地图坐标系下由 hit 点到 origin 的射线
    const float range = delta.norm();  // 该向量的模
    // 在下面的 if-else 语句中，完成 hit 点和 miss 点的筛选工作。
    // 参数 min_range() 和 max_range() 主要是设置雷达数据的最小距离和最大距离，
    // 配置参数的文件在 "cartographer/configuration_files/trajectory_builder_2d.lua" 中。
    if (range >= options_.min_range()) {
      if (range <= options_.max_range()) {
        // 如果该 hit 点的测量距离在 min_range 和 max_range 之间，则把它压入 accumulated_range_data_ 的 returns 集合中。
        accumulated_range_data_.returns.push_back(hit_in_local);
      } else {
        // 否则，如果该 hit 点的测量距离大于 max_range，则放入 accumulated_range_data_ 的 misses 集合中。
        // 参数 missing_data_ray_length() 在文件 "cartographer/configuration_files/trajectory_builder_2d.lua" 配置。
        accumulated_range_data_.misses.push_back(
            origin_in_local +
            options_.missing_data_ray_length() / range * delta);
      }
    }
  }
  ++num_accumulated_;

  // 每当累积的传感器数据数量超过了配置值 num_accumulated_range_data，就会调用函数 AddAccumulatedRangeData()
  // 完成 Local SLAM 的几项核心任务，并返回记录了子图插入结果的扫描匹配结果。
  // 参数 num_accumulated_range_data() 在文件 "trajectory_builder_2d.lua" 设置，默认数值为1。
  if (num_accumulated_ >= options_.num_accumulated_range_data()) {
    // 先将计数清零
    num_accumulated_ = 0;
    // 通过位姿估计器获取重力的方向。
    // 对于单线激光扫描消息 laser_scan，重力方向 gravity_alignment 的平移 translation 为 (0,0,0)，
    // 而旋转 rotation 为机器人在局部地图坐标系下的方向，以四元数表示。
    // 所以 gravity_alignment 包含机器人在局部地图坐标系下的方向信息。
    const transform::Rigid3d gravity_alignment = transform::Rigid3d::Rotation(
        extrapolator_->EstimateGravityOrientation(time));
    // TODO(gaschler): This assumes that 'range_data_poses.back()' is at time
    // 'time'.
    // 记录下当前的累积位姿，这是最后一个扫描点对应的机器人在局部地图坐标系下的位姿
    accumulated_range_data_.origin = range_data_poses.back().translation();
    /**
     * 1. 调用函数 AddAccumulatedRangeData() 进行扫描匹配、插入数据等工作，并返回扫描匹配的结果。
     * 2. 在给函数 AddAccumulatedRangeData() 传参的时候调用了函数 TransformToGravityAlignedFrameAndFilter()，
     *    这个函数主要是以重力方向为参考修正传感器数据后进行体素化滤波。
     *    实际上是把局部地图坐标系下的扫描数据平移到机器人坐标系上，但是不旋转。
     *    函数 TransformToGravityAlignedFrameAndFilter() 第一个参数的类型为 transform::Rigid3f，
     *    表示局部地图坐标系到机器人坐标系下的重力方向的变换，其中 translation 为局部地图坐标系到机器人坐标系的平移，
     *    而 rotation 以四元数表示，近似为 (0,0,0,1)，即旋转角度近似为0。
     */
    return AddAccumulatedRangeData(
        time,
        TransformToGravityAlignedFrameAndFilter(
            gravity_alignment.cast<float>() * range_data_poses.back().inverse(),
            accumulated_range_data_),
        gravity_alignment);
  }
  return nullptr;
}

// 添加累积的传感器数据，完成 Local SLAM 的几项核心任务，主要是进行扫描匹配、把数据插入子图等操作，
// 并返回记录了子图插入结果的扫描匹配结果。
// 它有三个参数：
// time 是当前同步时间，
// gravity_aligned_range_data 则是经过重力修正后的传感器数据，从局部地图坐标系平移到机器人坐标系下的扫描数据，但没有经过旋转，
// gravity_alignment 是重力方向，translation 近似为 (0,0,0)，rotation 为当前机器人在局部地图坐标系下的方向，
// 所以只包含机器人在局部地图坐标系下的方向信息。
std::unique_ptr<LocalTrajectoryBuilder2D::MatchingResult>
LocalTrajectoryBuilder2D::AddAccumulatedRangeData(
    const common::Time time,
    const sensor::RangeData& gravity_aligned_range_data,
    const transform::Rigid3d& gravity_alignment) {
  // 在函数的一开始先检查输入的数据不空
  if (gravity_aligned_range_data.returns.empty()) {
    LOG(WARNING) << "Dropped empty horizontal range data.";
    return nullptr;
  }

  // Computes a gravity aligned pose prediction.
  // 计算重力修正之后的机器人位姿。
  // 从位姿估计器 extrapolator_ 获取未经重力修正的机器人位姿。
  // 这里的位姿 non_gravity_aligned_pose_prediction 包含机器人在局部地图坐标系下的位置和方向，
  // 其中 non_gravity_aligned_pose_prediction.translation() 表示位置信息，
  // 而 non_gravity_aligned_pose_prediction.rotation() 则表示方向信息。
  const transform::Rigid3d non_gravity_aligned_pose_prediction =
      extrapolator_->ExtrapolatePose(time);
  // 计算重力修正之后的机器人位姿 pose_prediction。
  // 这里需要注意，pose_prediction 的 translation 为机器人在局部地图坐标系下的位置，
  // 而 rotation 用四元数表示后近似为 (0,0,0,1)。
  // 即 pose_prediction 只包含机器人在局部地图坐标系下的位置信息，但不包含方向信息。
  // gravity_alignment 是重力方向，translation 近似为 (0,0,0)，rotation 为当前机器人在局部地图坐标系下的方向。
  const transform::Rigid2d pose_prediction = transform::Project2D(
      non_gravity_aligned_pose_prediction * gravity_alignment.inverse());

  // local map frame <- gravity-aligned frame
  // 通过函数 ScanMatch() 进行扫描匹配，并返回新的位姿估计 pose_estimate_2d。
  // pose_estimate_2d 是机器人在局部地图坐标系下的位姿估计。
  // 同样地，pose_estimate_2d 的 translation 为机器人在局部地图坐标系下的位置估计，
  // 而 rotation 用四元数表示后近似为 (0,0,0,1)。
  // 所以，pose_estimate_2d 也只包含机器人在局部地图坐标系下的位置信息，但不包含方向信息。
  std::unique_ptr<transform::Rigid2d> pose_estimate_2d =
      ScanMatch(time, pose_prediction, gravity_aligned_range_data);
  // Eigen::AngleAxis<double> rotation_vector =
  //     Eigen::AngleAxis<double>(pose_estimate_2d->rotation().angle(),
  //                              Eigen::Matrix<double, 3, 1>::UnitZ());
  // Eigen::Quaternion<double> pose_estimate_2d_quaternion(rotation_vector);
  // std::cout << "pose_estimate_2d quaternion: "
  //           << pose_estimate_2d_quaternion.x() << " "
  //           << pose_estimate_2d_quaternion.y() << " "
  //           << pose_estimate_2d_quaternion.z() << " "
  //           << pose_estimate_2d_quaternion.w() << std::endl;
  // 如果匹配结果为空，表示匹配失败，直接返回
  if (pose_estimate_2d == nullptr) {
    LOG(WARNING) << "Scan matching failed.";
    return nullptr;
  }
  // 计算新的位姿估计 pose_estimate。
  // 通过函数 transform::Embed3D() 将 2d 的位姿变成 3d 的位姿，接着再利用重力方向进行修正。
  // 由于 pose_estimate_2d 只包含机器人的位置信息，而 gravity_alignment 只包含方向信息，
  // 所以经过计算之后，pose_estimate 则包含机器人在局部地图坐标系下的位置和方向信息。
  const transform::Rigid3d pose_estimate =
      transform::Embed3D(*pose_estimate_2d) * gravity_alignment;
  // 然后将新的位姿估计反馈给位姿估计器对象 extrapolator_
  extrapolator_->AddPose(time, pose_estimate);

  /**
   * 1. 利用新的位姿估计，将传感器的数据转换到局部地图坐标系下。
   * 2. 这里需要注意，pose_estimate_2d 是优化后的位姿估计，机器人在局部地图坐标系下的位置，只包含位置信息。
   * 3. gravity_aligned_range_data 是经过重力修正后的传感器数据，从局部地图坐标系平移到机器人坐标系下的扫描数据，但没有经过旋转，
   *    包含三个字段，其中 origin 近似为 (0,0,0)，而 returns 和 misses 则分别是
   *    局部地图坐标系下的 hit 点和 miss 点经过平移后在机器人坐标系下的空间坐标，但没有经过旋转。
   * 4. 所以，计算之后得到的 range_data_in_local 是在局部地图坐标系下的扫描数据，包含三个字段，
   *    其中 origin 是当次扫描测量时机器人在局部地图坐标系的位置，
   *    returns 和 misses 则分别记录了扫描数据的 hit 点和 miss 点在局部地图坐标系下的空间坐标。
   */
  sensor::RangeData range_data_in_local =
      TransformRangeData(gravity_aligned_range_data,
                         transform::Embed3D(pose_estimate_2d->cast<float>()));
  /**
   * 1. 通过函数 InsertIntoSubmap() 将新的扫描数据插入到子图中。
   * 2. range_data_in_local 是在优化之后的位姿估计下观测到的 hit 点和 miss 点在局部地图坐标系下的点云数据，
   *    包含三个字段，其中 origin 是当次扫描测量时机器人在局部地图坐标系的位置，
   *    returns 和 misses 则分别记录了 hit 点和 miss 点在局部地图坐标系下的空间坐标。
   * 3. gravity_aligned_range_data 是扫描匹配之前执行了重力修正的扫描数据，从局部地图坐标系平移到机器人坐标系下的扫描数据，但没有经过旋转，
   *    包含三个字段，其中 origin 近似为 (0,0,0)，而 returns 和 misses 则分别是
   *    局部地图坐标系下的 hit 点和 miss 点经过平移后在机器人坐标系下的空间坐标，但没有经过旋转。
   * 4. pose_estimate 是优化之后的位姿估计，机器人在局部地图坐标系下的位姿，包含位置和方向信息。
   * 5. gravity_alignment 是重力方向，表示当前机器人在局部地图坐标系下的方向。
   */
  std::unique_ptr<InsertionResult> insertion_result =
      InsertIntoSubmap(time, range_data_in_local, gravity_aligned_range_data,
                       pose_estimate, gravity_alignment.rotation());
  // 然后统计数据累积的时间。kLocalSlamLatencyMetric 应该是用来监视延迟的。
  auto duration = std::chrono::steady_clock::now() - accumulation_started_;
  kLocalSlamLatencyMetric->Set(
      std::chrono::duration_cast<std::chrono::seconds>(duration).count());
  /**
   *  
   * 1. 返回匹配结果，为一个指向数据类型 MatchingResult 的智能指针，有4个字段。
   * 2. time 是当前同步时间；
   * 3. pose_estimate 是优化后机器人在局部地图坐标系下的位姿，包含位置和方向信息；
   * 4. range_data_in_local 是在优化之后的位姿估计下观测到的 hit 点和 miss 点在局部地图坐标系下的点云数据，
   *    包含三个字段，其中 origin 是当次扫描测量时机器人在局部地图坐标系的位置，
   *    returns 和 misses 则分别记录了 hit 点和 miss 点在局部地图坐标系下的空间坐标。
   * 5. insertion_result 是子图插入结果，它是指向类型 InsertionResult 的智能指针，有两个字段。
   *    (1) constant_data 是插入的节点数据，类型为 TrajectoryNode::Data，这里包含以下4个更新的字段：
   *        time 是当前同步时间；
   *        gravity_alignment 是重力方向，机器人在局部地图坐标系下的方向；
   *        filtered_gravity_aligned_point_cloud 是经过滤波和重力修正后的扫描数据，从局部地图坐标系平移到机器人坐标系下的扫描数据；
   *        local_pose 为优化后机器人在局部地图坐标系下的位姿，包含位置和方向信息。
   *    (2) insertion_submaps 是扫描数据所插入的 active_submaps_ 当前维护的子图对象，一般 size = 2。
   */
  return common::make_unique<MatchingResult>(
      MatchingResult{time, pose_estimate, std::move(range_data_in_local),
                     std::move(insertion_result)});
}

// 将传感器数据插入到当前正在维护的子图中，并返回插入结果。
// 该函数有5个参数：
// time 是当前同步时间，
// range_data_in_local 是在优化之后的位姿估计下观测到的 hit 点和 miss 点在局部地图坐标系下的点云数据，
// gravity_aligned_range_data 中记录的则是扫描匹配之前执行了重力修正的扫描数据，从局部地图坐标系平移到机器人坐标系下的扫描数据，但没有经过旋转，
// pose_estimate 则是优化之后的位姿估计，机器人在局部地图坐标系下的位姿，包含位置和方向信息，
// gravity_alignment 描述了重力方向，表示当前机器人在局部地图坐标系下的方向。
std::unique_ptr<LocalTrajectoryBuilder2D::InsertionResult>
LocalTrajectoryBuilder2D::InsertIntoSubmap(
    const common::Time time, const sensor::RangeData& range_data_in_local,
    const sensor::RangeData& gravity_aligned_range_data,
    const transform::Rigid3d& pose_estimate,
    const Eigen::Quaterniond& gravity_alignment) {
  // 在函数的一开始，先通过运动滤波器来判定当前是否发生了明显的运动，若未发生则直接返回退出。这样可以减少很多数据量。
  if (motion_filter_.IsSimilar(time, pose_estimate)) {
    return nullptr;
  }

  // Querying the active submaps must be done here before calling
  // InsertRangeData() since the queried values are valid for next insertion.
  // 在调用 InsertRangeData() 之前，必须在此处完成对 active submaps 的查询，因为查询的值对于下一次插入有效。
  //
  // 在通知对象 active_submaps_ 接收新的数据更新地图之前，先把其维护的子图暂时保存在临时的容器 insertion_submaps 中。
  // 构图起始阶段，active_submaps_ 维护的子图数量为1。
  // 随着接收数据更新地图之后，当系统总的子图数量大于1时，active_submaps_ 维护的子图数量会一直维持在2。
  std::vector<std::shared_ptr<const Submap2D>> insertion_submaps;
  for (const std::shared_ptr<Submap2D>& submap : active_submaps_.submaps()) {
    insertion_submaps.push_back(submap);
  }
  // 调用对象 active_submaps_ 的函数 InsertRangeData() 将传感器数据插入，更新子图。
  active_submaps_.InsertRangeData(range_data_in_local);

  // 接下来出于闭环检测的目的，构建一个自适应的体素滤波器对重力修正的扫描数据进行滤波。
  sensor::AdaptiveVoxelFilter adaptive_voxel_filter(
      options_.loop_closure_adaptive_voxel_filter_options());
  const sensor::PointCloud filtered_gravity_aligned_point_cloud =
      adaptive_voxel_filter.Filter(gravity_aligned_range_data.returns);

  // 最后构建一个 InsertionResult 对象并返回
  return common::make_unique<InsertionResult>(InsertionResult{
      std::make_shared<const TrajectoryNode::Data>(TrajectoryNode::Data{
          time,                                  // 当前同步时间
          gravity_alignment,                     // 重力方向，机器人在局部地图坐标系下的方向
          filtered_gravity_aligned_point_cloud,  // 经过滤波和重力修正后的扫描数据，从局部地图坐标系平移到机器人坐标系下的扫描数据
          {},  // 'high_resolution_point_cloud' is only used in 3D.
          {},  // 'low_resolution_point_cloud' is only used in 3D.
          {},  // 'rotational_scan_matcher_histogram' is only used in 3D.
          pose_estimate}),                       // 优化之后的位姿估计，机器人在局部地图坐标系下的位姿，包含位置和方向信息
      std::move(insertion_submaps)});            // 扫描数据所插入的 active_submaps_ 当前维护的子图对象
}

// 接收 IMU 数据，并完成位姿估计器 extrapolator_ 的初始化工作
void LocalTrajectoryBuilder2D::AddImuData(const sensor::ImuData& imu_data) {
  // 检查配置项是否要求使用 IMU 数据
  CHECK(options_.use_imu_data()) << "An unexpected IMU packet was added.";
  // 通过函数 InitializeExtrapolator() 完成位姿估计器 extrapolator_ 的初始化工作
  InitializeExtrapolator(imu_data.time);
  // 最后将 IMU 的数据喂给 extrapolator_ 对象
  extrapolator_->AddImuData(imu_data);
}

// 接收里程计数据
void LocalTrajectoryBuilder2D::AddOdometryData(
    const sensor::OdometryData& odometry_data) {
  // 需要先通过检查对象 extrapolator_ 是否为空指针来判定位姿估计器是否已经完成初始化工作了
  if (extrapolator_ == nullptr) {
    // Until we've initialized the extrapolator we cannot add odometry data.
    // 在位姿估计器完成初始化之前，我们无法添加里程计数据。
    LOG(INFO) << "Extrapolator not yet initialized.";
    return;
  }
  // 若通过则直接将里程计的数据喂给 extrapolator_ 对象
  extrapolator_->AddOdometryData(odometry_data);
}

// 初始化位姿估计器对象 extrapolator_
void LocalTrajectoryBuilder2D::InitializeExtrapolator(const common::Time time) {
  // 在函数的一开始检查对象 extrapolator_ 是否是一个空指针，
  // 如果不是意味着已经创建了一个位姿估计器对象，直接返回。
  if (extrapolator_ != nullptr) {
    return;
  }
  // We derive velocities from poses which are at least 1 ms apart for numerical
  // stability. Usually poses known to the extrapolator will be further apart
  // in time and thus the last two are used.
  // 我们从至少 1ms 的位姿中得出速度，以实现数值稳定性。通常，位姿估计器已知的位姿会在时间上分开，因此使用最后两个位姿。
  // 该参数配置的是 PoseExtrapolator 中 Pose 队列的持续时间。设置为1ms。
  constexpr double kExtrapolationEstimationTimeSec = 0.001;
  // TODO(gaschler): Consider using InitializeWithImu as 3D does.
  // 创建一个 PoseExtrapolator 类型的位姿估计器
  extrapolator_ = common::make_unique<PoseExtrapolator>(
      ::cartographer::common::FromSeconds(kExtrapolationEstimationTimeSec),
      options_.imu_gravity_time_constant());
  // 最后添加一个初始位姿 I
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
