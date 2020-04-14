/*
 * Copyright 2017 The Cartographer Authors
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

#ifndef CARTOGRAPHER_MAPPING_POSE_EXTRAPOLATOR_H_
#define CARTOGRAPHER_MAPPING_POSE_EXTRAPOLATOR_H_

#include <deque>
#include <memory>

#include "cartographer/common/time.h"
#include "cartographer/mapping/imu_tracker.h"
#include "cartographer/sensor/imu_data.h"
#include "cartographer/sensor/odometry_data.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace mapping {

// Keep poses for a certain duration to estimate linear and angular velocity.
// Uses the velocities to extrapolate motion. Uses IMU and/or odometry data if
// available to improve the extrapolation.
// 保持位姿一定的持续时间，以估计线速度和角速度。
// 使用速度来推断运动。使用 IMU 和/或里程计数据（如果有）来改善 extrapolation。
// PoseExtrapolator 的主要作用是对 IMU、里程计数据进行融合，估计机器人的实时位姿。
/*
 * 从 cartographer 算法的流程框图我们可以知道，PoseExtrapolator 的输入有三个：从里程计传来的数据、
 * 经过 ImuTracker 处理过的对重力进行 aligned 过的 IMU 数据、从 Scan Matching 输出的Pose Observation。
 * PoseExtrapolator 对这三个输入进行融合后输出估计出来的 PoseEstimate。
 * 
 * 从该类开头的一段注释我们也可以知道，该类通过从 Scan Matching 输出的 PoseObservation 持续一段时间来跟踪 Poses，
 * 从而估计机器人的线速度和角速度。通过速度来解算机器人的运动。当 IMU 或里程计数据可用时，可与这些数据融合来提升解算结果。
 */
class PoseExtrapolator {
 public:
  // 构造函数
  explicit PoseExtrapolator(common::Duration pose_queue_duration,
                            double imu_gravity_time_constant);

  PoseExtrapolator(const PoseExtrapolator&) = delete;
  PoseExtrapolator& operator=(const PoseExtrapolator&) = delete;

  // 根据 IMU 数据来初始化一个 PoseExtrapolator
  static std::unique_ptr<PoseExtrapolator> InitializeWithImu(
      common::Duration pose_queue_duration, double imu_gravity_time_constant,
      const sensor::ImuData& imu_data);

  // Returns the time of the last added pose or Time::min() if no pose was added
  // yet.
  common::Time GetLastPoseTime() const;
  common::Time GetLastExtrapolatedTime() const;

  // 在时刻 time 往 Pose 队列中添加一个 Pose
  void AddPose(common::Time time, const transform::Rigid3d& pose);
  // 把新的 IMU 数据添加到队列中，删去队列中的过期数据
  void AddImuData(const sensor::ImuData& imu_data);
  void AddOdometryData(const sensor::OdometryData& odometry_data);
  transform::Rigid3d ExtrapolatePose(common::Time time);

  // Gravity alignment estimate.
  Eigen::Quaterniond EstimateGravityOrientation(common::Time time);

 private:
  // 从一个 Pose 队列中估计机器人的线速度和角速度
  void UpdateVelocitiesFromPoses();
  // 删去队列中无用的 IMU 数据
  void TrimImuData();
  // 删去队列中无用的里程计数据
  void TrimOdometryData();
  // 从 IMU 数据队列中取出最新的数据，更新 ImuTracker 的状态到指定的时间 time
  void AdvanceImuTracker(common::Time time, ImuTracker* imu_tracker) const;
  Eigen::Quaterniond ExtrapolateRotation(common::Time time,
                                         ImuTracker* imu_tracker) const;
  Eigen::Vector3d ExtrapolateTranslation(common::Time time);

  // Duration 是在/common/time.h中定义的别名
  // using Duration = UniversalTimeScaleClock::duration; //微秒,0.1us
  // UniversalTimeScaleClock 类实现 c++11 的 clock 接口，以 0.1us 为时间精度。
  const common::Duration pose_queue_duration_;
  struct TimedPose {
    // 带时间的 Pose
    common::Time time;
    transform::Rigid3d pose;
  };
  // 带时间的 Pose 队列
  std::deque<TimedPose> timed_pose_queue_;  // 存储要持续跟踪的 Poses,应该是从 ScanMatching 输出的 PoseObservation
  // 从持续跟踪一段时间的 Pose 队列中估计出来的线速度，初始化为0
  Eigen::Vector3d linear_velocity_from_poses_ = Eigen::Vector3d::Zero();
  // 从持续跟踪一段时间的 Pose 队列中估计出来的线速度，初始化为0
  Eigen::Vector3d angular_velocity_from_poses_ = Eigen::Vector3d::Zero();

  const double gravity_time_constant_;  // 重力的时间间隔？
  std::deque<sensor::ImuData> imu_data_;  // 存储 IMU 数据的队列
  std::unique_ptr<ImuTracker> imu_tracker_;  // ImuTracker，定义在/mapping/imu_tracker.h中
  std::unique_ptr<ImuTracker> odometry_imu_tracker_;  // 跟踪里程计
  std::unique_ptr<ImuTracker> extrapolation_imu_tracker_;  // 三个 ImuTracker 的智能指针，其中区别还不太懂
  // imu_tracker_ 是存放由 IMU 数据得到的信息
  // odometry_imu_tracker_ 是存放由里程计得到的信息
  // extrapolation_imu_tracker_ 是存放经过数据融合后的结果
  TimedPose cached_extrapolated_pose_;  // 缓存的一个带时间的 Pose

  std::deque<sensor::OdometryData> odometry_data_;  // 里程计数据
  // 从里程计获取到的线速度，初始化为0
  Eigen::Vector3d linear_velocity_from_odometry_ = Eigen::Vector3d::Zero();
  // 从里程计获取到的角速度，初始化为0
  Eigen::Vector3d angular_velocity_from_odometry_ = Eigen::Vector3d::Zero();
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_POSE_EXTRAPOLATOR_H_
