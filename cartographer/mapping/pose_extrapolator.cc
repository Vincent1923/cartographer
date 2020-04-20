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

#include "cartographer/mapping/pose_extrapolator.h"

#include <algorithm>

#include "cartographer/common/make_unique.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

// 构造函数
PoseExtrapolator::PoseExtrapolator(const common::Duration pose_queue_duration,
                                   double imu_gravity_time_constant)
    : pose_queue_duration_(pose_queue_duration),
      gravity_time_constant_(imu_gravity_time_constant),
      cached_extrapolated_pose_{common::Time::min(),
                                transform::Rigid3d::Identity()} {}  // 初始化 cached_extrapolated_pose_ 为单位矩阵I

// 根据 IMU 数据来初始化一个 PoseExtrapolator
std::unique_ptr<PoseExtrapolator> PoseExtrapolator::InitializeWithImu(
    const common::Duration pose_queue_duration,
    const double imu_gravity_time_constant, const sensor::ImuData& imu_data) {
  // 首先生成一个 PoseExtrapolator 的变量
  auto extrapolator = common::make_unique<PoseExtrapolator>(
      pose_queue_duration, imu_gravity_time_constant);
  // 添加 IMU 数据
  extrapolator->AddImuData(imu_data);
  // 生成 ImuTracker 并更新
  extrapolator->imu_tracker_ =
      common::make_unique<ImuTracker>(imu_gravity_time_constant, imu_data.time);
  extrapolator->imu_tracker_->AddImuLinearAccelerationObservation(
      imu_data.linear_acceleration);
  extrapolator->imu_tracker_->AddImuAngularVelocityObservation(
      imu_data.angular_velocity);
  extrapolator->imu_tracker_->Advance(imu_data.time);
  // 添加一个 Pose。
  extrapolator->AddPose(
      imu_data.time,
      transform::Rigid3d::Rotation(extrapolator->imu_tracker_->orientation()));
  return extrapolator;
}

// 返回 Pose 队列的最新时间
common::Time PoseExtrapolator::GetLastPoseTime() const {
  // 如果 Pose 队列为空
  if (timed_pose_queue_.empty()) {
    return common::Time::min();
  }
  return timed_pose_queue_.back().time;
}

// 返回解算结果的最新时间
common::Time PoseExtrapolator::GetLastExtrapolatedTime() const {
  if (!extrapolation_imu_tracker_) {
    return common::Time::min();
  }
  return extrapolation_imu_tracker_->time();
}

// 在时刻 time 往 Pose 队列中添加一个 Pose
void PoseExtrapolator::AddPose(const common::Time time,
                               const transform::Rigid3d& pose) {
  // 如果 ImuTracker 还没有建立，则需要建立一个 ImuTracker
  if (imu_tracker_ == nullptr) {
    common::Time tracker_start = time;
    // 如果 IMU 数据队列不为空，则以当前时间和 IMU 数据中的最早时刻的较小值为初始时刻建立一个 ImuTracker
    if (!imu_data_.empty()) {
      tracker_start = std::min(tracker_start, imu_data_.front().time);
    }
    //　生成 ImuTracker
    imu_tracker_ =
        common::make_unique<ImuTracker>(gravity_time_constant_, tracker_start);
  }
  //　把 Pose 压入队列
  timed_pose_queue_.push_back(TimedPose{time, pose});
  // Pose 队列大于2，并且时间间隔已经大于我们设定的 pose_queue_duration_ 时，则把队列之前的元素删除
  while (timed_pose_queue_.size() > 2 &&
         timed_pose_queue_[1].time <= time - pose_queue_duration_) {
    timed_pose_queue_.pop_front();
  }
  UpdateVelocitiesFromPoses();  // 更新最新估计的速度
  AdvanceImuTracker(time, imu_tracker_.get());  // 更新 ImuTracker
  TrimImuData();  // 更新  IMU数据队列
  TrimOdometryData();  // 更新里程计数据队列
  // 里程计和融合结果都以当前 IMU 的 tracker 为准
  odometry_imu_tracker_ = common::make_unique<ImuTracker>(*imu_tracker_);
  extrapolation_imu_tracker_ = common::make_unique<ImuTracker>(*imu_tracker_);
}

// 把新的 IMU 数据添加到队列中，删去队列中的过期数据
void PoseExtrapolator::AddImuData(const sensor::ImuData& imu_data) {
  CHECK(timed_pose_queue_.empty() ||
        imu_data.time >= timed_pose_queue_.back().time);
  imu_data_.push_back(imu_data);
  TrimImuData();  // 删去队列中无用的 IMU 数据
}

void PoseExtrapolator::AddOdometryData(
    const sensor::OdometryData& odometry_data) {
  CHECK(timed_pose_queue_.empty() ||
        odometry_data.time >= timed_pose_queue_.back().time);
  odometry_data_.push_back(odometry_data);
  TrimOdometryData();
  if (odometry_data_.size() < 2) {
    return;
  }
  // TODO(whess): Improve by using more than just the last two odometry poses.
  // Compute extrapolation in the tracking frame.
  const sensor::OdometryData& odometry_data_oldest = odometry_data_.front();
  const sensor::OdometryData& odometry_data_newest = odometry_data_.back();
  const double odometry_time_delta =
      common::ToSeconds(odometry_data_oldest.time - odometry_data_newest.time);
  const transform::Rigid3d odometry_pose_delta =
      odometry_data_newest.pose.inverse() * odometry_data_oldest.pose;
  angular_velocity_from_odometry_ =
      transform::RotationQuaternionToAngleAxisVector(
          odometry_pose_delta.rotation()) /
      odometry_time_delta;
  if (timed_pose_queue_.empty()) {
    return;
  }
  const Eigen::Vector3d
      linear_velocity_in_tracking_frame_at_newest_odometry_time =
          odometry_pose_delta.translation() / odometry_time_delta;
  const Eigen::Quaterniond orientation_at_newest_odometry_time =
      timed_pose_queue_.back().pose.rotation() *
      ExtrapolateRotation(odometry_data_newest.time,
                          odometry_imu_tracker_.get());
  linear_velocity_from_odometry_ =
      orientation_at_newest_odometry_time *
      linear_velocity_in_tracking_frame_at_newest_odometry_time;
}

transform::Rigid3d PoseExtrapolator::ExtrapolatePose(const common::Time time) {
  const TimedPose& newest_timed_pose = timed_pose_queue_.back();
  CHECK_GE(time, newest_timed_pose.time);
  if (cached_extrapolated_pose_.time != time) {
    const Eigen::Vector3d translation =
        ExtrapolateTranslation(time) + newest_timed_pose.pose.translation();
    const Eigen::Quaterniond rotation =
        newest_timed_pose.pose.rotation() *
        ExtrapolateRotation(time, extrapolation_imu_tracker_.get());
    cached_extrapolated_pose_ =
        TimedPose{time, transform::Rigid3d{translation, rotation}};
  }
  return cached_extrapolated_pose_.pose;
}

Eigen::Quaterniond PoseExtrapolator::EstimateGravityOrientation(
    const common::Time time) {
  ImuTracker imu_tracker = *imu_tracker_;
  AdvanceImuTracker(time, &imu_tracker);
  return imu_tracker.orientation();
}

// 从一个 Pose 队列中估计机器人的线速度和角速度
// 取出 timed_pose_queue_ 这个队列中最早和最新的两个 Pose 做差，然后除以时间得到机器人的速度。
void PoseExtrapolator::UpdateVelocitiesFromPoses() {
  // 判断这个 Pose 队列的长度，如果小于2则没法进行估计
  if (timed_pose_queue_.size() < 2) {
    // We need two poses to estimate velocities.
    return;
  }
  CHECK(!timed_pose_queue_.empty());  // 检查队列是否为空
  // 取出队列最末尾的一个 Pose,也就是最新时间点的 Pose,并记录相应的时间
  const TimedPose& newest_timed_pose = timed_pose_queue_.back();
  const auto newest_time = newest_timed_pose.time;
  // 取出队列最开头的一个 Pose，也就是最旧时间点的 Pose,并记录相应的时间
  const TimedPose& oldest_timed_pose = timed_pose_queue_.front();
  const auto oldest_time = oldest_timed_pose.time;
  // 两者的时间差
  const double queue_delta = common::ToSeconds(newest_time - oldest_time);
  // 如果时间差小于1ms，则估计不准，弹出警告信息
  if (queue_delta < 0.001) {  // 1 ms
    LOG(WARNING) << "Queue too short for velocity estimation. Queue duration: "
                 << queue_delta << " ms";
    return;
  }
  // 获取两个时刻各自的 Pose
  const transform::Rigid3d& newest_pose = newest_timed_pose.pose;
  const transform::Rigid3d& oldest_pose = oldest_timed_pose.pose;
  // 线速度即为两个 Pose 的 translation 部分相减后除以间隔时间
  linear_velocity_from_poses_ =
      (newest_pose.translation() - oldest_pose.translation()) / queue_delta;
  // 角速度是两个 Pose 的 rotation 部分的差
  // 这里特殊说明一下，姿态的差是转成了一个角轴向量
  // oldest_pose.rotation().inverse() 是计算四元数的逆
  angular_velocity_from_poses_ =
      transform::RotationQuaternionToAngleAxisVector(
          oldest_pose.rotation().inverse() * newest_pose.rotation()) /  // 计算旋转向量
      queue_delta;
}

// 删去队列中无用的 IMU 数据
void PoseExtrapolator::TrimImuData() {
  // 需要满足三个条件：IMU 数据队列大于1，Pose 的队列不为空，IMU 数据队列的第一个元素时间小于 Pose 队列的最后一个元素的时间
  // 最后一个条件意味着当 IMU 数据的时间比一个最新的 Pose 的时间要早时，说明这个 IMU 数据已经过期了。所以从队列中删掉就可以了。
  // 知道 IMU 数据的时间要比最新的 Pose 时间晚，那么说明这时候这个数据还有用，
  // 这种情况就不再删了，跳出循环，等待其他程序取出队列最开头的 IMU 数据进行融合。
  while (imu_data_.size() > 1 && !timed_pose_queue_.empty() &&
         imu_data_[1].time <= timed_pose_queue_.back().time) {
    imu_data_.pop_front();
  }
}

// 与处理 IMU 队列逻辑相同，删去队列中无用的里程计数据
void PoseExtrapolator::TrimOdometryData() {
  while (odometry_data_.size() > 2 && !timed_pose_queue_.empty() &&
         odometry_data_[1].time <= timed_pose_queue_.back().time) {
    odometry_data_.pop_front();
  }
}

// 从 IMU 数据队列中取出最新的数据，更新 ImuTracker 的状态到指定的时间 time
void PoseExtrapolator::AdvanceImuTracker(const common::Time time,
                                         ImuTracker* const imu_tracker) const {
  // 当前时间是否晚于 ImuTracker 的时间，如果是，需要更新 ImuTracker
  CHECK_GE(time, imu_tracker->time());
  if (imu_data_.empty() || time < imu_data_.front().time) {
    // There is no IMU data until 'time', so we advance the ImuTracker and use
    // the angular velocities from poses and fake gravity to help 2D stability.
    // 如果 IMU 数据队列为空，或当前时间要比 IMU 数据队列中最早期的时间还要早。说明没有可用的 IMU 数据。
    // 这时候根据时间推算。对于角速度，是用从 Pose 中估计的角速度或从里程计获得的角速度更新 ImuTracker 的角速度。
    imu_tracker->Advance(time);
    // 用一个 fake 的重力方向更新线加速度。这个 fake 的重力方向就是[0,0,1]。理想情况。
    imu_tracker->AddImuLinearAccelerationObservation(Eigen::Vector3d::UnitZ());
    imu_tracker->AddImuAngularVelocityObservation(
        odometry_data_.size() < 2 ? angular_velocity_from_poses_
                                  : angular_velocity_from_odometry_);
    return;
  }
  // 如果 ImuTracker 维护的时间早于 IMU 数据队列最早期的时间
  if (imu_tracker->time() < imu_data_.front().time) {
    // Advance to the beginning of 'imu_data_'.
    // 先把 ImuTracker 更新到 IMU 数据来临的那一刻
    imu_tracker->Advance(imu_data_.front().time);
  }
  // 然后依次取出 IMU 数据队列中的数据，更新 ImuTracker，直到 IMU 数据的时间比指定时间 time 要晚。
  auto it = std::lower_bound(
      imu_data_.begin(), imu_data_.end(), imu_tracker->time(),
      [](const sensor::ImuData& imu_data, const common::Time& time) {
        return imu_data.time < time;
      });
  while (it != imu_data_.end() && it->time < time) {
    imu_tracker->Advance(it->time);
    imu_tracker->AddImuLinearAccelerationObservation(it->linear_acceleration);
    imu_tracker->AddImuAngularVelocityObservation(it->angular_velocity);
    ++it;
  }
  imu_tracker->Advance(time);
}

// 解算姿态的变化量
Eigen::Quaterniond PoseExtrapolator::ExtrapolateRotation(
    const common::Time time, ImuTracker* const imu_tracker) const {
  // 检查指定时间是否大于等于 ImuTracker 的时间
  CHECK_GE(time, imu_tracker->time());
  // 更新 ImuTracker 到指定的 time
  AdvanceImuTracker(time, imu_tracker);
  // 上一时刻的姿态
  const Eigen::Quaterniond last_orientation = imu_tracker_->orientation();
  // 求取姿态变化量：上一时刻姿态的逆乘以当前的姿态
  return last_orientation.inverse() * imu_tracker->orientation();
}

// 解算位移的增长量
Eigen::Vector3d PoseExtrapolator::ExtrapolateTranslation(common::Time time) {
  // 取出 Pose 队列中最新的时刻
  const TimedPose& newest_timed_pose = timed_pose_queue_.back();
  // 算时间差
  const double extrapolation_delta =
      common::ToSeconds(time - newest_timed_pose.time);
  // 没有里程计数据的话则把从 Pose 队列中估计的线速度乘以时间
  if (odometry_data_.size() < 2) {
    return extrapolation_delta * linear_velocity_from_poses_;
  }
  // 如果有里程计数据，则更信任里程计速度，直接把从里程计处获得的线速度乘以时间
  return extrapolation_delta * linear_velocity_from_odometry_;
}

}  // namespace mapping
}  // namespace cartographer
