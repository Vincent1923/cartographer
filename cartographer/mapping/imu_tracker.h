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

#ifndef CARTOGRAPHER_MAPPING_IMU_TRACKER_H_
#define CARTOGRAPHER_MAPPING_IMU_TRACKER_H_

#include "Eigen/Geometry"
#include "cartographer/common/time.h"

namespace cartographer {
namespace mapping {

// Keeps track of the orientation using angular velocities and linear
// accelerations from an IMU. Because averaged linear acceleration (assuming
// slow movement) is a direct measurement of gravity, roll/pitch does not drift,
// though yaw does.
// 使用IMU的角速度和线性加速度跟踪方向。
// ImuTracker 的主要作用就是根据 IMU 的读数维护传感器当前的姿态、线加速度(经过重力校正的)、当前姿态、
// 重力方向、角速度等量。这些量都是以 ImuTracker 刚建立时的那一时刻 IMU 本身的坐标系为基准坐标系。
class ImuTracker {
 public:
 // 构造函数
  ImuTracker(double imu_gravity_time_constant, common::Time time);

  // Advances to the given 'time' and updates the orientation to reflect this.
  // 把 ImuTracker 更新到指定时刻 time，并把响应的 orientation_，gravity_vector_ 和 time_ 进行更新
  // 指定的时间 time 要比 ImuTracker 维护的时间 time_ 晚，否则不需要更新。
  void Advance(common::Time time);

  // Updates from an IMU reading (in the IMU frame).
  // 根据传感器读数更新传感器的最新状态，得到经过重力校正的线加速度、角速度等。
  void AddImuLinearAccelerationObservation(
      const Eigen::Vector3d& imu_linear_acceleration);
  void AddImuAngularVelocityObservation(
      const Eigen::Vector3d& imu_angular_velocity);

  // Query the current time.
  // 返回当前时间
  common::Time time() const { return time_; }

  // Query the current orientation estimate.
  // 返回当前姿态
  Eigen::Quaterniond orientation() const { return orientation_; }

 private:
  const double imu_gravity_time_constant_;      // align 重力的时间间隔
  common::Time time_;                           // 当前时间
  common::Time last_linear_acceleration_time_;  // 记录的上一个线加速度的时刻
  Eigen::Quaterniond orientation_;              // 当前姿态
  Eigen::Vector3d gravity_vector_;              // 当前重力方向
  Eigen::Vector3d imu_angular_velocity_;        // 角速度
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_IMU_TRACKER_H_
