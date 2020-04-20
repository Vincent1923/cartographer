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

#include "cartographer/mapping/imu_tracker.h"

#include <cmath>
#include <limits>

#include "cartographer/common/math.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

// 构造函数
ImuTracker::ImuTracker(const double imu_gravity_time_constant,
                       const common::Time time)
    : imu_gravity_time_constant_(imu_gravity_time_constant),
      time_(time),
      last_linear_acceleration_time_(common::Time::min()),
      orientation_(Eigen::Quaterniond::Identity()),      // 初始姿态为I
      gravity_vector_(Eigen::Vector3d::UnitZ()),         // 重力方向初始化为[0,0,1]
      imu_angular_velocity_(Eigen::Vector3d::Zero()) {}  // 角速度初始化为0

// 把 ImuTracker 更新到指定时刻 time，并把响应的 orientation_，gravity_vector_ 和 time_ 进行更新
// 指定的时间 time 要比 ImuTracker 维护的时间 time_ 晚，否则不需要更新。
void ImuTracker::Advance(const common::Time time) {
  // 检查 ImuTracker 维护的时间 time_ 是否小于等于指定时间 time
  CHECK_LE(time_, time);
  // 求指定时间相对当前时间的时间差
  const double delta_t = common::ToSeconds(time - time_);
  // 角速度乘以时间，然后转化成 RotationnQuaternion，这是这段时间的姿态变化量
  const Eigen::Quaterniond rotation =
      transform::AngleAxisVectorToRotationQuaternion(
          Eigen::Vector3d(imu_angular_velocity_ * delta_t));
  // 以当前姿态 orientation_ 为基准，再乘以姿态变化量，得到最新的姿态
  orientation_ = (orientation_ * rotation).normalized();
  // 更新重力方向
  gravity_vector_ = rotation.conjugate() * gravity_vector_;
  // 更新时间
  time_ = time;
}

// 根据读数更新线加速度。这里的线加速度是经过重力校正的。
void ImuTracker::AddImuLinearAccelerationObservation(
    const Eigen::Vector3d& imu_linear_acceleration) {
  // Update the 'gravity_vector_' with an exponential moving average using the
  // 'imu_gravity_time_constant'.
  // 求时间差
  const double delta_t =
      last_linear_acceleration_time_ > common::Time::min()
          ? common::ToSeconds(time_ - last_linear_acceleration_time_)
          : std::numeric_limits<double>::infinity();
  // 更新一下 last_linear_acceleration_time_
  last_linear_acceleration_time_ = time_;
  const double alpha = 1. - std::exp(-delta_t / imu_gravity_time_constant_);
  // 注意这里时间差 delta_t 越大，alpha 值就越大，而 imu_linear_acceleration 的权重就越大。
  // 理解1：重力校正那里就是根据加速度计的输出来修正重力方向，alpha 是修正因数，受修正时间间隔的控制。、
  // 间隔大则更多的依赖于加速度，间隔小则更多依赖陀螺积分结果。
  // 理解2：互补滤波就是在短时间内采用陀螺仪得到的角度（gravity_vector_）做为最优值，定时对加速度采样
  // （imu_linear_acceleration）来对加速度值进行互补来校正陀螺仪的得到的角度。
  // 所以，下面计算 imu_gravity 时，t 越大，alpha 值就越大，imu_linear_acceleration 的权重就越大。
  gravity_vector_ =
      (1. - alpha) * gravity_vector_ + alpha * imu_linear_acceleration;
  // Change the 'orientation_' so that it agrees with the current
  // 'gravity_vector_'.
  // 更改“orientation_”，使其与当前的“gravity_vector_”一致
  const Eigen::Quaterniond rotation = Eigen::Quaterniond::FromTwoVectors(
      gravity_vector_, orientation_.conjugate() * Eigen::Vector3d::UnitZ());
  orientation_ = (orientation_ * rotation).normalized();
  CHECK_GT((orientation_ * gravity_vector_).z(), 0.);
  CHECK_GT((orientation_ * gravity_vector_).normalized().z(), 0.99);
}

// 根据读数更新角速度
void ImuTracker::AddImuAngularVelocityObservation(
    const Eigen::Vector3d& imu_angular_velocity) {
  imu_angular_velocity_ = imu_angular_velocity;
}

}  // namespace mapping
}  // namespace cartographer
