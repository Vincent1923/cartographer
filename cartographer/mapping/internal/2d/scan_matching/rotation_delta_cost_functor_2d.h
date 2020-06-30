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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_ROTATION_DELTA_COST_FUNCTOR_2D_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_ROTATION_DELTA_COST_FUNCTOR_2D_H_

#include "Eigen/Core"
#include "ceres/ceres.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {

// Computes the cost of rotating 'pose' to 'target_angle'. Cost increases with
// the solution's distance from 'target_angle'.
// 计算将位姿 "pose" 旋转为 "target_angle" 的代价。代价随着解与 "target_angle" 距离的增加而增加。
class RotationDeltaCostFunctor2D {
 public:
  /**
   * @brief CreateAutoDiffCostFunction  提供使用 Ceres 原生的自动求导方法的角度偏差代价函数的计算。
   *                                    它是类 RotationDeltaCostFunctor2D 公开的静态成员。
   *                                    该函数最终构造和返回的是 Ceres 的 AutoDiffCostFunction 对象。
   *                                    从模板列表中可以看到类 RotationDeltaCostFunctor2D 只提供一个残差项，其参与优化的参数有3个。
   * @param scaling_factor              角度偏差的权重，可以通过配置文件指定
   * @param target_angle                参考角度，实际的调用传参是优化之前的角度估计
   * @return                            最终构造和返回的是 Ceres 的 AutoDiffCostFunction 对象。
   */
  static ceres::CostFunction* CreateAutoDiffCostFunction(
      const double scaling_factor, const double target_angle) {
    return new ceres::AutoDiffCostFunction<
        RotationDeltaCostFunctor2D, 1 /* residuals */, 3 /* pose variables */>(
        new RotationDeltaCostFunctor2D(scaling_factor, target_angle));
  }

  // 对运算符 "()" 的重载。
  // 它有一个残差项的代价需要计算，对应着角度偏差量。
  template <typename T>
  bool operator()(const T* const pose, T* residual) const {
    // 计算角度偏差量并乘上权重得到残差项的代价。
    // 这里的 scaling_factor_ 和 angle_ 是在类 RotationDeltaCostFunctor2D 中定义的两个私有成员变量，
    // 分别记录了权重和参考角度。它们通过构造函数和静态函数 CreateAutoDiffCostFunction() 的传参赋值。
    residual[0] = scaling_factor_ * (pose[2] - angle_);
    return true;
  }

 private:
  /**
   * @brief RotationDeltaCostFunctor2D  构造函数。显式的将唯一的构造函数定义为私有的。
   *                                    这就意味着我们不能直接的构造其对象，只能通过其静态函数 CreateAutoDiffCostFunction() 间接的创建。
   * @param scaling_factor              角度偏差的权重，可以通过配置文件指定
   * @param target_angle                参考角度，实际的调用传参是优化之前的角度估计。
   */
  explicit RotationDeltaCostFunctor2D(const double scaling_factor,
                                      const double target_angle)
      : scaling_factor_(scaling_factor), angle_(target_angle) {}

  // 屏蔽了拷贝构造和拷贝赋值
  RotationDeltaCostFunctor2D(const RotationDeltaCostFunctor2D&) = delete;
  RotationDeltaCostFunctor2D& operator=(const RotationDeltaCostFunctor2D&) =
      delete;

  const double scaling_factor_;  // 角度偏差的权重
  const double angle_;           // 参考角度
};

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_ROTATION_DELTA_COST_FUNCTOR_2D_H_
