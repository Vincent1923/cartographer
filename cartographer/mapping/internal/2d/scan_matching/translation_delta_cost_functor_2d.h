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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_TRANSLATION_DELTA_COST_FUNCTOR_2D_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_TRANSLATION_DELTA_COST_FUNCTOR_2D_H_

#include "Eigen/Core"
#include "ceres/ceres.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {

// Computes the cost of translating 'pose' to 'target_translation'.
// Cost increases with the solution's distance from 'target_translation'.
// 计算将位姿 "pose" 转换为 "target_translation" 的代价。代价随着解与 "target_translation" 距离的增加而增加。
class TranslationDeltaCostFunctor2D {
 public:
  /**
   * @brief CreateAutoDiffCostFunction  提供使用 Ceres 原生的自动求导方法的位移偏差代价函数的计算。
   *                                    它是类 TranslationDeltaCostFunctor2D 公开的静态成员。
   *                                    该函数最终构造和返回的是 Ceres 的 AutoDiffCostFunction 对象。
   *                                    从模板列表中可以看到类 TranslationDeltaCostFunctor2D 提供了2个残差项，其参与优化的参数有3个。
   * @param scaling_factor              位移偏差的权重，可以通过配置文件指定
   * @param target_translation          参考位置，实际的调用传参是优化之前的坐标估计
   * @return                            最终构造和返回的是 Ceres 的 AutoDiffCostFunction 对象。
   */
  static ceres::CostFunction* CreateAutoDiffCostFunction(
      const double scaling_factor, const Eigen::Vector2d& target_translation) {
    return new ceres::AutoDiffCostFunction<TranslationDeltaCostFunctor2D,
                                           2 /* residuals */,
                                           3 /* pose variables */>(
        new TranslationDeltaCostFunctor2D(scaling_factor, target_translation));
  }

  // 对运算符 "()" 的重载。
  // 它有两个残差项的代价需要计算，分别对应着 x 轴和 y 轴上的偏差量。
  template <typename T>
  bool operator()(const T* const pose, T* residual) const {
    // 计算位置偏差量并乘上权重来更新对应残差项的代价。
    // 这里的 scaling_factor_, x_ 和 y_ 是在类 RotationDeltaCostFunctor2D 中定义的3个私有成员变量，
    // 分别记录了权重和参考坐标。它们通过构造函数和静态函数 CreateAutoDiffCostFunction() 的传参赋值。
    residual[0] = scaling_factor_ * (pose[0] - x_);
    residual[1] = scaling_factor_ * (pose[1] - y_);
    return true;
  }

 private:
  // Constructs a new TranslationDeltaCostFunctor2D from the given
  // 'target_translation' (x, y).
  // 从给定的参考坐标 "target_translation"（x，y）构造一个新的 TranslationDeltaCostFunctor2D 对象。
  /**
   * @brief TranslationDeltaCostFunctor2D  构造函数。显式的将唯一的构造函数定义为私有的。
   *                                       这就意味着我们不能直接的构造其对象，只能通过其静态函数 CreateAutoDiffCostFunction() 间接的创建。
   * @param scaling_factor                 位移偏差的权重，可以通过配置文件指定
   * @param target_translation             参考位置，实际的调用传参是优化之前的坐标估计
   */
  explicit TranslationDeltaCostFunctor2D(
      const double scaling_factor, const Eigen::Vector2d& target_translation)
      : scaling_factor_(scaling_factor),
        x_(target_translation.x()),
        y_(target_translation.y()) {}

  // 屏蔽了拷贝构造和拷贝赋值
  TranslationDeltaCostFunctor2D(const TranslationDeltaCostFunctor2D&) = delete;
  TranslationDeltaCostFunctor2D& operator=(
      const TranslationDeltaCostFunctor2D&) = delete;

  const double scaling_factor_;  // 位移偏差的权重
  const double x_;               // 位移偏差量在 x 轴的参考坐标
  const double y_;               // 位移偏差量在 y 轴的参考坐标
};

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_TRANSLATION_DELTA_COST_FUNCTOR_2D_H_
