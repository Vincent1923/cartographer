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

#include "cartographer/mapping/internal/2d/scan_matching/ceres_scan_matcher_2d.h"

#include <utility>
#include <vector>

#include "Eigen/Core"
#include "cartographer/common/ceres_solver_options.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/mapping/2d/grid_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/occupied_space_cost_function_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/rotation_delta_cost_functor_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/translation_delta_cost_functor_2d.h"
#include "cartographer/transform/transform.h"
#include "ceres/ceres.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {

proto::CeresScanMatcherOptions2D CreateCeresScanMatcherOptions2D(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::CeresScanMatcherOptions2D options;
  options.set_occupied_space_weight(
      parameter_dictionary->GetDouble("occupied_space_weight"));
  options.set_translation_weight(
      parameter_dictionary->GetDouble("translation_weight"));
  options.set_rotation_weight(
      parameter_dictionary->GetDouble("rotation_weight"));
  *options.mutable_ceres_solver_options() =
      common::CreateCeresSolverOptionsProto(
          parameter_dictionary->GetDictionary("ceres_solver_options").get());
  return options;
}

// 构造函数。
// 记录扫描匹配器的相关配置项，通过函数 CreateCeresSolverOptions() 和配置项 ceres_solver_options 装载 Ceres 优化库的配置。
CeresScanMatcher2D::CeresScanMatcher2D(
    const proto::CeresScanMatcherOptions2D& options)
    : options_(options),
      ceres_solver_options_(
          common::CreateCeresSolverOptions(options.ceres_solver_options())) {
  ceres_solver_options_.linear_solver_type = ceres::DENSE_QR;
}

// 析构函数
CeresScanMatcher2D::~CeresScanMatcher2D() {}

// 使用优化库 ceres 进行扫描匹配。
// 在给定机器人的初始位姿估计 initial_pose_estimate 的情况下，尽可能的将扫描的点云数据 point_cloud
// 放置到栅格地图 grid 上，同时返回优化后的位姿估计 pose_estimate 和优化迭代信息summary。
// 而参数 target_translation 主要用于约束位姿估计的 xy 坐标， 在 Local SLAM 的业务主线 LocalTrajectoryBuilder2D
// 调用该函数的时候传递的是位姿估计器的估计值，Cartographer 认为优化后的机器人位置应当与该估计值的偏差不大。
void CeresScanMatcher2D::Match(const Eigen::Vector2d& target_translation,
                               const transform::Rigid2d& initial_pose_estimate,
                               const sensor::PointCloud& point_cloud,
                               const Grid2D& grid,
                               transform::Rigid2d* const pose_estimate,
                               ceres::Solver::Summary* const summary) const {
  // 在函数的一开始用一个 double 数组记录下初始位姿，作为 Ceres 优化迭代的初值。
  double ceres_pose_estimate[3] = {initial_pose_estimate.translation().x(),
                                   initial_pose_estimate.translation().y(),
                                   initial_pose_estimate.rotation().angle()};
  // 接下来构建了 ceres::Problem 对象，并通过接口 AddResidualBlock() 添加残差项。
  // 从代码看来，残差主要有三个方面的来源：
  // (1) 占用栅格与扫描数据的匹配度，
  // (2) 优化后的位置相对于 target_translation 的距离，
  // (3) 旋转角度相对于迭代初值的偏差。
  ceres::Problem problem;
  CHECK_GT(options_.occupied_space_weight(), 0.);  // 检查 occupied_space_weight 是否大于0
  problem.AddResidualBlock(
      OccupiedSpaceCostFunction2D::CreateAutoDiffCostFunction(
          options_.occupied_space_weight() /
              std::sqrt(static_cast<double>(point_cloud.size())),
          point_cloud, grid),
      nullptr /* loss function */, ceres_pose_estimate);
  CHECK_GT(options_.translation_weight(), 0.);  // 检查 translation_weight 是否大于0
  problem.AddResidualBlock(
      TranslationDeltaCostFunctor2D::CreateAutoDiffCostFunction(
          options_.translation_weight(), target_translation),
      nullptr /* loss function */, ceres_pose_estimate);
  CHECK_GT(options_.rotation_weight(), 0.);  // 检查 rotation_weight 是否大于0
  problem.AddResidualBlock(
      RotationDeltaCostFunctor2D::CreateAutoDiffCostFunction(
          options_.rotation_weight(), ceres_pose_estimate[2]),
      nullptr /* loss function */, ceres_pose_estimate);

  ceres::Solve(ceres_solver_options_, &problem, summary);

  // 最后求解并更新位姿估计。
  *pose_estimate = transform::Rigid2d(
      {ceres_pose_estimate[0], ceres_pose_estimate[1]}, ceres_pose_estimate[2]);
}

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer
