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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_CERES_SCAN_MATCHER_2D_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_CERES_SCAN_MATCHER_2D_H_

#include <memory>
#include <vector>

#include "Eigen/Core"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/mapping/2d/grid_2d.h"
#include "cartographer/mapping/proto/scan_matching/ceres_scan_matcher_options_2d.pb.h"
#include "cartographer/sensor/point_cloud.h"
#include "ceres/ceres.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {

proto::CeresScanMatcherOptions2D CreateCeresScanMatcherOptions2D(
    common::LuaParameterDictionary* parameter_dictionary);

// Align scans with an existing map using Ceres.
// 使用 Ceres 将扫描数据与现有地图对齐。
class CeresScanMatcher2D {
 public:
  /**
   * @brief CeresScanMatcher2D  构造函数。记录扫描匹配器的相关配置项，通过函数 CreateCeresSolverOptions() 和配置项 ceres_solver_options
   *                            装载 Ceres 优化库的配置。
   * @param options             扫描匹配器的相关配置
   */
  explicit CeresScanMatcher2D(const proto::CeresScanMatcherOptions2D& options);
  // 析构函数
  virtual ~CeresScanMatcher2D();

  // 屏蔽了拷贝构造和拷贝赋值
  CeresScanMatcher2D(const CeresScanMatcher2D&) = delete;
  CeresScanMatcher2D& operator=(const CeresScanMatcher2D&) = delete;

  // Aligns 'point_cloud' within the 'grid' given an
  // 'initial_pose_estimate' and returns a 'pose_estimate' and the solver
  // 'summary'.
  // 在给定 "initial_pose_estimate" 的情况下，在栅格 "grid" 内对齐点云 "point_cloud"，
  // 并返回位姿估计 "pose_estimate" 和求解器 "summary"。
  /**
   * @brief Match                  使用优化库 ceres 进行扫描的匹配。
   * @param target_translation     位姿估计器预测的位姿，机器人在局部地图坐标系下的位置，只包含空间坐标。
   * @param initial_pose_estimate  机器人的初始位姿估计，机器人在局部地图坐标系下的位姿，也是根据位姿估计器获取。
   *                               translation 为机器人在局部地图坐标系下的位置，而 rotation 用四元数表示后近似为 (0,0,0,1)。
   *                               所以它只包含机器人在局部地图坐标系下的位置信息，但不包含方向信息。
   * @param point_cloud            扫描的点云数据，从局部地图坐标系平移到机器人坐标系下的扫描数据，但没有经过旋转。
   * @param grid                   用于扫描匹配的旧图的栅格地图
   * @param pose_estimate          优化后的位姿估计，机器人在局部地图坐标系下的位姿估计。
   *                               translation 为机器人在局部地图坐标系下的位置估计，而 rotation 用四元数表示后近似为 (0,0,0,1)。
   *                               所以它只包含机器人在局部地图坐标系下的位置信息，但不包含方向信息，不过这里的方向也经过优化。
   * @param summary                优化迭代信息
   */
  void Match(const Eigen::Vector2d& target_translation,
             const transform::Rigid2d& initial_pose_estimate,
             const sensor::PointCloud& point_cloud, const Grid2D& grid,
             transform::Rigid2d* pose_estimate,
             ceres::Solver::Summary* summary) const;

 private:
  const proto::CeresScanMatcherOptions2D options_;  // 记录扫描匹配器的相关配置
  ceres::Solver::Options ceres_solver_options_;     // 优化过程的配置
};

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_CERES_SCAN_MATCHER_2D_H_
