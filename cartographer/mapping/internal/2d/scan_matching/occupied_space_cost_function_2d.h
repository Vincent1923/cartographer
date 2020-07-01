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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_OCCUPIED_SPACE_COST_FUNCTION_2D_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_OCCUPIED_SPACE_COST_FUNCTION_2D_H_

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/mapping/2d/grid_2d.h"
#include "cartographer/mapping/probability_values.h"
#include "cartographer/sensor/point_cloud.h"
#include "ceres/ceres.h"
#include "ceres/cubic_interpolation.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {

// Computes a cost for matching the 'point_cloud' to the 'grid' with
// a 'pose'. The cost increases with poorer correspondence of the grid and the
// point observation (e.g. points falling into less occupied space).
// 在带有 "pose" 的情况下，计算将 "point_cloud" 与 "grid" 相匹配的代价。
// 栅格地图和观测点的匹配程度越低（例如，点落入较少的占用空间），那么代价越大。
class OccupiedSpaceCostFunction2D {
 public:
  /**
   * @brief CreateAutoDiffCostFunction  使用 Ceres 原生的自动求导方法，计算激光点云数据和栅格地图的匹配程度偏差的代价函数。
   *                                    它是类 OccupiedSpaceCostFunction2D 公开的静态成员。
   *                                    该函数最终构造和返回的是 Ceres 的 AutoDiffCostFunction 对象。
   *                                    从模板列表中可以看到类 TranslationDeltaCostFunctor2D 提供的残差项数量为动态的，其参与优化的参数有3个。
   * @param scaling_factor              激光点云数据和栅格地图的匹配程度偏差的权重，可以通过配置文件指定
   * @param point_cloud                 待匹配的激光点云数据
   * @param grid                        栅格地图
   * @return                            最终构造和返回的是 Ceres 的 AutoDiffCostFunction 对象。
   */
  static ceres::CostFunction* CreateAutoDiffCostFunction(
      const double scaling_factor, const sensor::PointCloud& point_cloud,
      const Grid2D& grid) {
    return new ceres::AutoDiffCostFunction<OccupiedSpaceCostFunction2D,
                                           ceres::DYNAMIC /* residuals */,
                                           3 /* pose variables */>(
        new OccupiedSpaceCostFunction2D(scaling_factor, point_cloud, grid),
        point_cloud.size());
  }

  // 对运算符 "()" 的重载。
  template <typename T>
  bool operator()(const T* const pose, T* residual) const {
    // 在函数的一开始，先把迭代查询的输入参数 pose 转换为坐标变换 Tε，用临时变量 transform 记录。
    // pose[0] 和 pose[1] 表示机器人的位置，pose[2] 表示朝向角度。
    Eigen::Matrix<T, 2, 1> translation(pose[0], pose[1]);
    Eigen::Rotation2D<T> rotation(pose[2]);
    Eigen::Matrix<T, 2, 2> rotation_matrix = rotation.toRotationMatrix();
    Eigen::Matrix<T, 3, 3> transform;
    transform << rotation_matrix, translation, T(0.), T(0.), T(1.);

    // 然后使用 Ceres 库原生提供的双三次插值迭代器。
    const GridArrayAdapter adapter(grid_);
    ceres::BiCubicInterpolator<GridArrayAdapter> interpolator(adapter);
    const MapLimits& limits = grid_.limits();

    // 接着根据公式针对每个 hit 点计算对应的残差代价。
    for (size_t i = 0; i < point_cloud_.size(); ++i) {
      // Note that this is a 2D point. The third component is a scaling factor.
      const Eigen::Matrix<T, 3, 1> point((T(point_cloud_[i].x())),
                                         (T(point_cloud_[i].y())), T(1.));
      // 通过 hit 点坐标与 transform 的相乘得到其在地图坐标系下的坐标 Tε*hk
      const Eigen::Matrix<T, 3, 1> world = transform * point;
      // 通过刚刚构建的迭代器和地图坐标，获取在 hit 点出现的概率。
      // 该函数调用有三个参数，前两个参数用来描述 x,y 轴索引，第三个参数用于记录插值后的结果。
      // 这里的 xy 索引计算的比较奇怪，它通过 GridArrayAdapter 类中成员函数 GetValue() 获取栅格数据，这里不再细述。
      // 此外由于占用栅格中原本存储的就是栅格空闲的概率，所以这里查询出来的概率就是 (1−Msmooth(Tεhk))。
      interpolator.Evaluate(
          (limits.max().x() - world[0]) / limits.resolution() - 0.5 +
              static_cast<double>(kPadding),
          (limits.max().y() - world[1]) / limits.resolution() - 0.5 +
              static_cast<double>(kPadding),
          &residual[i]);
      residual[i] = scaling_factor_ * residual[i];
    }
    // 最后返回退出
    return true;
  }

 private:
  static constexpr int kPadding = INT_MAX / 4;
  class GridArrayAdapter {
   public:
    enum { DATA_DIMENSION = 1 };

    explicit GridArrayAdapter(const Grid2D& grid) : grid_(grid) {}

    void GetValue(const int row, const int column, double* const value) const {
      if (row < kPadding || column < kPadding || row >= NumRows() - kPadding ||
          column >= NumCols() - kPadding) {
        *value = kMaxCorrespondenceCost;
      } else {
        *value = static_cast<double>(grid_.GetCorrespondenceCost(
            Eigen::Array2i(column - kPadding, row - kPadding)));
      }
    }

    int NumRows() const {
      return grid_.limits().cell_limits().num_y_cells + 2 * kPadding;
    }

    int NumCols() const {
      return grid_.limits().cell_limits().num_x_cells + 2 * kPadding;
    }

   private:
    const Grid2D& grid_;
  };

  /**
   * @brief OccupiedSpaceCostFunction2D  构造函数。
   *                                     将唯一的构造函数定义为私有的。这就意味着我们不能直接的构造其对象，
   *                                     只能通过其静态函数 CreateAutoDiffCostFunction() 间接的创建。
   * @param scaling_factor               激光点云数据和栅格地图的匹配程度偏差的权重，可以通过配置文件指定
   * @param point_cloud                  待匹配的激光点云数据
   * @param grid                         栅格地图
   */
  OccupiedSpaceCostFunction2D(const double scaling_factor,
                              const sensor::PointCloud& point_cloud,
                              const Grid2D& grid)
      : scaling_factor_(scaling_factor),
        point_cloud_(point_cloud),
        grid_(grid) {}

  // 屏蔽了拷贝构造和拷贝赋值
  OccupiedSpaceCostFunction2D(const OccupiedSpaceCostFunction2D&) = delete;
  OccupiedSpaceCostFunction2D& operator=(const OccupiedSpaceCostFunction2D&) =
      delete;

  const double scaling_factor_;            // 激光点云数据和栅格地图的匹配程度偏差的权重
  const sensor::PointCloud& point_cloud_;  // 待匹配的激光点云数据
  const Grid2D& grid_;                     // 栅格地图
};

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_OCCUPIED_SPACE_COST_FUNCTION_2D_H_
