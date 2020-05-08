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

#include "cartographer/mapping/internal/2d/scan_matching/fast_correlative_scan_matcher_2d.h"

#include <algorithm>
#include <cmath>
#include <deque>
#include <functional>
#include <limits>

#include "Eigen/Geometry"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/math.h"
#include "cartographer/mapping/2d/grid_2d.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {
namespace {

// A collection of values which can be added and later removed, and the maximum
// of the current values in the collection can be retrieved.
// All of it in (amortized) O(1).
class SlidingWindowMaximum {
 public:
  void AddValue(const float value) {
    while (!non_ascending_maxima_.empty() &&
           value > non_ascending_maxima_.back()) {
      non_ascending_maxima_.pop_back();
    }
    non_ascending_maxima_.push_back(value);
  }

  void RemoveValue(const float value) {
    // DCHECK for performance, since this is done for every value in the
    // precomputation grid.
    DCHECK(!non_ascending_maxima_.empty());
    DCHECK_LE(value, non_ascending_maxima_.front());
    if (value == non_ascending_maxima_.front()) {
      non_ascending_maxima_.pop_front();
    }
  }

  float GetMaximum() const {
    // DCHECK for performance, since this is done for every value in the
    // precomputation grid.
    DCHECK_GT(non_ascending_maxima_.size(), 0);
    return non_ascending_maxima_.front();
  }

  void CheckIsEmpty() const { CHECK_EQ(non_ascending_maxima_.size(), 0); }

 private:
  // Maximum of the current sliding window at the front. Then the maximum of the
  // remaining window that came after this values first occurrence, and so on.
  std::deque<float> non_ascending_maxima_;
};

}  // namespace

proto::FastCorrelativeScanMatcherOptions2D
CreateFastCorrelativeScanMatcherOptions2D(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::FastCorrelativeScanMatcherOptions2D options;
  options.set_linear_search_window(
      parameter_dictionary->GetDouble("linear_search_window"));
  options.set_angular_search_window(
      parameter_dictionary->GetDouble("angular_search_window"));
  options.set_branch_and_bound_depth(
      parameter_dictionary->GetInt("branch_and_bound_depth"));
  return options;
}

PrecomputationGrid2D::PrecomputationGrid2D(
    const Grid2D& grid, const CellLimits& limits, const int width,
    std::vector<float>* reusable_intermediate_grid)
    : offset_(-width + 1, -width + 1),
      wide_limits_(limits.num_x_cells + width - 1,
                   limits.num_y_cells + width - 1),
      min_score_(1.f - grid.GetMaxCorrespondenceCost()),
      max_score_(1.f - grid.GetMinCorrespondenceCost()),
      cells_(wide_limits_.num_x_cells * wide_limits_.num_y_cells) {
  CHECK_GE(width, 1);
  CHECK_GE(limits.num_x_cells, 1);
  CHECK_GE(limits.num_y_cells, 1);
  const int stride = wide_limits_.num_x_cells;
  // First we compute the maximum probability for each (x0, y) achieved in the
  // span defined by x0 <= x < x0 + width.
  std::vector<float>& intermediate = *reusable_intermediate_grid;
  intermediate.resize(wide_limits_.num_x_cells * limits.num_y_cells);
  for (int y = 0; y != limits.num_y_cells; ++y) {
    SlidingWindowMaximum current_values;
    current_values.AddValue(
        1.f - std::abs(grid.GetCorrespondenceCost(Eigen::Array2i(0, y))));
    for (int x = -width + 1; x != 0; ++x) {
      intermediate[x + width - 1 + y * stride] = current_values.GetMaximum();
      if (x + width < limits.num_x_cells) {
        current_values.AddValue(1.f - std::abs(grid.GetCorrespondenceCost(
                                          Eigen::Array2i(x + width, y))));
      }
    }
    for (int x = 0; x < limits.num_x_cells - width; ++x) {
      intermediate[x + width - 1 + y * stride] = current_values.GetMaximum();
      current_values.RemoveValue(
          1.f - std::abs(grid.GetCorrespondenceCost(Eigen::Array2i(x, y))));
      current_values.AddValue(1.f - std::abs(grid.GetCorrespondenceCost(
                                        Eigen::Array2i(x + width, y))));
    }
    for (int x = std::max(limits.num_x_cells - width, 0);
         x != limits.num_x_cells; ++x) {
      intermediate[x + width - 1 + y * stride] = current_values.GetMaximum();
      current_values.RemoveValue(
          1.f - std::abs(grid.GetCorrespondenceCost(Eigen::Array2i(x, y))));
    }
    current_values.CheckIsEmpty();
  }
  // For each (x, y), we compute the maximum probability in the width x width
  // region starting at each (x, y) and precompute the resulting bound on the
  // score.
  for (int x = 0; x != wide_limits_.num_x_cells; ++x) {
    SlidingWindowMaximum current_values;
    current_values.AddValue(intermediate[x]);
    for (int y = -width + 1; y != 0; ++y) {
      cells_[x + (y + width - 1) * stride] =
          ComputeCellValue(current_values.GetMaximum());
      if (y + width < limits.num_y_cells) {
        current_values.AddValue(intermediate[x + (y + width) * stride]);
      }
    }
    for (int y = 0; y < limits.num_y_cells - width; ++y) {
      cells_[x + (y + width - 1) * stride] =
          ComputeCellValue(current_values.GetMaximum());
      current_values.RemoveValue(intermediate[x + y * stride]);
      current_values.AddValue(intermediate[x + (y + width) * stride]);
    }
    for (int y = std::max(limits.num_y_cells - width, 0);
         y != limits.num_y_cells; ++y) {
      cells_[x + (y + width - 1) * stride] =
          ComputeCellValue(current_values.GetMaximum());
      current_values.RemoveValue(intermediate[x + y * stride]);
    }
    current_values.CheckIsEmpty();
  }
}

uint8 PrecomputationGrid2D::ComputeCellValue(const float probability) const {
  const int cell_value = common::RoundToInt(
      (probability - min_score_) * (255.f / (max_score_ - min_score_)));
  CHECK_GE(cell_value, 0);
  CHECK_LE(cell_value, 255);
  return cell_value;
}

PrecomputationGridStack2D::PrecomputationGridStack2D(
    const Grid2D& grid,
    const proto::FastCorrelativeScanMatcherOptions2D& options) {
  CHECK_GE(options.branch_and_bound_depth(), 1);
  const int max_width = 1 << (options.branch_and_bound_depth() - 1);
  precomputation_grids_.reserve(options.branch_and_bound_depth());
  std::vector<float> reusable_intermediate_grid;
  const CellLimits limits = grid.limits().cell_limits();
  reusable_intermediate_grid.reserve((limits.num_x_cells + max_width - 1) *
                                     limits.num_y_cells);
  for (int i = 0; i != options.branch_and_bound_depth(); ++i) {
    const int width = 1 << i;
    precomputation_grids_.emplace_back(grid, limits, width,
                                       &reusable_intermediate_grid);
  }
}

// 构造函数，它有两个输入参数。
// grid 是子图的占用栅格，options 则记录了各种配置项。
// 在它的成员构造列表中，完成了三个成员的构造。
FastCorrelativeScanMatcher2D::FastCorrelativeScanMatcher2D(
    const Grid2D& grid,
    const proto::FastCorrelativeScanMatcherOptions2D& options)
    : options_(options),
      limits_(grid.limits()),
      precomputation_grid_stack_(
          common::make_unique<PrecomputationGridStack2D>(grid, options)) {}

FastCorrelativeScanMatcher2D::~FastCorrelativeScanMatcher2D() {}

// 定点的局部匹配，它有5个输入参数。
// initial_pose_estimate 描述了初始的位姿估计；point_cloud 则是将要考察的路径节点下的激光点云数据；
// min_score 是一个搜索节点的最小得分，也就是前文中提到的 score_threshold；
// 指针 score 和 pose_estimate 是两个输出参数，用于成功匹配后返回匹配度和位姿估计。
bool FastCorrelativeScanMatcher2D::Match(
    const transform::Rigid2d& initial_pose_estimate,
    const sensor::PointCloud& point_cloud, const float min_score, float* score,
    transform::Rigid2d* pose_estimate) const {
  // 构建了一个 SearchParameters 类型的对象，这是一个定义在 "correlative_scan_matcher_2d.h" 中的一个结构体，它描述了搜索窗口以及分辨率。
  const SearchParameters search_parameters(options_.linear_search_window(),
                                           options_.angular_search_window(),
                                           point_cloud, limits_.resolution());
  // 通过调用函数 MatchWithSearchParameters 来实际完成扫描匹配，进而实现闭环检测。
  return MatchWithSearchParameters(search_parameters, initial_pose_estimate,
                                   point_cloud, min_score, score,
                                   pose_estimate);
}

// 全地图匹配。
// 少了描述初始位姿的输入参数，其他参数的意义都与定点局部匹配是一样的。
// 殊途同归，全地图匹配也是调用函数 MatchWithSearchParameters 来实际完成扫描匹配的。
// 所不同的是，它需要提供一个以子图中心为搜索起点，覆盖整个子图的搜索窗口。所以我们会看到构建的对象 search_parameters 和 center。
bool FastCorrelativeScanMatcher2D::MatchFullSubmap(
    const sensor::PointCloud& point_cloud, float min_score, float* score,
    transform::Rigid2d* pose_estimate) const {
  // Compute a search window around the center of the submap that includes it
  // fully.
  const SearchParameters search_parameters(
      1e6 * limits_.resolution(),  // Linear search window, 1e6 cells/direction.
      M_PI,  // Angular search window, 180 degrees in both directions.
      point_cloud, limits_.resolution());
  const transform::Rigid2d center = transform::Rigid2d::Translation(
      limits_.max() - 0.5 * limits_.resolution() *
                          Eigen::Vector2d(limits_.cell_limits().num_y_cells,
                                          limits_.cell_limits().num_x_cells));
  return MatchWithSearchParameters(search_parameters, center, point_cloud,
                                   min_score, score, pose_estimate);
}

// 深度优先的分支定界搜索算法。它有6个输入参数。
// search_parameters 描述了窗口大小、分辨率等搜索信息。
// initial_pose_estimate 描述了初始的位姿估计；point_cloud 则是将要考察的路径节点下的激光点云数据；
// min_score 是一个搜索节点的最小得分，也就是前文中提到的 score_threshold；
// 指针 score 和 pose_estimate 是两个输出参数，用于成功匹配后返回匹配度和位姿估计。
bool FastCorrelativeScanMatcher2D::MatchWithSearchParameters(
    SearchParameters search_parameters,
    const transform::Rigid2d& initial_pose_estimate,
    const sensor::PointCloud& point_cloud, float min_score, float* score,
    transform::Rigid2d* pose_estimate) const {
  // 在函数的一开始先检查一下指针 score 和 pose_estimate 非空，因为在后续的计算过程中将要修改这两个指针所指对象的数据内容。
  CHECK_NOTNULL(score);
  CHECK_NOTNULL(pose_estimate);

  // 获取初始位姿估计的方向角，将激光点云中的点都绕Z轴转动相应的角度得到 rotated_point_cloud
  const Eigen::Rotation2Dd initial_rotation = initial_pose_estimate.rotation();
  const sensor::PointCloud rotated_point_cloud = sensor::TransformPointCloud(
      point_cloud,
      transform::Rigid3f::Rotation(Eigen::AngleAxisf(
          initial_rotation.cast<float>().angle(), Eigen::Vector3f::UnitZ())));
  // 调用定义在 "correlative_scan_matcher_2d.cc" 中的函数 GenerateRotatedScans 获得搜索窗口下机器人朝向各个方向角时的点云数据。
  // 也就是说容器 rotated_scans 中保存了姿态为 ε0+(0,0,δθjθ),jθ∈Z,−wθ≤jθ≤wθ 时所有的点云数据，
  // 其中 ε0 就是这里的初始位姿估计 initial_pose_estimate，δθ 则是角度搜索步长，它由对象 search_parameters 描述。
  const std::vector<sensor::PointCloud> rotated_scans =
      GenerateRotatedScans(rotated_point_cloud, search_parameters);
  // 接着通过同样定义在 "correlative_scan_matcher_2d.cc" 中的函数 DiscretizeScans，
  // 完成对旋转后的点云数据离散化的操作，即将浮点类型的点云数据转换成整型的栅格单元索引。
  // 类型 DiscreteScan2D 就是通过 typedef std::vector<Eigen::Array2i> DiscreteScan2D; 定义的整型容器。
  const std::vector<DiscreteScan2D> discrete_scans = DiscretizeScans(
      limits_, rotated_scans,
      Eigen::Translation2f(initial_pose_estimate.translation().x(),
                           initial_pose_estimate.translation().y()));
  // 通过对象 search_parameters 的接口 ShrinkToFit 尽可能的缩小搜索窗口的大小，以减小搜索空间，提高搜索效率。
  search_parameters.ShrinkToFit(discrete_scans, limits_.cell_limits());

  // 准备工作做完之后，下面就要进入实际的搜索过程了。
  // 首先通过函数 ComputeLowestResolutionCandidates 完成对搜索空间的第一次分割，得到初始子空间节点集合{C0}。
  // 该函数在最低分辨率的栅格地图上查表得到各个搜索节点 c∈{C0} 的上界，并降序排列。
  // 输入：discrete_scans 为离散化之后的各搜索方向上的点云数据，search_parameters 为搜索配置。
  const std::vector<Candidate2D> lowest_resolution_candidates =
      ComputeLowestResolutionCandidates(discrete_scans, search_parameters);
  // 最后调用函数 BranchAndBound 完成分支定界搜索，搜索的结果将被保存在 best_candidate中。
  const Candidate2D best_candidate = BranchAndBound(
      discrete_scans, search_parameters, lowest_resolution_candidates,
      precomputation_grid_stack_->max_depth(), min_score);
  // 检查最优解的值，如果大于指定阈值 min_score 就认为匹配成功，修改输入参数指针 score 和 pose_estimate 所指的对象。
  // 否则认为不匹配，不存在闭环，直接返回。
  if (best_candidate.score > min_score) {
    *score = best_candidate.score;
    *pose_estimate = transform::Rigid2d(
        {initial_pose_estimate.translation().x() + best_candidate.x,
         initial_pose_estimate.translation().y() + best_candidate.y},
        initial_rotation * Eigen::Rotation2Dd(best_candidate.orientation));
    return true;
  }
  return false;
}

// 以离散化之后的各搜索方向上的点云数据 discrete_scans 和搜索配置 search_parameters 为输入
std::vector<Candidate2D>
FastCorrelativeScanMatcher2D::ComputeLowestResolutionCandidates(
    const std::vector<DiscreteScan2D>& discrete_scans,
    const SearchParameters& search_parameters) const {
  // 调用函数 GenerateLowestResolutionCandidates 完成对搜索空间的初始分割
  std::vector<Candidate2D> lowest_resolution_candidates =
      GenerateLowestResolutionCandidates(search_parameters);
  // 通过函数 ScoreCandidates 计算各个候选点的评分并排序
  ScoreCandidates(
      precomputation_grid_stack_->Get(precomputation_grid_stack_->max_depth()),
      discrete_scans, search_parameters, &lowest_resolution_candidates);
  return lowest_resolution_candidates;
}

// 根据搜索配置来完成初始分割
std::vector<Candidate2D>
FastCorrelativeScanMatcher2D::GenerateLowestResolutionCandidates(
    const SearchParameters& search_parameters) const {
  // 首先根据预算图的金字塔高度计算初始分割的粒度 2^(h0)
  // linear_step_size 是顶层步长，precomputation_grid_stack_->max_depth() 是在 "pose_graph.lua" 定义的分支定界的深度。
  // 这里左移 precomputation_grid_stack_->max_depth() 位，表示2的 precomputation_grid_stack_->max_depth() 次方。
  const int linear_step_size = 1 << precomputation_grid_stack_->max_depth();
  int num_candidates = 0;
  // 遍历所有搜索方向，累计各个方向下空间的分割数量，得到 num_candidates 描述了搜索空间初始分割的子空间数量。
  // 下面举例进行说明：
  // 假设 x 和 y 方向的最大搜索索引为 wx = wy= 140，那么x方向的搜索数量为 2 * wx = 280
  // 如果 depth = 6，那么顶层步长为：2^6 = 64
  // 那么顶层的线性搜索空间大小就是 280/64，大约为 5（向下取整加1？）
  // 角度搜索索引由 cell 大小（1 pixel），和 scan 的最大扫描距离决定，大约为 300
  // 所以顶层 candidate 数量是：5 × 5 × 300。
  //
  // search_parameters.num_scans 为角度的搜索索引的数量 2 * wθ + 1，wθ 为搜索角度的最大索引。
  // c0 = jθ, j0 ∈ Z, −wθ ≤ jθ ≤ wθ
  for (int scan_index = 0; scan_index != search_parameters.num_scans;
       ++scan_index) {
    // num_lowest_resolution_linear_x_candidates 为搜索空间所对应的 x 轴的搜索索引的数量 2 * wx + 1，
    // wx 是 x 方向上最大的搜索索引。
    const int num_lowest_resolution_linear_x_candidates =
        (search_parameters.linear_bounds[scan_index].max_x -
         search_parameters.linear_bounds[scan_index].min_x + linear_step_size) /
        linear_step_size;
    // num_lowest_resolution_linear_y_candidates 为搜索空间所对应的 y 轴的搜索索引的数量 2 * wy + 1，
    // wy 是 y 方向上最大的搜索索引。
    const int num_lowest_resolution_linear_y_candidates =
        (search_parameters.linear_bounds[scan_index].max_y -
         search_parameters.linear_bounds[scan_index].min_y + linear_step_size) /
        linear_step_size;
    // num_candidates 描述了搜索空间初始分割的子空间数量。
    num_candidates += num_lowest_resolution_linear_x_candidates *
                      num_lowest_resolution_linear_y_candidates;
  }
  // 接下来在一个三层的嵌套for循环中，构建各个候选点。
  std::vector<Candidate2D> candidates;
  candidates.reserve(num_candidates);
  for (int scan_index = 0; scan_index != search_parameters.num_scans;
       ++scan_index) {
    for (int x_index_offset = search_parameters.linear_bounds[scan_index].min_x;
         x_index_offset <= search_parameters.linear_bounds[scan_index].max_x;
         x_index_offset += linear_step_size) {
      for (int y_index_offset =
               search_parameters.linear_bounds[scan_index].min_y;
           y_index_offset <= search_parameters.linear_bounds[scan_index].max_y;
           y_index_offset += linear_step_size) {
        // scan_index 表示每个候选点（节点）的角度搜索索引 c0 = jθ, j0 ∈ Z, −wθ ≤ jθ ≤ wθ，在这里实际上应该是 0 ≤ jθ ≤ 2 * wθ + 1
        // x_index_offset 表示每个候选点（节点）在搜索空间所对应的 x 轴索引 cx = -wx + 2^(h0) * jx, jx ∈ Z, 0 ≤ 2^(h0) * jx ≤ 2wx
        // y_index_offset 表示每个候选点（节点）在搜索空间所对应的 y 轴索引 cy = -wy + 2^(h0) * jy, jy ∈ Z, 0 ≤ 2^(h0) * jy ≤ 2wy
        candidates.emplace_back(scan_index, x_index_offset, y_index_offset,
                                search_parameters);
      }
    }
  }
  // 最后检查一下候选点数量，通过后返回。
  CHECK_EQ(candidates.size(), num_candidates);
  return candidates;
}

// 函数 ScoreCandidates 计算各个候选点的评分并排序，该函数有四个参数。
// precomputation_grid 是将要查询的预算图，discrete_scans 是离散化的各搜索角度下的激光点云，
// search_parameters 是搜索配置，candidates 是候选点集合将在本函数中计算得分并排序。
void FastCorrelativeScanMatcher2D::ScoreCandidates(
    const PrecomputationGrid2D& precomputation_grid,
    const std::vector<DiscreteScan2D>& discrete_scans,
    const SearchParameters& search_parameters,
    std::vector<Candidate2D>* const candidates) const {
  // 在一个 for 循环中遍历所有的候选点，并为之计算所有点云的 hit 概率。
  for (Candidate2D& candidate : *candidates) {
    int sum = 0;
    for (const Eigen::Array2i& xy_index :
         discrete_scans[candidate.scan_index]) {
      const Eigen::Array2i proposed_xy_index(
          xy_index.x() + candidate.x_index_offset,
          xy_index.y() + candidate.y_index_offset);
      // 计算每一个候选点的点云的 hit 概率
      sum += precomputation_grid.GetValue(proposed_xy_index);
    }
    // 计算候选点的评分
    candidate.score = precomputation_grid.ToScore(
        sum / static_cast<float>(discrete_scans[candidate.scan_index].size()));
  }
  // 最后通过 std::sort 完成排序
  std::sort(candidates->begin(), candidates->end(),
            std::greater<Candidate2D>());
}

// 具体完成了分支定界搜索过程
// discrete_scans 离散化的各搜索角度下的激光点云
// search_parameters 搜索配置参数
// candidates 候选点集
// candidate_depth 搜索树高度，前文提到的c_h
// min_score 候选点最小评分
Candidate2D FastCorrelativeScanMatcher2D::BranchAndBound(
    const std::vector<DiscreteScan2D>& discrete_scans,
    const SearchParameters& search_parameters,
    const std::vector<Candidate2D>& candidates, const int candidate_depth,
    float min_score) const {
  // 这个函数是以递归调用的方式求解的。首先给出了递归终止的条件，就是如果搜索树高为0，意味着我们搜索到了一个叶子节点。
  // 同时由于每次迭代过程我们都是对新扩展的候选点进行降序排列，所以可以认为队首的这个叶子节点就是我们想要的最优解，直接返回即可。
  if (candidate_depth == 0) {
    // Return the best candidate.
    return *candidates.begin();  // 队首的这个叶子节点就是我们想要的最优解，直接返回
  }

  // 然后创建一个临时的候选点对象 best_high_resolution_candidate，为之赋予最小的评分。
  Candidate2D best_high_resolution_candidate(0, 0, 0, search_parameters);
  best_high_resolution_candidate.score = min_score;
  // 在一个 for 循环中遍历所有的候选点
  for (const Candidate2D& candidate : candidates) {
    // 如果遇到一个候选点的评分很低，意味着以后的候选点中也没有合适的解。可以直接跳出循环退出了，说明没有构成闭环。
    if (candidate.score <= min_score) {
      break;
    }
    // 如果 for 循环能够继续运行，说明当前候选点是一个更优的选择，需要对其进行分支。
    // 新生成的候选点将被保存在容器 higher_resolution_candidates 中。
    std::vector<Candidate2D> higher_resolution_candidates;
    // 分支后搜索树的高度减1，half_width 是分支后的步长 2^(candidate_depth - 1)
    const int half_width = 1 << (candidate_depth - 1);
    // for 循环从顶层 C0 扩展到下一层 C1，扩展后的 C1 包含四个新的候选点
    for (int x_offset : {0, half_width}) {
      // x_offset 是分支后的 x 轴索引的偏移量，第一次循环是0，第二次循环是分支后的步长 half_width
      // 检查分支后新的索引值 candidate.x_index_offset + x_offset 是否大于搜索空间的 x 轴的最大索引
      if (candidate.x_index_offset + x_offset >
          search_parameters.linear_bounds[candidate.scan_index].max_x) {
        break;
      }
      // y_offset 是分支后的 y 轴索引的偏移量，第一次循环是0，第二次循环是分支后的步长 half_width
      // 检查分支后新的索引值 candidate.y_index_offset + y_offset 是否大于搜索空间的 y 轴的最大索引
      for (int y_offset : {0, half_width}) {
        if (candidate.y_index_offset + y_offset >
            search_parameters.linear_bounds[candidate.scan_index].max_y) {
          break;
        }
        // 把分支后新生成的候选点保存在容器 higher_resolution_candidates 中
        higher_resolution_candidates.emplace_back(
            candidate.scan_index, candidate.x_index_offset + x_offset,
            candidate.y_index_offset + y_offset, search_parameters);
      }
    }
    // 调用刚刚介绍过的函数 ScoreCandidates 对新扩展的候选点定界并排序，
    // 即计算这四个新的 C1 的 target 得分，从大到小排序。
    ScoreCandidates(precomputation_grid_stack_->Get(candidate_depth - 1),
                    discrete_scans, search_parameters,
                    &higher_resolution_candidates);
    // 递归调用 BranchAndBound 对新扩展的 higher_resolution_candidates 进行搜索。
    // 这样就可以实现深度优先的搜索，先一直沿着最有可能的分支向下搜索，直到找到一个解。并将该解作为目前的最优解保存在
    // best_high_resolution_candidate 中。以后通过递归调用发现了更优的解都将通过 std::max 函数来更新已知的最优解。
    best_high_resolution_candidate = std::max(
        best_high_resolution_candidate,
        BranchAndBound(discrete_scans, search_parameters,
                       higher_resolution_candidates, candidate_depth - 1,
                       best_high_resolution_candidate.score));
  }
  // 当遍历完所有的候选点之后，对象 best_high_resolution_candidate 中就记录了最优的解，将之返回即可。
  return best_high_resolution_candidate;
}

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer
