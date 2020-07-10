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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_CONSTRAINTS_CONSTRAINT_BUILDER_2D_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_CONSTRAINTS_CONSTRAINT_BUILDER_2D_H_

#include <array>
#include <deque>
#include <functional>
#include <limits>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/common/fixed_ratio_sampler.h"
#include "cartographer/common/histogram.h"
#include "cartographer/common/math.h"
#include "cartographer/common/mutex.h"
#include "cartographer/common/task.h"
#include "cartographer/common/thread_pool.h"
#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/ceres_scan_matcher_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/fast_correlative_scan_matcher_2d.h"
#include "cartographer/mapping/pose_graph_interface.h"
#include "cartographer/mapping/proto/pose_graph/constraint_builder_options.pb.h"
#include "cartographer/metrics/family_factory.h"
#include "cartographer/sensor/internal/voxel_filter.h"
#include "cartographer/sensor/point_cloud.h"

namespace cartographer {
namespace mapping {
namespace constraints {

// Returns (map <- submap) where 'submap' is a coordinate system at the origin
// of the Submap.
// 返回(map <- submap)，其中 "submap" 是子图原点的坐标系。
transform::Rigid2d ComputeSubmapPose(const Submap2D& submap);

// Asynchronously computes constraints.
// 异步计算约束。
//
// Intermingle an arbitrary number of calls to 'MaybeAddConstraint',
// 'MaybeAddGlobalConstraint', and 'NotifyEndOfNode', then call 'WhenDone' once.
// After all computations are done the 'callback' will be called with the result
// and another MaybeAdd(Global)Constraint()/WhenDone() cycle can follow.
// 混合任意数量的对 "MaybeAddConstraint"，"MaybeAddGlobalConstraint" 和 "NotifyEndOfNode" 的调用，
// 然后一次调用 "WhenDone"。完成所有计算后，将使用计算结果调用回调 "callback"，然后可以执行另一个
// MaybeAdd(Global)Constraint()/WhenDone() 循环。
//
// This class is thread-safe.
// 此类是线程安全的。
//
// Cartographer 使用一个叫做 ConstraintBuilder2D 的类来进行闭环检测并构建约束。
// 根据它的注释，可以看出它主要是异步地计算约束。
// 我们可以在任意调用 MaybeAddConstraint、MaybeAddGlobalConstraint、NotifyEndOfNode 之后，
// 调用一次 WhenDone 接口，来注册回调函数结束一轮操作。
// 如此不断的重复这个过程就可以持续地进行闭环检测，添加必要的约束。
class ConstraintBuilder2D {
 public:
  using Constraint = PoseGraphInterface::Constraint;
  using Result = std::vector<Constraint>;

  /**
   * @brief ConstraintBuilder2D  构造函数
   * @param options              约束构建器的配置项。
   *                             这个配置项 options 早在 Cartographer 的 ROS 入口中就已经从配置文件中加载了。
   * @param thread_pool          线程池。线程池对象 thread_pool 指向地图构建器对象 map_builder 的一个成员变量。
   */
  ConstraintBuilder2D(const proto::ConstraintBuilderOptions& options,
                      common::ThreadPoolInterface* thread_pool);
  ~ConstraintBuilder2D();

  // 屏蔽了拷贝构造函数和拷贝赋值操作符
  ConstraintBuilder2D(const ConstraintBuilder2D&) = delete;
  ConstraintBuilder2D& operator=(const ConstraintBuilder2D&) = delete;

  // Schedules exploring a new constraint between 'submap' identified by
  // 'submap_id', and the 'compressed_point_cloud' for 'node_id'. The
  // 'initial_relative_pose' is relative to the 'submap'.
  // 计划探索在由 "submap_id" 标识的 "submap" 和由 "node_id" 标识的 "compressed_point_cloud" 之间的新约束。
  // "initial_relative_pose" 是相对于 "submap" 的位姿。
  //
  // The pointees of 'submap' and 'compressed_point_cloud' must stay valid until
  // all computations are finished.
  // "submap" 和 "compressed_point_cloud" 的指针必须在所有计算完成之前保持有效。
  /**
   * @brief MaybeAddConstraint     计算子图和路径节点之间是否存在可能的约束。
   * @param submap_id              子图的索引
   * @param submap                 指向考察的子图对象，这个对象的生命周期应当能够覆盖后端优化的计算过程。
   * @param node_id                路径节点的索引
   * @param constant_data          指向路径节点中记录的激光点云数据，这个对象的生命周期应当能够覆盖后端优化的计算过程。
   *                               这里包含以下4个更新的字段：
   *                               time 是当前同步时间；
   *                               gravity_alignment 是重力方向，机器人在局部地图坐标系下的方向；
   *                               filtered_gravity_aligned_point_cloud 是经过滤波和重力修正后的扫描数据，从局部地图坐标系平移到机器人坐标系下的扫描数据；
   *                               local_pose 为优化后的机器人在局部地图坐标系下的位姿估计，包含位置和方向信息。
   * @param initial_relative_pose  记录了路径节点相对于子图的初始位姿，提供了优化迭代的一个初值。
   *                               translation 为路径节点在子图的位置，而 rotation 用四元数表示后近似为 (0,0,0,1)。
   *                               所以它只包含位置信息，但不包含方向信息。
   */
  void MaybeAddConstraint(const SubmapId& submap_id, const Submap2D* submap,
                          const NodeId& node_id,
                          const TrajectoryNode::Data* const constant_data,
                          const transform::Rigid2d& initial_relative_pose);

  // Schedules exploring a new constraint between 'submap' identified by
  // 'submap_id' and the 'compressed_point_cloud' for 'node_id'.
  // This performs full-submap matching.
  //
  // The pointees of 'submap' and 'compressed_point_cloud' must stay valid until
  // all computations are finished.
  /**
   * @brief MaybeAddGlobalConstraint  计算子图和路径节点之间是否存在可能的约束。与 MaybeAddConstraint() 不同的是，
   *                                  该接口只有四个输入参数，没有提供初始相对位姿，而且它的扫描匹配是在整个子图上进行的。
   * @param submap_id                 子图的索引
   * @param submap                    指向考察的子图对象，这个对象的生命周期应当能够覆盖后端优化的计算过程。
   * @param node_id                   路径节点的索引
   * @param constant_data             指向路径节点中记录的激光点云数据，这个对象的生命周期应当能够覆盖后端优化的计算过程。
   *                                  这里包含以下4个更新的字段：
   *                                  time 是当前同步时间；
   *                                  gravity_alignment 是重力方向，机器人在局部地图坐标系下的方向；
   *                                  filtered_gravity_aligned_point_cloud 是经过滤波和重力修正后的扫描数据，从局部地图坐标系平移到机器人坐标系下的扫描数据；
   *                                  local_pose 为优化后的机器人在局部地图坐标系下的位姿估计，包含位置和方向信息。
   */
  void MaybeAddGlobalConstraint(
      const SubmapId& submap_id, const Submap2D* submap, const NodeId& node_id,
      const TrajectoryNode::Data* const constant_data);

  // Must be called after all computations related to one node have been added.
  // 必须在添加与一个节点有关的所有计算完成后，才能调用。
  /**
   * @brief NotifyEndOfNode  通知对象 constraint_builder_ 完成了一个路径节点的插入工作
   */
  void NotifyEndOfNode();

  // Registers the 'callback' to be called with the results, after all
  // computations triggered by 'MaybeAdd*Constraint' have finished.
  // 'callback' is executed in the 'ThreadPool'.
  // 在由 "MaybeAdd*Constraint" 触发的所有计算完成之后，使用计算结果来注册要调用的回调 "callback"。
  // "callback" 在 "ThreadPool" 中执行。
  /**
   * @brief WhenDone  当所有的闭环检测任务计算完成之后，注册要调用的回调函数
   * @param callback  记录了当所有的闭环检测任务结束之后的回调函数
   */
  void WhenDone(const std::function<void(const Result&)>& callback);

  // Returns the number of consecutive finished nodes.
  int GetNumFinishedNodes();

  // Delete data related to 'submap_id'.
  void DeleteScanMatcher(const SubmapId& submap_id);

  static void RegisterMetrics(metrics::FamilyFactory* family_factory);

 private:
  // 扫描匹配器
  struct SubmapScanMatcher {
    const Grid2D* grid;  // 记录子图的占用栅格
    std::unique_ptr<scan_matching::FastCorrelativeScanMatcher2D>
        fast_correlative_scan_matcher;  // 扫描匹配器内核
    std::weak_ptr<common::Task> creation_task_handle;  // 线程池任务句柄
  };

  // The returned 'grid' and 'fast_correlative_scan_matcher' must only be
  // accessed after 'creation_task_handle' has completed.
  // 仅在完成 "creation_task_handle" 之后，才可以访问返回的 "grid" 和 "fast_correlative_scan_matcher"。
  /**
   * @brief DispatchScanMatcherConstruction  用于为新增的子图创建一个扫描匹配器，
   *                                         成功创建的匹配器将以 submap_id 为索引被保存在容器 submap_scan_matchers_ 中。
   * @param submap_id                        子图的索引。这里的子图实际上是处于 kFinished 状态的子图。
   * @param grid                             子图的占用栅格地图
   * @return                                 成功构建的扫描匹配器对象
   */
  const SubmapScanMatcher* DispatchScanMatcherConstruction(
      const SubmapId& submap_id, const Grid2D* grid) REQUIRES(mutex_);

  // Runs in a background thread and does computations for an additional
  // constraint, assuming 'submap' and 'compressed_point_cloud' do not change
  // anymore. As output, it may create a new Constraint in 'constraint'.
  // 假设子图 "submap" 和 "compressed_point_cloud" 不再更改，则在后台线程中运行并针对附加约束进行计算。
  // 作为输出，它可以在 "constraint" 中创建一个新的 Constraint。
  /**
   * @brief ComputeConstraint      在后台线程中完成子图与路径节点之间可能的约束计算。
   * @param submap_id              待考察的子图的索引。这里的子图实际上是处于 kFinished 状态的子图。
   * @param submap                 待考察的子图的数据内容
   * @param node_id                待考察的路径节点的索引
   * @param match_full_submap      用于指示是否完成遍历子图
   * @param constant_data          待考察的路径节点的数据内容。这里包含以下4个更新的字段：
   *                               time 是当前同步时间；
   *                               gravity_alignment 是重力方向，机器人在局部地图坐标系下的方向；
   *                               filtered_gravity_aligned_point_cloud 是经过滤波和重力修正后的扫描数据，从局部地图坐标系平移到机器人坐标系下的扫描数据；
   *                               local_pose 为优化后的机器人在局部地图坐标系下的位姿估计，包含位置和方向信息。
   * @param initial_relative_pose  描述了路径节点相对于子图的初始位姿，提供了优化迭代的一个初值。
   *                               translation 为路径节点在子图的位置，而 rotation 用四元数表示后近似为 (0,0,0,1)。
   *                               所以它只包含位置信息，但不包含方向信息。
   * @param submap_scan_matcher    进行扫描匹配时所用的匹配器对象
   * @param constraint             用于输出约束结果。
   *                               这是一个指向智能指针类型 std::unique_ptr<Constraint> 的指针。
   */
  void ComputeConstraint(const SubmapId& submap_id, const Submap2D* submap,
                         const NodeId& node_id, bool match_full_submap,
                         const TrajectoryNode::Data* const constant_data,
                         const transform::Rigid2d& initial_relative_pose,
                         const SubmapScanMatcher& submap_scan_matcher,
                         std::unique_ptr<Constraint>* constraint)
      EXCLUDES(mutex_);
  /**
   * @brief RunWhenDoneCallback  当一轮 MaybeAdd-WhenDone 任务结束后，用来调用 WhenDone() 接口注册的回调函数的
   */
  void RunWhenDoneCallback() EXCLUDES(mutex_);

/****************************************** 成员变量 ******************************************/
  const constraints::proto::ConstraintBuilderOptions options_;  // 关于约束构建器的各种配置
  common::ThreadPoolInterface* thread_pool_;  // 线程池，用于并行地完成闭环检测
  common::Mutex mutex_;  // 保护公共资源的互斥量

  // 'callback' set by WhenDone().
  // 通过接口 WhenDone 注册的回调函数对象
  std::unique_ptr<std::function<void(const Result&)>> when_done_
      GUARDED_BY(mutex_);

  // TODO(gaschler): Use atomics instead of mutex to access these counters.
  // Number of the node in reaction to which computations are currently
  // added. This is always the number of nodes seen so far, even when older
  // nodes are matched against a new submap.
  int num_started_nodes_ GUARDED_BY(mutex_) = 0;  // 一个计数器，记录了当前需要考虑的轨迹节点数量。

  int num_finished_nodes_ GUARDED_BY(mutex_) = 0;  // 一个计数器，记录了当前已完成约束计算的轨迹节点数量。

  std::unique_ptr<common::Task> finish_node_task_ GUARDED_BY(mutex_);  // 完成轨迹节点约束计算的任务状态机

  std::unique_ptr<common::Task> when_done_task_ GUARDED_BY(mutex_);  // WhenDone 的任务状态机

  // Constraints currently being computed in the background. A deque is used to
  // keep pointers valid when adding more entries. Constraint search results
  // with below-threshold scores are also 'nullptr'.
  // 当前在后台计算的约束。当添加更多条目时，使用双端队列使指针保持有效。低于阈值分数的约束搜索结果也为 "nullptr"。
  //
  // 用于保存后台计算的约束的双端队列
  std::deque<std::unique_ptr<Constraint>> constraints_ GUARDED_BY(mutex_);

  // Map of dispatched or constructed scan matchers by 'submap_id'.
  // 记录各个子图的扫描匹配器的容器，以 SubmapId 为索引。
  std::map<SubmapId, SubmapScanMatcher> submap_scan_matchers_
      GUARDED_BY(mutex_);

  common::FixedRatioSampler sampler_;  // 采样器
  scan_matching::CeresScanMatcher2D ceres_scan_matcher_;  // 基于 Ceres 库的扫描匹配器

  // Histogram of scan matcher scores.
  // 扫描匹配得分的直方图
  common::Histogram score_histogram_ GUARDED_BY(mutex_);
};

}  // namespace constraints
}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_CONSTRAINTS_CONSTRAINT_BUILDER_2D_H_
