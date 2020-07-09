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

#include "cartographer/mapping/internal/constraints/constraint_builder_2d.h"

#include <cmath>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>

#include "Eigen/Eigenvalues"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/math.h"
#include "cartographer/common/thread_pool.h"
#include "cartographer/mapping/proto/scan_matching//ceres_scan_matcher_options_2d.pb.h"
#include "cartographer/mapping/proto/scan_matching//fast_correlative_scan_matcher_options_2d.pb.h"
#include "cartographer/metrics/counter.h"
#include "cartographer/metrics/gauge.h"
#include "cartographer/metrics/histogram.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace constraints {

static auto* kConstraintsSearchedMetric = metrics::Counter::Null();
static auto* kConstraintsFoundMetric = metrics::Counter::Null();
static auto* kGlobalConstraintsSearchedMetric = metrics::Counter::Null();
static auto* kGlobalConstraintsFoundMetric = metrics::Counter::Null();
static auto* kQueueLengthMetric = metrics::Gauge::Null();
static auto* kConstraintScoresMetric = metrics::Histogram::Null();
static auto* kGlobalConstraintScoresMetric = metrics::Histogram::Null();

transform::Rigid2d ComputeSubmapPose(const Submap2D& submap) {
  return transform::Project2D(submap.local_pose());
}

// 构造函数，它有两个输入参数，分别记录了配置项和线程池对象。
// 这个配置项 options 早在 Cartographer的ROS入口中就已经从配置文件中加载了。
// 而线程池对象 thread_pool 则指向地图构建器对象 map_builder 的一个成员变量。
// 在构造函数成员构造列表中，直接将这两个对象赋予了成员变量 options_ 和 thread_pool_。
// 并且完成了任务状态机对象 finish_node_task_ 和 when_done_task_ 的初始构造，
// 同时根据参数配置完成了采样器和基于 Ceres 的扫描匹配器的构造。
ConstraintBuilder2D::ConstraintBuilder2D(
    const constraints::proto::ConstraintBuilderOptions& options,
    common::ThreadPoolInterface* const thread_pool)
    : options_(options),
      thread_pool_(thread_pool),
      finish_node_task_(common::make_unique<common::Task>()),
      when_done_task_(common::make_unique<common::Task>()),
      sampler_(options.sampling_ratio()),
      ceres_scan_matcher_(options.ceres_scan_matcher_options()) {}

// 析构函数，该函数实际没有什么具体的操作，只检查了各种状态，保证释放对象的时候没有任务在后台运行。
ConstraintBuilder2D::~ConstraintBuilder2D() {
  common::MutexLocker locker(&mutex_);
  CHECK_EQ(finish_node_task_->GetState(), common::Task::NEW);
  CHECK_EQ(when_done_task_->GetState(), common::Task::NEW);
  CHECK_EQ(constraints_.size(), 0) << "WhenDone() was not called";
  CHECK_EQ(num_started_nodes_, num_finished_nodes_);
  CHECK(when_done_ == nullptr);
}

// 接口 MaybeAddConstraint 用于检查子图和路径节点之间是否存在可能的约束。
// 它有5个输入参数：
// submap_id 和 node_id 分别是子图和路径节点的索引；
// 指针 submap 和 constant_data 分别指向了考察的子图对象和路径节点中记录了激光点云数据，
// 需要注意的是这两个对象的生命周期应当能够覆盖后端优化的计算过程；
// initial_relative_pose 记录了路径节点相对于子图的初始位姿，提供了优化迭代的一个初值。
void ConstraintBuilder2D::MaybeAddConstraint(
    const SubmapId& submap_id, const Submap2D* const submap,
    const NodeId& node_id, const TrajectoryNode::Data* const constant_data,
    const transform::Rigid2d& initial_relative_pose) {
  /**
   * 1. 如果初始的相对位姿显示，路径节点与子图相差很远，就直接返回不再两者之间建立约束。
   *    这个距离阈值可以通过配置项 max_constraint_distance 来设定。
   * 2. 这样做有两个好处，其一可以一定程度上降低约束的数量，减少全局图优化的计算量，提高系统的运行效率；
   *    其二，如果两者相差太远，很可能会受到累积误差的影响，导致添加错误的约束，给最终的全局优化带来负面的影响，
   *    这样直接抛弃一些不太合理的约束，看似遗漏了很多信息，但对于优化结果而言可能是一件好事。
   */  
  if (initial_relative_pose.translation().norm() >
      options_.max_constraint_distance()) {
    return;
  }
  // 如果采样器暂停了，也将直接退出。我目前还不是很清楚这个采样器是干什么用的。
  if (!sampler_.Pulse()) return;

  // 通过互斥量对象 mutex_ 加锁。
  // 然后检查回调函数对象 when_done_ 是否存在，若存在则说明上一个闭环检测迭代还没有完全结束，所以通过日志报警。
  common::MutexLocker locker(&mutex_);
  if (when_done_) {
    LOG(WARNING)
        << "MaybeAddConstraint was called while WhenDone was scheduled.";
  }
  // 然后就向容器 constraints_ 中添加新的约束。至于那个 kQueueLengthMetric 我们暂时先不去管它。
  constraints_.emplace_back();
  kQueueLengthMetric->Set(constraints_.size());
  // constraint 是常量指针，必须初始化，而且一旦初始化完成，则它的值（也就是存放在指针中的那个地址）就不能再改变了。
  // constraint 是一个指向智能指针(std::unique_ptr<Constraint>)的常量指针。
  auto* const constraint = &constraints_.back();
  // 接着通过函数 DispatchScanMatcherConstruction() 构建了一个扫描匹配器，一会儿我们就会看到，
  // 这个函数并没有完成扫描匹配器的构建，而是通过 lambda 表达式和线程池推后实现的。
  const auto* scan_matcher =
      DispatchScanMatcherConstruction(submap_id, submap->grid());
  // 构建一个线程池所用的任务对象 constraint_task，并通过接口 SetWorkItem 和 lambda 表达式具体描述工作内容。
  // 其 lambda 表达式中调用的函数 ComputeConstraint 具体完成了约束的计算。
  auto constraint_task = common::make_unique<common::Task>();
  constraint_task->SetWorkItem([=]() EXCLUDES(mutex_) {
    ComputeConstraint(submap_id, submap, node_id, false, /* match_full_submap */
                      constant_data, initial_relative_pose, *scan_matcher,
                      constraint);
  });
  // 由于具体的约束计算过程需要用到刚刚构建的扫描匹配器对象 scan_matcher，而该对象的核心也是在线程池中的一个 Task 完成构建的。
  // 所以这里通过给 Task 添加依赖关系来保证，在进行约束计算之前扫描匹配器就已经完成构建和初始化了。
  // 这里注意，constraint_task 是类型为 common::Task 的智能指针，
  // 而 scan_matcher->creation_task_handle 则是线程池任务句柄，类型为 std::weak_ptr<common::Task>。
  constraint_task->AddDependency(scan_matcher->creation_task_handle);
  // 最后将计算约束的任务 constraint_task 添加到线程池的调度队列中，并将其设置为完成轨迹节点约束计算任务的依赖，
  // 保证在完成了所有计算约束的任务之后才会执行 constraint_task 的计算任务。
  auto constraint_task_handle =
      thread_pool_->Schedule(std::move(constraint_task));
  // finish_node_task_ 是类型为 common::Task 的智能指针，
  // 而 constraint_task_handle 则是线程池任务句柄，类型为 std::weak_ptr<common::Task>。
  finish_node_task_->AddDependency(constraint_task_handle);
}

// 接口 MaybeAddGlobalConstraint() 与 MaybeAddConstraint() 的功能类似，也是计算子图和路径节点之间是否存在可能的约束。
// 所不同的是，该接口只有四个输入参数，没有提供初始相对位姿，而且它的扫描匹配是在整个子图上进行的。
// 该接口与 MaybeAddConstraint() 的套路基本一样。
void ConstraintBuilder2D::MaybeAddGlobalConstraint(
    const SubmapId& submap_id, const Submap2D* const submap,
    const NodeId& node_id, const TrajectoryNode::Data* const constant_data) {
  common::MutexLocker locker(&mutex_);
  if (when_done_) {
    LOG(WARNING)
        << "MaybeAddGlobalConstraint was called while WhenDone was scheduled.";
  }
  constraints_.emplace_back();
  kQueueLengthMetric->Set(constraints_.size());
  auto* const constraint = &constraints_.back();
  const auto* scan_matcher =
      DispatchScanMatcherConstruction(submap_id, submap->grid());
  auto constraint_task = common::make_unique<common::Task>();
  constraint_task->SetWorkItem([=]() EXCLUDES(mutex_) {
    ComputeConstraint(submap_id, submap, node_id, true, /* match_full_submap */
                      constant_data, transform::Rigid2d::Identity(),
                      *scan_matcher, constraint);
  });
  constraint_task->AddDependency(scan_matcher->creation_task_handle);
  auto constraint_task_handle =
      thread_pool_->Schedule(std::move(constraint_task));
  finish_node_task_->AddDependency(constraint_task_handle);
}

// 回顾位姿图的更新过程，我们会发现闭环检测是在每次向后端添加路径节点的时候触发的。除了要建立路径节点与新旧子图之间的内部约束之外，
// 还要与所有处于 kFinished 状态的子图进行一次匹配计算可能存在的约束，如果旧子图切换到 kFinished 状态还需要将之与所有路径节点进行匹配。
// 完成了这些操作之后，它就会调用接口 NotifyEndOfNode，通知对象 constraint_builder_ 完成了一个路径节点的插入工作。
void ConstraintBuilder2D::NotifyEndOfNode() {
  common::MutexLocker locker(&mutex_);
  CHECK(finish_node_task_ != nullptr);
  // 以下的 lambda 表达式，可以看到当完成了轨迹节点的约束计算后，就会增加计数器 num_finished_nodes_ 的计数，
  // 标识着一个新的路径节点被添加到后端。
  // finish_node_task_ 是类型为 common::Task 的智能指针。
  finish_node_task_->SetWorkItem([this] {
    common::MutexLocker locker(&mutex_);
    ++num_finished_nodes_;
  });
  // 通过线程池的 Schedule 接口，将任务对象 finish_node_task_ 添加到线程池的调度队列中。
  auto finish_node_task_handle =
      thread_pool_->Schedule(std::move(finish_node_task_));
  // 重新构建一个任务状态机对象 finish_node_task_ 
  finish_node_task_ = common::make_unique<common::Task>();
  // 将该对象添加到 WhenDone 任务的依赖列表中。
  // when_done_task_ 是类型为 common::Task 的智能指针，
  // 而 finish_node_task_handle 是线程池任务句柄，类型为 std::weak_ptr<common::Task>。
  when_done_task_->AddDependency(finish_node_task_handle);
  ++num_started_nodes_;  // 控制计数器 num_started_nodes_ 自加
}

// 函数的作用是当所有的闭环检测任务计算完成之后，注册要调用的回调函数。
// 该函数只有一个输入参数，记录了当所有的闭环检测任务结束之后的回调函数。
void ConstraintBuilder2D::WhenDone(
    const std::function<void(const ConstraintBuilder2D::Result&)>& callback) {
  common::MutexLocker locker(&mutex_);
  CHECK(when_done_ == nullptr);
  // TODO(gaschler): Consider using just std::function, it can also be empty.
  // 用指针 when_done_ 记录下回调函数
  when_done_ =
      common::make_unique<std::function<void(const Result&)>>(callback);
  CHECK(when_done_task_ != nullptr);
  // 指定 when_done_task_ 的具体工作内容，调用函数 RunWhenDoneCallback()。
  // when_done_task_ 是类型为 common::Task 的智能指针。
  when_done_task_->SetWorkItem([this] { RunWhenDoneCallback(); });
  // 将 when_done_task_ 添加到线程池的调度队列中
  thread_pool_->Schedule(std::move(when_done_task_));
  // 重新构建一个任务状态机对象 when_done_task_
  when_done_task_ = common::make_unique<common::Task>();
}

// 函数 DispatchScanMatcherConstruction 用于为新增的子图创建一个扫描匹配器，
// 成功创建的匹配器将以 submap_id 为索引被保存在容器 submap_scan_matchers_ 中。
// 它有两个输入参数，其中 submap_id 记录了子图的索引，而 grid 则是子图的占用栅格表示。
const ConstraintBuilder2D::SubmapScanMatcher*
ConstraintBuilder2D::DispatchScanMatcherConstruction(const SubmapId& submap_id,
                                                     const Grid2D* const grid) {
  // 先通过容器 submap_scan_matchers_ 查询一下是否曾经为该子图创建过扫描匹配器。若是则直接返回所对应的扫描匹配器对象。
  if (submap_scan_matchers_.count(submap_id) != 0) {
    return &submap_scan_matchers_.at(submap_id);
  }
  // 然后扩展容器 submap_scan_matchers_，并将输入的子图占用栅格填充到新扩展的扫描匹配器中。
  auto& submap_scan_matcher = submap_scan_matchers_[submap_id];
  submap_scan_matcher.grid = grid;
  // 从配置项中获取扫描匹配器的配置
  auto& scan_matcher_options = options_.fast_correlative_scan_matcher_options();
  // 同时创建一个临时的任务对象 scan_matcher_task，在它的 lambda 表达式中，使用刚刚获取的配置和占用栅格创建了一个
  // FastCorrelativeScanMatcher2D 类型的对象。该对象是 Cartographer 通过分支定界进行闭环检测的算法实现。
  auto scan_matcher_task = common::make_unique<common::Task>();
  scan_matcher_task->SetWorkItem(
      [&submap_scan_matcher, &scan_matcher_options]() {
        submap_scan_matcher.fast_correlative_scan_matcher =
            common::make_unique<scan_matching::FastCorrelativeScanMatcher2D>(
                *submap_scan_matcher.grid, scan_matcher_options);
      });
  // 将构建扫描匹配器的任务放置到线程池的调度队列中
  submap_scan_matcher.creation_task_handle =
      thread_pool_->Schedule(std::move(scan_matcher_task));
  // 将刚刚扩展得到的匹配器对象返回
  return &submap_scan_matchers_.at(submap_id);
}

// 函数 ComputeConstraint 在后台线程中完成子图与路径节点之间可能的约束计算。
// submap_id、submap、node_id、constant_data 分别是待考察的子图和路径节点的索引和数据内容。
// 布尔变量 match_full_submap 用于指示是否完成遍历子图。
// initial_relative_pose 描述了路径节点相对于子图的初始位姿，submap_scan_matcher 则是进行扫描匹配时所用的匹配器对象，
// constraint 则用于输出约束结果。
void ConstraintBuilder2D::ComputeConstraint(
    const SubmapId& submap_id, const Submap2D* const submap,
    const NodeId& node_id, bool match_full_submap,
    const TrajectoryNode::Data* const constant_data,
    const transform::Rigid2d& initial_relative_pose,
    const SubmapScanMatcher& submap_scan_matcher,
    std::unique_ptr<ConstraintBuilder2D::Constraint>* constraint) {
  // 在函数的一开始，先构建几个临时变量。
  // initial_pose 用于描述在世界坐标系下路径节点与子图之间的相对位置关系。
  const transform::Rigid2d initial_pose =
      ComputeSubmapPose(*submap) * initial_relative_pose;

  // The 'constraint_transform' (submap i <- node j) is computed from:
  // - a 'filtered_gravity_aligned_point_cloud' in node j,
  // - the initial guess 'initial_pose' for (map <- node j),
  // - the result 'pose_estimate' of Match() (map <- node j).
  // - the ComputeSubmapPose() (map <- submap i)
  // score 和 pose_estimate 用于记录扫描匹配之后的得分和位姿估计。
  float score = 0.;
  transform::Rigid2d pose_estimate = transform::Rigid2d::Identity();

  // Compute 'pose_estimate' in three stages:
  // 1. Fast estimate using the fast correlative scan matcher.
  // 2. Prune if the score is too low.
  // 3. Refine.
  // 根据输入参数 match_full_submap 选择不同的扫描匹配方式，如果没有匹配成功则直接退出。
  if (match_full_submap) {
    kGlobalConstraintsSearchedMetric->Increment();
    if (submap_scan_matcher.fast_correlative_scan_matcher->MatchFullSubmap(
            constant_data->filtered_gravity_aligned_point_cloud,
            options_.global_localization_min_score(), &score, &pose_estimate)) {
      CHECK_GT(score, options_.global_localization_min_score());
      CHECK_GE(node_id.trajectory_id, 0);
      CHECK_GE(submap_id.trajectory_id, 0);
      kGlobalConstraintsFoundMetric->Increment();
      kGlobalConstraintScoresMetric->Observe(score);
    } else {
      return;
    }
  } else {
    kConstraintsSearchedMetric->Increment();
    if (submap_scan_matcher.fast_correlative_scan_matcher->Match(
            initial_pose, constant_data->filtered_gravity_aligned_point_cloud,
            options_.min_score(), &score, &pose_estimate)) {
      // We've reported a successful local match.
      CHECK_GT(score, options_.min_score());
      kConstraintsFoundMetric->Increment();
      kConstraintScoresMetric->Observe(score);
    } else {
      return;
    }
  }
  // 接着在一个局部的 locker 的保护下，将新获得的约束得分统计到一个直方图中。
  {
    common::MutexLocker locker(&mutex_);
    score_histogram_.Add(score);
  }

  // Use the CSM estimate as both the initial and previous pose. This has the
  // effect that, in the absence of better information, we prefer the original
  // CSM estimate.
  // 最后使用 Ceres 扫描匹配进一步的对刚刚构建的约束进行优化。
  ceres::Solver::Summary unused_summary;
  ceres_scan_matcher_.Match(pose_estimate.translation(), pose_estimate,
                            constant_data->filtered_gravity_aligned_point_cloud,
                            *submap_scan_matcher.grid, &pose_estimate,
                            &unused_summary);

  // 并将这种新建的约束标记为 INTER_SUBMAP 类型。
  const transform::Rigid2d constraint_transform =
      ComputeSubmapPose(*submap).inverse() * pose_estimate;
  constraint->reset(new Constraint{submap_id,
                                   node_id,
                                   {transform::Embed3D(constraint_transform),
                                    options_.loop_closure_translation_weight(),
                                    options_.loop_closure_rotation_weight()},
                                   Constraint::INTER_SUBMAP});

  if (options_.log_matches()) {
    std::ostringstream info;
    info << "Node " << node_id << " with "
         << constant_data->filtered_gravity_aligned_point_cloud.size()
         << " points on submap " << submap_id << std::fixed;
    if (match_full_submap) {
      info << " matches";
    } else {
      const transform::Rigid2d difference =
          initial_pose.inverse() * pose_estimate;
      info << " differs by translation " << std::setprecision(2)
           << difference.translation().norm() << " rotation "
           << std::setprecision(3) << std::abs(difference.normalized_angle());
    }
    info << " with score " << std::setprecision(1) << 100. * score << "%.";
    LOG(INFO) << info.str();
  }
}

// 函数 RunWhenDoneCallback 是当一轮 MaybeAdd-WhenDone 任务结束后，用来调用 WhenDone 接口注册的回调函数的。
void ConstraintBuilder2D::RunWhenDoneCallback() {
  Result result;
  std::unique_ptr<std::function<void(const Result&)>> callback;
  {
    common::MutexLocker locker(&mutex_);
    CHECK(when_done_ != nullptr);
    // 将 MaybeAdd 过程中得到的约束放到 result 对象中
    for (const std::unique_ptr<Constraint>& constraint : constraints_) {
      if (constraint == nullptr) continue;
      result.push_back(*constraint);
    }
    if (options_.log_matches()) {
      LOG(INFO) << constraints_.size() << " computations resulted in "
                << result.size() << " additional constraints.";
      LOG(INFO) << "Score histogram:\n" << score_histogram_.ToString(10);
    }
    constraints_.clear();  // 清空约束列表 constraints_
    callback = std::move(when_done_);  // 用临时变量 callback 记录下回调函数对象
    when_done_.reset();  // 释放 when_done_ 指针
    kQueueLengthMetric->Set(constraints_.size());
  }
  (*callback)(result);  // 最后将 result 作为参数传递给回调函数
}

int ConstraintBuilder2D::GetNumFinishedNodes() {
  common::MutexLocker locker(&mutex_);
  return num_finished_nodes_;
}

void ConstraintBuilder2D::DeleteScanMatcher(const SubmapId& submap_id) {
  common::MutexLocker locker(&mutex_);
  if (when_done_) {
    LOG(WARNING)
        << "DeleteScanMatcher was called while WhenDone was scheduled.";
  }
  submap_scan_matchers_.erase(submap_id);
}

void ConstraintBuilder2D::RegisterMetrics(metrics::FamilyFactory* factory) {
  auto* counts = factory->NewCounterFamily(
      "mapping_internal_constraints_constraint_builder_2d_constraints",
      "Constraints computed");
  kConstraintsSearchedMetric =
      counts->Add({{"search_region", "local"}, {"matcher", "searched"}});
  kConstraintsFoundMetric =
      counts->Add({{"search_region", "local"}, {"matcher", "found"}});
  kGlobalConstraintsSearchedMetric =
      counts->Add({{"search_region", "global"}, {"matcher", "searched"}});
  kGlobalConstraintsFoundMetric =
      counts->Add({{"search_region", "global"}, {"matcher", "found"}});
  auto* queue_length = factory->NewGaugeFamily(
      "mapping_internal_constraints_constraint_builder_2d_queue_length",
      "Queue length");
  kQueueLengthMetric = queue_length->Add({});
  auto boundaries = metrics::Histogram::FixedWidth(0.05, 20);
  auto* scores = factory->NewHistogramFamily(
      "mapping_internal_constraints_constraint_builder_2d_scores",
      "Constraint scores built", boundaries);
  kConstraintScoresMetric = scores->Add({{"search_region", "local"}});
  kGlobalConstraintScoresMetric = scores->Add({{"search_region", "global"}});
}

}  // namespace constraints
}  // namespace mapping
}  // namespace cartographer
