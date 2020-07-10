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

#include "cartographer/mapping/internal/2d/pose_graph_2d.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <memory>
#include <sstream>
#include <string>

#include "Eigen/Eigenvalues"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/math.h"
#include "cartographer/mapping/proto/pose_graph/constraint_builder_options.pb.h"
#include "cartographer/sensor/compressed_point_cloud.h"
#include "cartographer/sensor/internal/voxel_filter.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

// 构造函数
// 函数体是空的什么也没有做，只是在构造列表中完成了一些成员变量构造。
// 它有三个输入参数：
// options 是从配置文件中装载的关于位姿图的配置项，
// optimization_problem 是一个智能指针指向后端优化问题求解器，
// thread_pool 则是一个线程池。
PoseGraph2D::PoseGraph2D(
    const proto::PoseGraphOptions& options,
    std::unique_ptr<optimization::OptimizationProblem2D> optimization_problem,
    common::ThreadPool* thread_pool)
    : options_(options),
      optimization_problem_(std::move(optimization_problem)),
      constraint_builder_(options_.constraint_builder_options(), thread_pool) {}

// 析构函数
PoseGraph2D::~PoseGraph2D() {
  WaitForAllComputations();
  common::MutexLocker locker(&mutex_);
  CHECK(work_queue_ == nullptr);
}

// 将子图的初始位姿提供给后端优化器，有三个输入参数。
// trajectory_id 是运行轨迹的索引，
// time 则是调用 AddNode 时对应路径节点的时间戳，
// insertion_submaps 则是从 Local SLAM 一路传递过来的新旧子图。
// 该函数的输出是一个 vector 容器，它将记录 insertion_submaps 中各个子图分配的 SubmapId。
std::vector<SubmapId> PoseGraph2D::InitializeGlobalSubmapPoses(
    const int trajectory_id, const common::Time time,
    const std::vector<std::shared_ptr<const Submap2D>>& insertion_submaps) {
  // 检查 insertion_submaps 非空
  CHECK(!insertion_submaps.empty());
  // 并获取后端优化器的子图位姿信息记录到临时对象 submap_data 中。
  const auto& submap_data = optimization_problem_->submap_data();
  // 根据 Local SLAM 中子图的维护方式，如果输入参数 insertion_submaps 中只有一个子图，意味着重新开始了一条新的轨迹。
  if (insertion_submaps.size() == 1) {
    // If we don't already have an entry for the first submap, add one.
    // 此时检查后端优化器中是否已经存在一条索引为 trajectory_id 的轨迹，若没有则创建一个。
    if (submap_data.SizeOfTrajectoryOrZero(trajectory_id) == 0) {
      // 检查 trajectory_id 是否与之前的轨迹有关联，若有则根据 initial_trajectory_poses_ 的描述构建连接关系
      if (initial_trajectory_poses_.count(trajectory_id) > 0) {
        trajectory_connectivity_state_.Connect(
            trajectory_id,
            initial_trajectory_poses_.at(trajectory_id).to_trajectory_id, time);
      }
      // 通过后端优化器的接口 AddSubmap 创建一条新的轨迹，并将子图的全局位姿信息喂给优化器。
      optimization_problem_->AddSubmap(
          trajectory_id,
          transform::Project2D(ComputeLocalToGlobalTransform(
                                   global_submap_poses_, trajectory_id) *
                               insertion_submaps[0]->local_pose()));
    }
    // 检查一下数据关系，为新建的子图赋予唯一的 SubmapId，并返回之。
    CHECK_EQ(1, submap_data.SizeOfTrajectoryOrZero(trajectory_id));
    const SubmapId submap_id{trajectory_id, 0};
    CHECK(submap_data_.at(submap_id).submap == insertion_submaps.front());
    return {submap_id};
  }
  // 如果函数的控制流走到了下面的代码片段，说明 trajectory_id 下已经至少有了一个子图，
  // 此时输入的 insertion_submaps 中应当有两个子图，也就是 Local SLAM 中维护的新旧子图。
  CHECK_EQ(2, insertion_submaps.size());
  const auto end_it = submap_data.EndOfTrajectory(trajectory_id);
  CHECK(submap_data.BeginOfTrajectory(trajectory_id) != end_it);
  // 用局部变量 last_submap_id 记录下后端优化器中最新子图的索引。
  const SubmapId last_submap_id = std::prev(end_it)->id;
  // 然后根据 last_submap_id 检查一下后端优化器中最新的子图是否与 insertion_submaps 中的旧图是同一个对象。
  // submap_data_.at(last_submap_id).submap 是后端优化器中最新的子图，insertion_submaps.front() 是输入的旧图。
  if (submap_data_.at(last_submap_id).submap == insertion_submaps.front()) {
    // In this case, 'last_submap_id' is the ID of
    // 'insertions_submaps.front()' and 'insertions_submaps.back()' is new.
    // 若是，则说明新图是 Local SLAM 新建的子图，后端尚未记录。此时需要将新图的全局位姿提供给后端优化器，并分配一个 SubmapId。
    // 然后将新旧子图的 SubmapId 放到容器中一并返回。
    // first_submap_pose 是旧图的全局位姿。
    const auto& first_submap_pose = submap_data.at(last_submap_id).global_pose;
    // 将新图的全局位姿提供给后端优化器，并分配一个 SubmapId
    optimization_problem_->AddSubmap(
        trajectory_id,
        // 计算新图的全局位姿
        first_submap_pose *
            constraints::ComputeSubmapPose(*insertion_submaps[0]).inverse() *
            constraints::ComputeSubmapPose(*insertion_submaps[1]));
    // 将新旧子图的 SubmapId 放到容器中一并返回
    return {last_submap_id,
            SubmapId{trajectory_id, last_submap_id.submap_index + 1}};
  }
  // 最后只剩下一种情况，Local SLAM 并没有再新建子图了，此时后端中记录了所有的子图，只需要将新旧子图对应的 SubmapId 返回即可。
  CHECK(submap_data_.at(last_submap_id).submap == insertion_submaps.back());
  const SubmapId front_submap_id{trajectory_id,
                                 last_submap_id.submap_index - 1};
  CHECK(submap_data_.at(front_submap_id).submap == insertion_submaps.front());
  return {front_submap_id, last_submap_id};
}

// 创建一个轨迹节点，并把前端的输出结果喂给后端，最后返回新建的轨迹节点索引。
// 有三个输入参数：
// constant_data 记录了更新子图时的点云信息以及相对位姿；
// trajectory_id 记录了轨迹索引；
// insertion_submaps 则是更新的子图。
NodeId PoseGraph2D::AddNode(
    std::shared_ptr<const TrajectoryNode::Data> constant_data,
    const int trajectory_id,
    const std::vector<std::shared_ptr<const Submap2D>>& insertion_submaps) {
  // 在函数一开始先将局部位姿转换成为世界坐标系下的位姿。
  // 函数 GetLocalToGlobalTransform() 根据最新一次优化之后的子图位姿生成局部坐标系到世界坐标系的坐标变换关系。
  // constant_data->local_pose 是优化后机器人在局部地图坐标系下的位姿估计。
  // 所以，optimized_pose 是扫描数据插入子图时机器人在世界坐标系下的位姿，即节点在世界坐标系下的位姿。
  const transform::Rigid3d optimized_pose(
      GetLocalToGlobalTransform(trajectory_id) * constant_data->local_pose);

  // 接下来先对信号量加锁，以保证多线程的安全。
  // 并通过函数 AddTrajectoryIfNeeded() 来维护各个轨迹之间的连接关系。
  common::MutexLocker locker(&mutex_);   
  AddTrajectoryIfNeeded(trajectory_id);
  // 根据输入的数据和刚刚生成的全局位姿构建一个 TrajectoryNode 的对象，并将之添加到节点的容器 trajectory_nodes_中。
  // 至此我们就添加了一个新的节点，并为之分配了一个 NodeId。
  const NodeId node_id = trajectory_nodes_.Append(
      trajectory_id, TrajectoryNode{constant_data, optimized_pose});
  ++num_trajectory_nodes_;  // 在 "pose_graph_2d.h" 中初始化为0

  // Test if the 'insertion_submap.back()' is one we never saw before.
  // 测试 "insertion_submap.back()" 是否是我们从未见过的。
  //
  // 我们可以认为输入参数 insertion_submaps.back() 中记录了最新的子图。
  // 所以下面的条件语句中先查询一下容器 submap_data_ 中是否有轨迹索引为 trajectory_id 的子图，
  // 再判定 insertion_submaps.back() 中的子图是否是新生成的。
  // 若是则将之添加到容器 submap_data_ 中，同时分配一个 SubmapId。
  if (submap_data_.SizeOfTrajectoryOrZero(trajectory_id) == 0 ||
      std::prev(submap_data_.EndOfTrajectory(trajectory_id))->data.submap !=
          insertion_submaps.back()) {
    // We grow 'submap_data_' as needed. This code assumes that the first
    // time we see a new submap is as 'insertion_submaps.back()'.
    // 我们根据需要增长 "submap_data_"。此代码假定我们第一次看到新的子图为 "insertion_submaps.back()"。
    //
    // 分配一个 SubmapId
    const SubmapId submap_id =
        submap_data_.Append(trajectory_id, InternalSubmapData());
    // 将新图 insertion_submaps.back() 添加到容器 submap_data_ 中
    submap_data_.at(submap_id).submap = insertion_submaps.back();
  }

  // We have to check this here, because it might have changed by the time we
  // execute the lambda.
  // 我们必须在此处进行检查，因为在执行 lambda 时可能已更改。
  /**
   * 1. 最后通过 lambda 表达式和函数 AddWorkItem() 注册一个为新增节点添加约束的任务 ComputeConstraintsForNode()。
   * 2. 根据 Cartographer 的思想，在该任务下应当会将新增的节点与所有已经处于 kFinished 状态的子图进行一次匹配建立可能存在的闭环约束。
   *    此外，当有新的子图进入 kFinished 状态时，还会将之与所有的节点进行一次匹配。
   *    所以这里会通过 insertion_submaps.front() 来查询旧图的更新状态。
   * 3. 注意：lambda 表达式中的[函数对象参数]为[=]，表示函数体内可以使用 Lambda 所在范围内所有可见的局部变量（包括 Lambda 所在类的 this），
   *    并且是值传递方式（相当于编译器自动为我们按值传递了所有局部变量）。
   */ 
  const bool newly_finished_submap = insertion_submaps.front()->finished();
  AddWorkItem([=]() REQUIRES(mutex_) {
    ComputeConstraintsForNode(node_id, insertion_submaps,
                              newly_finished_submap);
  });
  return node_id;
}

// 处理一个新的任务。
// 输入参数是一个能够兼容 lambda 表达式的 void() 类型的可调用对象。
// 其工作内容也十分简单，如果工作队列(work_queue_)存在就将任务(work_item)放到队列中，如果不存在就直接执行。
void PoseGraph2D::AddWorkItem(const std::function<void()>& work_item) {
  // 这里的工作队列 work_queue_ 是 PoseGraph2D 的一个成员变量，是一个指向双端队列的智能指针。
  // 在 PoseGraph2D 构造函数中没有创建过这一对象，所以一开始都是 nullptr。
  if (work_queue_ == nullptr) {
    // 若工作队列 work_queue_ 不存在，则直接执行任务 work_item
    work_item();
  } else {
    // 若工作队列 work_queue_ 存在，则把将任务 work_item 放到队列中
    work_queue_->push_back(work_item);
  }
}

void PoseGraph2D::AddTrajectoryIfNeeded(const int trajectory_id) {
  trajectory_connectivity_state_.Add(trajectory_id);
  // Make sure we have a sampler for this trajectory.
  if (!global_localization_samplers_[trajectory_id]) {
    global_localization_samplers_[trajectory_id] =
        common::make_unique<common::FixedRatioSampler>(
            options_.global_sampling_ratio());
  }
}

void PoseGraph2D::AddImuData(const int trajectory_id,
                             const sensor::ImuData& imu_data) {
  common::MutexLocker locker(&mutex_);
  AddWorkItem([=]() REQUIRES(mutex_) {
    optimization_problem_->AddImuData(trajectory_id, imu_data);
  });
}

void PoseGraph2D::AddOdometryData(const int trajectory_id,
                                  const sensor::OdometryData& odometry_data) {
  common::MutexLocker locker(&mutex_);
  AddWorkItem([=]() REQUIRES(mutex_) {
    optimization_problem_->AddOdometryData(trajectory_id, odometry_data);
  });
}

void PoseGraph2D::AddFixedFramePoseData(
    const int trajectory_id,
    const sensor::FixedFramePoseData& fixed_frame_pose_data) {
  LOG(FATAL) << "Not yet implemented for 2D.";
}

void PoseGraph2D::AddLandmarkData(int trajectory_id,
                                  const sensor::LandmarkData& landmark_data)
    EXCLUDES(mutex_) {
  common::MutexLocker locker(&mutex_);
  AddWorkItem([=]() REQUIRES(mutex_) {
    for (const auto& observation : landmark_data.landmark_observations) {
      landmark_nodes_[observation.id].landmark_observations.emplace_back(
          LandmarkNode::LandmarkObservation{
              trajectory_id, landmark_data.time,
              observation.landmark_to_tracking_transform,
              observation.translation_weight, observation.rotation_weight});
    }
  });
}

void PoseGraph2D::ComputeConstraint(const NodeId& node_id,
                                    const SubmapId& submap_id) {
  CHECK(submap_data_.at(submap_id).state == SubmapState::kFinished);

  const common::Time node_time = GetLatestNodeTime(node_id, submap_id);
  const common::Time last_connection_time =
      trajectory_connectivity_state_.LastConnectionTime(
          node_id.trajectory_id, submap_id.trajectory_id);
  if (node_id.trajectory_id == submap_id.trajectory_id ||
      node_time <
          last_connection_time +
              common::FromSeconds(
                  options_.global_constraint_search_after_n_seconds())) {
    // If the node and the submap belong to the same trajectory or if there
    // has been a recent global constraint that ties that node's trajectory to
    // the submap's trajectory, it suffices to do a match constrained to a
    // local search window.
    const transform::Rigid2d initial_relative_pose =
        optimization_problem_->submap_data()
            .at(submap_id)
            .global_pose.inverse() *
        optimization_problem_->node_data().at(node_id).global_pose_2d;
    constraint_builder_.MaybeAddConstraint(
        submap_id, submap_data_.at(submap_id).submap.get(), node_id,
        trajectory_nodes_.at(node_id).constant_data.get(),
        initial_relative_pose);
  } else if (global_localization_samplers_[node_id.trajectory_id]->Pulse()) {
    constraint_builder_.MaybeAddGlobalConstraint(
        submap_id, submap_data_.at(submap_id).submap.get(), node_id,
        trajectory_nodes_.at(node_id).constant_data.get());
  }
}

void PoseGraph2D::ComputeConstraintsForOldNodes(const SubmapId& submap_id) {
  const auto& submap_data = submap_data_.at(submap_id);
  for (const auto& node_id_data : optimization_problem_->node_data()) {
    const NodeId& node_id = node_id_data.id;
    if (submap_data.node_ids.count(node_id) == 0) {
      ComputeConstraint(node_id, submap_id);
    }
  }
}

// 函数 ComputeConstraintsForNode() 除了有添加约束的作用，还会触发工作队列的构建和运行。
// 它有三个输入参数：
// node_id 是待更新的节点索引；
// insertion_submaps 则是从 Local SLAM 一路传递过来的新旧子图；
// newly_finished_submap 表示旧图是否结束更新了。
void PoseGraph2D::ComputeConstraintsForNode(
    const NodeId& node_id,
    std::vector<std::shared_ptr<const Submap2D>> insertion_submaps,
    const bool newly_finished_submap) {
  // 在函数的一开始先获取节点数据，并通过函数 InitializeGlobalSubmapPoses() 获取新旧子图的索引。
  // 它除了获取ID之外还检查了新子图是否第一次被后端看见，若是则为之计算全局位姿并喂给后端优化器 optimization_problem_。
  const auto& constant_data = trajectory_nodes_.at(node_id).constant_data;
  const std::vector<SubmapId> submap_ids = InitializeGlobalSubmapPoses(  // submap_ids 记录了新旧子图的索引
      node_id.trajectory_id, constant_data->time, insertion_submaps);
  // 检查刚获取的新旧子图索引的数量是否等于输入子图数量
  CHECK_EQ(submap_ids.size(), insertion_submaps.size());
  // 接下来以旧图为参考，计算节点在局部地图坐标系下的位姿，以及它在世界坐标系下的位姿 εsj。
  // 并将之提供给后端优化器 optimization_problem_。
  const SubmapId matching_id = submap_ids.front();  // 旧图的索引
  // 计算节点在局部地图坐标系下的位姿。
  // constant_data->local_pose 是机器人在局部地图坐标系下的位姿，包含位置和方向信息，
  // 而 constant_data->gravity_alignment 是机器人在局部地图坐标系下的方向，
  // 所以计算得到的 local_pose_2d 是节点在局部地图坐标系下的位置，不包含方向信息。
  const transform::Rigid2d local_pose_2d = transform::Project2D(
      constant_data->local_pose *
      transform::Rigid3d::Rotation(constant_data->gravity_alignment.inverse()));
  // 计算节点在世界坐标系下的位姿 εsj。
  // 这里是通过后端优化器 optimization_problem_ 中记录的旧图的全局和局部位姿进行转化计算的。
  // 前面两项相乘得到的是局部地图坐标系在世界坐标系下的位姿，
  // 再用它左乘第三项，最后得到节点在世界坐标系下的位姿。
  const transform::Rigid2d global_pose_2d =
      optimization_problem_->submap_data().at(matching_id).global_pose *      // 经过后端优化后的旧图的全局位姿
      constraints::ComputeSubmapPose(*insertion_submaps.front()).inverse() *  // 旧图在局部地图坐标系下位姿的逆
      local_pose_2d;                                                          // 节点在局部地图坐标系下的位置，不包含方向信息
  // 提供给后端优化器 optimization_problem_
  optimization_problem_->AddTrajectoryNode(
      matching_id.trajectory_id,
      optimization::NodeSpec2D{constant_data->time, local_pose_2d,
                               global_pose_2d,
                               constant_data->gravity_alignment});
  // 然后为新增的节点和新旧子图之间添加 INTRA_SUBMAP 类型的约束。
  // 遍历处理每一个 insertion_submaps，实际上只有2个。
  for (size_t i = 0; i < insertion_submaps.size(); ++i) {
    const SubmapId submap_id = submap_ids[i];
    // Even if this was the last node added to 'submap_id', the submap will
    // only be marked as finished in 'submap_data_' further below.
    // 即使这是添加到 "submap_id" 的最后一个节点，该子图也只会在下面的 "submap_data_" 中标记为 finished。
    CHECK(submap_data_.at(submap_id).state == SubmapState::kActive);  // 检查子图是否为激活状态
    // 把新添加的节点 node_id 加入到 submap_data_ 对应的子图数据中
    submap_data_.at(submap_id).node_ids.emplace(node_id);
    // 计算节点相对于子图的相对位姿 εij
    const transform::Rigid2d constraint_transform =
        constraints::ComputeSubmapPose(*insertion_submaps[i]).inverse() *  // 子图在局部地图坐标系下位姿的逆
        local_pose_2d;                                                     // 节点在局部地图坐标系下的位置，不包含方向信息
    // 把约束压入约束集合中，节点 node_id 和子图 submap_id 建立起 INTRA_SUBMAP 类型的约束。
    constraints_.push_back(Constraint{submap_id,
                                      node_id,
                                      {transform::Embed3D(constraint_transform),
                                       options_.matcher_translation_weight(),
                                       options_.matcher_rotation_weight()},
                                      Constraint::INTRA_SUBMAP});
  }

  // 紧接着遍历所有已经处于 kFinished 状态的子图，建立它们与新增节点之间可能的约束。
  for (const auto& submap_id_data : submap_data_) {
    if (submap_id_data.data.state == SubmapState::kFinished) {   // 筛选出处于 kFinished 状态的子图
      CHECK_EQ(submap_id_data.data.node_ids.count(node_id), 0);  // 检查该子图还没有跟该节点产生约束
      ComputeConstraint(node_id, submap_id_data.id);  // 计算该节点与子图的约束
    }
  }

  // 如果旧图切换到 kFinished 状态，则遍历所有已经进行过优化的节点，建立它们与旧图之间可能的约束。
  // newly_finished_submap 表示旧图是否结束更新了。
  if (newly_finished_submap) {
    // 获取旧图的索引，submap_ids 记录了新旧子图的索引。
    const SubmapId finished_submap_id = submap_ids.front();
    // 获取旧图的数据
    InternalSubmapData& finished_submap_data =
        submap_data_.at(finished_submap_id);
    // 检查旧图是否为 kActive 状态，然后再把它设置成 kFinished 状态
    CHECK(finished_submap_data.state == SubmapState::kActive);
    finished_submap_data.state = SubmapState::kFinished;
    // We have a new completed submap, so we look into adding constraints for
    // old nodes.
    // 我们有一个新完成的子图，因此我们考虑为旧节点添加约束。
    //
    // 计算旧图和所有已经进行过优化的节点的约束
    ComputeConstraintsForOldNodes(finished_submap_id);
  }
  // 最后通知约束构建器新增节点的操作结束，增加计数器 num_nodes_since_last_loop_closure_。
  constraint_builder_.NotifyEndOfNode();
  ++num_nodes_since_last_loop_closure_;
  CHECK(!run_loop_closure_);  // 检查没进行过 Loop Closure
  // 如果节点数增长到一定地步，则调用 DispatchOptimization()。
  // 调用的函数 DispatchOptimization() 对于工作队列的运转有着重要的作用。
  // optimize_every_n_nodes 在文件 "cartographer/configuration_files/pose_graph.lua" 中配置，默认数值为 90。
  if (options_.optimize_every_n_nodes() > 0 &&
      num_nodes_since_last_loop_closure_ > options_.optimize_every_n_nodes()) {
    DispatchOptimization();
  }
}

// 调度优化（即闭环检测）运行。
// 我们已经看到函数 AddWorkItem() 会根据工作队列是否存在，选择直接运行工作任务还是放到队列中等待以后执行。
// 当时留下了两个问题，其一这个工作队列是什么时候被创建的？其二队列里的任务是如何调度的？
// 这一切都是从函数 DispatchOptimization() 开始的。
void PoseGraph2D::DispatchOptimization() {
  run_loop_closure_ = true;  // 标识当前正在进行闭环检测
  // If there is a 'work_queue_' already, some other thread will take care.
  // 如果已经存在一个 "work_queue_"，那么其他一些线程将负责处理。
  //
  // 判定工作队列是否存在，如果不存在就创建一个对象。
  if (work_queue_ == nullptr) {
    work_queue_ = common::make_unique<std::deque<std::function<void()>>>();
    // 通过约束构建器的 WhenDone() 接口注册了一个回调函数 HandleWorkQueue()。
    // 当约束构建器 constraint_builder_ 在后台完成了一些工作之后，就会调用这个函数 HandleWorkQueue()，来调度队列里的任务。
    constraint_builder_.WhenDone(
        std::bind(&PoseGraph2D::HandleWorkQueue, this, std::placeholders::_1));
  }
}

common::Time PoseGraph2D::GetLatestNodeTime(const NodeId& node_id,
                                            const SubmapId& submap_id) const {
  common::Time time = trajectory_nodes_.at(node_id).constant_data->time;
  const InternalSubmapData& submap_data = submap_data_.at(submap_id);
  if (!submap_data.node_ids.empty()) {
    const NodeId last_submap_node_id =
        *submap_data_.at(submap_id).node_ids.rbegin();
    time = std::max(
        time, trajectory_nodes_.at(last_submap_node_id).constant_data->time);
  }
  return time;
}

void PoseGraph2D::UpdateTrajectoryConnectivity(const Constraint& constraint) {
  CHECK_EQ(constraint.tag, Constraint::INTER_SUBMAP);
  const common::Time time =
      GetLatestNodeTime(constraint.node_id, constraint.submap_id);
  trajectory_connectivity_state_.Connect(constraint.node_id.trajectory_id,
                                         constraint.submap_id.trajectory_id,
                                         time);
}

// 运行优化，执行修饰器并处理工作队列。
// 输入是由约束构建器 ConstraintBuilder2D 建立起来的约束向量。
void PoseGraph2D::HandleWorkQueue(
    const constraints::ConstraintBuilder2D::Result& result) {
  // 在函数的一开始，将构建的约束添加到容器 constraints_ 中，然后调用函数 RunOptimization() 进行优化。
  {
    // 在处理数据时加上互斥锁，防止出现数据访问错误
    common::MutexLocker locker(&mutex_);
    // 将构建的约束 result 添加到容器 constraints_ 中
    constraints_.insert(constraints_.end(), result.begin(), result.end());
  }
  // 调用函数 RunOptimization() 进行优化
  RunOptimization();

  // 如果提供了全局优化之后的回调函数，则调用回调函数。
  if (global_slam_optimization_callback_) {
    // 设置回调函数的两个参数
    // 准备 trajectory_id_to_last_optimized_submap_id 和 trajectory_id_to_last_optimized_node_id
    std::map<int, NodeId> trajectory_id_to_last_optimized_node_id;
    std::map<int, SubmapId> trajectory_id_to_last_optimized_submap_id;
    {
      common::MutexLocker locker(&mutex_);
      const auto& submap_data = optimization_problem_->submap_data();
      const auto& node_data = optimization_problem_->node_data();
      // 把 optimization_problem_ 中的 node 和 submap 的数据拷贝到两个参数中
      for (const int trajectory_id : node_data.trajectory_ids()) {
        trajectory_id_to_last_optimized_node_id[trajectory_id] =
            std::prev(node_data.EndOfTrajectory(trajectory_id))->id;
        trajectory_id_to_last_optimized_submap_id[trajectory_id] =
            std::prev(submap_data.EndOfTrajectory(trajectory_id))->id;
      }
    }
    // 调用该回调函数进行处理。猜测应该是更新一下地图结果之类的操作。
    global_slam_optimization_callback_(
        trajectory_id_to_last_optimized_submap_id,
        trajectory_id_to_last_optimized_node_id);
  }

  // 更新轨迹之间的连接关系，并调用修饰器完成地图的修饰。
  common::MutexLocker locker(&mutex_);
  for (const Constraint& constraint : result) {
    UpdateTrajectoryConnectivity(constraint);  // 更新轨迹之间的连接关系
  }
  // 调用 trimmers_ 中每一个 trimmer 的 Trim 函数进行处理。但还是不清楚这个 trimming 的含义是什么。
  TrimmingHandle trimming_handle(this);
  for (auto& trimmer : trimmers_) {
    trimmer->Trim(&trimming_handle);
  }
  // Trim 之后把他们从 trimmers_ 这个向量中清除，trimmers_ 将重新记录等待新一轮的 Loop Closure 过程中产生的数据
  trimmers_.erase(
      std::remove_if(trimmers_.begin(), trimmers_.end(),
                     [](std::unique_ptr<PoseGraphTrimmer>& trimmer) {
                       return trimmer->IsFinished();
                     }),
      trimmers_.end());

  // 此时，我们应当是已经完成了一次地图的后端优化。所以需要将计数器 num_nodes_since_last_loop_closure_ 清零，
  // 并更新 run_loop_closure_ 表示当前系统空闲没有进行闭环检测。
  num_nodes_since_last_loop_closure_ = 0;
  run_loop_closure_ = false;
  // 接着就在一个 for 循环中处理掉 work_queue_ 中所有等待的任务，这些任务主要是添加节点、添加传感器数据到位姿图中。
  while (!run_loop_closure_) {
    if (work_queue_->empty()) {
      // 如果工作队列为空，重置工作队列，返回
      work_queue_.reset();
      return;
    }
    // 否则不停取出队列最前端的任务进行处理，直至所有任务都处理完成
    work_queue_->front()();
    work_queue_->pop_front();
  }
  // 有时还没有完全处理完队列中的所有任务，就因为状态 run_loop_closure_ 再次为 true 开启新的闭环检测而退出。
  // 此时需要重新注册一下回调函数。
  LOG(INFO) << "Remaining work items in queue: " << work_queue_->size();
  // We have to optimize again.
  // 我们必须再次进行优化。
  constraint_builder_.WhenDone(
      std::bind(&PoseGraph2D::HandleWorkQueue, this, std::placeholders::_1));
}

void PoseGraph2D::WaitForAllComputations() {
  bool notification = false;
  common::MutexLocker locker(&mutex_);
  const int num_finished_nodes_at_start =
      constraint_builder_.GetNumFinishedNodes();
  while (!locker.AwaitWithTimeout(
      [this]() REQUIRES(mutex_) {
        return ((constraint_builder_.GetNumFinishedNodes() ==
                 num_trajectory_nodes_) &&
                !work_queue_);
      },
      common::FromSeconds(1.))) {
    std::ostringstream progress_info;
    progress_info << "Optimizing: " << std::fixed << std::setprecision(1)
                  << 100. *
                         (constraint_builder_.GetNumFinishedNodes() -
                          num_finished_nodes_at_start) /
                         (num_trajectory_nodes_ - num_finished_nodes_at_start)
                  << "%...";
    std::cout << "\r\x1b[K" << progress_info.str() << std::flush;
  }
  std::cout << "\r\x1b[KOptimizing: Done.     " << std::endl;
  constraint_builder_.WhenDone(
      [this,
       &notification](const constraints::ConstraintBuilder2D::Result& result) {
        common::MutexLocker locker(&mutex_);
        constraints_.insert(constraints_.end(), result.begin(), result.end());
        notification = true;
      });
  locker.Await([&notification]() { return notification; });
}

void PoseGraph2D::FinishTrajectory(const int trajectory_id) {
  common::MutexLocker locker(&mutex_);
  AddWorkItem([this, trajectory_id]() REQUIRES(mutex_) {
    CHECK_EQ(finished_trajectories_.count(trajectory_id), 0);
    finished_trajectories_.insert(trajectory_id);

    for (const auto& submap : submap_data_.trajectory(trajectory_id)) {
      submap_data_.at(submap.id).state = SubmapState::kFinished;
    }
    CHECK(!run_loop_closure_);
    DispatchOptimization();
  });
}

bool PoseGraph2D::IsTrajectoryFinished(const int trajectory_id) const {
  return finished_trajectories_.count(trajectory_id) > 0;
}

void PoseGraph2D::FreezeTrajectory(const int trajectory_id) {
  common::MutexLocker locker(&mutex_);
  trajectory_connectivity_state_.Add(trajectory_id);
  AddWorkItem([this, trajectory_id]() REQUIRES(mutex_) {
    CHECK_EQ(frozen_trajectories_.count(trajectory_id), 0);
    frozen_trajectories_.insert(trajectory_id);
  });
}

bool PoseGraph2D::IsTrajectoryFrozen(const int trajectory_id) const {
  return frozen_trajectories_.count(trajectory_id) > 0;
}

void PoseGraph2D::AddSubmapFromProto(
    const transform::Rigid3d& global_submap_pose, const proto::Submap& submap) {
  if (!submap.has_submap_2d()) {
    return;
  }

  const SubmapId submap_id = {submap.submap_id().trajectory_id(),
                              submap.submap_id().submap_index()};
  std::shared_ptr<const Submap2D> submap_ptr =
      std::make_shared<const Submap2D>(submap.submap_2d());
  const transform::Rigid2d global_submap_pose_2d =
      transform::Project2D(global_submap_pose);

  common::MutexLocker locker(&mutex_);
  AddTrajectoryIfNeeded(submap_id.trajectory_id);
  submap_data_.Insert(submap_id, InternalSubmapData());
  submap_data_.at(submap_id).submap = submap_ptr;
  // Immediately show the submap at the 'global_submap_pose'.
  global_submap_poses_.Insert(
      submap_id, optimization::SubmapSpec2D{global_submap_pose_2d});
  AddWorkItem([this, submap_id, global_submap_pose_2d]() REQUIRES(mutex_) {
    submap_data_.at(submap_id).state = SubmapState::kFinished;
    optimization_problem_->InsertSubmap(submap_id, global_submap_pose_2d);
  });
}

void PoseGraph2D::AddNodeFromProto(const transform::Rigid3d& global_pose,
                                   const proto::Node& node) {
  const NodeId node_id = {node.node_id().trajectory_id(),
                          node.node_id().node_index()};
  std::shared_ptr<const TrajectoryNode::Data> constant_data =
      std::make_shared<const TrajectoryNode::Data>(FromProto(node.node_data()));

  common::MutexLocker locker(&mutex_);
  AddTrajectoryIfNeeded(node_id.trajectory_id);
  trajectory_nodes_.Insert(node_id, TrajectoryNode{constant_data, global_pose});

  AddWorkItem([this, node_id, global_pose]() REQUIRES(mutex_) {
    const auto& constant_data = trajectory_nodes_.at(node_id).constant_data;
    const auto gravity_alignment_inverse = transform::Rigid3d::Rotation(
        constant_data->gravity_alignment.inverse());
    optimization_problem_->InsertTrajectoryNode(
        node_id,
        optimization::NodeSpec2D{
            constant_data->time,
            transform::Project2D(constant_data->local_pose *
                                 gravity_alignment_inverse),
            transform::Project2D(global_pose * gravity_alignment_inverse),
            constant_data->gravity_alignment});
  });
}

void PoseGraph2D::SetTrajectoryDataFromProto(
    const proto::TrajectoryData& data) {
  LOG(ERROR) << "not implemented";
}

void PoseGraph2D::AddNodeToSubmap(const NodeId& node_id,
                                  const SubmapId& submap_id) {
  common::MutexLocker locker(&mutex_);
  AddWorkItem([this, node_id, submap_id]() REQUIRES(mutex_) {
    submap_data_.at(submap_id).node_ids.insert(node_id);
  });
}

void PoseGraph2D::AddSerializedConstraints(
    const std::vector<Constraint>& constraints) {
  common::MutexLocker locker(&mutex_);
  AddWorkItem([this, constraints]() REQUIRES(mutex_) {
    for (const auto& constraint : constraints) {
      CHECK(trajectory_nodes_.Contains(constraint.node_id));
      CHECK(submap_data_.Contains(constraint.submap_id));
      CHECK(trajectory_nodes_.at(constraint.node_id).constant_data != nullptr);
      CHECK(submap_data_.at(constraint.submap_id).submap != nullptr);
      switch (constraint.tag) {
        case Constraint::Tag::INTRA_SUBMAP:
          CHECK(submap_data_.at(constraint.submap_id)
                    .node_ids.emplace(constraint.node_id)
                    .second);
          break;
        case Constraint::Tag::INTER_SUBMAP:
          UpdateTrajectoryConnectivity(constraint);
          break;
      }
      const Constraint::Pose pose = {
          constraint.pose.zbar_ij *
              transform::Rigid3d::Rotation(
                  trajectory_nodes_.at(constraint.node_id)
                      .constant_data->gravity_alignment.inverse()),
          constraint.pose.translation_weight, constraint.pose.rotation_weight};
      constraints_.push_back(Constraint{
          constraint.submap_id, constraint.node_id, pose, constraint.tag});
    }
    LOG(INFO) << "Loaded " << constraints.size() << " constraints.";
  });
}

// 添加修饰器。
// 其输入参数就是一个指向修饰器对象的智能指针。
void PoseGraph2D::AddTrimmer(std::unique_ptr<PoseGraphTrimmer> trimmer) {
  common::MutexLocker locker(&mutex_);
  // C++11 does not allow us to move a unique_ptr into a lambda.
  PoseGraphTrimmer* const trimmer_ptr = trimmer.release();
  /**
   * 1. 通过函数 AddWorkItem() 将一个 lambda 表达式添加到工作队列中。
   * 2. 这里的 lambda 表达式描述的是一个 void() 类型的函数，它没有返回值也没有参数列表。
   *    [capture list] 中获取的是在函数体中用到的一些变量。捕获 this 指针，是为了能够访问成员变量 trimmers_，
   *    而 trimmer_ptr 则是从输入参数中获取的修饰器对象指针。这个表达式的作用就是将传参的修饰器指针放入容器 trimmers_ 中。
   * 3. 但是 lambda 表达式并不会立即执行，它将被当做一个类似于函数指针或者仿函数这样的可执行对象
   *    传参给函数 AddWorkItem，在合适的条件下被调用。
   */ 
  AddWorkItem([this, trimmer_ptr]()
                  REQUIRES(mutex_) { trimmers_.emplace_back(trimmer_ptr); });
}

void PoseGraph2D::RunFinalOptimization() {
  {
    common::MutexLocker locker(&mutex_);
    AddWorkItem([this]() REQUIRES(mutex_) {
      optimization_problem_->SetMaxNumIterations(
          options_.max_num_final_iterations());
      DispatchOptimization();
    });
    AddWorkItem([this]() REQUIRES(mutex_) {
      optimization_problem_->SetMaxNumIterations(
          options_.optimization_problem_options()
              .ceres_solver_options()
              .max_num_iterations());
    });
  }
  WaitForAllComputations();
}

void PoseGraph2D::RunOptimization() {
  // 先检查一下是否给后端优化器喂过数据
  if (optimization_problem_->submap_data().empty()) {
    return;
  }

  // No other thread is accessing the optimization_problem_, constraints_,
  // frozen_trajectories_ and landmark_nodes_ when executing the Solve. Solve is
  // time consuming, so not taking the mutex before Solve to avoid blocking
  // foreground processing.
  // 程序运行到这里的时候，实际上没有其他线程访问对象 optimization_problem_, constraints_, frozen_trajectories_
  // 和 landmark_nodes_这四个对象。又因为这个 Solve 接口实在太耗时了，所以没有在该函数之前加锁，以防止阻塞其他任务。
  //
  // 通过后端优化器的接口 Solve 进行 SPA 优化。
  optimization_problem_->Solve(constraints_, frozen_trajectories_,
                               landmark_nodes_);
  common::MutexLocker locker(&mutex_);

  // 需要完成框图中 Global SLAM 的第三个任务，对在后端进行 SPA 优化过程中新增的节点的位姿进行调整，以适应优化后的世界地图和运动轨迹。
  // 获取后端优化器中子图和路径节点的数据，用临时对象 submap_data 和 node_data 记录之。
  // submap_data 是优化后的子图位姿，类型为 MapById<SubmapId, SubmapSpec2D>
  // node_data 是优化后的轨迹节点位姿，类型为 MapById<NodeId, NodeSpec2D>
  const auto& submap_data = optimization_problem_->submap_data();
  const auto& node_data = optimization_problem_->node_data();
  // 遍历所有的轨迹
  for (const int trajectory_id : node_data.trajectory_ids()) {
    // 先在一个 for 循环中遍历所有的节点，用优化后的位姿来更新轨迹点的世界坐标。
    // 即用优化后的轨迹节点的位姿 node_data 来更新轨迹节点集合 trajectory_nodes_ 中对应 NodeId 的节点位姿。
    for (const auto& node : node_data.trajectory(trajectory_id)) {
      auto& mutable_trajectory_node = trajectory_nodes_.at(node.id);
      mutable_trajectory_node.global_pose =
          transform::Embed3D(node.data.global_pose_2d) *
          transform::Rigid3d::Rotation(
              mutable_trajectory_node.constant_data->gravity_alignment);
    }

    // Extrapolate all point cloud poses that were not included in the
    // 'optimization_problem_' yet.
    // 外推所有尚未包含在“optimization_problem_”中的点云位姿
    //
    // 计算 SPA 优化前后的世界坐标变换关系，并将之左乘在后来新增的路径节点的全局位姿上，得到修正后的轨迹。
    const auto local_to_new_global =
        ComputeLocalToGlobalTransform(submap_data, trajectory_id);
    const auto local_to_old_global =
        ComputeLocalToGlobalTransform(global_submap_poses_, trajectory_id);
    // 计算 SPA 优化前后的世界坐标变换关系，old_global_to_new_global 表示优化前的全局位姿到优化后的全局位姿的变换
    const transform::Rigid3d old_global_to_new_global =
        local_to_new_global * local_to_old_global.inverse();

    // last_optimized_node_id 是优化后的轨迹节点的位姿 node_data 的最后一个元素的 id
    // std::prev 返回迭代器的上一个元素
    const NodeId last_optimized_node_id =
        std::prev(node_data.EndOfTrajectory(trajectory_id))->id;
    // node_it 是优化后的轨迹节点的位姿 node_data 的最后一个元素的下一个元素，即后来新增的还没有经过后端优化器处理的路径节点
    // std::next 返回迭代器的下一个元素
    auto node_it = std::next(trajectory_nodes_.find(last_optimized_node_id));
    // 遍历 trajectory_nodes_ 中后来新增的还没有经过后端优化器处理的路径节点
    for (; node_it != trajectory_nodes_.EndOfTrajectory(trajectory_id);
         ++node_it) {
      auto& mutable_trajectory_node = trajectory_nodes_.at(node_it->id);
      // 将 SPA 优化前后的世界坐标变换关系左乘在后来新增的路径节点的全局位姿上，得到修正后的轨迹。
      mutable_trajectory_node.global_pose =
          old_global_to_new_global * mutable_trajectory_node.global_pose;
    }
  }
  // 更新路标位姿
  for (const auto& landmark : optimization_problem_->landmark_data()) {
    landmark_nodes_[landmark.first].global_landmark_pose = landmark.second;
  }
  // 用成员变量 global_submap_poses_ 记录下当前的子图位姿
  global_submap_poses_ = submap_data;
}

MapById<NodeId, TrajectoryNode> PoseGraph2D::GetTrajectoryNodes() const {
  common::MutexLocker locker(&mutex_);
  return trajectory_nodes_;
}

MapById<NodeId, TrajectoryNodePose> PoseGraph2D::GetTrajectoryNodePoses()
    const {
  MapById<NodeId, TrajectoryNodePose> node_poses;
  common::MutexLocker locker(&mutex_);
  for (const auto& node_id_data : trajectory_nodes_) {
    common::optional<TrajectoryNodePose::ConstantPoseData> constant_pose_data;
    if (node_id_data.data.constant_data != nullptr) {
      constant_pose_data = TrajectoryNodePose::ConstantPoseData{
          node_id_data.data.constant_data->time,
          node_id_data.data.constant_data->local_pose};
    }
    node_poses.Insert(
        node_id_data.id,
        TrajectoryNodePose{node_id_data.data.global_pose, constant_pose_data});
  }
  return node_poses;
}

std::map<std::string, transform::Rigid3d> PoseGraph2D::GetLandmarkPoses()
    const {
  std::map<std::string, transform::Rigid3d> landmark_poses;
  common::MutexLocker locker(&mutex_);
  for (const auto& landmark : landmark_nodes_) {
    // Landmark without value has not been optimized yet.
    if (!landmark.second.global_landmark_pose.has_value()) continue;
    landmark_poses[landmark.first] =
        landmark.second.global_landmark_pose.value();
  }
  return landmark_poses;
}

void PoseGraph2D::SetLandmarkPose(const std::string& landmark_id,
                                  const transform::Rigid3d& global_pose) {
  common::MutexLocker locker(&mutex_);
  AddWorkItem([=]() REQUIRES(mutex_) {
    landmark_nodes_[landmark_id].global_landmark_pose = global_pose;
  });
}

sensor::MapByTime<sensor::ImuData> PoseGraph2D::GetImuData() const {
  common::MutexLocker locker(&mutex_);
  return optimization_problem_->imu_data();
}

sensor::MapByTime<sensor::OdometryData> PoseGraph2D::GetOdometryData() const {
  common::MutexLocker locker(&mutex_);
  return optimization_problem_->odometry_data();
}

std::map<std::string /* landmark ID */, PoseGraphInterface::LandmarkNode>
PoseGraph2D::GetLandmarkNodes() const {
  common::MutexLocker locker(&mutex_);
  return landmark_nodes_;
}

std::map<int, PoseGraphInterface::TrajectoryData>
PoseGraph2D::GetTrajectoryData() const {
  return {};  // Not implemented yet in 2D.
}

sensor::MapByTime<sensor::FixedFramePoseData>
PoseGraph2D::GetFixedFramePoseData() const {
  return {};  // Not implemented yet in 2D.
}

std::vector<PoseGraphInterface::Constraint> PoseGraph2D::constraints() const {
  std::vector<PoseGraphInterface::Constraint> result;
  common::MutexLocker locker(&mutex_);
  for (const Constraint& constraint : constraints_) {
    result.push_back(Constraint{
        constraint.submap_id, constraint.node_id,
        Constraint::Pose{constraint.pose.zbar_ij *
                             transform::Rigid3d::Rotation(
                                 trajectory_nodes_.at(constraint.node_id)
                                     .constant_data->gravity_alignment),
                         constraint.pose.translation_weight,
                         constraint.pose.rotation_weight},
        constraint.tag});
  }
  return result;
}

// 指定新的运动轨迹的起始位姿。
// 有四个输入参数：
// from_trajectory_id 是新轨迹的索引；
// to_trajectory_id 是参考轨迹的索引；
// pose 是新轨迹(from_trajectory_id)起点相对于参考轨迹(to_trajectory_id)的位姿；
// time 是添加新轨迹的时刻。
void PoseGraph2D::SetInitialTrajectoryPose(const int from_trajectory_id,
                                           const int to_trajectory_id,
                                           const transform::Rigid3d& pose,
                                           const common::Time time) {
  common::MutexLocker locker(&mutex_);
  // 在函数中，直接根据后三个参数创建一个 InitialTrajectoryPose 类型的对象，放置到容器 initial_trajectory_poses_ 中。
  // initial_trajectory_poses_ 是一个 std::map<int, InitialTrajectoryPose> 类型的容器，记录所有轨迹的初始位姿。
  initial_trajectory_poses_[from_trajectory_id] =
      InitialTrajectoryPose{to_trajectory_id, pose, time};
}

transform::Rigid3d PoseGraph2D::GetInterpolatedGlobalTrajectoryPose(
    const int trajectory_id, const common::Time time) const {
  CHECK_GT(trajectory_nodes_.SizeOfTrajectoryOrZero(trajectory_id), 0);
  const auto it = trajectory_nodes_.lower_bound(trajectory_id, time);
  if (it == trajectory_nodes_.BeginOfTrajectory(trajectory_id)) {
    return trajectory_nodes_.BeginOfTrajectory(trajectory_id)->data.global_pose;
  }
  if (it == trajectory_nodes_.EndOfTrajectory(trajectory_id)) {
    return std::prev(trajectory_nodes_.EndOfTrajectory(trajectory_id))
        ->data.global_pose;
  }
  return transform::Interpolate(
             transform::TimestampedTransform{std::prev(it)->data.time(),
                                             std::prev(it)->data.global_pose},
             transform::TimestampedTransform{it->data.time(),
                                             it->data.global_pose},
             time)
      .transform;
}

transform::Rigid3d PoseGraph2D::GetLocalToGlobalTransform(
    const int trajectory_id) const {
  common::MutexLocker locker(&mutex_);
  return ComputeLocalToGlobalTransform(global_submap_poses_, trajectory_id);
}

std::vector<std::vector<int>> PoseGraph2D::GetConnectedTrajectories() const {
  return trajectory_connectivity_state_.Components();
}

PoseGraphInterface::SubmapData PoseGraph2D::GetSubmapData(
    const SubmapId& submap_id) const {
  common::MutexLocker locker(&mutex_);
  return GetSubmapDataUnderLock(submap_id);
}

MapById<SubmapId, PoseGraphInterface::SubmapData>
PoseGraph2D::GetAllSubmapData() const {
  common::MutexLocker locker(&mutex_);
  return GetSubmapDataUnderLock();
}

MapById<SubmapId, PoseGraphInterface::SubmapPose>
PoseGraph2D::GetAllSubmapPoses() const {
  common::MutexLocker locker(&mutex_);
  MapById<SubmapId, SubmapPose> submap_poses;
  for (const auto& submap_id_data : submap_data_) {
    auto submap_data = GetSubmapDataUnderLock(submap_id_data.id);
    submap_poses.Insert(
        submap_id_data.id,
        PoseGraph::SubmapPose{submap_data.submap->num_range_data(),
                              submap_data.pose});
  }
  return submap_poses;
}

transform::Rigid3d PoseGraph2D::ComputeLocalToGlobalTransform(
    const MapById<SubmapId, optimization::SubmapSpec2D>& global_submap_poses,
    const int trajectory_id) const {
  auto begin_it = global_submap_poses.BeginOfTrajectory(trajectory_id);
  auto end_it = global_submap_poses.EndOfTrajectory(trajectory_id);
  if (begin_it == end_it) {
    const auto it = initial_trajectory_poses_.find(trajectory_id);
    if (it != initial_trajectory_poses_.end()) {
      return GetInterpolatedGlobalTrajectoryPose(it->second.to_trajectory_id,
                                                 it->second.time) *
             it->second.relative_pose;
    } else {
      return transform::Rigid3d::Identity();
    }
  }
  const SubmapId last_optimized_submap_id = std::prev(end_it)->id;
  // Accessing 'local_pose' in Submap is okay, since the member is const.
  return transform::Embed3D(
             global_submap_poses.at(last_optimized_submap_id).global_pose) *
         submap_data_.at(last_optimized_submap_id)
             .submap->local_pose()
             .inverse();
}

PoseGraphInterface::SubmapData PoseGraph2D::GetSubmapDataUnderLock(
    const SubmapId& submap_id) const {
  const auto it = submap_data_.find(submap_id);
  if (it == submap_data_.end()) {
    return {};
  }
  auto submap = it->data.submap;
  if (global_submap_poses_.Contains(submap_id)) {
    // We already have an optimized pose.
    return {submap,
            transform::Embed3D(global_submap_poses_.at(submap_id).global_pose)};
  }
  // We have to extrapolate.
  return {submap, ComputeLocalToGlobalTransform(global_submap_poses_,
                                                submap_id.trajectory_id) *
                      submap->local_pose()};
}

PoseGraph2D::TrimmingHandle::TrimmingHandle(PoseGraph2D* const parent)
    : parent_(parent) {}

int PoseGraph2D::TrimmingHandle::num_submaps(const int trajectory_id) const {
  const auto& submap_data = parent_->optimization_problem_->submap_data();
  return submap_data.SizeOfTrajectoryOrZero(trajectory_id);
}

MapById<SubmapId, PoseGraphInterface::SubmapData>
PoseGraph2D::TrimmingHandle::GetOptimizedSubmapData() const {
  MapById<SubmapId, PoseGraphInterface::SubmapData> submaps;
  for (const auto& submap_id_data : parent_->submap_data_) {
    if (submap_id_data.data.state != SubmapState::kFinished ||
        !parent_->global_submap_poses_.Contains(submap_id_data.id)) {
      continue;
    }
    submaps.Insert(submap_id_data.id,
                   SubmapData{submap_id_data.data.submap,
                              transform::Embed3D(parent_->global_submap_poses_
                                                     .at(submap_id_data.id)
                                                     .global_pose)});
  }
  return submaps;
}

std::vector<SubmapId> PoseGraph2D::TrimmingHandle::GetSubmapIds(
    int trajectory_id) const {
  std::vector<SubmapId> submap_ids;
  const auto& submap_data = parent_->optimization_problem_->submap_data();
  for (const auto& it : submap_data.trajectory(trajectory_id)) {
    submap_ids.push_back(it.id);
  }
  return submap_ids;
}

const MapById<NodeId, TrajectoryNode>&
PoseGraph2D::TrimmingHandle::GetTrajectoryNodes() const {
  return parent_->trajectory_nodes_;
}

const std::vector<PoseGraphInterface::Constraint>&
PoseGraph2D::TrimmingHandle::GetConstraints() const {
  return parent_->constraints_;
}

bool PoseGraph2D::TrimmingHandle::IsFinished(const int trajectory_id) const {
  return parent_->IsTrajectoryFinished(trajectory_id);
}

void PoseGraph2D::TrimmingHandle::MarkSubmapAsTrimmed(
    const SubmapId& submap_id) {
  // TODO(hrapp): We have to make sure that the trajectory has been finished
  // if we want to delete the last submaps.
  CHECK(parent_->submap_data_.at(submap_id).state == SubmapState::kFinished);

  // Compile all nodes that are still INTRA_SUBMAP constrained once the submap
  // with 'submap_id' is gone.
  std::set<NodeId> nodes_to_retain;
  for (const Constraint& constraint : parent_->constraints_) {
    if (constraint.tag == Constraint::Tag::INTRA_SUBMAP &&
        constraint.submap_id != submap_id) {
      nodes_to_retain.insert(constraint.node_id);
    }
  }
  // Remove all 'constraints_' related to 'submap_id'.
  std::set<NodeId> nodes_to_remove;
  {
    std::vector<Constraint> constraints;
    for (const Constraint& constraint : parent_->constraints_) {
      if (constraint.submap_id == submap_id) {
        if (constraint.tag == Constraint::Tag::INTRA_SUBMAP &&
            nodes_to_retain.count(constraint.node_id) == 0) {
          // This node will no longer be INTRA_SUBMAP contrained and has to be
          // removed.
          nodes_to_remove.insert(constraint.node_id);
        }
      } else {
        constraints.push_back(constraint);
      }
    }
    parent_->constraints_ = std::move(constraints);
  }
  // Remove all 'constraints_' related to 'nodes_to_remove'.
  {
    std::vector<Constraint> constraints;
    for (const Constraint& constraint : parent_->constraints_) {
      if (nodes_to_remove.count(constraint.node_id) == 0) {
        constraints.push_back(constraint);
      }
    }
    parent_->constraints_ = std::move(constraints);
  }

  // Mark the submap with 'submap_id' as trimmed and remove its data.
  CHECK(parent_->submap_data_.at(submap_id).state == SubmapState::kFinished);
  parent_->submap_data_.Trim(submap_id);
  parent_->constraint_builder_.DeleteScanMatcher(submap_id);
  parent_->optimization_problem_->TrimSubmap(submap_id);

  // Remove the 'nodes_to_remove' from the pose graph and the optimization
  // problem.
  for (const NodeId& node_id : nodes_to_remove) {
    parent_->trajectory_nodes_.Trim(node_id);
    parent_->optimization_problem_->TrimTrajectoryNode(node_id);
  }
}

MapById<SubmapId, PoseGraphInterface::SubmapData>
PoseGraph2D::GetSubmapDataUnderLock() const {
  MapById<SubmapId, PoseGraphInterface::SubmapData> submaps;
  for (const auto& submap_id_data : submap_data_) {
    submaps.Insert(submap_id_data.id,
                   GetSubmapDataUnderLock(submap_id_data.id));
  }
  return submaps;
}

void PoseGraph2D::SetGlobalSlamOptimizationCallback(
    PoseGraphInterface::GlobalSlamOptimizationCallback callback) {
  global_slam_optimization_callback_ = callback;
}

}  // namespace mapping
}  // namespace cartographer
