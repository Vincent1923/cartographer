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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_2D_POSE_GRAPH_2D_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_2D_POSE_GRAPH_2D_H_

#include <deque>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <unordered_map>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/common/fixed_ratio_sampler.h"
#include "cartographer/common/mutex.h"
#include "cartographer/common/thread_pool.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/mapping/internal/constraints/constraint_builder_2d.h"
#include "cartographer/mapping/internal/optimization/optimization_problem_2d.h"
#include "cartographer/mapping/internal/trajectory_connectivity_state.h"
#include "cartographer/mapping/pose_graph.h"
#include "cartographer/mapping/pose_graph_trimmer.h"
#include "cartographer/sensor/fixed_frame_pose_data.h"
#include "cartographer/sensor/landmark_data.h"
#include "cartographer/sensor/odometry_data.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"

namespace cartographer {
namespace mapping {

// Implements the loop closure method called Sparse Pose Adjustment (SPA) from
// Konolige, Kurt, et al. "Efficient sparse pose adjustment for 2d mapping."
// Intelligent Robots and Systems (IROS), 2010 IEEE/RSJ International Conference
// on (pp. 22--29). IEEE, 2010.
//
// It is extended for submapping:
// Each node has been matched against one or more submaps (adding a constraint
// for each match), both poses of nodes and of submaps are to be optimized.
// All constraints are between a submap i and a node j.
//
// 它被扩展用于构建子图：
// 每个节点已经针对一个或多个子图进行了匹配（为每个匹配添加了约束），节点和子图的位姿都将得到优化。
// 所有约束都在子图i和节点j之间。
class PoseGraph2D : public PoseGraph {
 public:
  PoseGraph2D(
      const proto::PoseGraphOptions& options,
      std::unique_ptr<optimization::OptimizationProblem2D> optimization_problem,
      common::ThreadPool* thread_pool);
  ~PoseGraph2D() override;

  PoseGraph2D(const PoseGraph2D&) = delete;
  PoseGraph2D& operator=(const PoseGraph2D&) = delete;

  // Adds a new node with 'constant_data'. Its 'constant_data->local_pose' was
  // determined by scan matching against 'insertion_submaps.front()' and the
  // node data was inserted into the 'insertion_submaps'. If
  // 'insertion_submaps.front().finished()' is 'true', data was inserted into
  // this submap for the last time.
  NodeId AddNode(
      std::shared_ptr<const TrajectoryNode::Data> constant_data,
      int trajectory_id,
      const std::vector<std::shared_ptr<const Submap2D>>& insertion_submaps)
      EXCLUDES(mutex_);

  void AddImuData(int trajectory_id, const sensor::ImuData& imu_data) override
      EXCLUDES(mutex_);
  void AddOdometryData(int trajectory_id,
                       const sensor::OdometryData& odometry_data) override
      EXCLUDES(mutex_);
  void AddFixedFramePoseData(
      int trajectory_id,
      const sensor::FixedFramePoseData& fixed_frame_pose_data) override
      EXCLUDES(mutex_);
  void AddLandmarkData(int trajectory_id,
                       const sensor::LandmarkData& landmark_data) override
      EXCLUDES(mutex_);

  void FinishTrajectory(int trajectory_id) override;
  bool IsTrajectoryFinished(int trajectory_id) const override REQUIRES(mutex_);
  void FreezeTrajectory(int trajectory_id) override;
  bool IsTrajectoryFrozen(int trajectory_id) const override REQUIRES(mutex_);
  void AddSubmapFromProto(const transform::Rigid3d& global_submap_pose,
                          const proto::Submap& submap) override;
  void AddNodeFromProto(const transform::Rigid3d& global_pose,
                        const proto::Node& node) override;
  void SetTrajectoryDataFromProto(const proto::TrajectoryData& data) override;
  void AddNodeToSubmap(const NodeId& node_id,
                       const SubmapId& submap_id) override;
  void AddSerializedConstraints(
      const std::vector<Constraint>& constraints) override;
  void AddTrimmer(std::unique_ptr<PoseGraphTrimmer> trimmer) override;
  void RunFinalOptimization() override;
  std::vector<std::vector<int>> GetConnectedTrajectories() const override;
  PoseGraphInterface::SubmapData GetSubmapData(const SubmapId& submap_id) const
      EXCLUDES(mutex_) override;
  MapById<SubmapId, PoseGraphInterface::SubmapData> GetAllSubmapData() const
      EXCLUDES(mutex_) override;
  MapById<SubmapId, SubmapPose> GetAllSubmapPoses() const
      EXCLUDES(mutex_) override;
  transform::Rigid3d GetLocalToGlobalTransform(int trajectory_id) const
      EXCLUDES(mutex_) override;
  MapById<NodeId, TrajectoryNode> GetTrajectoryNodes() const override
      EXCLUDES(mutex_);
  MapById<NodeId, TrajectoryNodePose> GetTrajectoryNodePoses() const override
      EXCLUDES(mutex_);
  std::map<std::string, transform::Rigid3d> GetLandmarkPoses() const override
      EXCLUDES(mutex_);
  void SetLandmarkPose(const std::string& landmark_id,
                       const transform::Rigid3d& global_pose) override
      EXCLUDES(mutex_);
  sensor::MapByTime<sensor::ImuData> GetImuData() const override
      EXCLUDES(mutex_);
  sensor::MapByTime<sensor::OdometryData> GetOdometryData() const override
      EXCLUDES(mutex_);
  sensor::MapByTime<sensor::FixedFramePoseData> GetFixedFramePoseData()
      const override EXCLUDES(mutex_);
  std::map<std::string /* landmark ID */, PoseGraph::LandmarkNode>
  GetLandmarkNodes() const override EXCLUDES(mutex_);
  std::map<int, TrajectoryData> GetTrajectoryData() const override
      EXCLUDES(mutex_);
  std::vector<Constraint> constraints() const override EXCLUDES(mutex_);
  void SetInitialTrajectoryPose(int from_trajectory_id, int to_trajectory_id,
                                const transform::Rigid3d& pose,
                                const common::Time time) override
      EXCLUDES(mutex_);
  void SetGlobalSlamOptimizationCallback(
      PoseGraphInterface::GlobalSlamOptimizationCallback callback) override;
  transform::Rigid3d GetInterpolatedGlobalTrajectoryPose(
      int trajectory_id, const common::Time time) const REQUIRES(mutex_);

 private:
  // The current state of the submap in the background threads. When this
  // transitions to kFinished, all nodes are tried to match against this submap.
  // Likewise, all new nodes are matched against submaps which are finished.
  //
  // 子图在后台线程中的当前状态。当此过渡到 kFinished 时，将尝试所有节点与此子图进行匹配。
  // 同样，所有新节点都与完成的子图匹配。
  //
  // 一个枚举类，表征一个 Submap 的状态：kActive 或 kFinished。
  // 当一个 submap 由 Active 转为 kFinished时，所有的 Nodes 都要跟该 submap 做一个匹配。
  // 同样的，当在 trajectory 增长的过程中又有新增的 Nodes，这些新的 Nodes 也需要跟所有已经 finished 的 submap 做一下 match。
  enum class SubmapState { kActive, kFinished };
  // 存储与一个 Submap 相关联的 Node
  struct InternalSubmapData {
    std::shared_ptr<const Submap2D> submap;  // 共享指针 submap 记录了具体的子图对象

    // IDs of the nodes that were inserted into this map together with
    // constraints for them. They are not to be matched again when this submap
    // becomes 'finished'.
    // 插入到此地图中的节点索引及其约束。当该子图“完成”时，将不再匹配它们。
    //
    // 所有跟该 submap 有插入关系并存在约束的节点的ID的集合。当这个 submap 被 finished 时他们
    // 就不用重新跟该 submap 进行 match，而只需 match 新的 node 并把他们加入相应 submap 的这个集合中。
    std::set<NodeId> node_ids;  // 容器 node_ids 记录了所有直接插入 submap 的节点

    // 默认情况，submap 是 kActive，除非满足一定条件后把它置为 kFinished。
    //
    // 子图的状态主要是给后台的线程提供的。一开始子图的状态都是 kActive 的，当它切换到 kFinished 的状态下后
    // 就会与所有的节点进行一次扫描匹配操作。此外新增的节点也会与所有 kFinished 状态的子图进行扫描匹配。
    // 这一操作我们可以理解为是进行闭环检测，通过遍历与所有的 kFinished 状态的子图，或者节点，
    // 应当可以找到发生闭环的地点并建立一个约束来描述。
    SubmapState state = SubmapState::kActive;  // 枚举 state 记录了子图的状态
  };

  MapById<SubmapId, PoseGraphInterface::SubmapData> GetSubmapDataUnderLock()
      const REQUIRES(mutex_);

  // Handles a new work item.
  void AddWorkItem(const std::function<void()>& work_item) REQUIRES(mutex_);

  // Adds connectivity and sampler for a trajectory if it does not exist.
  void AddTrajectoryIfNeeded(int trajectory_id) REQUIRES(mutex_);

  // Grows the optimization problem to have an entry for every element of
  // 'insertion_submaps'. Returns the IDs for the 'insertion_submaps'.
  std::vector<SubmapId> InitializeGlobalSubmapPoses(
      int trajectory_id, const common::Time time,
      const std::vector<std::shared_ptr<const Submap2D>>& insertion_submaps)
      REQUIRES(mutex_);

  // Adds constraints for a node, and starts scan matching in the background.
  void ComputeConstraintsForNode(
      const NodeId& node_id,
      std::vector<std::shared_ptr<const Submap2D>> insertion_submaps,
      bool newly_finished_submap) REQUIRES(mutex_);

  // Computes constraints for a node and submap pair.
  void ComputeConstraint(const NodeId& node_id, const SubmapId& submap_id)
      REQUIRES(mutex_);

  // Adds constraints for older nodes whenever a new submap is finished.
  void ComputeConstraintsForOldNodes(const SubmapId& submap_id)
      REQUIRES(mutex_);

  // Runs the optimization, executes the trimmers and processes the work queue.
  void HandleWorkQueue(const constraints::ConstraintBuilder2D::Result& result)
      REQUIRES(mutex_);

  // Waits until we caught up (i.e. nothing is waiting to be scheduled), and
  // all computations have finished.
  void WaitForAllComputations() EXCLUDES(mutex_);

  // Runs the optimization. Callers have to make sure, that there is only one
  // optimization being run at a time.
  void RunOptimization() EXCLUDES(mutex_);

  // Computes the local to global map frame transform based on the given
  // 'global_submap_poses'.
  transform::Rigid3d ComputeLocalToGlobalTransform(
      const MapById<SubmapId, optimization::SubmapSpec2D>& global_submap_poses,
      int trajectory_id) const REQUIRES(mutex_);

  SubmapData GetSubmapDataUnderLock(const SubmapId& submap_id) const
      REQUIRES(mutex_);

  common::Time GetLatestNodeTime(const NodeId& node_id,
                                 const SubmapId& submap_id) const
      REQUIRES(mutex_);

  // Updates the trajectory connectivity structure with a new constraint.
  void UpdateTrajectoryConnectivity(const Constraint& constraint)
      REQUIRES(mutex_);

  const proto::PoseGraphOptions options_;  // 位姿图的各种配置
  GlobalSlamOptimizationCallback global_slam_optimization_callback_;  // 完成全局优化后的回调函数
  mutable common::Mutex mutex_;  // 用于多线程运行时保护重要数据资源的互斥量

  // If it exists, further work items must be added to this queue, and will be
  // considered later.
  // 这是一个智能指针形式的工作队列，用于记录将要完成的任务
  std::unique_ptr<std::deque<std::function<void()>>> work_queue_
      GUARDED_BY(mutex_);

  // How our various trajectories are related.
  // 描述不同轨迹之间的连接状态
  TrajectoryConnectivityState trajectory_connectivity_state_;

  // We globally localize a fraction of the nodes from each trajectory.
  // 这应该是一个以 trajectory_id 为索引的字典，用于对各个轨迹上的部分节点进行全局定位。
  std::unordered_map<int, std::unique_ptr<common::FixedRatioSampler>>
      global_localization_samplers_ GUARDED_BY(mutex_);

  // Number of nodes added since last loop closure.
  // 一个计数器，记录了自从上次闭环检测之后新增的节点数量。
  int num_nodes_since_last_loop_closure_ GUARDED_BY(mutex_) = 0;

  // Whether the optimization has to be run before more data is added.
  // 标识当前是否正在进行闭环检测。
  bool run_loop_closure_ GUARDED_BY(mutex_) = false;

  // Schedules optimization (i.e. loop closure) to run.
  void DispatchOptimization() REQUIRES(mutex_);

  // Current optimization problem.
  // 描述当前优化问题的对象，应该是 PoseGraph2D 的核心。
  std::unique_ptr<optimization::OptimizationProblem2D> optimization_problem_;
  // 约束构造器，用于异步的计算约束。
  constraints::ConstraintBuilder2D constraint_builder_ GUARDED_BY(mutex_);
  // 记录了位姿图中的所有约束。
  std::vector<Constraint> constraints_ GUARDED_BY(mutex_);

  // Submaps get assigned an ID and state as soon as they are seen, even
  // before they take part in the background computations.
  //
  // 一个 submap 一旦建立，就会立刻分配一个ID和状态{kActive, kFinished}。submap_data_ 就是存储一个 submap 相关信息。
  // MapById 定义在“/mapping/id.h”中，是一个工具类，方便管理。
  //
  // 该容器记录了所有的子图数据及其内部节点，其中 MapById 是对 std::map 的一个封装，
  // InternalSubmapData 除了描述了子图的数据之外还记录了所有内部的节点。
  MapById<SubmapId, InternalSubmapData> submap_data_ GUARDED_BY(mutex_);

  // Data that are currently being shown.
  // 记录轨迹节点的容器
  MapById<NodeId, TrajectoryNode> trajectory_nodes_ GUARDED_BY(mutex_);
  // 节点数
  int num_trajectory_nodes_ GUARDED_BY(mutex_) = 0;

  // Global submap poses currently used for displaying data.
  // 用于显示数据的当前全局子图位姿。会用于在 rviz 中显示建图效果。
  // 看源码中的注释说该容器主要是为了可视化准备的。那个 SubmapSpec2D 是一个结构体，实际上只有一个字段记录了一个全局位姿。
  MapById<SubmapId, optimization::SubmapSpec2D> global_submap_poses_
      GUARDED_BY(mutex_);

  // Global landmark poses with all observations.
  // 存储所有 landmark 的 Id 及他们的观测数据。
  // 记录路标点的容器。
  std::map<std::string /* landmark ID */, PoseGraph::LandmarkNode>
      landmark_nodes_ GUARDED_BY(mutex_);

  // List of all trimmers to consult when optimizations finish.
  // 用于指导修饰地图的修饰器。在 map_builder 的接口实现一文中我们曾见到过添加修饰器的操作。
  std::vector<std::unique_ptr<PoseGraphTrimmer>> trimmers_ GUARDED_BY(mutex_);

  // Set of all frozen trajectories not being optimized.
  // 记录所有当前冻结的轨迹。
  std::set<int> frozen_trajectories_ GUARDED_BY(mutex_);

  // Set of all finished trajectories.
  // 记录所有已经完结的轨迹。
  std::set<int> finished_trajectories_ GUARDED_BY(mutex_);

  // Set of all initial trajectory poses.
  // 记录所有轨迹的初始位姿。
  std::map<int, InitialTrajectoryPose> initial_trajectory_poses_
      GUARDED_BY(mutex_);

  // Allows querying and manipulating the pose graph by the 'trimmers_'. The
  // 'mutex_' of the pose graph is held while this class is used.
  class TrimmingHandle : public Trimmable {
   public:
    TrimmingHandle(PoseGraph2D* parent);
    ~TrimmingHandle() override {}

    int num_submaps(int trajectory_id) const override;
    std::vector<SubmapId> GetSubmapIds(int trajectory_id) const override;
    MapById<SubmapId, SubmapData> GetOptimizedSubmapData() const override
        REQUIRES(parent_->mutex_);
    const MapById<NodeId, TrajectoryNode>& GetTrajectoryNodes() const override
        REQUIRES(parent_->mutex_);
    const std::vector<Constraint>& GetConstraints() const override
        REQUIRES(parent_->mutex_);
    void MarkSubmapAsTrimmed(const SubmapId& submap_id)
        REQUIRES(parent_->mutex_) override;
    bool IsFinished(int trajectory_id) const override REQUIRES(parent_->mutex_);

   private:
    PoseGraph2D* const parent_;
  };
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_2D_POSE_GRAPH_2D_H_
