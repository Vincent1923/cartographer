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

#ifndef CARTOGRAPHER_MAPPING_POSE_GRAPH_INTERFACE_H_
#define CARTOGRAPHER_MAPPING_POSE_GRAPH_INTERFACE_H_

#include <vector>

#include "cartographer/common/optional.h"
#include "cartographer/mapping/id.h"
#include "cartographer/mapping/submaps.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace mapping {

class PoseGraphInterface {
 public:
  // A "constraint" as in the paper by Konolige, Kurt, et al. "Efficient sparse
  // pose adjustment for 2d mapping." Intelligent Robots and Systems (IROS),
  // 2010 IEEE/RSJ International Conference on (pp. 22--29). IEEE, 2010.
  //
  // Constraint 描述节点j相对于子图i的约束
  struct Constraint {
    struct Pose {
      transform::Rigid3d zbar_ij;  // 第j个节点相对于第i个子图的相对位姿
      // 以下两个变量描述相对位姿 zbar_ij 的可信度
      double translation_weight;   // 平移的权重
      double rotation_weight;      // 旋转的权重
    };

    // SubmapId 和 NodeId 都是定义在文件"id.h"中的结构体。它们有一个相同的字段 trajectory_id，用于标记当前跟踪的轨迹。
    // 各自有一个从0开始计数的 submap_index 和 node_index，分别为每个子图和节点提供一个唯一的编号。
    SubmapId submap_id;  // 'i' in the paper. 记录约束对应的子图索引（包括 trajectory_id 和 index）
    NodeId node_id;      // 'j' in the paper. 记录节点索引（包括 trajectory_id 和 index）

    // Pose of the node 'j' relative to submap 'i'.
    // 第j个节点相对于第i个子图的相对位姿
    Pose pose;

    // Differentiates between intra-submap (where node 'j' was inserted into
    // submap 'i') and inter-submap constraints (where node 'j' was not inserted
    // into submap 'i').
    //
    // 在 Cartographer 中有两类约束，被称为子图内约束(INTRA_SUBMAP)和子图间(INTER_SUBMAP)约束。
    // INTRA_SUBMAP 的约束是指在子图的更新过程中节点j被直接插入到子图i中。
    // 而 INTER_SUBMAP 类型的约束中节点j并不是直接插入到子图i中，我猜应该是因为闭环检测而添加的约束。
    enum Tag { INTRA_SUBMAP, INTER_SUBMAP } tag;
  };

  struct LandmarkNode {
    struct LandmarkObservation {
      int trajectory_id;
      common::Time time;
      transform::Rigid3d landmark_to_tracking_transform;
      double translation_weight;
      double rotation_weight;
    };
    std::vector<LandmarkObservation> landmark_observations;
    common::optional<transform::Rigid3d> global_landmark_pose;
  };

  struct SubmapPose {
    int version;
    transform::Rigid3d pose;  // submap 的绝对位姿
  };

  struct SubmapData {
    std::shared_ptr<const Submap> submap;  // 栅格概率图数据
    transform::Rigid3d pose;  // submap 的绝对位姿
  };

  struct TrajectoryData {
    double gravity_constant = 9.8;
    std::array<double, 4> imu_calibration{{1., 0., 0., 0.}};
    common::optional<transform::Rigid3d> fixed_frame_origin_in_map;
  };

  // 全局优化的回调函数的宏定义。
  // 满足一定条件则会调用该回调函数进行优化。可以看到，该函数传入的参数是一系列的子图索引 SubmapId 和节点索引 NodeId。
  using GlobalSlamOptimizationCallback =
      std::function<void(const std::map<int /* trajectory_id */, SubmapId>&,
                         const std::map<int /* trajectory_id */, NodeId>&)>;

  PoseGraphInterface() {}
  virtual ~PoseGraphInterface() {}

  PoseGraphInterface(const PoseGraphInterface&) = delete;
  PoseGraphInterface& operator=(const PoseGraphInterface&) = delete;

  // Waits for all computations to finish and computes optimized poses.
  // 等待所有计算完成并计算优化的位姿。
  virtual void RunFinalOptimization() = 0;

  // Returns data for all submaps.
  // 返回所有子图的数据。
  virtual MapById<SubmapId, SubmapData> GetAllSubmapData() const = 0;

  // Returns the global poses for all submaps.
  // 返回所有子图的全局位姿。
  virtual MapById<SubmapId, SubmapPose> GetAllSubmapPoses() const = 0;

  // Returns the transform converting data in the local map frame (i.e. the
  // continuous, non-loop-closed frame) into the global map frame (i.e. the
  // discontinuous, loop-closed frame).
  // 将局部地图坐标系（即连续的非闭环坐标系）中的转换数据返回到全局地图坐标系（即不连续的闭环坐标系）。
  //
  // 获取由局部坐标系到世界坐标系的变换矩阵
  virtual transform::Rigid3d GetLocalToGlobalTransform(
      int trajectory_id) const = 0;

  // Returns the current optimized trajectories.
  // 返回当前经过优化后的轨迹。
  //
  // 返回当前经过优化后的轨迹上的所有节点。这些节点构成了这条轨迹。
  virtual MapById<NodeId, TrajectoryNode> GetTrajectoryNodes() const = 0;

  // Returns the current optimized trajectory poses.
  // 返回当前经过优化后的所有的节点位姿。
  virtual MapById<NodeId, TrajectoryNodePose> GetTrajectoryNodePoses()
      const = 0;

  // Returns the current optimized landmark poses.
  // 返回当前经过优化后的 landmark 位姿
  virtual std::map<std::string, transform::Rigid3d> GetLandmarkPoses()
      const = 0;

  // Sets global pose of landmark 'landmark_id' to given 'global_pose'.
  // 将 landmark “landmark_id”的全局位姿设置为给定的“global_pose”。
  virtual void SetLandmarkPose(const std::string& landmark_id,
                               const transform::Rigid3d& global_pose) = 0;

  // Checks if the given trajectory is finished.
  // 检查给定的轨迹是否完成。
  virtual bool IsTrajectoryFinished(int trajectory_id) const = 0;

  // Checks if the given trajectory is frozen.
  // 检查给定的轨迹是否被冻结。
  virtual bool IsTrajectoryFrozen(int trajectory_id) const = 0;

  // Returns the trajectory data.
  // 返回 TrajectoryData
  virtual std::map<int, TrajectoryData> GetTrajectoryData() const = 0;

  // Returns the collection of constraints.
  // 返回约束的集合。
  virtual std::vector<Constraint> constraints() const = 0;

  // Serializes the constraints and trajectories.
  // 序列化约束和轨迹。
  virtual proto::PoseGraph ToProto() const = 0;

  // Sets the callback function that is invoked whenever the global optimization
  // problem is solved.
  // 设置在解决全局优化问题时调用的回调函数。
  virtual void SetGlobalSlamOptimizationCallback(
      GlobalSlamOptimizationCallback callback) = 0;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_POSE_GRAPH_INTERFACE_H_
