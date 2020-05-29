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

#include "cartographer/mapping/map_builder.h"

#include "cartographer/common/make_unique.h"
#include "cartographer/common/time.h"
#include "cartographer/io/internal/mapping_state_serialization.h"
#include "cartographer/io/proto_stream_deserializer.h"
#include "cartographer/mapping/internal/2d/local_trajectory_builder_2d.h"
#include "cartographer/mapping/internal/2d/overlapping_submaps_trimmer_2d.h"
#include "cartographer/mapping/internal/2d/pose_graph_2d.h"
#include "cartographer/mapping/internal/3d/local_trajectory_builder_3d.h"
#include "cartographer/mapping/internal/3d/pose_graph_3d.h"
#include "cartographer/mapping/internal/collated_trajectory_builder.h"
#include "cartographer/mapping/internal/global_trajectory_builder.h"
#include "cartographer/mapping/proto/internal/legacy_serialized_data.pb.h"
#include "cartographer/sensor/internal/collator.h"
#include "cartographer/sensor/internal/trajectory_collator.h"
#include "cartographer/sensor/internal/voxel_filter.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"

namespace cartographer {
namespace mapping {

namespace {

using mapping::proto::SerializedData;
/**
 * @brief SelectRangeSensorIds  从传感器数据的topic名字expected_sensor_ids中选择激光测距仪数据
 * @param expected_sensor_ids   所有输入的传感器数据topic名字，例如scan，imu，odom等。
 *                              MapBuilder::SensorId实际的定义为cartographer::mapping::TrajectoryBuilderInterface::SensorId，
 *                              SensorId把SensorType和传感器topic名称（类型为std::string）绑定在一起。
 * @return                      激光测距仪数据的topic名字
 */
std::vector<std::string> SelectRangeSensorIds(
    const std::set<MapBuilder::SensorId>& expected_sensor_ids) {
  std::vector<std::string> range_sensor_ids;  // 激光测距仪数据的topic名字
  LOG(WARNING) << "Select range sensor ids:";
  for (const MapBuilder::SensorId& sensor_id : expected_sensor_ids) {
    std::cout << "sensor_id.id: " << sensor_id.id << std::endl;
    if (sensor_id.type == MapBuilder::SensorId::SensorType::RANGE) {
      std::cout << "range_sensor_ids: " << sensor_id.id << std::endl;
      range_sensor_ids.push_back(sensor_id.id);
    }
  }
  return range_sensor_ids;
}

}  // namespace

proto::MapBuilderOptions CreateMapBuilderOptions(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::MapBuilderOptions options;
  options.set_use_trajectory_builder_2d(
      parameter_dictionary->GetBool("use_trajectory_builder_2d"));
  options.set_use_trajectory_builder_3d(
      parameter_dictionary->GetBool("use_trajectory_builder_3d"));
  options.set_num_background_threads(
      parameter_dictionary->GetNonNegativeInt("num_background_threads"));
  *options.mutable_pose_graph_options() = CreatePoseGraphOptions(
      parameter_dictionary->GetDictionary("pose_graph").get());
  CHECK_NE(options.use_trajectory_builder_2d(),
           options.use_trajectory_builder_3d());
  return options;
}

// 构造函数，主要是完成了对 options_、thread_pool_、pose_graph_、sensor_collator_的构建工作。
// 而用于建立子图的轨迹跟踪器的对象 trajectory_builders_ 则需要通过调用接口 AddTrajectoryBuilder() 来完成构建。
// 用成员变量 options_ 保存了配置选项，同时根据配置中关于线程数量的配置完成了线程池 thread_pool_ 的构造。
MapBuilder::MapBuilder(const proto::MapBuilderOptions& options)
    : options_(options), thread_pool_(options.num_background_threads()) {
  // 检查配置项，确保在 2D 建图和 3D 建图之间二选一
  CHECK(options.use_trajectory_builder_2d() ^
        options.use_trajectory_builder_3d());
  // 如果是 2D 建图，那么我们就构建一个 PoseGraph2D 的对象记录在 pose_graph_ 下
  if (options.use_trajectory_builder_2d()) {
    pose_graph_ = common::make_unique<PoseGraph2D>(
        options_.pose_graph_options(),
        common::make_unique<optimization::OptimizationProblem2D>(
            options_.pose_graph_options().optimization_problem_options()),
        &thread_pool_);
  }
  // 如果是一个 3D 建图，就构建一个 PoseGraph3D 的对象
  if (options.use_trajectory_builder_3d()) {
    pose_graph_ = common::make_unique<PoseGraph3D>(
        options_.pose_graph_options(),
        common::make_unique<optimization::OptimizationProblem3D>(
            options_.pose_graph_options().optimization_problem_options()),
        &thread_pool_);
  }
  /**
   * 1. 最后根据配置项 collate_by_trajectory 来构建修正器。
   * 2. sensor_collator_ 是一个接口 sensor::CollatorInterface 的智能指针。
   *    根据 options.collate_by_trajectory() 的不同，sensor::CollatorInterface 有两种不同的实现方式，
   *    分别是 sensor::TrajectoryCollator 和 sensor::Collator。一般默认是0。
   */ 
  if (options.collate_by_trajectory()) {
    sensor_collator_ = common::make_unique<sensor::TrajectoryCollator>();
  } else {
    sensor_collator_ = common::make_unique<sensor::Collator>();
  }
}

/*
 * （1）创建一个新的 TrajectoryBuilder 并返回它的 trajectory_id。
 * （2）一个 MapBuilder 的类对应了一次建图过程，在整个建图过程中，用于全局优化的 PoseGraph 的对象只有一个，
 *     即 pose_graph_，而这个变量是在构造函数中就生成了。在 AddTrajectorybuilder 函数中只需要检查一下
 *     pose_graph_ 是否符合 PoseGraph2D 或PoseGraph3D 的情况。
 * （3）而一个 trajectory 对应了机器人运行一圈。在图建好后机器人可能多次运行。每一次运行都是新增一条 trajectory，
 *     因此，需要动态地维护一个 trajectory 的列表。每生成一个 trajectory 时都是调用 AddTrajectoryBuilder 来创建的。
 */
// 创建一个新的轨迹跟踪器并返回该跟踪器的索引。它有三个参数，
// expected_sensor_ids 中记录了用于建图的所有传感器名称和类型，trajectory_options 是新建的轨迹跟踪器的配置，
// 最后的 local_slam_result_callback 则是一个回调函数对象，用于响应局部地图构建完成的事件。
int MapBuilder::AddTrajectoryBuilder(
    const std::set<SensorId>& expected_sensor_ids,
    const proto::TrajectoryBuilderOptions& trajectory_options,
    LocalSlamResultCallback local_slam_result_callback) {
  // 先通过容器 trajectory_builders_ 获取轨迹跟踪器的数量。如果构建成功，该值将作为新的跟踪器的索引。
  // 一般 trajectory_id 的索引从 0 开始。
  const int trajectory_id = trajectory_builders_.size();
  LOG(WARNING) << "MapBuilder::AddTrajectoryBuilder:";
  std::cout << "trajectory_builders_.size(): " << trajectory_builders_.size() << std::endl;
  std::cout << "trajectory_id: " << trajectory_id << std::endl;
  std::cout << "options_.use_trajectory_builder_2d(): " << options_.use_trajectory_builder_2d() << std::endl;
  std::cout << "options_.use_trajectory_builder_3d(): " << options_.use_trajectory_builder_3d() << std::endl;
  // 根据配置参数选择三维建图还是二维建图，从而创建新的轨迹跟踪器
  if (options_.use_trajectory_builder_3d()) {  // 进行三维建图，构建轨迹跟踪器
    std::unique_ptr<LocalTrajectoryBuilder3D> local_trajectory_builder;
    if (trajectory_options.has_trajectory_builder_3d_options()) {
      local_trajectory_builder = common::make_unique<LocalTrajectoryBuilder3D>(
          trajectory_options.trajectory_builder_3d_options(),
          SelectRangeSensorIds(expected_sensor_ids));
    }
    DCHECK(dynamic_cast<PoseGraph3D*>(pose_graph_.get()));
    trajectory_builders_.push_back(
        common::make_unique<CollatedTrajectoryBuilder>(
            sensor_collator_.get(), trajectory_id, expected_sensor_ids,
            CreateGlobalTrajectoryBuilder3D(
                std::move(local_trajectory_builder), trajectory_id,
                static_cast<PoseGraph3D*>(pose_graph_.get()),
                local_slam_result_callback)));
  } else {  // 进行二维建图，构建轨迹跟踪器
    // 先创建一个 LocalTrajectoryBuilder2D 类型的对象。
    // 这个对象并不是我们一直说的轨迹跟踪器，但它应该是轨迹跟踪器的核心。
    // 因为它几乎完成了一个局部 SLAM 的所有功能，包括位姿估计、扫描匹配等，就是没有闭环检测。
    std::unique_ptr<LocalTrajectoryBuilder2D> local_trajectory_builder;
    if (trajectory_options.has_trajectory_builder_2d_options()) {
      // 检查是否有关于 2D 的轨迹跟踪器的配置项，如果有的话，就根据其配置项以及传感器配置实例化对象 local_trajectory_builder。
      local_trajectory_builder = common::make_unique<LocalTrajectoryBuilder2D>(
          trajectory_options.trajectory_builder_2d_options(),
          // SelectRangeSensorIds() 函数返回 expected_sensor_ids 中所有激光传感器主题的名称
          SelectRangeSensorIds(expected_sensor_ids));
    }
    // 通过 dynamic_cast 将 pose_graph_ 对象强制转换为 PoseGraph2D，并检查数据类型是否正确。
    // dynamic_cast 是强制类型转化，把 PoseGraphInterface 的指针转化为 PoseGraph2D。
    DCHECK(dynamic_cast<PoseGraph2D*>(pose_graph_.get()));
    /**
     * 1. 创建轨迹跟踪器。
     * 2. 接下来的语句所构建的 CollatedTrajectoryBuilder 类型的对象才是所谓的轨迹跟踪器，它继承自接口类
     *    TrajectoryBuilderInterface，这个接口适用于 2D 和 3D 建图的对象，如此上层的代码就不必关心具体的建图内核，
     *    使用同一个容器 trajectory_builders_ 就可以保存两种 local SLAM 的对象。
     * 3. 构建好的对象就直接被塞进了容器 trajectory_builders_ 中。
     * 4. CollatedTrajectoryBuilder 类型有四个输入参数。
     *    (1)sensor_collator_: 这是一个类型为 sensor::CollatorInterface 的智能指针，该对象在 MapBuilder 的构造函数进行初始化，
     *                         主要作用是收集传感器数据。
     *    (2)trajectory_id: 新创建轨迹跟踪器的索引。
     *    (3)expected_sensor_ids: 记录了用于建图的所有传感器主题名称和类型。
     *    (4)通过函数 CreateGlobalTrajectoryBuilder2D 构建了一个 GlobalTrajectoryBuilder 类型的对象，
     *       它继承了 TrajectoryBuilderInterface。从它的参数列表中来看，除了含有刚刚构建的 local_trajectory_builder 对象之外，
     *       还引入了位姿图对象 pose_graph_。个人猜测这是一个具有闭环功能的 SLAM 对象，所以称之为Global的。
     */
    trajectory_builders_.push_back(
        common::make_unique<CollatedTrajectoryBuilder>(
            sensor_collator_.get(), trajectory_id, expected_sensor_ids,
            /*
             * （1）CreateGlobalTrajectoryBuilder2D() 函数生成一个 GlobalTrajectoryBuilder 的智能指针。
             *     GlobalTrajectoryBuilder 继承了 TrajectoryBuilderInterface。
             * （2）GlobalTrajectoryBuilder 是一个模板类，其模板列表中的 LocalTrajectoryBuilder2D 和 PoseGraph2D 分别是前端和后端的两个核心类型。
             *     LocalTrajectoryBuilder2D 负责接收来自激光雷达的数据，进行扫描匹配，估计机器人位姿，并将传感器数据插入子图中，更新子图。
             *     PoseGraph2D 在后台进行闭环检测全局优化。
             * （3）它的构造函数有4个参数：
             * local_trajectory_builder：位姿跟踪器，前端的核心对象，其数据类型是一个模板参数，这里实际使用的是 LocalTrajectoryBuilder2D。
             * trajectory_id：轨迹索引。
             * pose_graph：位姿图，后端的核心对象，其数据类型是一个模板参数，这里实际使用的是 PoseGraph2D。
             * local_slam_result_callback：前端数据更新后的回调函数。
             * （4）从它的参数列表中来看，除了含有刚刚构建的 local_trajectory_builder 对象之外，还引入了位姿图对象 pose_graph_。
             *     个人猜测这是一个具有闭环功能的 SLAM 对象，所以称之为 Global 的。
             */
            CreateGlobalTrajectoryBuilder2D(
                std::move(local_trajectory_builder), trajectory_id,
                static_cast<PoseGraph2D*>(pose_graph_.get()),
                local_slam_result_callback)));

    // 针对二维建图还有一个配置项 overlapping_submaps_trimmer_2d，决定是否为 pose_graph_ 对象添加一个
    // OverlappingSubmapsTrimmer2D 类型的修饰器，用于根据子图之间重叠的部分修饰地图。
    if (trajectory_options.has_overlapping_submaps_trimmer_2d()) {
      const auto& trimmer_options =
          trajectory_options.overlapping_submaps_trimmer_2d();
      pose_graph_->AddTrimmer(common::make_unique<OverlappingSubmapsTrimmer2D>(
          trimmer_options.fresh_submaps_count(),
          trimmer_options.min_covered_area() /
              common::Pow2(trajectory_options.trajectory_builder_2d_options()
                               .submaps_options()
                               .grid_options_2d()
                               .resolution()),
          trimmer_options.min_added_submaps_count()));
    }
  }
  // 如果配置项要求只进行定位，那么可以在 pose_graph_ 中添加一个 PureLocalizationTrimmer 类型修饰器，来完成这一功能。
  if (trajectory_options.pure_localization()) {
    constexpr int kSubmapsToKeep = 3;
    pose_graph_->AddTrimmer(common::make_unique<PureLocalizationTrimmer>(
        trajectory_id, kSubmapsToKeep));
  }
  /*
   * 如果该轨迹有初始pose；开始一条轨迹前我们是否已知初始位姿。
   * 这对应的情况就是比如说，我们检测到了一个Landmark。那么这时，我们可以新增加一条trajectory，
   * 增加新的trajectory时设置has.initial_trajectory_pose为真，
   * 然后根据机器人与Landmark之间的相对位姿推算机器人相对于世界坐标系的相对位姿。
   * 以该位姿作为新增加的trajectory的初始位姿。这样情况下，在检测到Landmark时就能有效降低累积误差。
   */
  // 如果开始建图之前已经有了初始位置，那么我们可以将初始位置提供给 pose_graph_ 对象。
  if (trajectory_options.has_initial_trajectory_pose()) {
    const auto& initial_trajectory_pose =
        trajectory_options.initial_trajectory_pose();
    // 调用pose_graph_中的方法，设置初始pose。跟全局相关的事情，都交给pose_graph_来处理。
    pose_graph_->SetInitialTrajectoryPose(  // 设置初始pose
        trajectory_id, initial_trajectory_pose.to_trajectory_id(),
        transform::ToRigid3(initial_trajectory_pose.relative_pose()),
        common::FromUniversal(initial_trajectory_pose.timestamp()));
  }
  // 将轨迹跟踪器的配置信息和传感器配置信息保存到容器 all_trajectory_builder_options_ 中。
  proto::TrajectoryBuilderOptionsWithSensorIds options_with_sensor_ids_proto;
  for (const auto& sensor_id : expected_sensor_ids) {
    *options_with_sensor_ids_proto.add_sensor_id() = ToProto(sensor_id);
  }
  *options_with_sensor_ids_proto.mutable_trajectory_builder_options() =
      trajectory_options;
  all_trajectory_builder_options_.push_back(options_with_sensor_ids_proto);
  // 检查轨迹跟踪器对象及其配置的数量，确保两者相等
  CHECK_EQ(trajectory_builders_.size(), all_trajectory_builder_options_.size());
  // 返回新建的轨迹跟踪器对象的索引
  return trajectory_id;
}

// 从序列化的数据中构造一条trajectory
int MapBuilder::AddTrajectoryForDeserialization(
    const proto::TrajectoryBuilderOptionsWithSensorIds&
        options_with_sensor_ids_proto) {
  const int trajectory_id = trajectory_builders_.size();
  /*
   * emplace_back和push_back都是向容器内添加数据。
   * 对于在容器中添加类的对象时, 相比于push_back,emplace_back可以避免额外类的复制和移动操作。
   */
  trajectory_builders_.emplace_back();
  // 配置项的添加
  all_trajectory_builder_options_.push_back(options_with_sensor_ids_proto);
  // 检查两者大小是否一致
  CHECK_EQ(trajectory_builders_.size(), all_trajectory_builder_options_.size());
  return trajectory_id;
}

// 结束一条轨迹；可以看到，分别调用sensor_collator和pose_graph_的成员函数来Finish一条轨迹
void MapBuilder::FinishTrajectory(const int trajectory_id) {
  // 现阶段猜测，sensor_collator_->FinishTrajectory应该是做的清除对传感器的占用等操作
  sensor_collator_->FinishTrajectory(trajectory_id);
  // 现阶段猜测，pose_graph_->FinishTrajectory应该是完成对刚刚的trajectory的全局优化
  pose_graph_->FinishTrajectory(trajectory_id);
}

// 根据指定的submap_id来查询submap，把结果放到SubmapQuery::Response中。
// 如果出现错误，返回error string; 成功则返回empty string。
std::string MapBuilder::SubmapToProto(
    const SubmapId& submap_id, proto::SubmapQuery::Response* const response) {
  if (submap_id.trajectory_id < 0 ||
      submap_id.trajectory_id >= num_trajectory_builders()) {
    return "Requested submap from trajectory " +
           std::to_string(submap_id.trajectory_id) + " but there are only " +
           std::to_string(num_trajectory_builders()) + " trajectories.";
  }  // 指定的submap的id必须合法

  // pose_graph_中应该是维护着一张submap的列表。通过pose_graph_获取指定id的子图。
  const auto submap_data = pose_graph_->GetSubmapData(submap_id);
  if (submap_data.submap == nullptr) {
    return "Requested submap " + std::to_string(submap_id.submap_index) +
           " from trajectory " + std::to_string(submap_id.trajectory_id) +
           " but it does not exist: maybe it has been trimmed.";
  }  // 一些子图可能在优化过程中被裁掉
  // 正常情况下返回数据
  submap_data.submap->ToResponseProto(submap_data.pose, response);
  return "";
}

// 调用io::WritePbStream工具，保存所有数据
void MapBuilder::SerializeState(io::ProtoStreamWriterInterface* const writer) {
  io::WritePbStream(*pose_graph_, all_trajectory_builder_options_, writer);
}

// 从一个proto流中加载SLAM状态
void MapBuilder::LoadState(io::ProtoStreamReaderInterface* const reader,
                           bool load_frozen_state) {
  // 反序列化读取工具
  io::ProtoStreamDeserializer deserializer(reader);

  // Create a copy of the pose_graph_proto, such that we can re-write the
  // trajectory ids.
  proto::PoseGraph pose_graph_proto = deserializer.pose_graph();
  const auto& all_builder_options_proto =
      deserializer.all_trajectory_builder_options();

  // 逐条trajectory恢复
  std::map<int, int> trajectory_remapping;
  for (auto& trajectory_proto : *pose_graph_proto.mutable_trajectory()) {
    const auto& options_with_sensor_ids_proto =
        all_builder_options_proto.options_with_sensor_ids(
            trajectory_proto.trajectory_id());
    const int new_trajectory_id =
        AddTrajectoryForDeserialization(options_with_sensor_ids_proto);
    CHECK(trajectory_remapping
              .emplace(trajectory_proto.trajectory_id(), new_trajectory_id)
              .second)
        << "Duplicate trajectory ID: " << trajectory_proto.trajectory_id();
    trajectory_proto.set_trajectory_id(new_trajectory_id);
    if (load_frozen_state) {
      pose_graph_->FreezeTrajectory(new_trajectory_id);
    }
  }

  // Apply the calculated remapping to constraints in the pose graph proto.
  // 恢复trajectory上的节点间的约束关系
  for (auto& constraint_proto : *pose_graph_proto.mutable_constraint()) {
    constraint_proto.mutable_submap_id()->set_trajectory_id(
        trajectory_remapping.at(constraint_proto.submap_id().trajectory_id()));
    constraint_proto.mutable_node_id()->set_trajectory_id(
        trajectory_remapping.at(constraint_proto.node_id().trajectory_id()));
  }

  // 恢复Submap
  MapById<SubmapId, transform::Rigid3d> submap_poses;
  for (const proto::Trajectory& trajectory_proto :
       pose_graph_proto.trajectory()) {
    for (const proto::Trajectory::Submap& submap_proto :
         trajectory_proto.submap()) {
      submap_poses.Insert(SubmapId{trajectory_proto.trajectory_id(),
                                   submap_proto.submap_index()},
                          transform::ToRigid3(submap_proto.pose()));
    }
  }

  // 恢复节点的pose
  MapById<NodeId, transform::Rigid3d> node_poses;
  for (const proto::Trajectory& trajectory_proto :
       pose_graph_proto.trajectory()) {
    for (const proto::Trajectory::Node& node_proto : trajectory_proto.node()) {
      node_poses.Insert(
          NodeId{trajectory_proto.trajectory_id(), node_proto.node_index()},
          transform::ToRigid3(node_proto.pose()));
    }
  }

  // Set global poses of landmarks.
  // 设置Landmark
  for (const auto& landmark : pose_graph_proto.landmark_poses()) {
    pose_graph_->SetLandmarkPose(landmark.landmark_id(),
                                 transform::ToRigid3(landmark.global_pose()));
  }

  // 不停读取，直到读完
  SerializedData proto;
  while (deserializer.ReadNextSerializedData(&proto)) {
    switch (proto.data_case()) {
      case SerializedData::kPoseGraph:
        LOG(ERROR) << "Found multiple serialized `PoseGraph`. Serialized "
                      "stream likely corrupt!.";
        break;
      case SerializedData::kAllTrajectoryBuilderOptions:
        LOG(ERROR) << "Found multiple serialized "
                      "`AllTrajectoryBuilderOptions`. Serialized stream likely "
                      "corrupt!.";
        break;
      case SerializedData::kSubmap: {
        proto.mutable_submap()->mutable_submap_id()->set_trajectory_id(
            trajectory_remapping.at(
                proto.submap().submap_id().trajectory_id()));
        const transform::Rigid3d& submap_pose = submap_poses.at(
            SubmapId{proto.submap().submap_id().trajectory_id(),
                     proto.submap().submap_id().submap_index()});
        pose_graph_->AddSubmapFromProto(submap_pose, proto.submap());
        break;
      }
      case SerializedData::kNode: {
        proto.mutable_node()->mutable_node_id()->set_trajectory_id(
            trajectory_remapping.at(proto.node().node_id().trajectory_id()));
        const transform::Rigid3d& node_pose =
            node_poses.at(NodeId{proto.node().node_id().trajectory_id(),
                                 proto.node().node_id().node_index()});
        pose_graph_->AddNodeFromProto(node_pose, proto.node());
        break;
      }
      case SerializedData::kTrajectoryData: {
        proto.mutable_trajectory_data()->set_trajectory_id(
            trajectory_remapping.at(proto.trajectory_data().trajectory_id()));
        pose_graph_->SetTrajectoryDataFromProto(proto.trajectory_data());
        break;
      }
      case SerializedData::kImuData: {
        if (load_frozen_state) break;
        pose_graph_->AddImuData(
            trajectory_remapping.at(proto.imu_data().trajectory_id()),
            sensor::FromProto(proto.imu_data().imu_data()));
        break;
      }
      case SerializedData::kOdometryData: {
        if (load_frozen_state) break;
        pose_graph_->AddOdometryData(
            trajectory_remapping.at(proto.odometry_data().trajectory_id()),
            sensor::FromProto(proto.odometry_data().odometry_data()));
        break;
      }
      case SerializedData::kFixedFramePoseData: {
        if (load_frozen_state) break;
        pose_graph_->AddFixedFramePoseData(
            trajectory_remapping.at(
                proto.fixed_frame_pose_data().trajectory_id()),
            sensor::FromProto(
                proto.fixed_frame_pose_data().fixed_frame_pose_data()));
        break;
      }
      case SerializedData::kLandmarkData: {
        if (load_frozen_state) break;
        pose_graph_->AddLandmarkData(
            trajectory_remapping.at(proto.landmark_data().trajectory_id()),
            sensor::FromProto(proto.landmark_data().landmark_data()));
        break;
      }
      default:
        LOG(WARNING) << "Skipping unknown message type in stream: "
                     << proto.GetTypeName();
    }
  }

  if (load_frozen_state) {
    // Add information about which nodes belong to which submap.
    // Required for 3D pure localization.
    for (const proto::PoseGraph::Constraint& constraint_proto :
         pose_graph_proto.constraint()) {
      if (constraint_proto.tag() !=
          proto::PoseGraph::Constraint::INTRA_SUBMAP) {
        continue;
      }
      pose_graph_->AddNodeToSubmap(
          NodeId{constraint_proto.node_id().trajectory_id(),
                 constraint_proto.node_id().node_index()},
          SubmapId{constraint_proto.submap_id().trajectory_id(),
                   constraint_proto.submap_id().submap_index()});
    }
  } else {
    // When loading unfrozen trajectories, 'AddSerializedConstraints' will
    // take care of adding information about which nodes belong to which
    // submap.
    pose_graph_->AddSerializedConstraints(
        FromProto(pose_graph_proto.constraint()));
  }
  CHECK(reader->eof());
}

}  // namespace mapping
}  // namespace cartographer
