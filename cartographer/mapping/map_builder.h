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

#ifndef CARTOGRAPHER_MAPPING_MAP_BUILDER_H_
#define CARTOGRAPHER_MAPPING_MAP_BUILDER_H_

#include "cartographer/mapping/map_builder_interface.h"

#include <memory>

#include "cartographer/common/thread_pool.h"
#include "cartographer/mapping/pose_graph.h"
#include "cartographer/mapping/proto/map_builder_options.pb.h"
#include "cartographer/sensor/collator_interface.h"

namespace cartographer {
namespace mapping {

proto::MapBuilderOptions CreateMapBuilderOptions(
    common::LuaParameterDictionary *const parameter_dictionary);

// Wires up the complete SLAM stack with TrajectoryBuilders (for local submaps)
// and a PoseGraph for loop closure.
// TrajectoryBuilder 用于 local submaps 的建立与维护；PoseGraph 部分用于 loop closure。
class MapBuilder : public MapBuilderInterface {
 public:
  /**
   * @brief MapBuilder  构造函数，主要是完成几个成员变量的构造
   * @param options     配置选项，这个参数的数据类型是通过 protobuf 根据 proto 文件生成的。
   */
  explicit MapBuilder(const proto::MapBuilderOptions &options);
  ~MapBuilder() override {}

  MapBuilder(const MapBuilder &) = delete;
  MapBuilder &operator=(const MapBuilder &) = delete;

  /**
   * @brief AddTrajectoryBuilder        创建一个新的轨迹跟踪器并返回该跟踪器的索引
   * @param expected_sensor_ids         记录了用于建图的所有传感器主题名称和类型。结构体 SensorId 有两个字段，
   *                                    type 通过枚举描述了传感器的类型，id 是一个字符串记录了传感器所对应的 ROS 主题名称。
   * @param trajectory_options          新建的轨迹跟踪器的配置，这个参数的数据类型是通过 protobuf 根据 proto 文件生成的，
   *                                    在 "node_main.cc" 中由函数 Run 从配置文件中获取。
   * @param local_slam_result_callback  回调函数对象，用于响应局部地图构建完成的事件
   * @return
   */
  int AddTrajectoryBuilder(
      const std::set<SensorId> &expected_sensor_ids,
      const proto::TrajectoryBuilderOptions &trajectory_options,
      LocalSlamResultCallback local_slam_result_callback) override;

  int AddTrajectoryForDeserialization(
      const proto::TrajectoryBuilderOptionsWithSensorIds
          &options_with_sensor_ids_proto) override;

  // 标记该轨迹已完成 data 采集，后续不再接收 data
  void FinishTrajectory(int trajectory_id) override;

  // 把轨迹 id 和子图索引对应的 submap，序列化到文件
  std::string SubmapToProto(const SubmapId &submap_id,
                            proto::SubmapQuery::Response *response) override;

  void SerializeState(io::ProtoStreamWriterInterface *writer) override;

  void LoadState(io::ProtoStreamReaderInterface *reader,
                 bool load_frozen_state) override;

  // 返回一个PoseGraphInterface的接口指针
  mapping::PoseGraphInterface *pose_graph() override {
    return pose_graph_.get();  // unique_ptr的get函数可返回被管理对象的指针
  }

  // 返回系统中当前已有的 trajectory_builder 的数量，即在建图的轨迹数量
  int num_trajectory_builders() const override {
    return trajectory_builders_.size();  // 向量的 size 即为 TrajectoryBuilder 的数量
  }

  // 根据轨迹 id trajectory_id 返回一个指向该轨迹的 TrajectoryBuilderInterface 对象指针。
  // 如果该 trajectory 没有一个 TrajectoryBuilder，则返回 nullptr。
  mapping::TrajectoryBuilderInterface *GetTrajectoryBuilder(
      int trajectory_id) const override {
    return trajectory_builders_.at(trajectory_id).get();  // 从列表中取出指定id的TrajectoryBuilder
  }

  // 获取所有TrajectoryBuilder的配置项
  const std::vector<proto::TrajectoryBuilderOptionsWithSensorIds>
      &GetAllTrajectoryBuilderOptions() const override {
    return all_trajectory_builder_options_;  // 所有配置项都存在该向量中
  }

 private:
  // options_ 用于记录运行配置，它使用了 google 的 protobuf 来处理结构化的数据。
  // 这些配置项是由 cartographer_ros 在系统运行之初从配置文件中加载的。
  const proto::MapBuilderOptions options_;
  // 线程池，其中的线程数量是固定的。
  // Cartographer 使用类 ThreadPool 对 C++11 的线程进行了封装，用于方便高效的管理多线程。
  common::ThreadPool thread_pool_;

  // 该对象用于在后台完成闭环检测，进行全局的地图优化。
  std::unique_ptr<PoseGraph> pose_graph_;

  // 应该是用来管理和收集传感器数据的
  std::unique_ptr<sensor::CollatorInterface> sensor_collator_;
  /*
   * （1）MapBuilder还维护了一个TrajectoryBuilder的向量列表，每一个TrajectoryBuilder对应了机器人运行了一圈。
   *     这个向量列表就管理了整个图中的所有submap。
   * （2）一个向量，管理所有的TrajectoryBuilderInterface；应该是每一条trajectory对应了该向量的一个元素。
   * （3）trajectory是机器人跑一圈时的轨迹，在这其中需要记录和维护传感器的数据。
   *     根据这个trajectory上传感器收集的数据，我们可以逐步构建出栅格化的地图Submap，
   *     但这个submap会随着时间或trajectory的增长而产生误差累积，当trajectory增长到超过一个阈值，则会新增一个submap。
   *     而PoseGraph是用来进行全局优化，将所有的Submap紧紧tie在一起，构成一个全局的Map，消除误差累积。
   */
  // 用于在前台构建子图。在系统运行的过程中，可能有不止一条轨迹，针对每一条轨迹 Cartographer 都建立了一个轨迹跟踪器。
  std::vector<std::unique_ptr<mapping::TrajectoryBuilderInterface>>
      trajectory_builders_;
  // 记录了所有轨迹跟踪器的配置。
  std::vector<proto::TrajectoryBuilderOptionsWithSensorIds>
      all_trajectory_builder_options_;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_MAP_BUILDER_H_
