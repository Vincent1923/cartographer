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
// TrajectoryBuilder用于local submaps的建立与维护；PoseGraph部分用于loop closure。
class MapBuilder : public MapBuilderInterface {
 public:
  explicit MapBuilder(const proto::MapBuilderOptions &options);
  ~MapBuilder() override {}

  MapBuilder(const MapBuilder &) = delete;
  MapBuilder &operator=(const MapBuilder &) = delete;

  // 创建一个TrajectoryBuilder并返回他的index，即trajectory_id
  int AddTrajectoryBuilder(
      const std::set<SensorId> &expected_sensor_ids,
      const proto::TrajectoryBuilderOptions &trajectory_options,
      LocalSlamResultCallback local_slam_result_callback) override;

  int AddTrajectoryForDeserialization(
      const proto::TrajectoryBuilderOptionsWithSensorIds
          &options_with_sensor_ids_proto) override;

  void FinishTrajectory(int trajectory_id) override;

  std::string SubmapToProto(const SubmapId &submap_id,
                            proto::SubmapQuery::Response *response) override;

  void SerializeState(io::ProtoStreamWriterInterface *writer) override;

  void LoadState(io::ProtoStreamReaderInterface *reader,
                 bool load_frozen_state) override;

  // 返回一个PoseGraphInterface的接口指针
  mapping::PoseGraphInterface *pose_graph() override {
    return pose_graph_.get();  // unique_ptr的get函数可返回被管理对象的指针
  }

  // 返回系统中当前已有的trajectory_builder的数量
  int num_trajectory_builders() const override {
    return trajectory_builders_.size();  // 向量的size即为TrajectoryBuilder的数量
  }

  // 根据trajectory_id返回一个TrajectoryBuilderInterface的指针。
  // 如果该trajectory没有一个TrajectoryBuilder，则返回nullptr。
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
  const proto::MapBuilderOptions options_;  // MapBuilder的配置项
  common::ThreadPool thread_pool_;          // 线程池。个人猜测，应该是为每一条trajectory都单独开辟一个线程

  /*
   * MapBuilder维护了一个PoseGraph的智能指针，该指针用来做Loop Closure
   */
  std::unique_ptr<PoseGraph> pose_graph_;

  std::unique_ptr<sensor::CollatorInterface> sensor_collator_;       // 收集传感器数据的智能指针
  /*
   * （1）MapBuilder还维护了一个TrajectoryBuilder的向量列表，每一个TrajectoryBuilder对应了机器人运行了一圈。
   *     这个向量列表就管理了整个图中的所有submap。
   * （2）一个向量，管理所有的TrajectoryBuilderInterface；应该是每一条trajectory对应了该向量的一个元素。
   * （3）trajectory是机器人跑一圈时的轨迹，在这其中需要记录和维护传感器的数据。
   *     根据这个trajectory上传感器收集的数据，我们可以逐步构建出栅格化的地图Submap，
   *     但这个submap会随着时间或trajectory的增长而产生误差累积，当trajectory增长到超过一个阈值，则会新增一个submap。
   *     而PoseGraph是用来进行全局优化，将所有的Submap紧紧tie在一起，构成一个全局的Map，消除误差累积。
   */
  std::vector<std::unique_ptr<mapping::TrajectoryBuilderInterface>>
      trajectory_builders_;
  std::vector<proto::TrajectoryBuilderOptionsWithSensorIds>          //与每个TrajectoryBuilderInterface相对应的配置项
      all_trajectory_builder_options_;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_MAP_BUILDER_H_
