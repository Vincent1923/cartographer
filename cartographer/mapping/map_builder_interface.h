/*
 * Copyright 2017 The Cartographer Authors
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

#ifndef CARTOGRAPHER_MAPPING_MAP_BUILDER_INTERFACE_H_
#define CARTOGRAPHER_MAPPING_MAP_BUILDER_INTERFACE_H_

#include <set>
#include <string>
#include <vector>

#include "Eigen/Geometry"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/port.h"
#include "cartographer/io/proto_stream_interface.h"
#include "cartographer/mapping/id.h"
#include "cartographer/mapping/pose_graph_interface.h"
#include "cartographer/mapping/proto/submap_visualization.pb.h"
#include "cartographer/mapping/proto/trajectory_builder_options.pb.h"
#include "cartographer/mapping/submaps.h"
#include "cartographer/mapping/trajectory_builder_interface.h"

namespace cartographer {
namespace mapping {

// This interface is used for both library and RPC implementations.
// Implementations wire up the complete SLAM stack.
class MapBuilderInterface {
 public:
  /*
   * 这里注意，LocalSlamResultCallback是一个回调函数。
   * 查看TrajectoryBuilderInterface的定义文件/mapping/trajectory_builder_interface.h:
   *   using LocalSlamResultCallback =
   *      std::function<void(int,                                       // trajectory ID
   *                         common::Time,                              //时间
   *                         transform::Rigid3d,                        // local pose estimate
   *                         sensor::RangeData,                         // in local frame
   *                         std::unique_ptr<const InsertionResult>)>;
   */
  // 先通过 using 关键字定义了两个类型别名，用来替代原来很长的类型名
  using LocalSlamResultCallback =
      TrajectoryBuilderInterface::LocalSlamResultCallback;

  using SensorId = TrajectoryBuilderInterface::SensorId;

  // 定义和实现了默认的构造函数和析构函数
  MapBuilderInterface() {}
  virtual ~MapBuilderInterface() {}

  // 禁用了拷贝构造函数和赋值运算符
  MapBuilderInterface(const MapBuilderInterface&) = delete;
  MapBuilderInterface& operator=(const MapBuilderInterface&) = delete;

  // Creates a new trajectory builder and returns its index.
  // 创建一个新的轨迹跟踪器并返回该跟踪器的索引
  /**
   * @brief AddTrajectoryBuilder        创建一个新的轨迹跟踪器并返回该跟踪器的索引
   * @param expected_sensor_ids         记录了用于建图的所有传感器名称和类型
   * @param trajectory_options          新建的轨迹跟踪器的配置，这个参数的数据类型是通过 protobuf 根据 proto 文件生成的
   * @param local_slam_result_callback  回调函数对象，用于响应局部地图构建完成的事件
   * @return                            新建的轨迹跟踪器的索引
   */
  virtual int AddTrajectoryBuilder(
      const std::set<SensorId>& expected_sensor_ids,
      const proto::TrajectoryBuilderOptions& trajectory_options,
      LocalSlamResultCallback local_slam_result_callback) = 0;

  // Creates a new trajectory and returns its index. Querying the trajectory
  // builder for it will return 'nullptr'.
  // 创建一个新的轨迹跟踪器并返回该跟踪器的索引。
  /**
   * @brief AddTrajectoryForDeserialization  从一个序列化的数据中创建一个新的轨迹跟踪器并返回该跟踪器的索引。
   *                                         Serialization 是序列化，Deserialization 是反序列化。
   * @param options_with_sensor_ids_proto    这个参数处理记录了轨迹跟踪器的配置还包含传感器的配置。
   *                                         这个参数的数据类型是通过 protobuf 根据 proto 文件生成的。
   * @return                                 新建的轨迹跟踪器的索引
   */
  virtual int AddTrajectoryForDeserialization(
      const proto::TrajectoryBuilderOptionsWithSensorIds&
          options_with_sensor_ids_proto) = 0;

  // Returns the 'TrajectoryBuilderInterface' corresponding to the specified
  // 'trajectory_id' or 'nullptr' if the trajectory has no corresponding
  // builder.
  /**
   * @brief GetTrajectoryBuilder  获取一个索引为 trajectory_id 的轨迹跟踪器对象，如果输入的索引不对应一个跟踪器对象则返回空指针 nullptr
   * @param trajectory_id         轨迹跟踪器的索引
   * @return                      索引为 trajectory_id 的轨迹跟踪器对象
   */
  virtual mapping::TrajectoryBuilderInterface* GetTrajectoryBuilder(
      int trajectory_id) const = 0;

  // Marks the TrajectoryBuilder corresponding to 'trajectory_id' as finished,
  // i.e. no further sensor data is expected.
  /**
   * @brief FinishTrajectory  关闭索引 trajectory_id 对应的轨迹跟踪器，该跟踪器将不再响应新的传感器数据
   * @param trajectory_id     轨迹跟踪器的索引
   */
  virtual void FinishTrajectory(int trajectory_id) = 0;

  // Fills the SubmapQuery::Response corresponding to 'submap_id'. Returns an
  // error string on failure, or an empty string on success.
  /**
   * @brief SubmapToProto  将子图索引 submap_id 所对应的子图信息填充到 response 中。
   *                       如果出错将错误信息以字符串的形式返回，若成功运行则返回空字符串。
   * @param submap_id      子图索引
   * @param response       查询子图返回的响应信息
   * @return               如果出错将返回错误信息的字符串，若成功则返回空字符串
   */
  virtual std::string SubmapToProto(const SubmapId& submap_id,
                                    proto::SubmapQuery::Response* response) = 0;

  // Serializes the current state to a proto stream.
  /**
   * @brief SerializeState  将当前的系统状态转换成一个 proto 的流，完成序列化。
   * @param writer
   */
  virtual void SerializeState(io::ProtoStreamWriterInterface* writer) = 0;

  // Loads the SLAM state from a proto stream.
  /**
   * @brief LoadState          SerializeState 的逆操作，用于从 proto 流中加载 SLAM 状态
   * @param reader
   * @param load_frozen_state
   */
  virtual void LoadState(io::ProtoStreamReaderInterface* reader,
                         bool load_frozen_state) = 0;

  /**
   * @brief num_trajectory_builders  获取当前轨迹跟踪器的数量
   * @return                         当前轨迹跟踪器的数量
   */
  virtual int num_trajectory_builders() const = 0;

  /**
   * @brief pose_graph  获取用于实现闭环检测的 PoseGraph 对象
   * @return            PoseGraph 对象
   */
  virtual mapping::PoseGraphInterface* pose_graph() = 0;

  /**
   * @brief GetAllTrajectoryBuilderOptions  获取所有的轨迹跟踪器的配置
   * @return                                所有的轨迹跟踪器的配置
   */
  virtual const std::vector<proto::TrajectoryBuilderOptionsWithSensorIds>&
  GetAllTrajectoryBuilderOptions() const = 0;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_MAP_BUILDER_INTERFACE_H_
