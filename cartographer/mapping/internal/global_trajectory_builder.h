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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_GLOBAL_TRAJECTORY_BUILDER_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_GLOBAL_TRAJECTORY_BUILDER_H_

#include <memory>

#include "cartographer/mapping/internal/2d/local_trajectory_builder_2d.h"
#include "cartographer/mapping/internal/2d/pose_graph_2d.h"
#include "cartographer/mapping/internal/3d/local_trajectory_builder_3d.h"
#include "cartographer/mapping/internal/3d/pose_graph_3d.h"
#include "cartographer/mapping/local_slam_result_data.h"
#include "cartographer/mapping/trajectory_builder_interface.h"
#include "cartographer/metrics/family_factory.h"

namespace cartographer {
namespace mapping {

/*
 * （1）CreateGlobalTrajectoryBuilder2D() 函数生成一个 GlobalTrajectoryBuilder 的智能指针。
 *     GlobalTrajectoryBuilder 继承了 TrajectoryBuilderInterface。
 * （2）GlobalTrajectoryBuilder 是一个模板类，其模板列表中的 LocalTrajectoryBuilder 和 PoseGraph 分别是前端和后端的两个核心类型。
 *     这里传入的模板为 LocalTrajectoryBuilder2D 和 PoseGraph2D，表示 2D 构图。
 *     LocalTrajectoryBuilder2D 负责接收来自激光雷达的数据，进行扫描匹配，估计机器人位姿，并将传感器数据插入子图中，更新子图。
 *     PoseGraph2D 在后台进行闭环检测全局优化。
 */
std::unique_ptr<TrajectoryBuilderInterface> CreateGlobalTrajectoryBuilder2D(
    std::unique_ptr<LocalTrajectoryBuilder2D> local_trajectory_builder,
    const int trajectory_id, mapping::PoseGraph2D* const pose_graph,
    const TrajectoryBuilderInterface::LocalSlamResultCallback&
        local_slam_result_callback);

std::unique_ptr<TrajectoryBuilderInterface> CreateGlobalTrajectoryBuilder3D(
    std::unique_ptr<LocalTrajectoryBuilder3D> local_trajectory_builder,
    const int trajectory_id, mapping::PoseGraph3D* const pose_graph,
    const TrajectoryBuilderInterface::LocalSlamResultCallback&
        local_slam_result_callback);

void GlobalTrajectoryBuilderRegisterMetrics(
    metrics::FamilyFactory* family_factory);

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_GLOBAL_TRAJECTORY_BUILDER_H_
