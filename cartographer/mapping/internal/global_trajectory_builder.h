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

/**
 * @brief CreateGlobalTrajectoryBuilder2D  构建一个 GlobalTrajectoryBuilder 类型的对象。它是连接前端与后端的桥梁，
 *                                         继承了 TrajectoryBuilderInterface。
 * @param local_trajectory_builder         位姿跟踪器，前端的核心对象，其数据类型是一个模板参数
 * @param trajectory_id                    轨迹索引
 * @param pose_graph                       位姿图，后端的核心对象，其数据类型是一个模板参数
 * @param local_slam_result_callback       前端数据更新后的回调函数
 * @return                                 GlobalTrajectoryBuilder 类型的对象
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
