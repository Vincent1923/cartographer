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

#ifndef CARTOGRAPHER_MAPPING_2D_SUBMAP_2D_H_
#define CARTOGRAPHER_MAPPING_2D_SUBMAP_2D_H_

#include <memory>
#include <vector>

#include "Eigen/Core"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/mapping/2d/grid_2d.h"
#include "cartographer/mapping/2d/map_limits.h"
#include "cartographer/mapping/proto/2d/submaps_options_2d.pb.h"
#include "cartographer/mapping/proto/serialization.pb.h"
#include "cartographer/mapping/proto/submap_visualization.pb.h"
#include "cartographer/mapping/range_data_inserter_interface.h"
#include "cartographer/mapping/submaps.h"
#include "cartographer/mapping/trajectory_node.h"
#include "cartographer/sensor/range_data.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace mapping {

proto::SubmapsOptions2D CreateSubmapsOptions2D(
    common::LuaParameterDictionary* parameter_dictionary);

class Submap2D : public Submap {
 public:
  /**
   * @brief Submap2D  构造函数
   * @param origin    原点坐标
   * @param grid      Grid2D变量，存储栅格化坐标和坐标上的概率值。
   *                  Grid2D定义在/mapping/2d/grid_2d.h中，继承了GridInterface(/mapping/grid_interface.h)，
   *                  Grid2D又被ProbabilityGrid继承，定义在/mapping/2d/probability_grid.h中。
   *                  基本数据都存储在Grid2D的成员变量grid_中。
   * @return
   */
  Submap2D(const Eigen::Vector2f& origin, std::unique_ptr<Grid2D> grid);
  explicit Submap2D(const proto::Submap2D& proto);

   // 实现接口Submap中的三个成员函数
  void ToProto(proto::Submap* proto,
               bool include_probability_grid_data) const override;
  void UpdateFromProto(const proto::Submap& proto) override;

  void ToResponseProto(const transform::Rigid3d& global_submap_pose,
                       proto::SubmapQuery::Response* response) const override;

  // 返回grid_
  const Grid2D* grid() const { return grid_.get(); }

  // Insert 'range_data' into this submap using 'range_data_inserter'. The
  // submap must not be finished yet.
  // 利用RangeDataInserterInterface来插入并更新概率图。该接口在/mapping/range_data_inserter_interface.h中定义
  // ProbabilityGridRangeDataInserter2D继承了该接口，定义在/mapping/2d/probability_grid_range_data_inserter_2d.h中
  void InsertRangeData(const sensor::RangeData& range_data,
                       const RangeDataInserterInterface* range_data_inserter);
  void Finish();

 private:
  std::unique_ptr<Grid2D> grid_;  // 概率图数据存储在这里
};

// Except during initialization when only a single submap exists, there are
// always two submaps into which range data is inserted: an old submap that is
// used for matching, and a new one, which will be used for matching next, that
// is being initialized.
// 除了在初始化期间只有一个 submap 存在的情况之外，range data 总是会插入到两个 submap 中：
// 一个用于匹配的旧 submap，另一个正在初始化，用于接下来要匹配的新 submap。
//
// Once a certain number of range data have been inserted, the new submap is
// considered initialized: the old submap is no longer changed, the "new" submap
// is now the "old" submap and is used for scan-to-map matching. Moreover, a
// "new" submap gets created. The "old" submap is forgotten by this object.
// 一旦插入一定数量的 range data，就将新 submap 视为已初始化：旧 submap 不再更改，
// “新”submap 现在是“旧”submap，并用于 scan-to-map 匹配。
// 此外，将创建一个“新”submap。该对象会遗忘掉“旧”submap。
//
// 在 cartographer 中总是同时存在着两个 Submap：Old Submap 和 New Submap。
// Old Submap 是用来做 matching，New submap 则用来 matching next。
// 每一帧 RangeData 数据都要同时插入到两个 submap 中。当插入 Old Submap 中的传感器帧数达到一定数量
// （在配置文件/src/cartographer/configuration_files/trajectory_builder_2d.lua中设置-submap/num_range_data）时，
// Old submap 就不再改变，这时 Old Submap 开始进行 Loop Closure，被加入到 submap 的 list 中，
// 设置 matching_submap_index 增加1，然后被 ActiveSubmap 这个 object 所遗忘，
// 而原先的 New submap 则变成新的 Old Submap，同时通过 AddSubmap 函数重新初始化一个 submap。
class ActiveSubmaps2D {
 public:
  explicit ActiveSubmaps2D(const proto::SubmapsOptions2D& options);

  ActiveSubmaps2D(const ActiveSubmaps2D&) = delete;
  ActiveSubmaps2D& operator=(const ActiveSubmaps2D&) = delete;

  // Returns the index of the newest initialized Submap which can be
  // used for scan-to-map matching.
  // 返回正在执行 scan-to-map matching 的 submap 的 index
  int matching_index() const;

  // Inserts 'range_data' into the Submap collection.
  // 将“range_data”插入 Submap 集合
  // 插入传感器数据
  void InsertRangeData(const sensor::RangeData& range_data);

  std::vector<std::shared_ptr<Submap2D>> submaps() const;

 private:
  std::unique_ptr<RangeDataInserterInterface> CreateRangeDataInserter();
  std::unique_ptr<GridInterface> CreateGrid(const Eigen::Vector2f& origin);
  void FinishSubmap();
  void AddSubmap(const Eigen::Vector2f& origin);

  const proto::SubmapsOptions2D options_;
  int matching_submap_index_ = 0;
  std::vector<std::shared_ptr<Submap2D>> submaps_;
  std::unique_ptr<RangeDataInserterInterface> range_data_inserter_;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_SUBMAP_2D_H_
