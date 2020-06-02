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
   * @param grid      Grid2D 变量，存储栅格化坐标和坐标上的概率值。
   *                  Grid2D 定义在 “/mapping/2d/grid_2d.h” 中，继承了 GridInterface(/mapping/grid_interface.h)，
   *                  Grid2D 又被 ProbabilityGrid 继承，定义在 “/mapping/2d/probability_grid.h” 中。
   *                  基本数据都存储在类型为 Grid2D 的成员变量 grid_ 中。
   * @return
   */
  Submap2D(const Eigen::Vector2f& origin, std::unique_ptr<Grid2D> grid);
  explicit Submap2D(const proto::Submap2D& proto);

   // 实现接口 Submap 中的三个成员函数
  void ToProto(proto::Submap* proto,
               bool include_probability_grid_data) const override;
  void UpdateFromProto(const proto::Submap& proto) override;

  void ToResponseProto(const transform::Rigid3d& global_submap_pose,
                       proto::SubmapQuery::Response* response) const override;

  // 返回 grid_
  const Grid2D* grid() const { return grid_.get(); }

  // Insert 'range_data' into this submap using 'range_data_inserter'. The
  // submap must not be finished yet.
  // 利用 RangeDataInserterInterface 来插入并更新概率图。该接口在 “/mapping/range_data_inserter_interface.h” 中定义。
  // ProbabilityGridRangeDataInserter2D 继承了该接口，定义在 “/mapping/2d/probability_grid_range_data_inserter_2d.h” 中。
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
// 除了在初始化期间只有一个子图存在的情况之外，扫描数据总是会插入到两个子图中：
// 一个是旧图用来做匹配，另一个是正在初始化的新图，用于接下来做匹配。
//
// Once a certain number of range data have been inserted, the new submap is
// considered initialized: the old submap is no longer changed, the "new" submap
// is now the "old" submap and is used for scan-to-map matching. Moreover, a
// "new" submap gets created. The "old" submap is forgotten by this object.
// 新图一旦插入一定数量的扫描数据，就视为完成初始化：旧图不再更改，“新”图会变成“旧”图，并用于扫描匹配。
// 此外，将生成一个“新”图。该对象将抛弃原来的“旧”图。
//
// 除了刚开始构建该对象的时候，只有一个子图(Submap2D)，其他时候它都维护着两个子图对象。
// 一个子图用于进行扫描匹配，称为旧图。另一个子图被称为新图用于插入扫描数据。
// 当新图中插入一定数量的数据完成了初始化操作之后，它就会被当作旧图，用于扫描匹配。
// 对象将抛弃原来的旧图，并重新构建一个新图。
class ActiveSubmaps2D {
 public:
  /**
   * @brief ActiveSubmaps2D  构造函数。该函数有一个参数用于配置子图的选项。
   * @param options          子图的配置选项
   */
  explicit ActiveSubmaps2D(const proto::SubmapsOptions2D& options);

  ActiveSubmaps2D(const ActiveSubmaps2D&) = delete;
  ActiveSubmaps2D& operator=(const ActiveSubmaps2D&) = delete;

  // Returns the index of the newest initialized Submap which can be
  // used for scan-to-map matching.
  // 获取当前的匹配子图索引
  int matching_index() const;

  // Inserts 'range_data' into the Submap collection.
  // 将 "range_data" 插入子图集合
  /**
   * @brief InsertRangeData  将扫描数据插入到子图中
   * @param range_data       扫描数据
   */
  void InsertRangeData(const sensor::RangeData& range_data);

  // 获取子图容器 submaps_
  std::vector<std::shared_ptr<Submap2D>> submaps() const;

 private:
  /**
   * @brief AddSubmap  构建一个插入器对象
   * @return           新建的插入器对象
   */
  std::unique_ptr<RangeDataInserterInterface> CreateRangeDataInserter();
  /**
   * @brief CreateGrid  为子图创建栅格信息存储结构
   * @param origin      新建子图的原点坐标
   */
  std::unique_ptr<GridInterface> CreateGrid(const Eigen::Vector2f& origin);
  /**
   * @brief FinishSubmap  完成新旧图的切换
   */
  void FinishSubmap();
  /**
   * @brief AddSubmap  新建一个子图
   * @param origin     新建子图的原点坐标
   */
  void AddSubmap(const Eigen::Vector2f& origin);

  const proto::SubmapsOptions2D options_;                            // 子图的配置选项
  int matching_submap_index_ = 0;                                    // 记录了当前用于扫描匹配的旧图索引
  std::vector<std::shared_ptr<Submap2D>> submaps_;                   // 保存当前维护子图的容器
  std::unique_ptr<RangeDataInserterInterface> range_data_inserter_;  // 用于将扫描数据插入子图的工具，我们称它为插入器
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_SUBMAP_2D_H_
