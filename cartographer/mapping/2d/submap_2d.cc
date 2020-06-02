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

#include "cartographer/mapping/2d/submap_2d.h"

#include <cinttypes>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <limits>

#include "Eigen/Geometry"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/port.h"
#include "cartographer/mapping/2d/probability_grid_range_data_inserter_2d.h"
#include "cartographer/mapping/range_data_inserter_interface.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

/*
 * （1）CreateSubmapsOptions2D()函数的功能为获取 submaps 的配置参数。
 * （2）proto::SubmapsOptions2D 是一个 ProtocolBuffer 消息类型，消息类型定义在
 *     “src/cartographer/cartographer/mapping/proto/2d/submaps_options_2d.proto”文件中。
 * （3）参数配置在“/src/cartographer/configuration_files/trajectory_builder_2d.lua”文件中。
 */
proto::SubmapsOptions2D CreateSubmapsOptions2D(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::SubmapsOptions2D options;
  // num_range_data 主要配置submaps的尺寸，2d 构图的默认设置为90。
  // 添加一个新的submap之前的测距仪数据的数量。每个submap将获得插入的测距仪数据的两倍数量：首先进行初始化而不匹配，然后进行匹配。
  // 这个参数主要配置submaps的尺寸，在调试的时候：一方面，submaps必须足够小，这样它们内部的偏移就会低于分辨率，因此它们在局部是正确的；
  // 另一方面，它应该足够大，保证闭环能够正常工作。
  options.set_num_range_data(
      parameter_dictionary->GetNonNegativeInt("num_range_data"));
  // grid_options_2d 主要配置栅格地图的选项。
  // 其中，grid_type 默认为“PROBABILITY_GRID”；而 resolution 为地图的分辨率，以米为单位，默认为0.05。
  *options.mutable_grid_options_2d() = CreateGridOptions2D(
      parameter_dictionary->GetDictionary("grid_options_2d").get());
  *options.mutable_range_data_inserter_options() =
      CreateRangeDataInserterOptions(
          parameter_dictionary->GetDictionary("range_data_inserter").get());

  bool valid_range_data_inserter_grid_combination = false;
  const proto::GridOptions2D_GridType& grid_type =  // 获取 grid_type，一般为"PROBABILITY_GRID"
      options.grid_options_2d().grid_type();
  const proto::RangeDataInserterOptions_RangeDataInserterType&
      range_data_inserter_type =  // 获取 range_data_inserter_type，一般为"PROBABILITY_GRID_INSERTER_2D"
          options.range_data_inserter_options().range_data_inserter_type();
  // 判断 grid_type 是否为"PROBABILITY_GRID"，以及 range_data_inserter_type 是否为"PROBABILITY_GRID_INSERTER_2D"
  if (grid_type == proto::GridOptions2D::PROBABILITY_GRID &&
      range_data_inserter_type ==
          proto::RangeDataInserterOptions::PROBABILITY_GRID_INSERTER_2D) {
    valid_range_data_inserter_grid_combination = true;
  }
  CHECK(valid_range_data_inserter_grid_combination)
      << "Invalid combination grid_type " << grid_type
      << " with range_data_inserter_type " << range_data_inserter_type;
  CHECK_GT(options.num_range_data(), 0);
  return options;
}

// 构造函数。
// 该函数有两个参数，其中 origin 给出了子图的位置，而 grid 则指示了子图实际用于保存数据的对象。
// 在参数列表中通过父类 Submap 的构造函数设置了子图的位姿。
// 可以看到，在构造 2D 子图时，Submap 的坐标系旋转角度设置为0，而原点由参数 origin 给出。
Submap2D::Submap2D(const Eigen::Vector2f& origin, std::unique_ptr<Grid2D> grid)
    : Submap(transform::Rigid3d::Translation(
          Eigen::Vector3d(origin.x(), origin.y(), 0.))) {
  // 通过 std::move 赋予成员变量 grid_
  grid_ = std::move(grid);
}

// 从消息类型为 proto::Submap2D 的 proto 流中构建 Submap2D
// proto::Submap2D 是一个 ProtocolBuffer 消息类型，消息类型定义在
// “src/cartographer/cartographer/mapping/proto/submap.proto”文件中。
Submap2D::Submap2D(const proto::Submap2D& proto)
    : Submap(transform::ToRigid3(proto.local_pose())) {  // 设置 local_pose_
  if (proto.has_grid()) {
    CHECK(proto.grid().has_probability_grid_2d());
    grid_ = common::make_unique<ProbabilityGrid>(proto.grid());
  }
  set_num_range_data(proto.num_range_data());  // 设置 num_range_data_
  set_finished(proto.finished());  // 设置 finished_
}

// 序列化，存到 proto 中
// proto::Submap 是一个 ProtocolBuffer 消息类型，消息类型定义在
// “src/cartographer/cartographer/mapping/proto/submap.proto”文件中，
// 定义文件里包含 Submap2D 和 Submap3D 两个消息类型。
void Submap2D::ToProto(proto::Submap* const proto,
                       bool include_probability_grid_data) const {
  auto* const submap_2d = proto->mutable_submap_2d();  // 返回一个指向 Submap2D 的指针
  *submap_2d->mutable_local_pose() = transform::ToProto(local_pose());  // 保存 local_pose
  submap_2d->set_num_range_data(num_range_data());  // 保存 num_range_data
  submap_2d->set_finished(finished());  // 保存 finished
  if (include_probability_grid_data) {
    CHECK(grid_);
    *submap_2d->mutable_grid() = grid_->ToProto();  // 调用 grid_ 中的 ToProto 函数把概率图保存到 proto 中
  }
}

// 从消息类型为 proto::Submap 的 proto 流中获取 Submap2D
void Submap2D::UpdateFromProto(const proto::Submap& proto) {
  CHECK(proto.has_submap_2d());  // 检查 proto 流中是否含有 Submap2D 分量
  const auto& submap_2d = proto.submap_2d();
  set_num_range_data(submap_2d.num_range_data());  // 设置 num_range_data_
  set_finished(submap_2d.finished());  // 设置 finished_
  if (proto.submap_2d().has_grid()) {
    CHECK(proto.submap_2d().grid().has_probability_grid_2d());
    grid_ = common::make_unique<ProbabilityGrid>(submap_2d.grid());
  }
}

// 放到 response 中
// proto::SubmapQuery::Response 是一个 ProtocolBuffer 消息类型，消息类型定义在
// “src/cartographer/cartographer/mapping/proto/submap_visualization.proto”文件中。
void Submap2D::ToResponseProto(
    const transform::Rigid3d&,
    proto::SubmapQuery::Response* const response) const {
  if (!grid_) return;
  response->set_submap_version(num_range_data());  // 设置 response 中的 submap_version 字段
  proto::SubmapQuery::Response::SubmapTexture* const texture =
      response->add_textures();
  grid()->DrawToSubmapTexture(texture, local_pose());  // 调用 grid_ 中的 DrawToSubmapTexture 函数设置 response 中的 SubmapTexture 字段
}

// 将激光的扫描数据插入到 grid_ 对象中。该函数有两个参数。
// range_data: 将要插入的扫描数据。
// range_data_inserter: 是一个辅助的工具，实际就是类 ActiveSubmaps2D 中的成员
//                      range_data_inserter_，具体负责插入数据的方式方法。
void Submap2D::InsertRangeData(
    const sensor::RangeData& range_data,
    const RangeDataInserterInterface* range_data_inserter) {
  CHECK(grid_);  // 检查是否栅格化
  CHECK(!finished());  // 检查图是否已被 finished
  // 调用 RangeDataInserterInterface 来更新概率图
  // range_data 表示激光测距仪一帧的数据，其中 origin 表示原点，returns 表示 hits 的点集合，misses 表示 free 的点集合。
  range_data_inserter->Insert(range_data, grid_.get());
  set_num_range_data(num_range_data() + 1);  // 插入的数据+1
}

// 负责终止子图的更新
void Submap2D::Finish() {
  CHECK(grid_);
  CHECK(!finished());
  // 通知 grid_ 对象裁剪重叠的栅格
  grid_ = grid_->ComputeCroppedGrid();
  // 通过父类的接口 set_finished() 更新完成标志
  set_finished(true);
}

// 构造函数。
// 该函数有一个参数 options 用于配置子图的选项，在它的成员初始化列表中，直接用来构建对象 options_。
// 该函数还通过私有的成员函数 CreateRangeDataInserter() 构建了一个插入器对象，并赋予了 range_data_inserter_。
ActiveSubmaps2D::ActiveSubmaps2D(const proto::SubmapsOptions2D& options)
    : options_(options),
      range_data_inserter_(std::move(CreateRangeDataInserter())) {
  // We always want to have at least one likelihood field which we can return,
  // and will create it at the origin in absence of a better choice.
  // 我们总是希望至少有一个可以返回的似然字段，并在没有更好选择的情况下在原点创建它。
  // 通过函数 AddSubmap()，构建了第一个子图。这里设置原点为 [0,0]。
  AddSubmap(Eigen::Vector2f::Zero());
}

// 获取子图容器 submaps_
std::vector<std::shared_ptr<Submap2D>> ActiveSubmaps2D::submaps() const {
  return submaps_;
}

// 获取当前的匹配子图索引
int ActiveSubmaps2D::matching_index() const { return matching_submap_index_; }

// 将扫描数据插入到子图中
void ActiveSubmaps2D::InsertRangeData(const sensor::RangeData& range_data) {
  // 先依次将输入参数 range_data 填进容器 submaps_ 的子图中
  for (auto& submap : submaps_) {
    submap->InsertRangeData(range_data, range_data_inserter_.get());
  }
  // 然后，检查新图中插入的数据数量，当超过配置项 num_range_data 的时候就会调用 AddSubmap() 抛弃旧图创建新图。
  if (submaps_.back()->num_range_data() == options_.num_range_data()) {
    AddSubmap(range_data.origin.head<2>());  // 新创建子图的坐标原点为扫描数据的原点
  }
}

// 构建一个插入器对象
std::unique_ptr<RangeDataInserterInterface>
ActiveSubmaps2D::CreateRangeDataInserter() {
  // 将实例化的插入器返回。
  // 该函数的返回值和用于保存插入器对象的变量的数据类型都是接口类 RangeDataInserterInterface，
  // 而真正的插入器对象的数据类型是 ProbabilityGridRangeDataInserter2D。这应该是一种概率栅格形式的地图。
  return common::make_unique<ProbabilityGridRangeDataInserter2D>(
      options_.range_data_inserter_options()
          .probability_grid_range_data_inserter_options_2d());
}

// 为子图创建栅格信息存储结构
std::unique_ptr<GridInterface> ActiveSubmaps2D::CreateGrid(
    const Eigen::Vector2f& origin) {
  // 获取子图尺寸和分辨率信息
  constexpr int kInitialSubmapSize = 100;
  float resolution = options_.grid_options_2d().resolution();
  // 构建一个 ProbabilityGrid 类型的栅格存储
  return common::make_unique<ProbabilityGrid>(
      MapLimits(resolution,
                origin.cast<double>() + 0.5 * kInitialSubmapSize * resolution *
                                            Eigen::Vector2d::Ones(),
                CellLimits(kInitialSubmapSize, kInitialSubmapSize)));
}

// 完成新旧图的切换
void ActiveSubmaps2D::FinishSubmap() {
  // 获取当前的旧图，通过旧图对象的成员函数 Finish() 完成一些收尾工作。
  Submap2D* submap = submaps_.front().get();  // 容器的第一个元素是旧图
  submap->Finish();
  // 增加 matching_submap_index_ 记录当前新图索引
  ++matching_submap_index_;
  // 最后抛弃旧图对象。
  // 需要强调一点，这里所说的抛弃只是不在容器 submaps_ 中存放对象了，
  // 并不意味者对象就此销毁了，因为容器是以共享指针的形式保存子图对象的。
  submaps_.erase(submaps_.begin());
}

// 新建一个子图
// origin 是新建子图的原点坐标
void ActiveSubmaps2D::AddSubmap(const Eigen::Vector2f& origin) {
  // 先检查一下容器 submaps_ 中的子图数量，如果不只有一个子图，
  // 就需要在函数 FinishSubmap() 中完成新旧图的切换工作。
  if (submaps_.size() > 1) {
    // This will crop the finished Submap before inserting a new Submap to
    // reduce peak memory usage a bit.
    FinishSubmap();
  }

  // 在构造函数中我们是第一次调用该函数，会直接构建一个新的 Submap2D 类型的对象放置到容器 submaps_ 中，
  // 它将作为一个新图用于插入数据。
  // 在构建 Submap2D 类型的对象的时候，还调用了函数 CreateGrid() 为该对象提供了一个保存栅格占用信息的存储结构。
  submaps_.push_back(common::make_unique<Submap2D>(
      origin, std::unique_ptr<Grid2D>(
                  static_cast<Grid2D*>(CreateGrid(origin).release()))));
  LOG(INFO) << "Added submap " << matching_submap_index_ + submaps_.size();
}

}  // namespace mapping
}  // namespace cartographer
