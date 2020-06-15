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

#include "cartographer/mapping/probability_values.h"

#include "cartographer/common/make_unique.h"

namespace cartographer {
namespace mapping {

namespace {

// 0 is unknown, [1, 32767] maps to [lower_bound, upper_bound].
// 将一个 uint16 型的 value 转成一个浮点数。value 的范围是 [1,32767]。
// 若 value 为 0，表示是 unknown。
// 若是 [1,32767]，则映射到浮点型的范围 [lower_bound, upper_bound].
float SlowValueToBoundedFloat(const uint16 value, const uint16 unknown_value,
                              const float unknown_result,
                              const float lower_bound,
                              const float upper_bound) {
  CHECK_GE(value, 0);
  CHECK_LE(value, 32767);
  if (value == unknown_value) return unknown_result;
  const float kScale = (upper_bound - lower_bound) / 32766.f;
  return value * kScale + (lower_bound - kScale);
}

// 把 [1,32767] 之间的所有 value 预先计算出来其映射到 [lower_bound, upper_bound] 这个区间的对应浮点值，存到一个浮点型向量中：
std::unique_ptr<std::vector<float>> PrecomputeValueToBoundedFloat(
    const uint16 unknown_value, const float unknown_result,
    const float lower_bound, const float upper_bound) {
  auto result = common::make_unique<std::vector<float>>();
  // Repeat two times, so that both values with and without the update marker
  // can be converted to a probability.
  for (int repeat = 0; repeat != 2; ++repeat) {
    for (int value = 0; value != 32768; ++value) {
      result->push_back(SlowValueToBoundedFloat(
          value, unknown_value, unknown_result, lower_bound, upper_bound));
    }
  }
  return result;
}

// 将 [1,32767] 之间的所有 value 值映射到 Probability
std::unique_ptr<std::vector<float>> PrecomputeValueToProbability() {
  return PrecomputeValueToBoundedFloat(kUnknownProbabilityValue,
                                       kMinProbability, kMinProbability,
                                       kMaxProbability);
}

// 将 [1,32767] 之间的所有 value 值映射到 CorrespondenceCost
std::unique_ptr<std::vector<float>> PrecomputeValueToCorrespondenceCost() {
  return PrecomputeValueToBoundedFloat(
      kUnknownCorrespondenceValue, kMaxCorrespondenceCost,
      kMinCorrespondenceCost, kMaxCorrespondenceCost);
}

}  // namespace

// 把预先计算出来的 Probability 和 CorrespondenceCost 放到在 probability_values.h 中定义的
// 两个向量 kValueToProbability 和 kValueToCorrespondenceCost 中。
// 这样，以后直接以 value 为索引值查表就可以获得其对应的 probability 或 correspondence_cost。
const std::vector<float>* const kValueToProbability =
    PrecomputeValueToProbability().release();

const std::vector<float>* const kValueToCorrespondenceCost =
    PrecomputeValueToCorrespondenceCost().release();

// 该函数的含义是，对于一个 value~[1,32767], 如果有一个新的 odds 值的观测后，更新后的 value 应该是什么。
// 这里对所有可能的 value 都进行了计算，存在了一个列表中。odds 只有两种情况，hit 或 misses。
// 因此，可以预先计算出来两个列表。这样，在有一个新的 odds 时可根据原有的 value 值查表得到一个新的 value 值，更新。
std::vector<uint16> ComputeLookupTableToApplyOdds(const float odds) {
  std::vector<uint16> result;
  // 这个表示这个表中的第一个元素对应了如果之前该点是 unknown 状态，更新的 value 应该是什么
  result.push_back(ProbabilityToValue(ProbabilityFromOdds(odds)) +
                   kUpdateMarker);
  for (int cell = 1; cell != 32768; ++cell) {
    result.push_back(ProbabilityToValue(ProbabilityFromOdds(
                         odds * Odds((*kValueToProbability)[cell]))) +
                     kUpdateMarker);
  }
  return result;
}
// 但在 ComputeLookupTableToApplyOdds 这个转化里都加了一个 kUpdateMarker，相当于有了一个偏移，但为什么我没想明白

// 函数 ComputeLookupTableToApplyCorrespondenceCostOdds() 用于构建查找表。
// 它的输入参数是 C(hit) 或 C(miss)，分别表示 hit 事件和 miss 事件发生时的更新系数。
// 它的作用是处理某一个单元格的空闲概率数值(CorrespondenceCostValue)如何进行更新的情况。
std::vector<uint16> ComputeLookupTableToApplyCorrespondenceCostOdds(
    float odds) {
  // 在函数一开始先临时建了一个容器用于保存返回结果。
  std::vector<uint16> result;
  // 计算未知栅格更新后的数值。
  // 首先对 odds 求逆将其转换成概率值(ProbabilityFromOdds)，再取反(ProbabilityToCorrespondenceCost)，
  // 然后转换成为存储值(CorrespondenceCostToValue)，最后加上 kUpdateMarker 把结果塞进了 result 中。
  result.push_back(CorrespondenceCostToValue(ProbabilityToCorrespondenceCost(
                       ProbabilityFromOdds(odds))) +
                   kUpdateMarker);
  // 然后在一个 for 循环中以同样费劲方式依次计算存储值从1到32767所对应的更新值并塞进 result 中返回。
  // (*kValueToCorrespondenceCost)[cell] 是栅格存储值为 cell 时所对应的栅格的空闲概率。
  for (int cell = 1; cell != 32768; ++cell) {
    result.push_back(
        CorrespondenceCostToValue(
            ProbabilityToCorrespondenceCost(ProbabilityFromOdds(
                odds * Odds(CorrespondenceCostToProbability(
                           (*kValueToCorrespondenceCost)[cell]))))) +
        kUpdateMarker);
  }
  return result;
}

}  // namespace mapping
}  // namespace cartographer
