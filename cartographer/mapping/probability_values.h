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

#ifndef CARTOGRAPHER_MAPPING_PROBABILITY_VALUES_H_
#define CARTOGRAPHER_MAPPING_PROBABILITY_VALUES_H_

#include <cmath>
#include <vector>

#include "cartographer/common/math.h"
#include "cartographer/common/port.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

namespace {

// cartongrapher 为了尽量避免浮点运算，将 [kMinProbability, kMaxProbability]
// 或 [kMinCorrespondenceCost, kMaxCorrespondenceCost] 之间的浮点数映射到了整数区间：[1, 32767]。
// BoundedFloatToValue() 为由浮点数到整数区间的映射函数。
inline uint16 BoundedFloatToValue(const float float_value,
                                  const float lower_bound,
                                  const float upper_bound) {
  const int value =
      common::RoundToInt(
          (common::Clamp(float_value, lower_bound, upper_bound) - lower_bound) *
          (32766.f / (upper_bound - lower_bound))) +
      1;
  // DCHECK for performance.
  DCHECK_GE(value, 1);      // 检查是否大于等于1
  DCHECK_LE(value, 32767);  // 是否小于等于32767
  return value;
}

}  // namespace

// 由 Probability 计算 odds
inline float Odds(float probability) {
  return probability / (1.f - probability);
}

// 由 odds 计算 probability
inline float ProbabilityFromOdds(const float odds) {
  return odds / (odds + 1.f);
}

// Probability 转 CorrespondenceCost
inline float ProbabilityToCorrespondenceCost(const float probability) {
  return 1.f - probability;
}

// CorrespondenceCost 转 Probability
inline float CorrespondenceCostToProbability(const float correspondence_cost) {
  return 1.f - correspondence_cost;
}

// 定义的几个常量
constexpr float kMinProbability = 0.1f;                          // 最小概率为 0.1
constexpr float kMaxProbability = 1.f - kMinProbability;         // 最大概率为 1-0.1=0.9
constexpr float kMinCorrespondenceCost = 1.f - kMaxProbability;  // 最小 Free 概率为 0.1
constexpr float kMaxCorrespondenceCost = 1.f - kMinProbability;  // 最大 Free 概率为 0.9

// clamp 函数的含义是，如果给定参数小于最小值，则返回最小值；如果大于最大值则返回最大值；其他情况正常返回参数。
// Clamps probability to be in the range [kMinProbability, kMaxProbability].
inline float ClampProbability(const float probability) {
  return common::Clamp(probability, kMinProbability, kMaxProbability);
}
// Clamps correspondece cost to be in the range [kMinCorrespondenceCost,
// kMaxCorrespondenceCost].
inline float ClampCorrespondenceCost(const float correspondence_cost) {
  return common::Clamp(correspondence_cost, kMinCorrespondenceCost,
                       kMaxCorrespondenceCost);
}

// 在没有任何先验信息情况下，Occupied 和 Free 概率值都为 0
constexpr uint16 kUnknownProbabilityValue = 0;
constexpr uint16 kUnknownCorrespondenceValue = kUnknownProbabilityValue;
// 运算符 << 是左移，左移一位当于乘以2，所以这里 kUpdateMarker 等于2的15次方：32768，
// 也就是所以概率值转化成整数 value 之后的最大范围。所以程序中有判断是否越界。
constexpr uint16 kUpdateMarker = 1u << 15;

// Converts a correspondence_cost to a uint16 in the [1, 32767] range.
// 把浮点数 correspondence_cost 映射到 uint16 的整数区间：[1, 32767]
inline uint16 CorrespondenceCostToValue(const float correspondence_cost) {
  return BoundedFloatToValue(correspondence_cost, kMinCorrespondenceCost,
                             kMaxCorrespondenceCost);
}

// Converts a probability to a uint16 in the [1, 32767] range.
// 把浮点数 probability 映射到 uint16 的整数区间：[1, 32767]
inline uint16 ProbabilityToValue(const float probability) {
  return BoundedFloatToValue(probability, kMinProbability, kMaxProbability);
}

// 两张查找 uint16 的整数区间数值 value 映射到浮点数区间的表格
extern const std::vector<float>* const kValueToProbability;         // 整数 value 映射到浮点数 probability 的表格
extern const std::vector<float>* const kValueToCorrespondenceCost;  // 整数 value 映射到浮点数 correspondence_cost 的表格

// Converts a uint16 (which may or may not have the update marker set) to a
// probability in the range [kMinProbability, kMaxProbability].
// uint16 的整数区间数值 value 映射到浮点数 probability 的区间：[kMinProbability, kMaxProbability]
inline float ValueToProbability(const uint16 value) {
  return (*kValueToProbability)[value];
}

// Converts a uint16 (which may or may not have the update marker set) to a
// correspondence cost in the range [kMinCorrespondenceCost,
// kMaxCorrespondenceCost].
// uint16 的整数区间数值 value 映射到浮点数 correspondence_cost 的区间：
// [kMinCorrespondenceCost, kMaxCorrespondenceCost]
inline float ValueToCorrespondenceCost(const uint16 value) {
  return (*kValueToCorrespondenceCost)[value];
}

// Probability 的 Value 转成 CorrespondenceCost 的 Value:
inline uint16 ProbabilityValueToCorrespondenceCostValue(
    uint16 probability_value) {
  if (probability_value == kUnknownProbabilityValue) {
    // 如果是 Unknown 值还返回 unknown 值。Probability 和 CorrespondenceCost 的 Unknown 值都是0。
    return kUnknownCorrespondenceValue;
  }
  bool update_carry = false;
  if (probability_value > kUpdateMarker) {  // 如果该值超过最大范围：但什么情况下会导致出现该值超过范围还不清楚
    probability_value -= kUpdateMarker;  // 防止溢出范围
    update_carry = true;  // 如果存在过超出范围的行为，则将 update_carry 置为 true
  }
  // ProbabilityValue-->Probability-->CorrespondenceCost-->CorrespondenceCostValue
  uint16 result = CorrespondenceCostToValue(
      ProbabilityToCorrespondenceCost(ValueToProbability(probability_value)));
  if (update_carry) result += kUpdateMarker;  // 原先减去过一个最大范围，现在再加回来
  return result;
}

// CorrespondenceCost 的 Value 转成 Probability 的 Value
inline uint16 CorrespondenceCostValueToProbabilityValue(
    uint16 correspondence_cost_value) {
  if (correspondence_cost_value == kUnknownCorrespondenceValue)
    return kUnknownProbabilityValue;
  bool update_carry = false;
  if (correspondence_cost_value > kUpdateMarker) {
    correspondence_cost_value -= kUpdateMarker;
    update_carry = true;
  }
  uint16 result = ProbabilityToValue(CorrespondenceCostToProbability(
      ValueToCorrespondenceCost(correspondence_cost_value)));
  if (update_carry) result += kUpdateMarker;
  return result;
}

std::vector<uint16> ComputeLookupTableToApplyOdds(float odds);
/**
 * @brief ComputeLookupTableToApplyCorrespondenceCostOdds  用于构建查找表
 * @param odds                                             C(hit) 或 C(miss)，分别表示 hit 事件和 miss 事件发生时的更新系数
 * @return                                                 hit 事件或 miss 事件的查找表
 */
std::vector<uint16> ComputeLookupTableToApplyCorrespondenceCostOdds(float odds);

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_PROBABILITY_VALUES_H_
