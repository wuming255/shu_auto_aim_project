#include "vision/target_selector.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

namespace srm::vision
{
namespace
{
bool finite(double value)
{
  return std::isfinite(value);
}

bool valid_limit(double value)
{
  return (finite(value) && value >= 0.0) ||
         (std::isinf(value) && value > 0.0);
}

}  // namespace

bool TargetSelection::has_target() const noexcept
{
  return selected_index.has_value() && selected_index.value() < observations.size();
}

const TargetObservation * TargetSelection::selected() const noexcept
{
  if (!has_target()) return nullptr;
  return &observations[selected_index.value()];
}

TargetObservation * TargetSelection::selected() noexcept
{
  if (!has_target()) return nullptr;
  return &observations[selected_index.value()];
}

TargetSelector::TargetSelector(TargetSelectorConfig config) : config_(std::move(config))
{
  validate_config(config_);
}

TargetSelection TargetSelector::select(
  const DetectionResult & detections, const ArmorPoseSolver & pose_solver,
  std::chrono::steady_clock::time_point timestamp, std::uint64_t frame_sequence) const
{
  TargetSelection result;
  result.observations.reserve(detections.armors.size());

  for (std::size_t detection_index = 0; detection_index < detections.armors.size();
       ++detection_index) {
    TargetObservation observation;
    observation.frame_sequence = frame_sequence;
    observation.timestamp = timestamp;
    observation.detection_index = detection_index;
    observation.armor = detections.armors[detection_index];

    try {
      observation.pose = pose_solver.solve(observation.armor);
    } catch (const std::exception & exception) {
      // 检测器正常输出时点应为有限值；这里仍把单个坏候选隔离，避免一帧中其他
      // 有效目标被 OpenCV 输入检查异常连带丢弃。
      observation.pose.valid = false;
      observation.pose.error = exception.what();
    }

    observation.aim_error_rad = aim_error(observation.pose);
    result.observations.push_back(std::move(observation));
  }

  for (std::size_t observation_index = 0; observation_index < result.observations.size();
       ++observation_index) {
    const auto & candidate = result.observations[observation_index];
    if (!eligible(candidate)) continue;

    if (!result.selected_index.has_value()) {
      result.selected_index = observation_index;
      continue;
    }

    const auto & current = result.observations[result.selected_index.value()];
    const auto better = [this](const TargetObservation & lhs, const TargetObservation & rhs) {
      constexpr double kTieTolerance = 1e-12;
      if (lhs.aim_error_rad + kTieTolerance < rhs.aim_error_rad) return true;
      if (rhs.aim_error_rad + kTieTolerance < lhs.aim_error_rad) return false;

      if (lhs.pose.reprojection_rms_px + kTieTolerance < rhs.pose.reprojection_rms_px) {
        return true;
      }
      if (rhs.pose.reprojection_rms_px + kTieTolerance < lhs.pose.reprojection_rms_px) {
        return false;
      }

      if (lhs.pose.range_m + kTieTolerance < rhs.pose.range_m) return true;
      if (rhs.pose.range_m + kTieTolerance < lhs.pose.range_m) return false;

      // 最后使用原始检测顺序保证结果稳定，避免同分目标在不同 STL 实现中改变。
      return lhs.detection_index < rhs.detection_index;
    };

    if (better(candidate, current)) result.selected_index = observation_index;
  }

  return result;
}

const TargetSelectorConfig & TargetSelector::config() const noexcept
{
  return config_;
}

bool TargetSelector::eligible(const TargetObservation & observation) const noexcept
{
  if (!observation.pose.valid) return false;
  if (config_.filter_color && observation.armor.color != config_.required_color) return false;
  if (!finite(observation.aim_error_rad) || observation.aim_error_rad > config_.max_aim_error_rad) {
    return false;
  }
  if (!finite(observation.pose.range_m) || observation.pose.range_m > config_.max_range_m) {
    return false;
  }
  if (
    !finite(observation.pose.reprojection_rms_px) ||
    observation.pose.reprojection_rms_px > config_.max_reprojection_rms_px) {
    return false;
  }
  return true;
}

double TargetSelector::aim_error(const PoseSolution & pose) noexcept
{
  if (!pose.valid || !finite(pose.yaw_rad) || !finite(pose.pitch_rad)) {
    return std::numeric_limits<double>::infinity();
  }
  return std::hypot(pose.yaw_rad, pose.pitch_rad);
}

void TargetSelector::validate_config(const TargetSelectorConfig & config)
{
  if (config.filter_color && config.required_color == LightColor::unknown) {
    throw std::invalid_argument("required target color must be red or blue when filtering");
  }
  if (!valid_limit(config.max_aim_error_rad)) {
    throw std::invalid_argument("maximum target aim error must be finite and non-negative or +inf");
  }
  if (!valid_limit(config.max_range_m)) {
    throw std::invalid_argument("maximum target range must be finite and non-negative or +inf");
  }
  if (!valid_limit(config.max_reprojection_rms_px)) {
    throw std::invalid_argument(
      "maximum target reprojection RMS must be finite and non-negative or +inf");
  }
}

}  // namespace srm::vision
