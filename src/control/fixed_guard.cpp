#include "control/fixed_guard.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace srm::control
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
}

FixedGuardController::FixedGuardController(FixedGuardConfig config) : config_(std::move(config))
{
  validate_config(config_);
}

FixedGuardResult FixedGuardController::update(
  const vision::RotationEstimate & rotation, const AimCompensator & compensator,
  double bullet_speed_m_s, std::chrono::steady_clock::time_point timestamp)
{
  FixedGuardResult result;
  // 模型未就绪时也保留输入，避免把默认 0 误记为下位机弹速异常。
  result.aim.bullet_speed_m_s = bullet_speed_m_s;
  result.target_visible = rotation.current_visible;

  if (!rotation.current_visible && rotation.observation_age_seconds > config_.max_hold_seconds) {
    guard_locked_ = false;
    held_aim_.reset();
    last_model_yaw_valid_ = false;
    result.status = "long_loss_reacquire";
    return result;
  }

  if (!rotation.initialized || !rotation.prediction_valid || !rotation.stable_frame_valid ||
      (rotation.current_visible && !rotation.reliable)) {
    result.status = rotation.current_visible ? "guard_model_not_reliable" : "guard_unavailable";
    guard_locked_ = false;
    held_aim_.reset();
    last_model_yaw_valid_ = false;
    return result;
  }

  const bool center_changed =
    guard_locked_ && cv::norm(rotation.center_world_m - locked_center_world_m_) >
      config_.relocalization_center_threshold_m;
  const bool radius_changed =
    guard_locked_ && std::abs(rotation.radius_m - locked_radius_m_) >
      config_.relocalization_radius_threshold_m;

  const auto world_position = rotation.armor_position(0.0, rotation.associated_plate);
  const auto gimbal_position = rotation.rotation_gimbal_to_world.t() * world_position;
  const double model_yaw = std::atan2(gimbal_position[1], gimbal_position[0]);
  const bool yaw_changed = guard_locked_ && last_model_yaw_valid_ &&
    std::abs(phase_difference(model_yaw, last_model_yaw_rad_)) >
      config_.relocalization_yaw_threshold_rad;
  const bool should_relocalize = !guard_locked_ || center_changed || radius_changed || yaw_changed;

  if (should_relocalize) {
    const bool first_lock = !guard_locked_;
    guard_locked_ = true;
    locked_center_world_m_ = rotation.center_world_m;
    locked_radius_m_ = rotation.radius_m;
    if (first_lock) guard_phase_rad_ = rotation.phase_rad;
    result.relocalized = true;
  }
  last_model_yaw_rad_ = model_yaw;
  last_model_yaw_valid_ = finite(model_yaw);

  result.guard_locked = guard_locked_;
  result.guard_phase_rad = guard_phase_rad_;

  if (!rotation.current_visible && held_aim_.has_value()) {
    result.aim = *held_aim_;
    result.aim.valid = true;
    result.status = "temporarily_lost_hold";
  } else {
    auto guard_rotation = rotation;
    guard_rotation.center_world_m = locked_center_world_m_;
    guard_rotation.radius_m = locked_radius_m_;
    const auto target = make_target(guard_rotation, rotation.associated_plate);
    result.aim = compensator.solve(target, bullet_speed_m_s);
    if (!result.aim.valid) {
      result.status = "guard_ballistic_invalid";
      return result;
    }
    held_aim_ = result.aim;
    result.status = "guard_locked";
  }

  result.predicted_phase_rad = rotation.phase_rad +
    rotation.angular_velocity_rad_s * result.aim.total_prediction_lead_s;
  result.phase_error_rad = phase_difference(result.predicted_phase_rad, guard_phase_rad_);
  result.fire_window_valid = rotation.current_visible &&
    std::abs(result.phase_error_rad) <= config_.hit_window_half_width_rad;
  result.guard_locked = guard_locked_;
  result.guard_phase_rad = guard_phase_rad_;
  (void)timestamp;
  return result;
}

void FixedGuardController::reset() noexcept
{
  guard_locked_ = false;
  locked_center_world_m_ = {};
  locked_radius_m_ = 0.0;
  guard_phase_rad_ = 0.0;
  last_model_yaw_rad_ = 0.0;
  last_model_yaw_valid_ = false;
  held_aim_.reset();
}

const FixedGuardConfig & FixedGuardController::config() const noexcept
{
  return config_;
}

bool FixedGuardController::finite(double value) noexcept
{
  return std::isfinite(value);
}

void FixedGuardController::validate_config(const FixedGuardConfig & config)
{
  if (!finite(config.relocalization_center_threshold_m) ||
      config.relocalization_center_threshold_m <= 0.0 ||
      !finite(config.relocalization_radius_threshold_m) ||
      config.relocalization_radius_threshold_m <= 0.0 ||
      !finite(config.relocalization_yaw_threshold_rad) ||
      config.relocalization_yaw_threshold_rad <= 0.0 ||
      !finite(config.hit_window_half_width_rad) || config.hit_window_half_width_rad <= 0.0 ||
      config.hit_window_half_width_rad >= kPi || !finite(config.max_hold_seconds) ||
      config.max_hold_seconds <= 0.0) {
    throw std::invalid_argument("invalid fixed guard configuration");
  }
}

double FixedGuardController::phase_difference(double lhs, double rhs) noexcept
{
  return std::remainder(lhs - rhs, 2.0 * kPi);
}

vision::TargetTrackEstimate FixedGuardController::make_target(
  const vision::RotationEstimate & rotation, int plate)
{
  vision::TargetTrackEstimate target;
  target.state = vision::TargetTrackState::tracking;
  target.has_current_measurement = rotation.current_visible;
  target.has_last_measurement = true;
  target.prediction_valid = true;
  target.velocity_valid = true;

  const auto world_position = rotation.armor_position(0.0, plate);
  const auto position = rotation.rotation_gimbal_to_world.t() * world_position;
  const auto radial = world_position - rotation.center_world_m;
  const auto world_velocity = cv::Vec3d(
    -rotation.angular_velocity_rad_s * radial[1],
    rotation.angular_velocity_rad_s * radial[0], 0.0);
  const auto velocity = rotation.rotation_gimbal_to_world.t() * world_velocity;
  const double horizontal = std::hypot(position[0], position[1]);
  const double range = cv::norm(position);

  target.measured_yaw_rad = std::atan2(position[1], position[0]);
  target.unwrapped_yaw_rad = target.measured_yaw_rad;
  target.measured_pitch_rad = std::atan2(position[2], horizontal);
  target.measured_range_m = range;
  target.predicted_yaw_rad = target.measured_yaw_rad;
  target.predicted_pitch_rad = target.measured_pitch_rad;
  target.predicted_range_m = range;
  if (horizontal > 1e-9 && range > 1e-9) {
    target.yaw_velocity_rad_s =
      (position[0] * velocity[1] - position[1] * velocity[0]) / (horizontal * horizontal);
    target.pitch_velocity_rad_s =
      (horizontal * velocity[2] - position[2] *
        (position[0] * velocity[0] + position[1] * velocity[1]) / horizontal) /
      (range * range);
    target.range_velocity_m_s = position.dot(velocity) / range;
  }
  target.timestamp = rotation.timestamp;
  target.last_observation = rotation.current_observation;
  return target;
}

}  // namespace srm::control
