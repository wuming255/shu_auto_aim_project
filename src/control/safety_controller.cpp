#include "control/safety_controller.hpp"

#include <cmath>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace srm::control
{
namespace
{
std::string reason_text(const std::vector<SafetyReason> & reasons)
{
  std::ostringstream stream;
  for (std::size_t index = 0; index < reasons.size(); ++index) {
    if (index != 0) stream << ",";
    stream << to_string(reasons[index]);
  }
  return stream.str();
}

}  // namespace

const char * to_string(SafetyState state) noexcept
{
  switch (state) {
    case SafetyState::safe:
      return "safe";
    case SafetyState::aiming:
      return "aiming";
    case SafetyState::fire_ready:
      return "fire_ready";
    default:
      return "unknown";
  }
}

const char * to_string(SafetyReason reason) noexcept
{
  switch (reason) {
    case SafetyReason::none:
      return "none";
    case SafetyReason::transport_not_running:
      return "transport_not_running";
    case SafetyReason::transport_disconnected:
      return "transport_disconnected";
    case SafetyReason::transport_state_missing:
      return "transport_state_missing";
    case SafetyReason::transport_state_stale:
      return "transport_state_stale";
    case SafetyReason::frame_missing:
      return "frame_missing";
    case SafetyReason::frame_stale:
      return "frame_stale";
    case SafetyReason::processing_failed:
      return "processing_failed";
    case SafetyReason::target_unavailable:
      return "target_unavailable";
    case SafetyReason::aim_invalid:
      return "aim_invalid";
    case SafetyReason::bullet_speed_invalid:
      return "bullet_speed_invalid";
    case SafetyReason::fire_disabled:
      return "fire_disabled";
    case SafetyReason::target_not_visible:
      return "target_not_visible";
    case SafetyReason::fire_window_invalid:
      return "fire_window_invalid";
    case SafetyReason::gimbal_feedback_missing:
      return "gimbal_feedback_missing";
    case SafetyReason::gimbal_feedback_stale:
      return "gimbal_feedback_stale";
    case SafetyReason::gimbal_not_settled:
      return "gimbal_not_settled";
    default:
      return "unknown";
  }
}

SafetyController::SafetyController(SafetyControllerConfig config) : config_(std::move(config))
{
  validate_config(config_);
}

SafetyOutput SafetyController::decide(const SafetyInput & input) const
{
  SafetyOutput output;

  if (!input.transport_running) add_reason(output, SafetyReason::transport_not_running);
  if (!input.transport_connected) add_reason(output, SafetyReason::transport_disconnected);

  if (!input.frame_available) {
    add_reason(output, SafetyReason::frame_missing);
  } else if (!timestamp_fresh(input.now, input.frame_timestamp, config_.max_frame_age)) {
    add_reason(output, SafetyReason::frame_stale);
  }

  bool transport_state_timestamp_valid = false;
  bool transport_bullet_speed_valid = false;
  if (!input.transport_state.has_value()) {
    add_reason(output, SafetyReason::transport_state_missing);
  } else {
    transport_state_timestamp_valid =
      input.transport_state_timestamp.has_value() &&
      timestamp_fresh(
        input.now, input.transport_state_timestamp.value(), config_.max_transport_state_age);
    if (!transport_state_timestamp_valid) {
      add_reason(output, SafetyReason::transport_state_stale);
    }

    const double transport_bullet_speed =
      effective_bullet_speed(input.transport_state->shoot.bullet_speed);
    transport_bullet_speed_valid =
      finite(transport_bullet_speed) &&
      transport_bullet_speed >= config_.min_bullet_speed_m_s &&
      transport_bullet_speed <= config_.max_bullet_speed_m_s;
    if (!transport_bullet_speed_valid) {
      add_reason(output, SafetyReason::bullet_speed_invalid);
    }
  }

  if (!input.processing_ok) add_reason(output, SafetyReason::processing_failed);

  const auto * pipeline = input.pipeline_result;
  const bool follow_prediction = config_.allow_follow_prediction_fire && pipeline &&
    pipeline->follow_target && pipeline->tracking.prediction_valid &&
    pipeline->tracking.state == vision::TargetTrackState::temporarily_lost;
  const bool base_inputs_valid =
    input.transport_running && input.transport_connected && input.frame_available &&
    timestamp_fresh(input.now, input.frame_timestamp, config_.max_frame_age) &&
    input.transport_state.has_value() && transport_state_timestamp_valid &&
    transport_bullet_speed_valid && input.processing_ok;

  if (pipeline == nullptr) {
    add_reason(output, SafetyReason::target_unavailable);
  } else {
    const bool guard_hold = pipeline->guard_command_valid &&
      pipeline->tracking.state == vision::TargetTrackState::temporarily_lost;
    if (pipeline->tracking.state != vision::TargetTrackState::tracking && !guard_hold && !follow_prediction) {
      add_reason(output, SafetyReason::target_unavailable);
    }
    if (pipeline->guard_command_valid && !pipeline->guard_target_visible) {
      add_reason(output, SafetyReason::target_not_visible);
    }
    if (pipeline->guard_command_valid && !pipeline->fire_window_valid) {
      add_reason(output, SafetyReason::fire_window_invalid);
    }
    if (!pipeline->aim.valid) add_reason(output, SafetyReason::aim_invalid);
    if (
      !finite(pipeline->aim.bullet_speed_m_s) ||
      pipeline->aim.bullet_speed_m_s < config_.min_bullet_speed_m_s ||
      pipeline->aim.bullet_speed_m_s > config_.max_bullet_speed_m_s) {
      add_reason(output, SafetyReason::bullet_speed_invalid);
    }
  }

  const bool aim_valid =
    base_inputs_valid && pipeline != nullptr &&
    (pipeline->tracking.state == vision::TargetTrackState::tracking || follow_prediction) &&
    pipeline->aim.valid && finite(pipeline->aim.yaw_deg) && finite(pipeline->aim.pitch_deg) &&
    finite(pipeline->aim.bullet_speed_m_s) &&
    pipeline->aim.bullet_speed_m_s >= config_.min_bullet_speed_m_s &&
    pipeline->aim.bullet_speed_m_s <= config_.max_bullet_speed_m_s;

  const bool guard_hold_aim_valid =
    base_inputs_valid && pipeline != nullptr && pipeline->guard_command_valid &&
    pipeline->tracking.state == vision::TargetTrackState::temporarily_lost &&
    pipeline->aim.valid && finite(pipeline->aim.yaw_deg) && finite(pipeline->aim.pitch_deg) &&
    finite(pipeline->aim.bullet_speed_m_s) &&
    pipeline->aim.bullet_speed_m_s >= config_.min_bullet_speed_m_s &&
    pipeline->aim.bullet_speed_m_s <= config_.max_bullet_speed_m_s;
  const bool command_aim_valid = aim_valid || guard_hold_aim_valid;

  if (command_aim_valid) {
    double yaw_deg = pipeline->aim.yaw_deg;
    double pitch_deg = pipeline->aim.pitch_deg;
    bool feedback_valid = true;
    if (pipeline->follow_target || pipeline->guard_command_valid) {
      const auto & feedback = input.transport_state->gimbal;
      feedback_valid = finite(feedback.yaw) && finite(feedback.pitch) && finite(feedback.roll);
      // 同济参考输出绝对角，pitch 向下为正；yaw 保持反馈附近的连续周次。
      yaw_deg = feedback.yaw + std::remainder(yaw_deg - feedback.yaw, 360.0);
      pitch_deg = -pitch_deg;
    }
    const float yaw = static_cast<float>(yaw_deg);
    const float pitch = static_cast<float>(pitch_deg);
    if (feedback_valid && std::isfinite(yaw) && std::isfinite(pitch)) {
      output.command.gimbal = {yaw, pitch};
      output.aim_command_valid = true;
    } else {
      add_reason(output, SafetyReason::aim_invalid);
    }
  }

  bool gimbal_settled = true;
  if (config_.require_gimbal_feedback_for_fire) {
    if (!input.transport_state.has_value()) {
      add_reason(output, SafetyReason::gimbal_feedback_missing);
      gimbal_settled = false;
      gimbal_settle_start_.reset();
    } else if (!input.transport_state_timestamp.has_value()) {
      add_reason(output, SafetyReason::gimbal_feedback_missing);
      gimbal_settled = false;
      gimbal_settle_start_.reset();
    } else if (!timestamp_fresh(
      input.now, *input.transport_state_timestamp, config_.max_transport_state_age)) {
      add_reason(output, SafetyReason::gimbal_feedback_stale);
      gimbal_settled = false;
      gimbal_settle_start_.reset();
    } else if (!output.aim_command_valid || !finite(input.transport_state->gimbal.yaw) ||
               !finite(input.transport_state->gimbal.pitch)) {
      add_reason(output, SafetyReason::gimbal_not_settled);
      gimbal_settled = false;
      gimbal_settle_start_.reset();
    } else {
      const double yaw_error =
        std::abs(static_cast<double>(input.transport_state->gimbal.yaw) -
                 static_cast<double>(output.command.gimbal.yaw));
      const double pitch_error =
        std::abs(static_cast<double>(input.transport_state->gimbal.pitch) -
                 static_cast<double>(output.command.gimbal.pitch));
      if (!finite(yaw_error) || !finite(pitch_error) ||
          yaw_error > config_.max_gimbal_error_deg || pitch_error > config_.max_gimbal_error_deg) {
        add_reason(output, SafetyReason::gimbal_not_settled);
        gimbal_settled = false;
        gimbal_settle_start_.reset();
      } else {
        if (!gimbal_settle_start_.has_value()) gimbal_settle_start_ = input.now;
        const auto settled_for = input.now - *gimbal_settle_start_;
        gimbal_settled = settled_for >= config_.min_gimbal_settle_time;
        if (!gimbal_settled) add_reason(output, SafetyReason::gimbal_not_settled);
      }
    }
  } else {
    gimbal_settle_start_.reset();
  }

  const bool guard_fire_conditions = pipeline == nullptr || !pipeline->guard_command_valid ||
    (pipeline->guard_target_visible && pipeline->fire_window_valid);
  const bool fire_gate =
    command_aim_valid && output.aim_command_valid && config_.allow_visual_fire &&
    !config_.fixed_bullet_speed_m_s &&
    input.visual_fire_intent && gimbal_settled && guard_fire_conditions;
  if (fire_gate) {
    output.command.fire.fire_flag = 1;
    output.fire_allowed = true;
    output.state = SafetyState::fire_ready;
  } else {
    output.command.fire.fire_flag = 0;
    if (command_aim_valid && output.aim_command_valid) {
      output.state = SafetyState::aiming;
    } else {
      output.state = SafetyState::safe;
      output.command.gimbal = {};
      if (input.transport_state &&
          transport_state_timestamp_valid && input.transport_connected &&
          finite(input.transport_state->gimbal.yaw) && finite(input.transport_state->gimbal.pitch)) {
        output.command.gimbal = {input.transport_state->gimbal.yaw, input.transport_state->gimbal.pitch};
      }
    }
    if (input.visual_fire_intent && (!config_.allow_visual_fire || config_.fixed_bullet_speed_m_s)) {
      add_reason(output, SafetyReason::fire_disabled);
    }
  }

  output.summary = reason_text(output.reasons);
  if (output.reasons.empty()) output.summary = "none";
  return output;
}

double SafetyController::effective_bullet_speed(double reported_speed_m_s) const noexcept
{
  return config_.fixed_bullet_speed_m_s.value_or(
    reported_speed_m_s == 0.0 ? kDefaultBulletSpeedMps : reported_speed_m_s);
}

const SafetyControllerConfig & SafetyController::config() const noexcept
{
  return config_;
}

bool SafetyController::finite(double value) noexcept
{
  return std::isfinite(value);
}

bool SafetyController::timestamp_fresh(
  std::chrono::steady_clock::time_point now,
  std::chrono::steady_clock::time_point timestamp,
  std::chrono::milliseconds max_age) noexcept
{
  const auto age = now - timestamp;
  return age >= std::chrono::steady_clock::duration::zero() && age <= max_age;
}

void SafetyController::validate_config(const SafetyControllerConfig & config)
{
  if (config.max_frame_age <= std::chrono::milliseconds::zero()) {
    throw std::invalid_argument("maximum frame age must be positive");
  }
  if (config.max_transport_state_age <= std::chrono::milliseconds::zero()) {
    throw std::invalid_argument("maximum transport state age must be positive");
  }
  if (!finite(config.min_bullet_speed_m_s) || config.min_bullet_speed_m_s <= 0.0 ||
      !finite(config.max_bullet_speed_m_s) ||
      config.max_bullet_speed_m_s < config.min_bullet_speed_m_s) {
    throw std::invalid_argument("invalid safety bullet speed range");
  }
  if (config.fixed_bullet_speed_m_s &&
      (!finite(*config.fixed_bullet_speed_m_s) ||
       *config.fixed_bullet_speed_m_s < config.min_bullet_speed_m_s ||
       *config.fixed_bullet_speed_m_s > config.max_bullet_speed_m_s)) {
    throw std::invalid_argument("fixed bullet speed is outside configured range");
  }
  if (!finite(config.max_gimbal_error_deg) || config.max_gimbal_error_deg <= 0.0 ||
      config.min_gimbal_settle_time <= std::chrono::milliseconds::zero()) {
    throw std::invalid_argument("invalid gimbal settling configuration");
  }
}

void SafetyController::add_reason(SafetyOutput & output, SafetyReason reason)
{
  if (!has_reason(output, reason)) output.reasons.push_back(reason);
}

bool SafetyController::has_reason(const SafetyOutput & output, SafetyReason reason) noexcept
{
  for (const auto existing : output.reasons) {
    if (existing == reason) return true;
  }
  return false;
}

}  // namespace srm::control
