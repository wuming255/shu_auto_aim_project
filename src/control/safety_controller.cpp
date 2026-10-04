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
      static_cast<double>(input.transport_state->shoot.bullet_speed);
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
  const bool base_inputs_valid =
    input.transport_running && input.transport_connected && input.frame_available &&
    timestamp_fresh(input.now, input.frame_timestamp, config_.max_frame_age) &&
    input.transport_state.has_value() && transport_state_timestamp_valid &&
    transport_bullet_speed_valid && input.processing_ok;

  if (pipeline == nullptr) {
    add_reason(output, SafetyReason::target_unavailable);
  } else {
    if (pipeline->tracking.state != vision::TargetTrackState::tracking) {
      add_reason(output, SafetyReason::target_unavailable);
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
    base_inputs_valid && pipeline != nullptr && pipeline->tracking.state == vision::TargetTrackState::tracking &&
    pipeline->aim.valid && finite(pipeline->aim.yaw_deg) && finite(pipeline->aim.pitch_deg) &&
    finite(pipeline->aim.bullet_speed_m_s) &&
    pipeline->aim.bullet_speed_m_s >= config_.min_bullet_speed_m_s &&
    pipeline->aim.bullet_speed_m_s <= config_.max_bullet_speed_m_s;

  if (aim_valid) {
    const float yaw = static_cast<float>(pipeline->aim.yaw_deg);
    const float pitch = static_cast<float>(pipeline->aim.pitch_deg);
    if (std::isfinite(yaw) && std::isfinite(pitch)) {
      output.command.gimbal = {yaw, pitch};
      output.aim_command_valid = true;
    } else {
      add_reason(output, SafetyReason::aim_invalid);
    }
  }

  const bool fire_gate =
    aim_valid && output.aim_command_valid && config_.allow_visual_fire &&
    input.visual_fire_intent;
  if (fire_gate) {
    output.command.fire.fire_flag = 1;
    output.fire_allowed = true;
    output.state = SafetyState::fire_ready;
  } else {
    output.command.fire.fire_flag = 0;
    if (aim_valid && output.aim_command_valid) {
      output.state = SafetyState::aiming;
    } else {
      output.state = SafetyState::safe;
      output.command.gimbal = {};
    }
    if (input.visual_fire_intent && !config_.allow_visual_fire) {
      add_reason(output, SafetyReason::fire_disabled);
    }
  }

  output.summary = reason_text(output.reasons);
  if (output.reasons.empty()) output.summary = "none";
  return output;
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
