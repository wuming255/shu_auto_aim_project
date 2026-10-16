#include "control/safety_controller.hpp"

#include <cassert>
#include <chrono>
#include <cmath>
#include <limits>

namespace
{
using Clock = std::chrono::steady_clock;
using srm::control::AutoAimFrameResult;
using srm::control::SafetyController;
using srm::control::SafetyControllerConfig;
using srm::control::SafetyInput;
using srm::control::SafetyReason;
using srm::control::SafetyState;
using srm::protocol::IncomingState;
using srm::vision::TargetTrackState;

bool has_reason(const srm::control::SafetyOutput & output, SafetyReason reason)
{
  for (const auto existing : output.reasons) {
    if (existing == reason) return true;
  }
  return false;
}

SafetyInput valid_input(
  AutoAimFrameResult & pipeline_result, IncomingState & transport_state,
  Clock::time_point now)
{
  pipeline_result.tracking.state = TargetTrackState::tracking;
  pipeline_result.aim.valid = true;
  pipeline_result.aim.yaw_deg = 12.5;
  pipeline_result.aim.pitch_deg = -3.25;
  pipeline_result.aim.bullet_speed_m_s = 20.0;

  transport_state.shoot.bullet_speed = 20.0F;

  SafetyInput input;
  input.now = now;
  input.frame_available = true;
  input.frame_timestamp = now;
  input.pipeline_result = &pipeline_result;
  input.processing_ok = true;
  input.transport_running = true;
  input.transport_connected = true;
  input.transport_state = transport_state;
  input.transport_state_timestamp = now;
  return input;
}

void test_valid_aim_is_non_firing_by_default()
{
  AutoAimFrameResult pipeline_result;
  IncomingState transport_state;
  const auto now = Clock::now();
  const auto input = valid_input(pipeline_result, transport_state, now);

  const SafetyController controller;
  const auto output = controller.decide(input);

  assert(output.state == SafetyState::aiming);
  assert(output.aim_command_valid);
  assert(!output.fire_allowed);
  assert(output.command.fire.fire_flag == 0);
  assert(std::fabs(output.command.gimbal.yaw - 12.5F) < 1e-5F);
  assert(std::fabs(output.command.gimbal.pitch + 3.25F) < 1e-5F);
  assert(output.reasons.empty());
}

void test_explicit_visual_fire_gate_is_separate()
{
  AutoAimFrameResult pipeline_result;
  IncomingState transport_state;
  const auto now = Clock::now();
  auto input = valid_input(pipeline_result, transport_state, now);
  input.visual_fire_intent = true;

  const SafetyController default_controller;
  const auto disabled = default_controller.decide(input);
  assert(disabled.state == SafetyState::aiming);
  assert(!disabled.fire_allowed);
  assert(disabled.command.fire.fire_flag == 0);
  assert(has_reason(disabled, SafetyReason::fire_disabled));

  SafetyControllerConfig enabled_config;
  enabled_config.allow_visual_fire = true;
  const SafetyController enabled_controller(enabled_config);
  const auto enabled = enabled_controller.decide(input);
  assert(enabled.state == SafetyState::fire_ready);
  assert(enabled.fire_allowed);
  assert(enabled.command.fire.fire_flag == 1);
}

void test_transport_lifecycle_and_state_guards()
{
  AutoAimFrameResult pipeline_result;
  IncomingState transport_state;
  const auto now = Clock::now();

  {
    auto input = valid_input(pipeline_result, transport_state, now);
    input.transport_running = false;
    const auto output = SafetyController().decide(input);
    assert(output.state == SafetyState::safe);
    assert(!output.aim_command_valid);
    assert(output.command.fire.fire_flag == 0);
    assert(has_reason(output, SafetyReason::transport_not_running));
  }

  {
    auto input = valid_input(pipeline_result, transport_state, now);
    input.transport_connected = false;
    const auto output = SafetyController().decide(input);
    assert(output.state == SafetyState::safe);
    assert(output.command.fire.fire_flag == 0);
    assert(has_reason(output, SafetyReason::transport_disconnected));
  }

  {
    auto input = valid_input(pipeline_result, transport_state, now);
    input.transport_state.reset();
    input.transport_state_timestamp.reset();
    const auto output = SafetyController().decide(input);
    assert(output.state == SafetyState::safe);
    assert(output.command.fire.fire_flag == 0);
    assert(has_reason(output, SafetyReason::transport_state_missing));
  }

  {
    auto input = valid_input(pipeline_result, transport_state, now);
    input.transport_state_timestamp = now - std::chrono::seconds(1);
    const auto output = SafetyController().decide(input);
    assert(output.state == SafetyState::safe);
    assert(output.command.fire.fire_flag == 0);
    assert(has_reason(output, SafetyReason::transport_state_stale));
  }
}

void test_frame_and_processing_guards()
{
  AutoAimFrameResult pipeline_result;
  IncomingState transport_state;
  const auto now = Clock::now();

  {
    auto input = valid_input(pipeline_result, transport_state, now);
    input.frame_available = false;
    const auto output = SafetyController().decide(input);
    assert(output.state == SafetyState::safe);
    assert(output.command.fire.fire_flag == 0);
    assert(has_reason(output, SafetyReason::frame_missing));
  }

  {
    auto input = valid_input(pipeline_result, transport_state, now);
    input.frame_timestamp = now - std::chrono::seconds(1);
    const auto output = SafetyController().decide(input);
    assert(output.state == SafetyState::safe);
    assert(output.command.fire.fire_flag == 0);
    assert(has_reason(output, SafetyReason::frame_stale));
  }

  {
    auto input = valid_input(pipeline_result, transport_state, now);
    input.processing_ok = false;
    const auto output = SafetyController().decide(input);
    assert(output.state == SafetyState::safe);
    assert(output.command.fire.fire_flag == 0);
    assert(has_reason(output, SafetyReason::processing_failed));
  }
}

void test_target_aim_and_bullet_guards()
{
  AutoAimFrameResult pipeline_result;
  IncomingState transport_state;
  const auto now = Clock::now();

  {
    auto input = valid_input(pipeline_result, transport_state, now);
    input.pipeline_result = nullptr;
    const auto output = SafetyController().decide(input);
    assert(output.state == SafetyState::safe);
    assert(output.command.fire.fire_flag == 0);
    assert(has_reason(output, SafetyReason::target_unavailable));
  }

  {
    auto input = valid_input(pipeline_result, transport_state, now);
    pipeline_result.tracking.state = TargetTrackState::temporarily_lost;
    const auto output = SafetyController().decide(input);
    assert(output.state == SafetyState::safe);
    assert(output.command.fire.fire_flag == 0);
    assert(has_reason(output, SafetyReason::target_unavailable));
  }

  {
    auto input = valid_input(pipeline_result, transport_state, now);
    pipeline_result.aim.valid = false;
    const auto output = SafetyController().decide(input);
    assert(output.state == SafetyState::safe);
    assert(output.command.fire.fire_flag == 0);
    assert(has_reason(output, SafetyReason::aim_invalid));
  }

  {
    auto input = valid_input(pipeline_result, transport_state, now);
    pipeline_result.aim.bullet_speed_m_s = std::numeric_limits<double>::quiet_NaN();
    const auto output = SafetyController().decide(input);
    assert(output.state == SafetyState::safe);
    assert(output.command.fire.fire_flag == 0);
    assert(has_reason(output, SafetyReason::bullet_speed_invalid));
  }

  {
    auto input = valid_input(pipeline_result, transport_state, now);
    transport_state.shoot.bullet_speed = std::numeric_limits<float>::quiet_NaN();
    input.transport_state = transport_state;
    const auto output = SafetyController().decide(input);
    assert(output.state == SafetyState::safe);
    assert(!output.aim_command_valid);
    assert(output.command.fire.fire_flag == 0);
    assert(has_reason(output, SafetyReason::bullet_speed_invalid));
  }
}

void test_gimbal_settling_and_visible_fire_window()
{
  AutoAimFrameResult pipeline_result;
  IncomingState transport_state;
  const auto now = Clock::now();
  auto input = valid_input(pipeline_result, transport_state, now);
  pipeline_result.guard_command_valid = true;
  pipeline_result.guard_target_visible = true;
  pipeline_result.fire_window_valid = true;
  input.visual_fire_intent = true;

  SafetyControllerConfig config;
  config.allow_visual_fire = true;
  config.require_gimbal_feedback_for_fire = true;
  config.max_gimbal_error_deg = 0.5;
  config.min_gimbal_settle_time = std::chrono::milliseconds(50);
  transport_state.gimbal.yaw = 12.5F;
  transport_state.gimbal.pitch = -3.25F;
  input.transport_state = transport_state;
  input.transport_state_timestamp = now;
  SafetyController controller(config);

  const auto settling = controller.decide(input);
  assert(settling.state == SafetyState::aiming);
  assert(!settling.fire_allowed);
  assert(has_reason(settling, SafetyReason::gimbal_not_settled));

  input.now = now + std::chrono::milliseconds(60);
  input.frame_timestamp = input.now;
  input.transport_state_timestamp = input.now;
  const auto ready = controller.decide(input);
  assert(ready.state == SafetyState::fire_ready);
  assert(ready.fire_allowed);
  assert(ready.command.fire.fire_flag == 1);

  pipeline_result.guard_target_visible = false;
  const auto temporarily_lost = controller.decide(input);
  assert(temporarily_lost.state == SafetyState::aiming);
  assert(temporarily_lost.aim_command_valid);
  assert(!temporarily_lost.fire_allowed);
  assert(temporarily_lost.command.fire.fire_flag == 0);
  assert(has_reason(temporarily_lost, SafetyReason::target_not_visible));
}

}  // namespace

int main()
{
  test_valid_aim_is_non_firing_by_default();
  test_explicit_visual_fire_gate_is_separate();
  test_transport_lifecycle_and_state_guards();
  test_frame_and_processing_guards();
  test_target_aim_and_bullet_guards();
  test_gimbal_settling_and_visible_fire_window();
  return 0;
}
