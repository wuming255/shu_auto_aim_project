#include "control/fixed_guard.hpp"

#include <cassert>
#include <chrono>
#include <cmath>

namespace
{
using Clock = std::chrono::steady_clock;
using srm::control::AimCompensator;
using srm::control::FixedGuardConfig;
using srm::control::FixedGuardController;
using srm::vision::RotationEstimate;

RotationEstimate reliable_rotation()
{
  RotationEstimate rotation;
  rotation.initialized = true;
  rotation.reliable = true;
  rotation.current_visible = true;
  rotation.prediction_valid = true;
  rotation.stable_frame_valid = true;
  rotation.center_world_m = {5.0, 0.2, 0.6};
  rotation.radius_m = 0.28;
  rotation.phase_rad = 0.4;
  rotation.angular_velocity_rad_s = 3.4;
  rotation.associated_plate = 0;
  rotation.observation_age_seconds = 0.0;
  rotation.rotation_gimbal_to_world = cv::Matx33d::eye();
  return rotation;
}

void test_lock_relocalize_hold_and_long_loss()
{
  FixedGuardConfig config;
  config.relocalization_center_threshold_m = 0.1;
  config.relocalization_radius_threshold_m = 0.05;
  config.hit_window_half_width_rad = 0.08;
  config.max_hold_seconds = 0.15;
  FixedGuardController guard(config);
  AimCompensator compensator;
  const auto t0 = Clock::time_point{};

  auto rotation = reliable_rotation();
  const auto first = guard.update(rotation, compensator, 20.0, t0);
  assert(first.aim.valid);
  assert(first.guard_locked);
  assert(first.relocalized);
  assert(first.target_visible);

  rotation.phase_rad += rotation.angular_velocity_rad_s * 0.02;
  const auto stable = guard.update(rotation, compensator, 20.0, t0 + std::chrono::milliseconds(20));
  assert(stable.aim.valid);
  assert(stable.guard_locked);
  assert(!stable.relocalized);
  assert(std::abs(stable.aim.yaw_rad - first.aim.yaw_rad) < config.relocalization_yaw_threshold_rad);

  rotation.current_visible = false;
  rotation.observation_age_seconds = 0.05;
  const auto held = guard.update(rotation, compensator, 20.0, t0 + std::chrono::milliseconds(70));
  assert(held.aim.valid);
  assert(held.guard_locked);
  assert(!held.target_visible);
  assert(!held.fire_window_valid);
  assert(held.status == "temporarily_lost_hold");

  rotation.current_visible = true;
  rotation.reliable = false;
  rotation.observation_age_seconds = 0.0;
  const auto unreliable = guard.update(
    rotation, compensator, 20.0, t0 + std::chrono::milliseconds(80));
  assert(!unreliable.aim.valid);
  assert(!unreliable.guard_locked);
  assert(unreliable.status == "guard_model_not_reliable");

  rotation.current_visible = false;
  rotation.observation_age_seconds = 0.3;
  const auto lost = guard.update(rotation, compensator, 20.0, t0 + std::chrono::milliseconds(320));
  assert(!lost.aim.valid);
  assert(!lost.guard_locked);
  assert(lost.status == "long_loss_reacquire");
}

void test_relocalization_and_ballistic_hit_window()
{
  FixedGuardConfig config;
  config.hit_window_half_width_rad = 0.02;
  FixedGuardController guard(config);
  AimCompensator compensator;
  const auto t0 = Clock::time_point{};
  auto rotation = reliable_rotation();

  const auto first = guard.update(rotation, compensator, 20.0, t0);
  assert(first.aim.valid);
  assert(first.aim.flight_time_s > 0.0);

  rotation.center_world_m[0] += 0.2;
  const auto moved = guard.update(rotation, compensator, 20.0, t0 + std::chrono::milliseconds(20));
  assert(moved.relocalized);
  assert(moved.aim.valid);

  rotation.center_world_m[0] -= 0.2;
  rotation.phase_rad = moved.guard_phase_rad -
    rotation.angular_velocity_rad_s * moved.aim.total_prediction_lead_s;
  const auto in_window = guard.update(rotation, compensator, 20.0, t0 + std::chrono::milliseconds(40));
  assert(in_window.aim.valid);
  assert(in_window.fire_window_valid);
}

}  // namespace

int main()
{
  test_lock_relocalize_hold_and_long_loss();
  test_relocalization_and_ballistic_hit_window();
  return 0;
}
