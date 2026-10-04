#include "control/aim_compensator.hpp"

#include <cassert>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace
{
using srm::control::AimCompensator;
using srm::control::AimCompensatorConfig;
using srm::control::AimSolution;
using srm::vision::TargetTrackEstimate;
using srm::vision::TargetTrackState;

constexpr double kPi = 3.14159265358979323846;
constexpr double kEpsilon = 1e-6;

void assert_near(double actual, double expected, double tolerance = kEpsilon)
{
  assert(std::abs(actual - expected) < tolerance);
}

TargetTrackEstimate make_target(
  double yaw, double pitch, double range, double yaw_velocity = 0.0,
  double pitch_velocity = 0.0, double range_velocity = 0.0)
{
  TargetTrackEstimate target;
  target.state = TargetTrackState::tracking;
  target.has_last_measurement = true;
  target.prediction_valid = true;
  target.predicted_yaw_rad = yaw;
  target.predicted_pitch_rad = pitch;
  target.predicted_range_m = range;
  target.yaw_velocity_rad_s = yaw_velocity;
  target.pitch_velocity_rad_s = pitch_velocity;
  target.range_velocity_m_s = range_velocity;
  return target;
}

void test_geometric_mode_without_gravity()
{
  AimCompensatorConfig config;
  config.enable_gravity_compensation = false;
  AimCompensator compensator(config);

  const double yaw = 3.0 * kPi;
  const double pitch = 0.1;
  const double range = 5.0;
  const double speed = 20.0;
  const auto result = compensator.solve(make_target(yaw, pitch, range), speed);

  assert(result.valid);
  assert_near(result.yaw_rad, kPi);
  assert_near(result.line_of_sight_pitch_rad, pitch);
  assert_near(result.pitch_rad, pitch);
  assert_near(result.ballistic_pitch_offset_rad, 0.0);
  assert_near(result.target_range_m, range);
  assert_near(result.flight_time_s, range / speed);
  assert_near(result.total_prediction_lead_s, range / speed);
}

void test_gravity_compensation_hits_synthetic_target()
{
  AimCompensatorConfig config;
  config.gravity_m_s2 = 9.80665;
  config.max_iterations = 6;
  AimCompensator compensator(config);

  const double horizontal = 5.0;
  const double vertical = 0.0;
  const double range = std::hypot(horizontal, vertical);
  const double speed = 20.0;
  const auto result = compensator.solve(make_target(0.0, 0.0, range), speed);

  assert(result.valid);
  assert(result.pitch_rad > 0.0);
  assert(result.ballistic_pitch_offset_rad > 0.0);
  assert(result.flight_time_s > 0.0);

  const double x = result.horizontal_distance_m;
  const double v = result.bullet_speed_m_s;
  const double theta = result.pitch_rad;
  const double simulated_height =
    x * std::tan(theta) - config.gravity_m_s2 * x * x /
      (2.0 * v * v * std::cos(theta) * std::cos(theta));
  assert(std::abs(simulated_height - result.vertical_distance_m) < 1e-6);
}

void test_motion_latency_is_iterated()
{
  AimCompensatorConfig config;
  config.enable_gravity_compensation = false;
  config.additional_latency_seconds = 0.1;
  config.max_iterations = 8;
  config.convergence_tolerance_seconds = 1e-10;
  AimCompensator compensator(config);

  const auto static_result =
    compensator.solve(make_target(0.0, 0.0, 4.0), 20.0);
  const auto moving_result =
    compensator.solve(make_target(0.0, 0.0, 4.0, 1.0, 0.0, 0.0), 20.0);

  assert(static_result.valid);
  assert(moving_result.valid);
  assert(moving_result.yaw_rad > static_result.yaw_rad);
  assert(moving_result.total_prediction_lead_s > config.additional_latency_seconds);
  assert_near(static_result.total_prediction_lead_s, 0.1 + 4.0 / 20.0);
}

void test_invalid_target_speed_and_unreachable_target()
{
  AimCompensator compensator;
  const auto target = make_target(0.0, 0.0, 5.0);

  const auto zero_speed = compensator.solve(target, 0.0);
  assert(!zero_speed.valid);
  assert(!zero_speed.error.empty());

  TargetTrackEstimate lost_target = target;
  lost_target.state = TargetTrackState::temporarily_lost;
  const auto lost_result = compensator.solve(lost_target, 20.0);
  assert(!lost_result.valid);

  TargetTrackEstimate invalid_prediction = target;
  invalid_prediction.prediction_valid = false;
  const auto invalid_result = compensator.solve(invalid_prediction, 20.0);
  assert(!invalid_result.valid);

  AimCompensatorConfig strict_config;
  strict_config.max_bullet_speed_m_s = 10.0;
  AimCompensator strict_compensator(strict_config);
  const auto out_of_range_speed = strict_compensator.solve(target, 20.0);
  assert(!out_of_range_speed.valid);

  TargetTrackEstimate unreachable = make_target(0.0, 0.0, 100.0);
  AimCompensatorConfig low_speed_config;
  low_speed_config.min_bullet_speed_m_s = 1.0;
  low_speed_config.max_bullet_speed_m_s = 2.0;
  low_speed_config.max_target_range_m = 200.0;
  AimCompensator low_speed_compensator(low_speed_config);
  const auto unreachable_result = low_speed_compensator.solve(unreachable, 2.0);
  assert(!unreachable_result.valid);
}

void test_invalid_configuration()
{
  bool bad_gravity_threw = false;
  try {
    AimCompensatorConfig config;
    config.gravity_m_s2 = -1.0;
    AimCompensator invalid(config);
  } catch (const std::invalid_argument &) {
    bad_gravity_threw = true;
  }
  assert(bad_gravity_threw);

  bool bad_iterations_threw = false;
  try {
    AimCompensatorConfig config;
    config.max_iterations = 0;
    AimCompensator invalid(config);
  } catch (const std::invalid_argument &) {
    bad_iterations_threw = true;
  }
  assert(bad_iterations_threw);
}

}  // namespace

int main()
{
  test_geometric_mode_without_gravity();
  test_gravity_compensation_hits_synthetic_target();
  test_motion_latency_is_iterated();
  test_invalid_target_speed_and_unreachable_target();
  test_invalid_configuration();
  return 0;
}
