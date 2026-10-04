#include "vision/target_tracker.hpp"

#include <cassert>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace
{
using srm::vision::TargetObservation;
using srm::vision::TargetSelection;
using srm::vision::TargetTrackState;
using srm::vision::TargetTracker;
using srm::vision::TargetTrackerConfig;

constexpr double kPi = 3.14159265358979323846;
constexpr double kEpsilon = 1e-6;
using TimePoint = std::chrono::steady_clock::time_point;

TargetObservation make_observation(double yaw, double pitch = 0.0, double range = 5.0)
{
  TargetObservation observation;
  observation.pose.valid = true;
  observation.pose.yaw_rad = yaw;
  observation.pose.pitch_rad = pitch;
  observation.pose.range_m = range;
  return observation;
}

TargetSelection make_selection(const TargetObservation & observation)
{
  TargetSelection selection;
  selection.observations.push_back(observation);
  selection.selected_index = 0;
  return selection;
}

TargetSelection no_target()
{
  return {};
}

TargetTrackerConfig make_config()
{
  TargetTrackerConfig config;
  config.min_confirmed_updates = 2;
  config.max_consecutive_misses = 2;
  config.min_dt_seconds = 1e-6;
  config.max_dt_seconds = 1.0;
  config.prediction_horizon_seconds = 0.1;
  config.max_prediction_age_seconds = 0.4;
  config.velocity_smoothing_factor = 1.0;
  config.max_abs_angular_velocity_rad_s = 100.0;
  return config;
}

void assert_near(double actual, double expected, double tolerance = kEpsilon)
{
  assert(std::abs(actual - expected) < tolerance);
}

void test_angle_unwrap_and_short_term_prediction()
{
  TargetTracker tracker(make_config());
  const TimePoint t0{};
  const auto first = tracker.update(make_selection(make_observation(3.10)), t0);
  assert(first.state == TargetTrackState::detecting);
  assert(first.has_current_measurement);
  assert(!first.velocity_valid);
  assert(first.prediction_valid);

  const double second_yaw = -3.08;
  const auto second = tracker.update(
    make_selection(make_observation(second_yaw, 0.02, 5.01)),
    t0 + std::chrono::milliseconds(100));

  const double expected_unwrapped = 3.10 + (second_yaw + 2.0 * kPi - 3.10);
  const double expected_velocity = (expected_unwrapped - 3.10) / 0.1;
  assert(second.state == TargetTrackState::tracking);
  assert(second.velocity_valid);
  assert_near(second.unwrapped_yaw_rad, expected_unwrapped);
  assert_near(second.yaw_velocity_rad_s, expected_velocity);
  assert_near(second.pitch_velocity_rad_s, 0.2);
  assert_near(second.range_velocity_m_s, 0.1);
  assert_near(
    second.predicted_yaw_rad, expected_unwrapped + expected_velocity * 0.1);
  assert_near(second.predicted_pitch_rad, 0.02 + 0.2 * 0.1);
  assert(second.predicted_range_m > second.measured_range_m);

  assert_near(TargetTracker::wrap_angle(3.0 * kPi), kPi);
  assert_near(TargetTracker::unwrap_angle(3.10, second_yaw), expected_unwrapped);
}

void test_temporary_loss_and_reset_after_misses()
{
  TargetTracker tracker(make_config());
  const TimePoint t0{};
  (void)tracker.update(make_selection(make_observation(0.0)), t0);
  (void)tracker.update(make_selection(make_observation(0.1)), t0 + std::chrono::milliseconds(100));

  const auto first_miss = tracker.update(no_target(), t0 + std::chrono::milliseconds(200));
  assert(first_miss.state == TargetTrackState::temporarily_lost);
  assert(!first_miss.has_current_measurement);
  assert(first_miss.has_last_measurement);
  assert(first_miss.consecutive_misses == 1);
  assert(first_miss.prediction_valid);

  const auto second_miss = tracker.update(no_target(), t0 + std::chrono::milliseconds(300));
  assert(second_miss.state == TargetTrackState::temporarily_lost);
  assert(second_miss.consecutive_misses == 2);

  const auto third_miss = tracker.update(no_target(), t0 + std::chrono::milliseconds(400));
  assert(third_miss.state == TargetTrackState::lost);
  assert(!third_miss.has_last_measurement);
  assert(!third_miss.prediction_valid);

  const auto recovered = tracker.update(
    make_selection(make_observation(-0.2)), t0 + std::chrono::milliseconds(500));
  assert(recovered.state == TargetTrackState::detecting);
  assert(recovered.has_current_measurement);
  assert(recovered.confirmed_updates == 1);
}

void test_timestamp_gap_starts_new_track()
{
  auto config = make_config();
  config.max_dt_seconds = 0.2;
  TargetTracker tracker(config);
  const TimePoint t0{};
  (void)tracker.update(make_selection(make_observation(0.0)), t0);
  (void)tracker.update(make_selection(make_observation(0.1)), t0 + std::chrono::milliseconds(100));

  const auto after_gap = tracker.update(
    make_selection(make_observation(-0.5)), t0 + std::chrono::seconds(1));
  assert(after_gap.reset_due_to_time_gap);
  assert(after_gap.state == TargetTrackState::detecting);
  assert(after_gap.confirmed_updates == 1);
  assert(!after_gap.velocity_valid);
  assert_near(after_gap.yaw_velocity_rad_s, 0.0);
}

void test_abnormal_angular_velocity_is_not_predicted()
{
  auto config = make_config();
  config.max_abs_angular_velocity_rad_s = 1.0;
  TargetTracker tracker(config);
  const TimePoint t0{};
  (void)tracker.update(make_selection(make_observation(0.0)), t0);
  const auto result = tracker.update(
    make_selection(make_observation(0.3)), t0 + std::chrono::milliseconds(100));

  assert(result.state == TargetTrackState::detecting);
  assert(result.confirmed_updates == 1);
  assert(!result.velocity_valid);
  assert(result.prediction_valid);
  assert_near(result.predicted_yaw_rad, result.unwrapped_yaw_rad);
  assert(result.status == "angular velocity rejected");
}

void test_invalid_configuration_and_invalid_selection()
{
  bool zero_confirmed_threw = false;
  try {
    auto config = make_config();
    config.min_confirmed_updates = 0;
    TargetTracker invalid(config);
  } catch (const std::invalid_argument &) {
    zero_confirmed_threw = true;
  }
  assert(zero_confirmed_threw);

  bool bad_smoothing_threw = false;
  try {
    auto config = make_config();
    config.velocity_smoothing_factor = 1.1;
    TargetTracker invalid(config);
  } catch (const std::invalid_argument &) {
    bad_smoothing_threw = true;
  }
  assert(bad_smoothing_threw);

  TargetTracker tracker(make_config());
  TargetSelection invalid_selection;
  TargetObservation invalid_observation = make_observation(0.0);
  invalid_observation.pose.valid = false;
  invalid_selection.observations.push_back(invalid_observation);
  invalid_selection.selected_index = 0;
  const auto result = tracker.update(invalid_selection, TimePoint{});
  assert(result.state == TargetTrackState::lost);
  assert(!result.has_current_measurement);
  assert(!result.has_last_measurement);
}

}  // namespace

int main()
{
  test_angle_unwrap_and_short_term_prediction();
  test_temporary_loss_and_reset_after_misses();
  test_timestamp_gap_starts_new_track();
  test_abnormal_angular_velocity_is_not_predicted();
  test_invalid_configuration_and_invalid_selection();
  return 0;
}
