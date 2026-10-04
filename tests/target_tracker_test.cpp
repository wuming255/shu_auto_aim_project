#include "vision/target_tracker.hpp"

#include <cassert>
#include <cmath>
#include <initializer_list>
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

TargetSelection make_candidates(
  std::initializer_list<TargetObservation> observations, std::size_t selected_index)
{
  TargetSelection selection;
  selection.observations.assign(observations.begin(), observations.end());
  selection.selected_index = selected_index;
  return selection;
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

void test_prediction_association_resists_candidate_switching()
{
  auto config = make_config();
  config.min_confirmed_updates = 2;
  config.max_association_angular_error_rad = 0.3;
  TargetTracker tracker(config);
  const TimePoint t0{};

  TargetObservation target_0 = make_observation(0.0);
  target_0.detection_index = 10;
  TargetObservation target_1 = make_observation(0.1);
  target_1.detection_index = 11;
  TargetObservation distractor_1 = make_observation(0.5);
  distractor_1.detection_index = 12;

  (void)tracker.update(make_selection(target_0), t0);
  const auto second = tracker.update(
    make_candidates({target_1, distractor_1}, 1), t0 + std::chrono::milliseconds(100));
  assert(second.has_current_measurement);
  assert(second.last_observation->detection_index == 11);
  assert(second.velocity_valid);

  // 当前帧选择器偏向零视轴的干扰候选（index 1）；跟踪器应依据预测运动选中
  // 位于 0.2 rad 的连续目标（index 20），而不是回跳到 0 rad 的干扰项。
  TargetObservation target_2 = make_observation(0.2);
  target_2.detection_index = 20;
  TargetObservation distractor_2 = make_observation(0.0);
  distractor_2.detection_index = 21;
  const auto third = tracker.update(
    make_candidates({target_2, distractor_2}, 1), t0 + std::chrono::milliseconds(200));
  assert(third.has_current_measurement);
  assert(third.last_observation->detection_index == 20);
  assert_near(third.measured_yaw_rad, 0.2);
}

void test_association_gate_treats_unrelated_candidate_as_miss()
{
  auto config = make_config();
  config.min_confirmed_updates = 1;
  config.max_association_angular_error_rad = 0.1;
  TargetTracker tracker(config);
  const TimePoint t0{};
  (void)tracker.update(make_selection(make_observation(0.0)), t0);

  TargetObservation unrelated = make_observation(0.8);
  unrelated.detection_index = 4;
  const auto result = tracker.update(
    make_selection(unrelated), t0 + std::chrono::milliseconds(100));
  assert(result.state == TargetTrackState::temporarily_lost);
  assert(!result.has_current_measurement);
  assert(result.has_last_measurement);
  assert(result.consecutive_misses == 1);
}

void test_noisy_sequence_and_occlusion_reacquisition()
{
  auto config = make_config();
  config.min_confirmed_updates = 2;
  config.max_consecutive_misses = 2;
  config.max_association_angular_error_rad = 0.12;
  config.max_association_range_error_m = 0.25;
  TargetTracker tracker(config);
  const TimePoint t0{};

  const double target_yaws[] = {0.0, 0.05, 0.10, 0.15};
  const double noise[] = {0.0, 0.004, -0.003, 0.005};

  TargetObservation first_target = make_observation(target_yaws[0] + noise[0], 0.01, 5.0);
  first_target.detection_index = 100;
  (void)tracker.update(make_selection(first_target), t0);

  for (std::size_t frame = 1; frame < 4; ++frame) {
    TargetObservation target = make_observation(target_yaws[frame] + noise[frame], 0.01, 5.0);
    target.detection_index = 100 + frame;
    TargetObservation distractor = make_observation(-0.10, 0.01, 5.0);
    distractor.detection_index = 200 + frame;

    const auto result = tracker.update(
      make_candidates({target, distractor}, 1),
      t0 + std::chrono::milliseconds(static_cast<int>(frame * 100)));
    assert(result.has_current_measurement);
    assert(result.last_observation->detection_index == target.detection_index);
  }

  const auto missed = tracker.update(
    no_target(), t0 + std::chrono::milliseconds(500));
  assert(missed.state == TargetTrackState::temporarily_lost);
  assert(missed.prediction_valid);

  TargetObservation reacquired = make_observation(0.33, 0.01, 5.0);
  reacquired.detection_index = 300;
  TargetObservation distractor = make_observation(0.0, 0.01, 5.0);
  distractor.detection_index = 301;
  const auto recovered = tracker.update(
    make_candidates({reacquired, distractor}, 1),
    t0 + std::chrono::milliseconds(600));
  assert(recovered.has_current_measurement);
  assert(recovered.last_observation->detection_index == 300);
}

void test_stale_prediction_does_not_drive_association()
{
  auto config = make_config();
  config.min_confirmed_updates = 1;
  config.max_association_angular_error_rad = 0.1;
  config.max_prediction_age_seconds = 0.15;
  TargetTracker tracker(config);
  const TimePoint t0{};

  (void)tracker.update(make_selection(make_observation(0.0)), t0);
  (void)tracker.update(make_selection(make_observation(0.1)), t0 + std::chrono::milliseconds(100));
  const auto stale = tracker.update(no_target(), t0 + std::chrono::milliseconds(300));
  assert(!stale.prediction_valid);

  TargetObservation last_position = make_observation(0.1);
  last_position.detection_index = 50;
  TargetObservation stale_velocity_position = make_observation(0.4);
  stale_velocity_position.detection_index = 51;
  const auto result = tracker.update(
    make_candidates({last_position, stale_velocity_position}, 1),
    t0 + std::chrono::milliseconds(400));
  assert(result.has_current_measurement);
  assert(result.last_observation->detection_index == 50);
}

void test_association_can_be_disabled()
{
  auto config = make_config();
  config.use_prediction_association = false;
  TargetTracker tracker(config);
  const TimePoint t0{};

  (void)tracker.update(make_selection(make_observation(0.0)), t0);
  TargetObservation selected = make_observation(0.4);
  selected.detection_index = 9;
  TargetObservation closer_to_history = make_observation(0.05);
  closer_to_history.detection_index = 10;
  const auto result = tracker.update(
    make_candidates({closer_to_history, selected}, 1),
    t0 + std::chrono::milliseconds(100));
  assert(result.has_current_measurement);
  assert(result.last_observation->detection_index == 9);
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

  bool bad_association_gate_threw = false;
  try {
    auto config = make_config();
    config.max_association_angular_error_rad = -0.1;
    TargetTracker invalid(config);
  } catch (const std::invalid_argument &) {
    bad_association_gate_threw = true;
  }
  assert(bad_association_gate_threw);

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
  test_prediction_association_resists_candidate_switching();
  test_association_gate_treats_unrelated_candidate_as_miss();
  test_noisy_sequence_and_occlusion_reacquisition();
  test_stale_prediction_does_not_drive_association();
  test_association_can_be_disabled();
  test_timestamp_gap_starts_new_track();
  test_abnormal_angular_velocity_is_not_predicted();
  test_invalid_configuration_and_invalid_selection();
  return 0;
}
