#include "vision/target_tracker.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace srm::vision
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;

bool finite(double value)
{
  return std::isfinite(value);
}

bool valid_non_negative_limit(double value)
{
  return (finite(value) && value >= 0.0) ||
         (std::isinf(value) && value > 0.0);
}

}  // namespace

const char * to_string(TargetTrackState state) noexcept
{
  switch (state) {
    case TargetTrackState::lost:
      return "lost";
    case TargetTrackState::detecting:
      return "detecting";
    case TargetTrackState::tracking:
      return "tracking";
    case TargetTrackState::temporarily_lost:
      return "temporarily_lost";
    default:
      return "unknown";
  }
}

TargetTracker::TargetTracker(TargetTrackerConfig config) : config_(std::move(config))
{
  validate_config(config_);
}

TargetTrackEstimate TargetTracker::update(
  const TargetSelection & selection, std::chrono::steady_clock::time_point timestamp)
{
  bool reset_due_to_time_gap = false;
  std::string status;
  bool association_rejected = false;

  if (has_update_timestamp_) {
    const double update_dt = elapsed_seconds(timestamp, last_update_timestamp_);
    if (!finite(update_dt) || update_dt < 0.0 || update_dt > config_.max_dt_seconds) {
      clear_track();
      reset_due_to_time_gap = true;
      status = "timestamp gap reset";
    }
  }
  last_update_timestamp_ = timestamp;
  has_update_timestamp_ = true;

  const auto * selected = choose_observation(selection, timestamp, association_rejected);

  if (selected == nullptr) {
    if (!has_measurement_) {
      state_ = TargetTrackState::lost;
      return make_estimate(
        timestamp, false, reset_due_to_time_gap,
        status.empty() ? (association_rejected ? "no associated target" : "no valid target")
                       : std::move(status));
    }

    ++consecutive_misses_;
    if (state_ == TargetTrackState::detecting) {
      clear_track();
      status = "target lost while detecting";
    } else if (state_ == TargetTrackState::tracking) {
      state_ = TargetTrackState::temporarily_lost;
      status = "target temporarily lost";
    } else if (
      state_ == TargetTrackState::temporarily_lost &&
      consecutive_misses_ > config_.max_consecutive_misses) {
      clear_track();
      status = "target lost after consecutive misses";
    } else {
      status = "target temporarily lost";
    }

    return make_estimate(timestamp, false, reset_due_to_time_gap, std::move(status));
  }

  if (!has_measurement_) {
    start_track(*selected, timestamp);
    status = (confirmed_updates_ >= config_.min_confirmed_updates) ? "tracking initialized"
                                                                   : "target detecting";
    return make_estimate(timestamp, true, reset_due_to_time_gap, std::move(status));
  }

  const double measurement_dt = elapsed_seconds(timestamp, last_measurement_timestamp_);
  if (!finite(measurement_dt) || measurement_dt < 0.0 || measurement_dt > config_.max_dt_seconds) {
    clear_track();
    reset_due_to_time_gap = true;
    start_track(*selected, timestamp);
    return make_estimate(timestamp, true, reset_due_to_time_gap, "measurement gap reset");
  }

  const double new_measured_yaw = wrap_angle(selected->pose.yaw_rad);
  const double new_measured_pitch = selected->pose.pitch_rad;
  const double new_measured_range = selected->pose.range_m;
  const double new_unwrapped_yaw = unwrap_angle(unwrapped_yaw_rad_, new_measured_yaw);

  bool velocity_rejected = false;
  if (measurement_dt >= config_.min_dt_seconds) {
    const double raw_yaw_velocity =
      (new_unwrapped_yaw - unwrapped_yaw_rad_) / measurement_dt;
    const double raw_pitch_velocity =
      (new_measured_pitch - measured_pitch_rad_) / measurement_dt;
    const double raw_range_velocity =
      (new_measured_range - measured_range_m_) / measurement_dt;

    const bool finite_velocity = finite(raw_yaw_velocity) && finite(raw_pitch_velocity) &&
                                 finite(raw_range_velocity);
    const bool angular_velocity_in_range =
      std::abs(raw_yaw_velocity) <= config_.max_abs_angular_velocity_rad_s &&
      std::abs(raw_pitch_velocity) <= config_.max_abs_angular_velocity_rad_s;

    if (finite_velocity && angular_velocity_in_range) {
      const double alpha = config_.velocity_smoothing_factor;
      if (velocity_initialized_) {
        yaw_velocity_rad_s_ =
          alpha * raw_yaw_velocity + (1.0 - alpha) * yaw_velocity_rad_s_;
        pitch_velocity_rad_s_ =
          alpha * raw_pitch_velocity + (1.0 - alpha) * pitch_velocity_rad_s_;
        range_velocity_m_s_ =
          alpha * raw_range_velocity + (1.0 - alpha) * range_velocity_m_s_;
      } else {
        yaw_velocity_rad_s_ = raw_yaw_velocity;
        pitch_velocity_rad_s_ = raw_pitch_velocity;
        range_velocity_m_s_ = raw_range_velocity;
      }
      velocity_initialized_ = true;
      velocity_valid_ = true;
    } else {
      velocity_initialized_ = false;
      velocity_valid_ = false;
      yaw_velocity_rad_s_ = 0.0;
      pitch_velocity_rad_s_ = 0.0;
      range_velocity_m_s_ = 0.0;
      velocity_rejected = true;
    }
  } else {
    // 时间间隔过小不足以稳定估计差分速度；保留本次位置，但不输出速度预测。
    velocity_initialized_ = false;
    velocity_valid_ = false;
    yaw_velocity_rad_s_ = 0.0;
    pitch_velocity_rad_s_ = 0.0;
    range_velocity_m_s_ = 0.0;
  }

  measured_yaw_rad_ = new_measured_yaw;
  measured_pitch_rad_ = new_measured_pitch;
  measured_range_m_ = new_measured_range;
  unwrapped_yaw_rad_ = new_unwrapped_yaw;
  last_measurement_timestamp_ = timestamp;
  last_observation_ = *selected;
  has_measurement_ = true;
  consecutive_misses_ = 0;

  if (velocity_rejected) {
    confirmed_updates_ = 1;
    state_ = (config_.min_confirmed_updates <= 1) ? TargetTrackState::tracking
                                                   : TargetTrackState::detecting;
    status = "angular velocity rejected";
  } else {
    ++confirmed_updates_;
    state_ = (confirmed_updates_ >= config_.min_confirmed_updates)
                ? TargetTrackState::tracking
                : TargetTrackState::detecting;
    status = (state_ == TargetTrackState::tracking) ? "tracking" : "target detecting";
  }

  return make_estimate(timestamp, true, reset_due_to_time_gap, std::move(status));
}

void TargetTracker::reset() noexcept
{
  clear_track();
  has_update_timestamp_ = false;
  last_update_timestamp_ = {};
}

const TargetTrackerConfig & TargetTracker::config() const noexcept
{
  return config_;
}

TargetTrackState TargetTracker::state() const noexcept
{
  return state_;
}

double TargetTracker::wrap_angle(double angle_rad) noexcept
{
  if (!finite(angle_rad)) return std::numeric_limits<double>::quiet_NaN();
  double wrapped = std::remainder(angle_rad, kTwoPi);
  // 采用 [-pi, pi]，并把边界的 -pi 统一表示为 +pi，保证日志和比较稳定。
  if (wrapped == -kPi) wrapped = kPi;
  if (wrapped == 0.0) wrapped = 0.0;
  return wrapped;
}

double TargetTracker::unwrap_angle(
  double previous_unwrapped_rad, double current_wrapped_rad) noexcept
{
  if (!finite(previous_unwrapped_rad) || !finite(current_wrapped_rad)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const double previous_wrapped = wrap_angle(previous_unwrapped_rad);
  return previous_unwrapped_rad + wrap_angle(current_wrapped_rad - previous_wrapped);
}

void TargetTracker::validate_config(const TargetTrackerConfig & config)
{
  if (config.min_confirmed_updates == 0) {
    throw std::invalid_argument("minimum confirmed updates must be positive");
  }
  if (config.max_consecutive_misses == 0) {
    throw std::invalid_argument("maximum consecutive misses must be positive");
  }
  if (!finite(config.min_dt_seconds) || config.min_dt_seconds < 0.0) {
    throw std::invalid_argument("minimum tracker dt must be finite and non-negative");
  }
  if (!finite(config.max_dt_seconds) || config.max_dt_seconds <= 0.0 ||
      config.max_dt_seconds < config.min_dt_seconds) {
    throw std::invalid_argument("maximum tracker dt must be finite and larger than minimum dt");
  }
  if (!finite(config.prediction_horizon_seconds) || config.prediction_horizon_seconds < 0.0) {
    throw std::invalid_argument("prediction horizon must be finite and non-negative");
  }
  if (!finite(config.max_prediction_age_seconds) || config.max_prediction_age_seconds < 0.0) {
    throw std::invalid_argument("maximum prediction age must be finite and non-negative");
  }
  if (!finite(config.velocity_smoothing_factor) || config.velocity_smoothing_factor < 0.0 ||
      config.velocity_smoothing_factor > 1.0) {
    throw std::invalid_argument("velocity smoothing factor must be in [0, 1]");
  }
  if (!valid_non_negative_limit(config.max_abs_angular_velocity_rad_s)) {
    throw std::invalid_argument(
      "maximum angular velocity must be finite and non-negative or +inf");
  }
  if (!valid_non_negative_limit(config.max_association_angular_error_rad)) {
    throw std::invalid_argument(
      "maximum association angular error must be finite and non-negative or +inf");
  }
  if (!valid_non_negative_limit(config.max_association_range_error_m)) {
    throw std::invalid_argument(
      "maximum association range error must be finite and non-negative or +inf");
  }
}

bool TargetTracker::valid_measurement(const TargetObservation & observation) noexcept
{
  if (!observation.pose.valid) return false;
  if (!finite(observation.pose.yaw_rad) || !finite(observation.pose.pitch_rad) ||
      !finite(observation.pose.range_m) || observation.pose.range_m <= 0.0) {
    return false;
  }
  return true;
}

double TargetTracker::elapsed_seconds(
  std::chrono::steady_clock::time_point newer,
  std::chrono::steady_clock::time_point older) noexcept
{
  return std::chrono::duration<double>(newer - older).count();
}

const TargetObservation * TargetTracker::choose_observation(
  const TargetSelection & selection, std::chrono::steady_clock::time_point timestamp,
  bool & association_rejected) const noexcept
{
  association_rejected = false;

  const auto * initially_selected = selection.selected();
  if (!has_measurement_ || !config_.use_prediction_association) {
    return initially_selected != nullptr && valid_measurement(*initially_selected)
             ? initially_selected
             : nullptr;
  }

  double predicted_yaw = unwrapped_yaw_rad_;
  double predicted_pitch = measured_pitch_rad_;
  double predicted_range = measured_range_m_;
  const double age_seconds = elapsed_seconds(timestamp, last_measurement_timestamp_);
  const bool prediction_age_valid =
    finite(age_seconds) && age_seconds >= 0.0 &&
    age_seconds <= config_.max_prediction_age_seconds;
  if (prediction_age_valid && velocity_valid_) {
    predicted_yaw += yaw_velocity_rad_s_ * age_seconds;
    predicted_pitch += pitch_velocity_rad_s_ * age_seconds;
    predicted_range += range_velocity_m_s_ * age_seconds;
  }
  if (!finite(predicted_yaw) || !finite(predicted_pitch) || !finite(predicted_range)) {
    return initially_selected != nullptr && valid_measurement(*initially_selected)
             ? initially_selected
             : nullptr;
  }

  const TargetObservation * best = nullptr;
  double best_angular_error = std::numeric_limits<double>::infinity();
  double best_range_error = std::numeric_limits<double>::infinity();
  std::size_t best_detection_index = std::numeric_limits<std::size_t>::max();

  for (const auto & candidate : selection.observations) {
    if (!valid_measurement(candidate)) continue;

    const double yaw_error = wrap_angle(candidate.pose.yaw_rad - wrap_angle(predicted_yaw));
    const double pitch_error = candidate.pose.pitch_rad - predicted_pitch;
    const double angular_error = std::hypot(yaw_error, pitch_error);
    const double range_error = std::abs(candidate.pose.range_m - predicted_range);
    if (!finite(angular_error) || !finite(range_error)) continue;
    if (angular_error > config_.max_association_angular_error_rad ||
        range_error > config_.max_association_range_error_m) {
      association_rejected = true;
      continue;
    }

    const bool better =
      angular_error < best_angular_error ||
      (angular_error == best_angular_error && range_error < best_range_error) ||
      (angular_error == best_angular_error && range_error == best_range_error &&
       candidate.detection_index < best_detection_index);
    if (better) {
      best = &candidate;
      best_angular_error = angular_error;
      best_range_error = range_error;
      best_detection_index = candidate.detection_index;
    }
  }

  if (best != nullptr) return best;
  return nullptr;
}

void TargetTracker::clear_track() noexcept
{
  state_ = TargetTrackState::lost;
  has_measurement_ = false;
  velocity_initialized_ = false;
  last_measurement_timestamp_ = {};
  last_observation_.reset();
  confirmed_updates_ = 0;
  consecutive_misses_ = 0;
  measured_yaw_rad_ = 0.0;
  measured_pitch_rad_ = 0.0;
  measured_range_m_ = 0.0;
  unwrapped_yaw_rad_ = 0.0;
  yaw_velocity_rad_s_ = 0.0;
  pitch_velocity_rad_s_ = 0.0;
  range_velocity_m_s_ = 0.0;
  velocity_valid_ = false;
}

void TargetTracker::start_track(
  const TargetObservation & observation, std::chrono::steady_clock::time_point timestamp)
{
  measured_yaw_rad_ = wrap_angle(observation.pose.yaw_rad);
  measured_pitch_rad_ = observation.pose.pitch_rad;
  measured_range_m_ = observation.pose.range_m;
  unwrapped_yaw_rad_ = measured_yaw_rad_;
  yaw_velocity_rad_s_ = 0.0;
  pitch_velocity_rad_s_ = 0.0;
  range_velocity_m_s_ = 0.0;
  velocity_initialized_ = false;
  velocity_valid_ = false;
  last_measurement_timestamp_ = timestamp;
  last_observation_ = observation;
  has_measurement_ = true;
  confirmed_updates_ = 1;
  consecutive_misses_ = 0;
  state_ = (config_.min_confirmed_updates <= 1) ? TargetTrackState::tracking
                                                 : TargetTrackState::detecting;
}

TargetTrackEstimate TargetTracker::make_estimate(
  std::chrono::steady_clock::time_point timestamp, bool current_measurement,
  bool reset_due_to_time_gap, std::string status) const
{
  TargetTrackEstimate estimate;
  estimate.state = state_;
  estimate.has_current_measurement = current_measurement;
  estimate.has_last_measurement = has_measurement_ && last_observation_.has_value();
  estimate.velocity_valid = velocity_valid_;
  estimate.reset_due_to_time_gap = reset_due_to_time_gap;
  estimate.confirmed_updates = confirmed_updates_;
  estimate.consecutive_misses = consecutive_misses_;
  estimate.timestamp = timestamp;
  estimate.last_observation = last_observation_;
  if (current_measurement) estimate.current_observation = last_observation_;
  estimate.measured_yaw_rad = measured_yaw_rad_;
  estimate.measured_pitch_rad = measured_pitch_rad_;
  estimate.measured_range_m = measured_range_m_;
  estimate.unwrapped_yaw_rad = unwrapped_yaw_rad_;
  estimate.yaw_velocity_rad_s = yaw_velocity_rad_s_;
  estimate.pitch_velocity_rad_s = pitch_velocity_rad_s_;
  estimate.range_velocity_m_s = range_velocity_m_s_;
  estimate.status = std::move(status);

  if (estimate.has_last_measurement) {
    const double age_seconds = elapsed_seconds(timestamp, last_measurement_timestamp_);
    fill_prediction(estimate, age_seconds);
  }
  return estimate;
}

void TargetTracker::fill_prediction(TargetTrackEstimate & estimate, double age_seconds) const
{
  if (!finite(age_seconds) || age_seconds < 0.0 ||
      age_seconds > config_.max_prediction_age_seconds) {
    estimate.prediction_valid = false;
    estimate.predicted_yaw_rad = estimate.unwrapped_yaw_rad;
    estimate.predicted_pitch_rad = estimate.measured_pitch_rad;
    estimate.predicted_range_m = estimate.measured_range_m;
    return;
  }

  const double lead_seconds = estimate.velocity_valid ? config_.prediction_horizon_seconds : 0.0;
  const double prediction_seconds = age_seconds + lead_seconds;
  const double predicted_yaw =
    estimate.unwrapped_yaw_rad + estimate.yaw_velocity_rad_s * prediction_seconds;
  const double predicted_pitch =
    estimate.measured_pitch_rad + estimate.pitch_velocity_rad_s * prediction_seconds;
  const double predicted_range =
    estimate.measured_range_m + estimate.range_velocity_m_s * prediction_seconds;

  if (!finite(predicted_yaw) || !finite(predicted_pitch) || !finite(predicted_range) ||
      predicted_range <= 0.0) {
    estimate.prediction_valid = false;
    estimate.predicted_yaw_rad = estimate.unwrapped_yaw_rad;
    estimate.predicted_pitch_rad = estimate.measured_pitch_rad;
    estimate.predicted_range_m = estimate.measured_range_m;
    return;
  }

  // 没有当前测量且没有可用速度时，不把陈旧的位置冒充为预测结果。
  if (!estimate.has_current_measurement && !estimate.velocity_valid) {
    estimate.prediction_valid = false;
    estimate.predicted_yaw_rad = estimate.unwrapped_yaw_rad;
    estimate.predicted_pitch_rad = estimate.measured_pitch_rad;
    estimate.predicted_range_m = estimate.measured_range_m;
    return;
  }

  estimate.prediction_valid = true;
  estimate.predicted_yaw_rad = predicted_yaw;
  estimate.predicted_pitch_rad = predicted_pitch;
  estimate.predicted_range_m = predicted_range;
}

}  // namespace srm::vision
