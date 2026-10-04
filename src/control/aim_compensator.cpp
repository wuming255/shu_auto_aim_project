#include "control/aim_compensator.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace srm::control
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kMinimumHorizontalDistanceM = 1e-6;
constexpr double kDiscriminantTolerance = 1e-10;
constexpr double kMinimumCosine = 1e-9;

}  // namespace

AimCompensator::AimCompensator(AimCompensatorConfig config) : config_(std::move(config))
{
  validate_config(config_);
}

AimSolution AimCompensator::solve(
  const vision::TargetTrackEstimate & target, double bullet_speed_m_s) const
{
  AimSolution result;
  result.bullet_speed_m_s = bullet_speed_m_s;

  if (target.state != vision::TargetTrackState::tracking) {
    result.error = "target is not in tracking state";
    return result;
  }
  if (!target.prediction_valid || !target.has_last_measurement) {
    result.error = "target prediction is not valid";
    return result;
  }
  if (!finite(bullet_speed_m_s) || bullet_speed_m_s < config_.min_bullet_speed_m_s ||
      bullet_speed_m_s > config_.max_bullet_speed_m_s) {
    result.error = "bullet speed is outside configured range";
    return result;
  }
  if (!finite(target.predicted_yaw_rad) || !finite(target.predicted_pitch_rad) ||
      !finite(target.predicted_range_m) || target.predicted_range_m <= 0.0 ||
      target.predicted_range_m > config_.max_target_range_m) {
    result.error = "target prediction has invalid range or angle";
    return result;
  }

  const double base_yaw = vision::TargetTracker::wrap_angle(target.predicted_yaw_rad);
  const double base_pitch = target.predicted_pitch_rad;
  const double base_range = target.predicted_range_m;
  if (!finite(base_yaw) || !finite(base_pitch) || !finite(base_range) ||
      !finite(target.yaw_velocity_rad_s) || !finite(target.pitch_velocity_rad_s) ||
      !finite(target.range_velocity_m_s)) {
    result.error = "target prediction contains non-finite values";
    return result;
  }

  double total_lead_s = config_.additional_latency_seconds;
  BallisticResult ballistic;
  double solved_yaw = base_yaw;
  double solved_range = base_range;
  double solved_horizontal = 0.0;
  double solved_vertical = 0.0;

  for (std::size_t iteration = 0; iteration < config_.max_iterations; ++iteration) {
    solved_yaw = vision::TargetTracker::wrap_angle(
      target.predicted_yaw_rad + target.yaw_velocity_rad_s * total_lead_s);
    const double line_pitch = target.predicted_pitch_rad +
                              target.pitch_velocity_rad_s * total_lead_s;
    solved_range = target.predicted_range_m + target.range_velocity_m_s * total_lead_s;
    if (!finite(solved_yaw) || !finite(line_pitch) || !finite(solved_range) ||
        solved_range <= 0.0 || solved_range > config_.max_target_range_m) {
      result.error = "target motion extrapolation is invalid";
      return result;
    }

    solved_horizontal = solved_range * std::cos(line_pitch);
    solved_vertical = solved_range * std::sin(line_pitch);
    if (!finite(solved_horizontal) || !finite(solved_vertical) ||
        solved_horizontal <= kMinimumHorizontalDistanceM) {
      result.error = "target horizontal distance is invalid";
      return result;
    }

    ballistic = solve_ballistic(
      solved_horizontal, solved_vertical, bullet_speed_m_s, config_.gravity_m_s2,
      config_.enable_gravity_compensation);
    if (!ballistic.valid) {
      result.error = ballistic.error;
      return result;
    }

    const double next_total_lead_s = config_.additional_latency_seconds + ballistic.flight_time_s;
    if (!finite(next_total_lead_s) || next_total_lead_s < 0.0 ||
        next_total_lead_s > config_.max_flight_time_s + config_.additional_latency_seconds) {
      result.error = "computed prediction lead exceeds configured limit";
      return result;
    }
    if (std::abs(next_total_lead_s - total_lead_s) <= config_.convergence_tolerance_seconds) {
      total_lead_s = next_total_lead_s;
      break;
    }
    total_lead_s = next_total_lead_s;
  }

  // Recompute once at the converged lead so output angles and flight time correspond to the same
  // target state. This also handles max_iterations == 1 deterministically.
  solved_yaw = vision::TargetTracker::wrap_angle(
    target.predicted_yaw_rad + target.yaw_velocity_rad_s * total_lead_s);
  const double final_line_pitch =
    target.predicted_pitch_rad + target.pitch_velocity_rad_s * total_lead_s;
  solved_range = target.predicted_range_m + target.range_velocity_m_s * total_lead_s;
  solved_horizontal = solved_range * std::cos(final_line_pitch);
  solved_vertical = solved_range * std::sin(final_line_pitch);
  if (!finite(solved_yaw) || !finite(final_line_pitch) || !finite(solved_range) ||
      solved_range <= 0.0 || solved_range > config_.max_target_range_m ||
      !finite(solved_horizontal) || !finite(solved_vertical) ||
      solved_horizontal <= kMinimumHorizontalDistanceM) {
    result.error = "final target extrapolation is invalid";
    return result;
  }
  ballistic = solve_ballistic(
    solved_horizontal, solved_vertical, bullet_speed_m_s, config_.gravity_m_s2,
    config_.enable_gravity_compensation);
  if (!ballistic.valid) {
    result.error = ballistic.error;
    return result;
  }

  const double final_total_lead_s = config_.additional_latency_seconds + ballistic.flight_time_s;
  if (!finite(final_total_lead_s) || final_total_lead_s < 0.0 ||
      final_total_lead_s > config_.max_flight_time_s + config_.additional_latency_seconds) {
    result.error = "final prediction lead exceeds configured limit";
    return result;
  }

  result.valid = true;
  result.error.clear();
  result.yaw_rad = solved_yaw;
  result.pitch_rad = ballistic.pitch_rad;
  result.yaw_deg = result.yaw_rad * 180.0 / kPi;
  result.pitch_deg = result.pitch_rad * 180.0 / kPi;
  result.line_of_sight_pitch_rad = final_line_pitch;
  result.ballistic_pitch_offset_rad = result.pitch_rad - final_line_pitch;
  result.horizontal_distance_m = solved_horizontal;
  result.vertical_distance_m = solved_vertical;
  result.target_range_m = solved_range;
  result.flight_time_s = ballistic.flight_time_s;
  result.total_prediction_lead_s = final_total_lead_s;
  return result;
}

const AimCompensatorConfig & AimCompensator::config() const noexcept
{
  return config_;
}

bool AimCompensator::finite(double value) noexcept
{
  return std::isfinite(value);
}

void AimCompensator::validate_config(const AimCompensatorConfig & config)
{
  if (!finite(config.gravity_m_s2) || config.gravity_m_s2 < 0.0) {
    throw std::invalid_argument("gravity must be finite and non-negative");
  }
  if (!finite(config.min_bullet_speed_m_s) || config.min_bullet_speed_m_s <= 0.0 ||
      !finite(config.max_bullet_speed_m_s) ||
      config.max_bullet_speed_m_s < config.min_bullet_speed_m_s) {
    throw std::invalid_argument("invalid bullet speed range");
  }
  if (!finite(config.max_target_range_m) || config.max_target_range_m <= 0.0) {
    throw std::invalid_argument("maximum target range must be finite and positive");
  }
  if (!finite(config.max_flight_time_s) || config.max_flight_time_s <= 0.0) {
    throw std::invalid_argument("maximum flight time must be finite and positive");
  }
  if (!finite(config.additional_latency_seconds) || config.additional_latency_seconds < 0.0) {
    throw std::invalid_argument("additional latency must be finite and non-negative");
  }
  if (config.max_iterations == 0) {
    throw std::invalid_argument("maximum ballistic iterations must be positive");
  }
  if (!finite(config.convergence_tolerance_seconds) ||
      config.convergence_tolerance_seconds <= 0.0) {
    throw std::invalid_argument("convergence tolerance must be finite and positive");
  }
}

AimCompensator::BallisticResult AimCompensator::solve_ballistic(
  double horizontal_distance_m, double vertical_distance_m, double bullet_speed_m_s,
  double gravity_m_s2, bool enable_gravity_compensation)
{
  BallisticResult result;
  if (!finite(horizontal_distance_m) || !finite(vertical_distance_m) ||
      horizontal_distance_m <= kMinimumHorizontalDistanceM || !finite(bullet_speed_m_s) ||
      bullet_speed_m_s <= 0.0 || !finite(gravity_m_s2) || gravity_m_s2 < 0.0) {
    result.error = "invalid ballistic input";
    return result;
  }

  const double line_pitch = std::atan2(vertical_distance_m, horizontal_distance_m);
  if (!enable_gravity_compensation || gravity_m_s2 == 0.0) {
    result.pitch_rad = line_pitch;
    result.flight_time_s =
      std::hypot(horizontal_distance_m, vertical_distance_m) / bullet_speed_m_s;
    result.valid = finite(result.pitch_rad) && finite(result.flight_time_s) &&
                   result.flight_time_s >= 0.0;
    if (!result.valid) result.error = "non-finite geometric flight time";
    return result;
  }

  const double speed_squared = bullet_speed_m_s * bullet_speed_m_s;
  const double discriminant =
    speed_squared * speed_squared -
    gravity_m_s2 * (gravity_m_s2 * horizontal_distance_m * horizontal_distance_m +
                    2.0 * vertical_distance_m * speed_squared);
  if (!finite(discriminant)) {
    result.error = "ballistic discriminant is non-finite";
    return result;
  }
  if (discriminant < -kDiscriminantTolerance) {
    result.error = "target is outside ballistic envelope";
    return result;
  }

  const double root = std::sqrt(std::max(0.0, discriminant));
  const double tangent = (speed_squared - root) / (gravity_m_s2 * horizontal_distance_m);
  result.pitch_rad = std::atan(tangent);
  const double cosine = std::cos(result.pitch_rad);
  if (!finite(result.pitch_rad) || !finite(cosine) || cosine <= kMinimumCosine) {
    result.error = "ballistic pitch is invalid";
    return result;
  }

  result.flight_time_s = horizontal_distance_m / (bullet_speed_m_s * cosine);
  if (!finite(result.flight_time_s) || result.flight_time_s < 0.0) {
    result.error = "ballistic flight time is invalid";
    return result;
  }
  result.valid = true;
  return result;
}

}  // namespace srm::control
