#ifndef SHU_AUTO_AIM_FIXED_GUARD_HPP
#define SHU_AUTO_AIM_FIXED_GUARD_HPP

#include <chrono>
#include <optional>
#include <string>

#include "control/aim_compensator.hpp"
#include "vision/rotating_target.hpp"

namespace srm::control
{

struct FixedGuardConfig
{
  double relocalization_center_threshold_m = 0.10;
  double relocalization_radius_threshold_m = 0.05;
  double relocalization_yaw_threshold_rad = 0.15;
  double hit_window_half_width_rad = 0.08;
  double max_hold_seconds = 0.15;
};

struct FixedGuardResult
{
  AimSolution aim;
  bool guard_locked = false;
  bool relocalized = false;
  bool target_visible = false;
  bool fire_window_valid = false;
  double guard_phase_rad = 0.0;
  double predicted_phase_rad = 0.0;
  double phase_error_rad = 0.0;
  std::string status = "uninitialized";
};

class FixedGuardController final
{
public:
  explicit FixedGuardController(FixedGuardConfig config = {});

  FixedGuardResult update(
    const vision::RotationEstimate & rotation, const AimCompensator & compensator,
    double bullet_speed_m_s, std::chrono::steady_clock::time_point timestamp);

  void reset() noexcept;

  const FixedGuardConfig & config() const noexcept;

private:
  static bool finite(double value) noexcept;
  static void validate_config(const FixedGuardConfig & config);
  static double phase_difference(double lhs, double rhs) noexcept;
  static vision::TargetTrackEstimate make_target(
    const vision::RotationEstimate & rotation, int plate);

  FixedGuardConfig config_;
  bool guard_locked_ = false;
  cv::Vec3d locked_center_world_m_{};
  double locked_radius_m_ = 0.0;
  double guard_phase_rad_ = 0.0;
  double last_model_yaw_rad_ = 0.0;
  bool last_model_yaw_valid_ = false;
  std::optional<AimSolution> held_aim_;
};

}  // namespace srm::control

#endif  // SHU_AUTO_AIM_FIXED_GUARD_HPP
