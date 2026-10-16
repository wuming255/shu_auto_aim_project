#ifndef SHU_AUTO_AIM_ROTATING_TARGET_HPP
#define SHU_AUTO_AIM_ROTATING_TARGET_HPP

#include <array>
#include <deque>
#include "vision/target_selector.hpp"

namespace srm::vision
{
struct RotatingTargetConfig
{
  bool enabled = false;
  bool assume_static_camera = false;  // 仅供明确声明固定相机的离线输入。
  bool feedback_frame_confirmed = false;
  std::optional<double> mounting_tilt_rad;
  double min_radius_m = 0.03;
  double max_radius_m = 1.0;
  double fit_window_seconds = 1.0;
  double short_loss_seconds = 0.15;
  double long_loss_seconds = 0.5;
  double max_position_residual_m = 0.08;
  double max_phase_residual_rad = 0.25;
  double max_tilt_error_rad = 0.20;
  double max_angular_speed_rad_s = 20.0;
  double min_normal_span_rad = 0.15;
  std::size_t min_updates = 8;
};

struct RotationEstimate
{
  bool initialized = false;
  bool reliable = false;
  bool current_visible = false;
  bool prediction_valid = false;
  bool stable_frame_valid = false;
  bool pose_ambiguous = false;
  cv::Vec3d center_world_m{};
  double radius_m = 0.0;
  double phase_rad = 0.0;  // 同一四甲轨道的连续相位，物理板编号只在模4意义下存在。
  double angular_velocity_rad_s = 0.0;
  double mounting_tilt_rad = 0.0;
  double position_rms_m = 0.0;
  double phase_rms_rad = 0.0;
  double radius_std_m = 0.0;
  double angular_velocity_std_rad_s = 0.0;
  double observation_age_seconds = 0.0;
  std::size_t confirmed_updates = 0;
  int associated_plate = 0;
  std::optional<TargetObservation> current_observation;
  std::chrono::steady_clock::time_point timestamp{};
  cv::Matx33d rotation_gimbal_to_world = cv::Matx33d::eye();
  std::string status = "uninitialized";
  cv::Vec3d armor_position(double dt_seconds, int plate) const noexcept;
};

class RotatingTarget final
{
public:
  explicit RotatingTarget(RotatingTargetConfig config = {}, PoseSolverConfig pose = {});
  RotationEstimate update(const TargetSelection & selection,
    std::chrono::steady_clock::time_point time,
    std::optional<cv::Matx33d> rotation_gimbal_to_world = std::nullopt);
  void reset() noexcept;
private:
  struct Sample { double time; cv::Vec3d position; cv::Vec2d radial; double phase; double tilt; };
  RotatingTargetConfig config_;
  PoseSolverConfig pose_;
  std::deque<Sample> samples_;
  RotationEstimate state_;
  std::optional<std::chrono::steady_clock::time_point> last_update_, last_seen_;
  bool last_stable_frame_ = false;
  bool last_fit_reliable_ = false;
  bool fit(double now);
};

// 显式约定 x前/y左/z上，yaw左转、pitch抬头、roll绕x右手；只能在现场确认后使用。
cv::Matx33d gimbal_to_world(double yaw_rad, double pitch_rad, double roll_rad);
}  // namespace srm::vision
#endif
