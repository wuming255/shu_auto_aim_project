#ifndef SHU_AUTO_AIM_VEHICLE_TARGET_HPP
#define SHU_AUTO_AIM_VEHICLE_TARGET_HPP

#include <Eigen/Core>
#include <deque>
#include "vision/target_tracker.hpp"

namespace srm::vision
{
struct VehicleTargetDiagnostics
{
  cv::Vec3d center_world_m{};
  cv::Vec3d observed_position_world_m{};
  double observed_phase_rad = 0;
  double phase_rad = 0, angular_velocity_rad_s = 0;
  double radius_m = 0, alternate_radius_m = 0, alternate_height_m = 0;
  double observation_age_s = 0, fit_span_s = 0, normal_span_rad = 0;
  double position_rms_m = 0, phase_rms_rad = 0, omega_std_rad_s = 0;
  double fit_angular_velocity_rad_s = 0, fit_omega_std_rad_s = 0;
  double normalized_position_error = 0, normalized_phase_error = 0;
  int associated_plate = -1;
  std::size_t generation = 0;
  bool initialized = false, pose_ambiguous = false;
  std::string reset_reason = "startup";
  std::string fit_status = "samples";
};

// Fixed-center four-plate inertial model, adapted from TongjiSuperPower Target (MIT).
// State: cx,cy,cz,phase,omega,r,alternate_radius_delta,alternate_height.
class VehicleTarget final
{
public:
  VehicleTarget(TargetTrackerConfig config, PoseSolverConfig pose, double mounting_tilt);
  void update(const TargetSelection & selection, std::chrono::steady_clock::time_point time,
    const cv::Matx33d & gimbal_to_world, const cv::Vec3d & muzzle_world);
  TargetTrackEstimate estimate(std::chrono::steady_clock::time_point time) const;
  cv::Vec3d armor_position(double dt, int plate) const;
  int choose_plate(double dt, const cv::Vec3d & muzzle_world, int previous) const;
  double prediction_seconds(std::chrono::steady_clock::time_point time) const;
  VehicleTargetDiagnostics diagnostics(std::chrono::steady_clock::time_point time) const;
  void reset() noexcept;

private:
  using State = Eigen::Matrix<double, 8, 1>;
  using Covariance = Eigen::Matrix<double, 8, 8>;
  struct Sample
  {
    double time, phase;
    cv::Vec3d position;
    int plate;
    Eigen::Matrix4d noise;
  };
  void predict(double dt);
  void correct(const cv::Vec3d & position, double phase, int plate, const Eigen::Matrix4d & noise);
  bool fit(double now);
  bool geometry_valid() const;
  TargetTrackerConfig config_;
  PoseSolverConfig pose_;
  double mounting_tilt_;
  State x_ = State::Zero();
  Covariance p_ = Covariance::Identity();
  std::optional<std::chrono::steady_clock::time_point> time_, seen_;
  std::optional<TargetObservation> observation_;
  std::size_t updates_ = 0, misses_ = 0;
  bool current_ = false;
  bool initialized_ = false;
  bool expired_ = false;
  std::deque<Sample> samples_;
  VehicleTargetDiagnostics diagnostics_;
};
}  // namespace srm::vision
#endif
