#ifndef SHU_AUTO_AIM_VEHICLE_TARGET_HPP
#define SHU_AUTO_AIM_VEHICLE_TARGET_HPP

#include <Eigen/Core>
#include "vision/target_tracker.hpp"

namespace srm::vision
{
// Four-plate inertial-frame model, adapted from TongjiSuperPower Target (MIT).
// State: cx,vx,cy,vy,cz,vz,phase,omega,r,alternate_radius_delta,alternate_height.
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
  void reset() noexcept;

private:
  using State = Eigen::Matrix<double, 11, 1>;
  using Covariance = Eigen::Matrix<double, 11, 11>;
  void predict(double dt);
  void correct(const cv::Vec3d & position, double phase, int plate);
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
};
}  // namespace srm::vision
#endif
