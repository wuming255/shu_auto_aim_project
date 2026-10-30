// State/plate model and Joseph EKF update adapted from sp_vision_25-main:
// tasks/auto_aim/target.cpp and tools/extended_kalman_filter.cpp.
// Copyright (c) 2025 TongjiSuperPower. MIT; see vehicle_target.LICENSE.
#include "vision/vehicle_target.hpp"

#include <Eigen/Cholesky>
#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <utility>

namespace srm::vision
{
namespace
{
constexpr double pi = 3.14159265358979323846;
constexpr double quarter = pi / 2;
double wrap(double angle) { return std::remainder(angle, 2*pi); }
double seconds(std::chrono::steady_clock::time_point a, std::chrono::steady_clock::time_point b)
{ return std::chrono::duration<double>(a-b).count(); }
}

VehicleTarget::VehicleTarget(TargetTrackerConfig config, PoseSolverConfig pose, double mounting_tilt)
: config_(std::move(config)), pose_(std::move(pose)), mounting_tilt_(mounting_tilt) {}

void VehicleTarget::reset() noexcept
{
  time_.reset(); seen_.reset(); observation_.reset();
  updates_ = misses_ = 0; current_ = false;
  x_.setZero(); p_.setIdentity();
}

cv::Vec3d VehicleTarget::armor_position(double dt, int plate) const
{
  const bool alternate = plate % 2;
  const double radius = x_[8] + (alternate ? x_[9] : 0);
  const double phase = x_[6] + x_[7]*dt + plate*quarter;
  // Our PnP +Z normal points outward; Tongji's armor yaw uses the opposite sign.
  return {x_[0]+x_[1]*dt+radius*std::cos(phase),
          x_[2]+x_[3]*dt+radius*std::sin(phase),
          x_[4]+x_[5]*dt+(alternate ? x_[10] : 0)};
}

void VehicleTarget::predict(double dt)
{
  auto f = Covariance::Identity().eval();
  auto q = Covariance::Zero().eval();
  for (int i : {0, 2, 4, 6}) {
    f(i,i+1) = dt;
    const double variance = i == 6 ? 400.0 : 100.0;
    q(i,i) = dt*dt*dt*dt*variance/4;
    q(i,i+1) = q(i+1,i) = dt*dt*dt*variance/2;
    q(i+1,i+1) = dt*dt*variance;
  }
  x_ = (f*x_).eval(); x_[6] = wrap(x_[6]);
  p_ = (f*p_*f.transpose()+q).eval();
}

void VehicleTarget::correct(const cv::Vec3d & position, double phase, int plate)
{
  const bool alternate = plate % 2;
  const double angle = x_[6]+plate*quarter;
  const double radius = x_[8]+(alternate ? x_[9] : 0);
  const double c = std::cos(angle), s = std::sin(angle);
  Eigen::Matrix<double,4,11> h = Eigen::Matrix<double,4,11>::Zero();
  h(0,0) = h(1,2) = h(2,4) = h(3,6) = 1;
  h(0,6) = -radius*s; h(1,6) = radius*c;
  h(0,8) = c; h(1,8) = s;
  if (alternate) { h(0,9) = c; h(1,9) = s; h(2,10) = 1; }
  const auto predicted = armor_position(0,plate);
  Eigen::Vector4d residual(position[0]-predicted[0], position[1]-predicted[1],
    position[2]-predicted[2], wrap(phase-angle));
  Eigen::Matrix4d r = Eigen::Matrix4d::Zero();
  // Position and normal are PnP observations, not simulator target truth.
  r.diagonal() << 0.0036, 0.0036, 0.0009, 0.0225;
  const Eigen::Matrix4d innovation = h*p_*h.transpose()+r;
  const Eigen::Matrix<double,11,4> k = innovation.ldlt().solve(h*p_).transpose();
  x_ += k*residual; x_[6] = wrap(x_[6]);
  const Covariance a = Covariance::Identity()-k*h;
  p_ = (a*p_*a.transpose()+k*r*k.transpose()).eval();
}

bool VehicleTarget::geometry_valid() const
{
  return x_.allFinite() && p_.allFinite() && x_[8] > 0.03 && x_[8] < 1.0 &&
    x_[8]+x_[9] > 0.03 && x_[8]+x_[9] < 1.0;
}

void VehicleTarget::update(const TargetSelection & selection,
  std::chrono::steady_clock::time_point time, const cv::Matx33d & orientation,
  const cv::Vec3d & muzzle_world)
{
  if (time_ && (time <= *time_ || seconds(time,*time_) > config_.max_dt_seconds ||
      (seen_ && seconds(time,*seen_) > config_.max_prediction_age_seconds))) reset();
  if (time_) predict(seconds(time,*time_));
  time_ = time; current_ = false;
  struct Candidate { TargetObservation observation; cv::Vec3d position; double phase, score; int plate; };
  std::vector<Candidate> candidates;
  for (const auto & observation : selection.observations) {
    if (!observation.eligible) continue;
    if (observation_ && !observation_->armor.classification.label.empty() &&
        observation.armor.classification.label != observation_->armor.classification.label) continue;
    if (!seen_ && selection.selected() != &observation) continue;
    const auto poses = observation.pose_hypotheses.empty() ?
      std::vector<PoseSolution>{observation.pose} : observation.pose_hypotheses;
    for (const auto & pose : poses) {
      if (!pose.valid) continue;
      cv::Matx33d board;
      cv::Rodrigues(pose.rotation_vector_camera,board);
      board = orientation*pose_.rotation_camera_to_gimbal*board;
      const double horizontal = std::hypot(board(0,2),board(1,2));
      if (horizontal < 0.5 || board(2,1) < 0.3) continue;
      const double tilt_error = std::abs(std::atan2(board(2,2),horizontal)-mounting_tilt_);
      if (tilt_error > 0.35) continue;
      const auto position = muzzle_world+orientation*pose.translation_gimbal_m;
      const double phase = std::atan2(board(1,2),board(0,2));
      double score = pose.reprojection_rms_px/pose_.max_reprojection_rms_px+tilt_error;
      int plate = 0;
      if (seen_) {
        double best = std::numeric_limits<double>::infinity();
        for (int id=0; id<4; ++id) {
          const double distance = cv::norm(position-armor_position(0,id));
          const double angle = std::abs(wrap(phase-x_[6]-id*quarter));
          if (distance > 0.6 || angle > 0.9) continue;
          const double error = distance/0.3+angle/0.5;
          if (error < best) { best = error; plate = id; }
        }
        if (!std::isfinite(best)) continue;
        score += best;
      }
      auto associated = observation; associated.pose = pose;
      candidates.push_back({std::move(associated),position,phase,score,plate});
    }
  }
  std::stable_sort(candidates.begin(),candidates.end(),
    [](const auto & a,const auto & b) { return a.score < b.score; });
  std::set<int> plates;
  std::set<std::size_t> detections;
  for (const auto & candidate : candidates) {
    if (plates.count(candidate.plate) || detections.count(candidate.observation.detection_index)) continue;
    plates.insert(candidate.plate); detections.insert(candidate.observation.detection_index);
    if (!seen_) {
      x_ << candidate.position[0]-0.2*std::cos(candidate.phase),0,
            candidate.position[1]-0.2*std::sin(candidate.phase),0,candidate.position[2],0,
            candidate.phase,0,0.2,0,0;
      p_.setZero(); p_.diagonal() << 1,64,1,64,1,64,0.4,100,0.01,0.01,0.01;
    } else correct(candidate.position,candidate.phase,candidate.plate);
    if (!geometry_valid()) { reset(); return; }
    if (!current_) observation_ = candidate.observation;
    current_ = true; seen_ = time;
  }
  if (current_) { ++updates_; misses_ = 0; }
  else ++misses_;
}

double VehicleTarget::prediction_seconds(std::chrono::steady_clock::time_point time) const
{ return time_ ? seconds(time,*time_) : 0; }

TargetTrackEstimate VehicleTarget::estimate(std::chrono::steady_clock::time_point time) const
{
  TargetTrackEstimate result;
  result.timestamp = time; result.confirmed_updates = updates_; result.consecutive_misses = misses_;
  if (!seen_ || time < *seen_ || seconds(time,*seen_) > config_.max_prediction_age_seconds) {
    result.status = "follow_prediction_expired"; return result;
  }
  result.last_observation = observation_; result.has_last_measurement = true;
  result.has_current_measurement = current_;
  if (current_) result.current_observation = observation_;
  const bool ready = updates_ >= config_.min_confirmed_updates;
  result.state = ready ? (current_ ? TargetTrackState::tracking : TargetTrackState::temporarily_lost) :
    TargetTrackState::detecting;
  result.prediction_valid = result.velocity_valid = ready;
  result.status = !ready ? "follow_ekf_detecting" : (current_ ? "follow_ekf_tracking" : "follow_ekf_coasting");
  if (observation_) {
    result.measured_yaw_rad = result.unwrapped_yaw_rad = observation_->pose.yaw_rad;
    result.measured_pitch_rad = observation_->pose.pitch_rad;
    result.measured_range_m = observation_->pose.range_m;
  }
  return result;
}

int VehicleTarget::choose_plate(double dt, const cv::Vec3d & muzzle, int previous) const
{
  const double facing = std::atan2(muzzle[1]-x_[2]-x_[3]*dt,muzzle[0]-x_[0]-x_[1]*dt);
  int best = 0;
  double best_error = std::numeric_limits<double>::infinity();
  for (int plate=0; plate<4; ++plate) {
    double error = std::abs(wrap(x_[6]+x_[7]*dt+plate*quarter-facing));
    if (plate == previous) error -= 0.15; // Keep the selected plate near the switching boundary.
    if (error < best_error) { best = plate; best_error = error; }
  }
  return best;
}
}  // namespace srm::vision
