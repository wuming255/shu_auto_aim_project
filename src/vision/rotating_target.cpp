#include "vision/rotating_target.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>
#include <utility>

namespace srm::vision
{
namespace
{
constexpr double pi = 3.14159265358979323846;
constexpr double quarter = pi/2;
double seconds(std::chrono::steady_clock::time_point time)
{ return std::chrono::duration<double>(time.time_since_epoch()).count(); }
int plate_index(double theta, double phase)
{
  const auto index = static_cast<long long>(std::llround((theta-phase)/quarter));
  return static_cast<int>((index%4+4)%4);
}
bool valid_rotation(const cv::Matx33d & rotation)
{
  return cv::checkRange(cv::Mat(rotation)) &&
    cv::norm(cv::Mat(rotation*rotation.t()-cv::Matx33d::eye())) < 1e-5 &&
    std::abs(cv::determinant(rotation)-1.0) < 1e-5;
}
}

RotatingTarget::RotatingTarget(RotatingTargetConfig config, PoseSolverConfig pose)
  : config_(std::move(config)), pose_(std::move(pose))
{
  for (const auto value : {config_.min_radius_m, config_.max_radius_m, config_.fit_window_seconds,
    config_.short_loss_seconds, config_.long_loss_seconds, config_.max_position_residual_m,
    config_.max_phase_residual_rad, config_.max_tilt_error_rad, config_.max_angular_speed_rad_s,
    config_.min_normal_span_rad}) {
    if (!std::isfinite(value) || value <= 0) throw std::invalid_argument("invalid rotation model limits");
  }
  if (config_.min_radius_m >= config_.max_radius_m || config_.short_loss_seconds >= config_.long_loss_seconds ||
      config_.fit_window_seconds < config_.long_loss_seconds || config_.min_updates < 3 ||
      config_.max_phase_residual_rad >= pi/4 || (config_.mounting_tilt_rad &&
      (!std::isfinite(*config_.mounting_tilt_rad) || std::abs(*config_.mounting_tilt_rad) >= pi/2))) {
    throw std::invalid_argument("invalid rotation geometry or observability limits");
  }
}

void RotatingTarget::reset() noexcept
{
  samples_.clear(); state_ = {}; last_update_.reset(); last_seen_.reset(); last_stable_frame_ = false;
  last_fit_reliable_ = false;
}

cv::Vec3d RotationEstimate::armor_position(double dt, int plate) const noexcept
{
  const double theta = phase_rad+angular_velocity_rad_s*dt+plate*quarter;
  return center_world_m+cv::Vec3d(radius_m*std::cos(theta),radius_m*std::sin(theta),0);
}

cv::Matx33d gimbal_to_world(double yaw, double pitch, double roll)
{
  const double cy=std::cos(yaw), sy=std::sin(yaw), cp=std::cos(pitch), sp=std::sin(pitch);
  const double cr=std::cos(roll), sr=std::sin(roll);
  return cv::Matx33d(cy,-sy,0, sy,cy,0, 0,0,1)*
    cv::Matx33d(cp,0,-sp, 0,1,0, sp,0,cp)*cv::Matx33d(1,0,0, 0,cr,-sr, 0,sr,cr);
}

bool RotatingTarget::fit(double now)
{
  if (samples_.size() < config_.min_updates) return false;
  cv::Vec3d mean_position{};
  cv::Vec2d mean_radial{};
  double mean_time=0, mean_phase=0, mean_tilt=0;
  for (const auto & sample : samples_) {
    mean_position += sample.position; mean_radial += sample.radial;
    mean_time += sample.time-now; mean_phase += sample.phase; mean_tilt += sample.tilt;
  }
  const double count = static_cast<double>(samples_.size());
  mean_position *= 1/count; mean_radial *= 1/count;
  mean_time /= count; mean_phase /= count; mean_tilt /= count;
  double numerator=0, denominator=0, time_variance=0, phase_covariance=0;
  for (const auto & sample : samples_) {
    const cv::Vec2d delta_n = sample.radial-mean_radial;
    numerator += cv::Vec2d(sample.position[0]-mean_position[0],sample.position[1]-mean_position[1]).dot(delta_n);
    denominator += delta_n.dot(delta_n);
    const double dt = sample.time-now-mean_time;
    time_variance += dt*dt; phase_covariance += dt*(sample.phase-mean_phase);
  }
  // 法向覆盖不足时半径不可观；不依赖任意固定初始半径宣称已收敛。
  if (denominator/count < 1-std::cos(config_.min_normal_span_rad/2) || time_variance < 1e-5) return false;
  const double radius = numerator/denominator, omega = phase_covariance/time_variance;
  if (!std::isfinite(radius) || radius < config_.min_radius_m || radius > config_.max_radius_m ||
      !std::isfinite(omega) || std::abs(omega) > config_.max_angular_speed_rad_s) return false;
  const cv::Vec3d center = mean_position-cv::Vec3d(radius*mean_radial[0],radius*mean_radial[1],0);
  const double phase = mean_phase-omega*mean_time;
  double position_error=0, phase_error=0;
  for (const auto & sample : samples_) {
    const auto residual = sample.position-center-cv::Vec3d(radius*sample.radial[0],radius*sample.radial[1],0);
    position_error += residual.dot(residual);
    const double dphase = sample.phase-(phase+omega*(sample.time-now));
    phase_error += dphase*dphase;
  }
  const double position_rms=std::sqrt(position_error/count), phase_rms=std::sqrt(phase_error/count);
  if (position_rms > config_.max_position_residual_m || phase_rms > config_.max_phase_residual_rad) return false;
  state_.center_world_m = center; state_.radius_m = radius;
  state_.phase_rad = phase; state_.angular_velocity_rad_s = omega;
  state_.mounting_tilt_rad = mean_tilt;
  state_.position_rms_m = position_rms; state_.phase_rms_rad = phase_rms;
  // 残差标准误只用于软件质量门；不是现场标定过的概率置信区间。
  state_.radius_std_m = position_rms/std::sqrt(denominator);
  state_.angular_velocity_std_rad_s = phase_rms/std::sqrt(time_variance);
  return true;
}

RotationEstimate RotatingTarget::update(const TargetSelection & selection,
  std::chrono::steady_clock::time_point time, std::optional<cv::Matx33d> orientation)
{
  const bool stable_frame = orientation.has_value() || config_.assume_static_camera;
  if ((orientation && !valid_rotation(*orientation))) {
    reset(); state_.timestamp = time; state_.status = "invalid_reference_rotation"; return state_;
  }
  const double dt = last_update_ ? seconds(time)-seconds(*last_update_) : 0;
  const double age = last_seen_ ? seconds(time)-seconds(*last_seen_) : 0;
  bool reset_time = last_update_ && (dt <= 0 || dt > config_.long_loss_seconds);
  const bool expired = last_seen_ && age > config_.long_loss_seconds;
  // 四甲相位以 pi/2 为周期。观测间隔可能跨过半个周期时，不能沿用旧模型
  // 给新观测展开相位，否则反向运动可能周期性匹配旧方向并永远无法重捕获。
  // 无观测期间仍按短失门限提供预测；新观测到达时才重新建立可观测历史。
  const bool ambiguous_gap = last_seen_ && config_.max_angular_speed_rad_s*age >= pi/4 &&
    std::any_of(selection.observations.begin(),selection.observations.end(),
      [](const auto & observation) { return observation.eligible; });
  if (reset_time || expired || ambiguous_gap || (last_update_ && stable_frame != last_stable_frame_)) reset();
  else if (last_update_ && state_.initialized) state_.phase_rad += state_.angular_velocity_rad_s*dt;
  last_update_ = time; last_stable_frame_ = stable_frame;
  state_.timestamp = time; state_.rotation_gimbal_to_world = orientation.value_or(cv::Matx33d::eye());
  state_.stable_frame_valid = stable_frame; state_.current_visible = false;
  state_.current_observation.reset(); state_.pose_ambiguous = false; state_.reliable = false;
  state_.prediction_valid = false;
  const double now = seconds(time);
  while (!samples_.empty() && now-samples_.front().time > config_.fit_window_seconds) samples_.pop_front();

  struct Candidate { TargetObservation observation; Sample sample; double theta; double score; };
  std::vector<Candidate> candidates;
  bool rejected = false;
  for (const auto & observation : selection.observations) {
    if (!observation.eligible) continue;
    std::vector<Candidate> hypotheses;
    const auto poses = observation.pose_hypotheses.empty() ?
      std::vector<PoseSolution>{observation.pose} : observation.pose_hypotheses;
    for (const auto & pose : poses) {
      if (!pose.valid || !cv::checkRange(cv::Mat(pose.translation_gimbal_m)) ||
          !cv::checkRange(cv::Mat(pose.rotation_vector_camera))) continue;
      cv::Matx33d camera_rotation;
      cv::Rodrigues(pose.rotation_vector_camera,camera_rotation);
      const cv::Matx33d world_rotation = state_.rotation_gimbal_to_world*pose_.rotation_camera_to_gimbal*camera_rotation;
      const cv::Vec3d normal(world_rotation(0,2),world_rotation(1,2),world_rotation(2,2));
      if (std::hypot(normal[0],normal[1]) < 0.5 || world_rotation(2,1) < 0.3) continue;
      const double theta = std::atan2(normal[1],normal[0]);
      const double tilt = std::atan2(normal[2],std::hypot(normal[0],normal[1]));
      const auto tilt_reference = config_.mounting_tilt_rad ? config_.mounting_tilt_rad :
        (state_.initialized ? std::optional<double>(state_.mounting_tilt_rad) : std::nullopt);
      const double tilt_error = tilt_reference ? std::abs(tilt-*tilt_reference) : 0;
      if (tilt_error > config_.max_tilt_error_rad) continue;
      const cv::Vec3d position = state_.rotation_gimbal_to_world*pose.translation_gimbal_m;
      const cv::Vec2d radial(std::cos(theta),std::sin(theta));
      double phase = theta;
      if (!samples_.empty() || state_.initialized) {
        const double predicted = state_.initialized ? state_.phase_rad : samples_.back().phase;
        phase = predicted+std::remainder(theta-predicted,quarter);
      }
      double score = pose.reprojection_rms_px/pose_.max_reprojection_rms_px + tilt_error/config_.max_tilt_error_rad;
      if (state_.initialized) {
        const double residual = cv::norm(position-state_.center_world_m-
          cv::Vec3d(state_.radius_m*radial[0],state_.radius_m*radial[1],0));
        if (residual > config_.max_position_residual_m ||
            std::abs(phase-state_.phase_rad) > config_.max_phase_residual_rad) continue;
        score += residual/config_.max_position_residual_m+
          std::abs(phase-state_.phase_rad)/config_.max_phase_residual_rad;
      }
      TargetObservation associated = observation;
      associated.pose = pose; // 实际采用的 IPPE 分支，供跟踪框和CSV使用。
      hypotheses.push_back({std::move(associated),{now,position,radial,phase,tilt},theta,score});
    }
    std::stable_sort(hypotheses.begin(),hypotheses.end(),[](const auto & a,const auto & b) { return a.score < b.score; });
    if (hypotheses.empty()) { rejected = true; continue; }
    if (hypotheses.size() > 1 && hypotheses[1].score-hypotheses[0].score < 0.03 &&
        std::abs(std::remainder(hypotheses[1].theta-hypotheses[0].theta,2*pi)) > 0.15) {
      state_.pose_ambiguous = true; rejected = true; continue;
    }
    candidates.push_back(std::move(hypotheses.front()));
  }
  std::stable_sort(candidates.begin(),candidates.end(),[](const auto & a,const auto & b) { return a.score < b.score; });
  std::set<int> seen;
  for (auto & candidate : candidates) {
    const double reference = state_.initialized ? state_.phase_rad :
      (samples_.empty() ? candidate.theta : samples_.back().phase);
    candidate.sample.phase = reference+std::remainder(candidate.theta-reference,quarter);
    const int index = plate_index(candidate.theta,reference);
    if (!seen.insert(index).second) continue;
    if (!state_.current_observation) {
      state_.current_observation = candidate.observation;
      state_.associated_plate = index;
    }
    samples_.push_back(candidate.sample);
  }
  while (samples_.size() > 512) samples_.pop_front();
  if (!seen.empty()) {
    state_.current_visible = true;
    last_seen_ = time;
    ++state_.confirmed_updates;
    const bool fitted = fit(now);
    state_.initialized = fitted;
    state_.observation_age_seconds = 0;
    state_.reliable = fitted && state_.confirmed_updates >= config_.min_updates &&
      stable_frame && pose_.feature_size_m.has_value() && config_.mounting_tilt_rad.has_value() &&
      state_.radius_std_m < config_.max_position_residual_m/2 &&
      state_.angular_velocity_std_rad_s < config_.max_angular_speed_rad_s/10;
    state_.prediction_valid = state_.reliable;
    last_fit_reliable_ = state_.reliable;
    state_.status = !fitted ? "collecting_observable_geometry" :
      (!stable_frame ? "reference_frame_unconfirmed" :
      (!pose_.feature_size_m ? "feature_geometry_unconfirmed" :
       (!config_.mounting_tilt_rad ? "mounting_tilt_unconfirmed" :
       (state_.reliable ? "tracking_four_plates" : "estimate_uncertain"))));
  } else {
    state_.observation_age_seconds = last_seen_ ? now-seconds(*last_seen_) : 0;
    state_.prediction_valid = state_.initialized && last_fit_reliable_ && stable_frame && pose_.feature_size_m.has_value() &&
      state_.observation_age_seconds <= config_.short_loss_seconds;
    state_.status = state_.prediction_valid ? "temporarily_lost_prediction_only" :
      (expired || reset_time ? "long_loss_reacquire" :
      (state_.pose_ambiguous ? "ambiguous_planar_pose" : (rejected ? "observation_rejected" : "no_observation")));
  }
  return state_;
}
}  // namespace srm::vision
