// State/plate model and Joseph EKF update adapted from sp_vision_25-main:
// tasks/auto_aim/target.cpp and tools/extended_kalman_filter.cpp.
// Copyright (c) 2025 TongjiSuperPower. MIT; see vehicle_target.LICENSE.
#include "vision/vehicle_target.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
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
constexpr double quarter = pi/2;
double wrap(double angle) { return std::remainder(angle,2*pi); }
double seconds(std::chrono::steady_clock::time_point a, std::chrono::steady_clock::time_point b)
{ return std::chrono::duration<double>(a-b).count(); }
double epoch(std::chrono::steady_clock::time_point time)
{ return std::chrono::duration<double>(time.time_since_epoch()).count(); }

Eigen::Matrix4d observation_noise(const TargetObservation & observation, const PoseSolution & pose,
  const cv::Vec3d & ray_world, double phase, bool ambiguous, double focal)
{
  const auto & points = observation.armor.points;
  const double height = std::max(4.0,0.5*(cv::norm(points[0]-points[3])+cv::norm(points[1]-points[2])));
  const double range = cv::norm(ray_world);
  const cv::Vec3d ray = ray_world/std::max(range,1e-6);
  const double incidence = std::clamp(-ray[0]*std::cos(phase)-ray[1]*std::sin(phase),0.1,1.0);
  const double pixel = std::hypot(0.5,pose.reprojection_rms_px);
  // Depth follows relative image-size error; transverse error follows angular error.
  // Rotate this anisotropic covariance with the actual world-frame sight ray.
  const double radial = std::clamp(2*range*pixel/height,0.02,0.5);
  const double transverse = std::max(0.002,range*pixel/std::max(focal,1.0));
  const double phase_sigma = (0.12+1.5*pixel/height+0.05*(1-incidence))*(ambiguous ? 2 : 1);
  Eigen::Vector3d direction(ray[0],ray[1],ray[2]);
  Eigen::Matrix4d noise = Eigen::Matrix4d::Zero();
  noise.topLeftCorner<3,3>() = Eigen::Matrix3d::Identity()*transverse*transverse+
    (radial*radial-transverse*transverse)*direction*direction.transpose();
  noise(3,3) = phase_sigma*phase_sigma;
  return noise;
}
}

VehicleTarget::VehicleTarget(TargetTrackerConfig config, PoseSolverConfig pose, double mounting_tilt)
: config_(std::move(config)), pose_(std::move(pose)), mounting_tilt_(mounting_tilt) {}

void VehicleTarget::reset() noexcept
{
  time_.reset(); seen_.reset(); recovery_started_.reset(); observation_.reset(); samples_.clear();
  updates_ = misses_ = 0; current_ = initialized_ = expired_ = false;
  x_.setZero(); p_.setIdentity();
  const auto generation = diagnostics_.generation+1;
  const auto reset_count = diagnostics_.reset_count+1;
  diagnostics_ = {}; diagnostics_.generation = generation; diagnostics_.reset_count = reset_count;
  diagnostics_.reset_reason = "explicit_reset";
}

cv::Vec3d VehicleTarget::armor_position(double dt, int plate) const
{
  const bool alternate = plate%2;
  const double radius = x_[5]+(alternate ? x_[6] : 0);
  const double phase = x_[3]+x_[4]*dt+plate*quarter;
  return {x_[0]+radius*std::cos(phase),x_[1]+radius*std::sin(phase),x_[2]+(alternate ? x_[7] : 0)};
}

void VehicleTarget::predict(double dt)
{
  auto f = Covariance::Identity().eval();
  auto q = Covariance::Zero().eval();
  f(3,4) = dt;
  // Unknown, stable angular speed. Center/geometry are slowly corrected by vision,
  // with no translational-velocity states to extrapolate into the impact point.
  const double acceleration_variance = initialized_ ? 0.0025 : 4.0;
  q(3,3) = std::pow(dt,4)*acceleration_variance/4;
  q(3,4) = q(4,3) = dt*dt*dt*acceleration_variance/2;
  q(4,4) = dt*dt*acceleration_variance;
  for (int i : {0,1,2,5,6,7}) q(i,i) = dt*1e-6;
  x_ = (f*x_).eval(); x_[3] = wrap(x_[3]);
  p_ = (f*p_*f.transpose()+q).eval();
}

Eigen::Matrix<double,4,8> VehicleTarget::observation_jacobian(double dt, int plate) const
{
  const bool alternate = plate%2;
  const double angle = x_[3]+x_[4]*dt+plate*quarter;
  const double radius = x_[5]+(alternate ? x_[6] : 0);
  const double c = std::cos(angle), s = std::sin(angle);
  Eigen::Matrix<double,4,8> h = Eigen::Matrix<double,4,8>::Zero();
  h(0,0) = h(1,1) = h(2,2) = h(3,3) = 1;
  h(0,3) = -radius*s; h(1,3) = radius*c;
  h(0,4) = -radius*s*dt; h(1,4) = radius*c*dt; h(3,4) = dt;
  h(0,5) = c; h(1,5) = s;
  if (alternate) { h(0,6) = c; h(1,6) = s; h(2,7) = 1; }
  return h;
}

bool VehicleTarget::point_prediction_valid(double dt, int plate, const cv::Vec3d & muzzle) const
{
  if (plate < 0 || plate >= 4 || !std::isfinite(dt) || dt < 0 || !geometry_valid()) return false;
  const auto h = observation_jacobian(dt,plate).topRows<3>().eval();
  const Eigen::Matrix3d covariance = h*p_*h.transpose();
  const auto point = armor_position(dt,plate)-muzzle;
  const double range = cv::norm(point);
  if (!covariance.allFinite() || range < 1e-3) return false;
  const Eigen::Vector3d ray(point[0]/range,point[1]/range,point[2]/range);
  const Eigen::Matrix3d transverse = Eigen::Matrix3d::Identity()-ray*ray.transpose();
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen(transverse*covariance*transverse);
  // Quality of this plate at impact matters, not identifiability of every radius.
  // Retain center/radius correlations: a static observed point can be well known
  // while its rotation center remains uncertain. Three sigma is below 0.7 deg.
  return eigen.info() == Eigen::Success && eigen.eigenvalues().maxCoeff() < std::pow(range*0.004,2) &&
    ray.dot(covariance*ray) < 0.09;
}

bool VehicleTarget::correct(const cv::Vec3d & position, double phase, int plate, const Eigen::Matrix4d & noise)
{
  const auto h = observation_jacobian(0,plate);
  const double angle = x_[3]+plate*quarter;
  const auto predicted = armor_position(0,plate);
  const Eigen::Vector4d residual(position[0]-predicted[0],position[1]-predicted[1],
    position[2]-predicted[2],wrap(phase-angle));
  const Eigen::Matrix4d innovation = h*p_*h.transpose()+noise;
  const auto decomposition = innovation.ldlt();
  if (decomposition.info() != Eigen::Success || !decomposition.isPositive() ||
      residual.dot(decomposition.solve(residual)) > 25) return false;
  const Eigen::Matrix<double,8,4> k = decomposition.solve(h*p_).transpose();
  const State previous_x = x_;
  const Covariance previous_p = p_;
  x_ += k*residual; x_[3] = wrap(x_[3]);
  const Covariance a = Covariance::Identity()-k*h;
  p_ = (a*p_*a.transpose()+k*noise*k.transpose()).eval();
  if (!geometry_valid()) { x_=previous_x; p_=previous_p; return false; }
  return true;
}

bool VehicleTarget::fit(double now)
{
  diagnostics_.fit_status = "samples";
  if (samples_.size() < std::max<std::size_t>(6,config_.min_confirmed_updates)) return false;
  diagnostics_.fit_span_s = samples_.back().time-samples_.front().time;
  diagnostics_.fit_status = "time_span";
  if (diagnostics_.fit_span_s < 0.4) return false;
  double weight=0,mean_time=0,mean_phase=0;
  for (const auto & sample : samples_) {
    const double w=1/sample.noise(3,3);
    weight+=w; mean_time+=w*(sample.time-now); mean_phase+=w*sample.phase;
  }
  mean_time/=weight; mean_phase/=weight;
  double time_variance=0,phase_covariance=0;
  for (const auto & sample : samples_) {
    const double dt=sample.time-now-mean_time,w=1/sample.noise(3,3);
    time_variance+=w*dt*dt; phase_covariance+=w*dt*(sample.phase-mean_phase);
  }
  if (time_variance<1e-6) return false;
  const double omega=phase_covariance/time_variance,phase=mean_phase-omega*mean_time;
  Eigen::Matrix4d information = Eigen::Matrix4d::Zero();
  Eigen::Vector4d rhs = Eigen::Vector4d::Zero();
  double z_weight[2]{}, z_sum[2]{};
  double min_phase[2]{1e10,1e10},max_phase[2]{-1e10,-1e10};
  for (const auto & sample : samples_) {
    const int group = sample.plate%2;
    min_phase[group] = std::min(min_phase[group],sample.phase);
    max_phase[group] = std::max(max_phase[group],sample.phase);
    const double angle = phase+omega*(sample.time-now)+sample.plate*quarter;
    Eigen::Matrix<double,2,4> h = Eigen::Matrix<double,2,4>::Zero();
    h(0,0)=h(1,1)=1; h(0,2+group)=std::cos(angle); h(1,2+group)=std::sin(angle);
    // Include normal uncertainty in geometry fitting; it is not exact ground truth.
    Eigen::Matrix2d noise = sample.noise.topLeftCorner<2,2>();
    noise.diagonal().array() += 0.09*sample.noise(3,3);
    const Eigen::Matrix2d precision = noise.ldlt().solve(Eigen::Matrix2d::Identity());
    information += h.transpose()*precision*h;
    rhs += h.transpose()*precision*Eigen::Vector2d(sample.position[0],sample.position[1]);
    const double wz = 1/sample.noise(2,2);
    z_weight[group] += wz; z_sum[group] += wz*sample.position[2];
  }
  diagnostics_.normal_span_rad = std::max(max_phase[0]-min_phase[0],max_phase[1]-min_phase[1]);
  diagnostics_.fit_status = "normal_coverage";
  if (z_weight[0] == 0 || z_weight[1] == 0 || diagnostics_.normal_span_rad < 0.7) return false;
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> eigen(information);
  diagnostics_.fit_status = "geometry_information";
  if (eigen.info() != Eigen::Success || eigen.eigenvalues()[0] < 1e-4) return false;
  const Eigen::Matrix4d geometry_covariance = information.ldlt().solve(Eigen::Matrix4d::Identity());
  const Eigen::Vector4d geometry = information.ldlt().solve(rhs);
  diagnostics_.fit_status = "geometry_bounds";
  if (!geometry.allFinite() || geometry[2] < 0.03 || geometry[2] > 1 || geometry[3] < 0.03 || geometry[3] > 1) return false;
  diagnostics_.fit_status = "geometry_uncertain";
  if (geometry_covariance(2,2) > 0.0036 || geometry_covariance(3,3) > 0.0036) return false;
  double position_error=0,phase_error=0,normalized_error=0,normalized_phase_error=0;
  for (const auto & sample : samples_) {
    const int group=sample.plate%2;
    const double angle=phase+omega*(sample.time-now)+sample.plate*quarter;
    const cv::Vec3d predicted(geometry[0]+geometry[2+group]*std::cos(angle),
      geometry[1]+geometry[2+group]*std::sin(angle),z_sum[group]/z_weight[group]);
    const auto residual=sample.position-predicted;
    position_error += residual.dot(residual);
    Eigen::Matrix3d noise=sample.noise.topLeftCorner<3,3>();
    noise(0,0) += 0.09*sample.noise(3,3); noise(1,1) += 0.09*sample.noise(3,3);
    const Eigen::Vector3d delta(residual[0],residual[1],residual[2]);
    normalized_error += delta.dot(noise.ldlt().solve(delta));
    phase_error += std::pow(sample.phase-phase-omega*(sample.time-now),2);
    normalized_phase_error += std::pow(sample.phase-phase-omega*(sample.time-now),2)/sample.noise(3,3);
  }
  diagnostics_.position_rms_m = std::sqrt(position_error/samples_.size());
  diagnostics_.phase_rms_rad = std::sqrt(phase_error/samples_.size());
  diagnostics_.normalized_position_error = normalized_error/samples_.size();
  diagnostics_.normalized_phase_error = normalized_phase_error/samples_.size();
  const double omega_variance = std::max(1.0,phase_error*weight/samples_.size()/samples_.size())/time_variance;
  diagnostics_.fit_angular_velocity_rad_s = omega;
  diagnostics_.fit_omega_std_rad_s = std::sqrt(omega_variance);
  diagnostics_.fit_status = "fit_residual";
  if (diagnostics_.normalized_position_error > 9 || diagnostics_.normalized_phase_error > 9 ||
      omega_variance > 0.0225 || std::abs(omega) > config_.max_abs_angular_velocity_rad_s) return false;
  if (!initialized_) {
    // One batch initialization REPLACES the provisional filter. These same samples
    // are never fused again as independent EKF measurements.
    x_ << geometry[0],geometry[1],z_sum[0]/z_weight[0],wrap(phase),omega,
      geometry[2],geometry[3]-geometry[2],z_sum[1]/z_weight[1]-z_sum[0]/z_weight[0];
    p_.setZero();
    Eigen::Matrix<double,8,4> transform = Eigen::Matrix<double,8,4>::Zero();
    transform(0,0)=transform(1,1)=transform(5,2)=transform(6,3)=1; transform(6,2)=-1;
    p_ = transform*geometry_covariance*transform.transpose();
    p_(2,2)=1/z_weight[0]; p_(7,7)=1/z_weight[0]+1/z_weight[1]; p_(2,7)=p_(7,2)=-1/z_weight[0];
    p_(3,3)=1/weight+mean_time*mean_time*omega_variance;
    p_(4,4)=omega_variance; p_(3,4)=p_(4,3)=-mean_time*omega_variance;
    p_.diagonal().array() += 1e-6;
  }
  diagnostics_.fit_status = "consistent";
  return true;
}

void VehicleTarget::fit_provisional(double now)
{
  if (samples_.size() < 3 || samples_.back().time-samples_.front().time < 0.06) return;
  // Fit the unknown speed from raw normals, independently of the radius prior.
  // Replacing the provisional posterior avoids fusing this window twice. Once
  // full geometry is established, only fresh observations update the EKF.
  double weight=0,mean_time=0,mean_phase=0;
  for (const auto & sample : samples_) {
    const double w=1/sample.noise(3,3);
    weight+=w; mean_time+=w*(sample.time-now); mean_phase+=w*sample.phase;
  }
  mean_time/=weight; mean_phase/=weight;
  double variance=0,covariance=0;
  for (const auto & sample : samples_) {
    const double dt=sample.time-now-mean_time,w=1/sample.noise(3,3);
    variance+=w*dt*dt; covariance+=w*dt*(sample.phase-mean_phase);
  }
  if (variance<1e-6) return;
  const double omega=covariance/variance,phase=mean_phase-omega*mean_time;
  if (!std::isfinite(omega) || std::abs(omega)>config_.max_abs_angular_velocity_rad_s) return;
  Eigen::Matrix2d information=Eigen::Matrix2d::Zero();
  Eigen::Vector2d rhs=Eigen::Vector2d::Zero();
  double z_weight[2]{},z_sum[2]{};
  for (const auto & sample : samples_) {
    const int group=sample.plate%2;
    const double angle=phase+omega*(sample.time-now)+sample.plate*quarter;
    const Eigen::Vector2d normal(std::cos(angle),std::sin(angle));
    // Phase uncertainty is propagated below with its center correlation.
    const Eigen::Matrix2d noise=sample.noise.topLeftCorner<2,2>();
    // The provisional radius is deliberately uncertain. An anisotropic ray
    // intersection with a fixed radius would move the center meters in depth
    // to explain centimeters of orbit mismatch. Use scalar quality weights
    // here and propagate the actual anisotropic noise of that weighted mean.
    const double position_weight=1/noise.trace();
    const Eigen::Matrix2d precision=Eigen::Matrix2d::Identity()*position_weight;
    information+=precision;
    rhs+=precision*(Eigen::Vector2d(sample.position[0],sample.position[1])-0.2*normal);
    const double wz=1/sample.noise(2,2);
    z_weight[group]+=wz; z_sum[group]+=wz*sample.position[2];
  }
  const Eigen::Vector2d center=information.ldlt().solve(rhs);
  if (!center.allFinite()) return;
  const double z0=z_weight[0]>0 ? z_sum[0]/z_weight[0] : z_sum[1]/z_weight[1];
  const double z1=z_weight[1]>0 ? z_sum[1]/z_weight[1] : z0;
  x_ << center[0],center[1],z0,wrap(phase),omega,0.2,0,z1-z0;
  // Joint batch information retains all center/radius/phase correlations and
  // lets observed point motion constrain speed even when its normal is noisy.
  // Two independent radius priors (r and r+dr); an unseen height remains broad.
  Covariance joint=Covariance::Identity()*1e-8;
  joint(5,5)+=50; joint(5,6)=joint(6,5)=25; joint(6,6)+=25; joint(7,7)+=25;
  for (const auto & sample : samples_) {
    const auto h=observation_jacobian(sample.time-now,sample.plate);
    joint+=h.transpose()*sample.noise.ldlt().solve(h);
  }
  p_=joint.ldlt().solve(Covariance::Identity());
  p_.diagonal().array()+=1e-8;
}

bool VehicleTarget::geometry_valid() const
{
  return x_.allFinite() && p_.allFinite() && x_[5] > 0.03 && x_[5] < 1.0 &&
    x_[5]+x_[6] > 0.03 && x_[5]+x_[6] < 1.0 &&
    std::abs(x_[4]) <= config_.max_abs_angular_velocity_rad_s;
}

void VehicleTarget::update(const TargetSelection & selection,
  std::chrono::steady_clock::time_point time, const cv::Matx33d & orientation,
  const cv::Vec3d & muzzle_world)
{
  if (time_ && (time <= *time_ || seconds(time,*time_) > config_.max_dt_seconds ||
      (seen_ && seconds(time,*seen_) > 1.5))) {
    const bool expired = seen_ && seconds(time,*seen_) > 1.5;
    reset(); diagnostics_.reset_reason = expired ? "history_expired" : "time_discontinuity";
  } else if (seen_ && !expired_ && seconds(time,*seen_) > config_.max_prediction_age_seconds) {
    // Output expires independently of the bounded geometry prior. New accepted
    // observations must reconfirm it; rejected observations never refresh seen_.
    expired_=true; updates_=0;
    if (!recovery_started_) recovery_started_=time;
    ++diagnostics_.generation; diagnostics_.reset_reason="observation_expired";
  }
  if (time_) predict(seconds(time,*time_));
  // A large predicted rotation is not ambiguity. An uncertain phase is. Keep
  // an established speed through short gaps, but cold-start an uncertain prior.
  if (seen_ && seconds(time,*seen_) > config_.max_prediction_age_seconds &&
      3*std::sqrt(std::max(0.0,p_(3,3))) >= quarter/2 &&
      std::any_of(selection.observations.begin(),selection.observations.end(),
        [](const auto & observation) { return observation.eligible; })) {
    reset(); diagnostics_.reset_reason="ambiguous_phase_gap";
  }
  time_ = time; current_ = false; diagnostics_.pose_ambiguous = false;
  const double now=epoch(time);
  while (!samples_.empty() && now-samples_.front().time > 1.5) samples_.pop_front();
  struct Candidate {
    TargetObservation observation; cv::Vec3d position, ray; double phase, score; int plate; bool ambiguous=false;
  };
  std::vector<Candidate> candidates;
  for (const auto & observation : selection.observations) {
    if (!observation.eligible) continue;
    if (observation_ && !observation_->armor.classification.label.empty() &&
        observation.armor.classification.label != observation_->armor.classification.label) continue;
    if (!seen_ && selection.selected() != &observation) continue;
    const auto poses = observation.pose_hypotheses.empty() ?
      std::vector<PoseSolution>{observation.pose} : observation.pose_hypotheses;
    std::vector<Candidate> hypotheses;
    for (const auto & hypothesis : poses) {
      if (!hypothesis.valid) continue;
      const auto & pose=hypothesis;
      cv::Matx33d board;
      cv::Rodrigues(pose.rotation_vector_camera,board);
      board = orientation*pose_.rotation_camera_to_gimbal*board;
      const double horizontal=std::hypot(board(0,2),board(1,2));
      if (horizontal < 0.5 || board(2,1) < 0.3) continue;
      const double tilt_error=std::abs(std::atan2(board(2,2),horizontal)-mounting_tilt_);
      if (tilt_error > 0.35) continue;
      const auto position=muzzle_world+orientation*pose.translation_gimbal_m;
      const auto ray=orientation*(pose.translation_gimbal_m-pose_.translation_camera_to_gimbal);
      const double phase=std::atan2(board(1,2),board(0,2));
      const auto noise=observation_noise(observation,pose,ray,phase,false,pose_.camera_matrix(0,0));
      double score=pose.reprojection_rms_px/pose_.max_reprojection_rms_px+tilt_error/0.35;
      int plate=0;
      if (seen_) {
        double best=std::numeric_limits<double>::infinity();
        double second=best;
        for (int id=0;id<4;++id) {
          const auto predicted=armor_position(0,id);
          const Eigen::Vector4d residual(position[0]-predicted[0],position[1]-predicted[1],
            position[2]-predicted[2],wrap(phase-x_[3]-id*quarter));
          const auto h=observation_jacobian(0,id);
          Eigen::Matrix4d innovation=h*p_*h.transpose()+noise;
          // Before a full fit, the center is conditional on a radius prior.
          // Its orbit mismatch must not reject the very views that identify it.
          if (!initialized_) innovation.topLeftCorner<3,3>().diagonal().array()+=0.04;
          const auto decomposition=innovation.ldlt();
          if (decomposition.info()!=Eigen::Success || !decomposition.isPositive()) continue;
          const double error=residual.dot(decomposition.solve(residual));
          if (error > 25 || std::abs(residual[3]) > 0.9 || cv::norm(position-predicted) > 1.5) continue;
          if (error < best) { second=best; best=error; plate=id; }
          else second=std::min(second,error);
        }
        if (!std::isfinite(best) || second-best < 1.0) continue;
        score += best;
      }
      auto associated=observation; associated.pose=pose;
      hypotheses.push_back({std::move(associated),position,ray,phase,score,plate});
    }
    std::stable_sort(hypotheses.begin(),hypotheses.end(),
      [](const auto & a,const auto & b) { return a.score < b.score; });
    if (hypotheses.empty()) continue;
    if (hypotheses.size()>1 && std::abs(wrap(hypotheses[1].phase-hypotheses[0].phase))>0.15) {
      diagnostics_.pose_ambiguous=true;
      if (hypotheses[1].score-hypotheses[0].score < 0.08) continue;
      hypotheses[0].ambiguous=true;
    }
    candidates.push_back(std::move(hypotheses.front()));
  }
  std::stable_sort(candidates.begin(),candidates.end(),
    [](const auto & a,const auto & b) { return a.score < b.score; });
  std::set<int> plates;
  for (const auto & candidate : candidates) {
    if (!plates.insert(candidate.plate).second) continue;
    const auto noise=observation_noise(candidate.observation,candidate.observation.pose,
      candidate.ray,candidate.phase,candidate.ambiguous,pose_.camera_matrix(0,0));
    if (!seen_) {
      x_ << candidate.position[0]-0.2*std::cos(candidate.phase),
        candidate.position[1]-0.2*std::sin(candidate.phase),candidate.position[2],candidate.phase,0,0.2,0,0;
      // c = measured_point - r*normal. A radius prior must move the center
      // with it, rather than inventing uncertainty in the observed plate.
      Eigen::Matrix<double,8,4> transform = Eigen::Matrix<double,8,4>::Zero();
      transform.topLeftCorner<3,3>().setIdentity(); transform(3,3)=1;
      transform(0,3)=0.2*std::sin(candidate.phase);
      transform(1,3)=-0.2*std::cos(candidate.phase);
      p_ = transform*noise*transform.transpose();
      State radius = State::Zero();
      radius[0]=-std::cos(candidate.phase); radius[1]=-std::sin(candidate.phase); radius[5]=1;
      p_ += 0.04*radius*radius.transpose();
      p_(4,4)=25; p_(6,6)=p_(7,7)=0.04;
      p_.diagonal().array() += 1e-8;
    } else if (initialized_ && !correct(candidate.position,candidate.phase,candidate.plate,noise)) {
      continue;
    }
    if (!geometry_valid()) { reset(); diagnostics_.reset_reason="invalid_geometry"; return; }
    const double reference=samples_.empty() ? x_[3] : samples_.back().phase+x_[4]*(now-samples_.back().time);
    const double phase=reference+wrap(candidate.phase-candidate.plate*quarter-reference);
    samples_.push_back({now,phase,candidate.position,candidate.plate,noise});
    if (!current_) {
      observation_=candidate.observation; diagnostics_.associated_plate=candidate.plate;
      diagnostics_.observed_position_world_m=candidate.position; diagnostics_.observed_phase_rad=candidate.phase;
    }
    current_=true; seen_=time; expired_=false;
  }
  while (samples_.size()>256) samples_.pop_front();
  if (current_) {
    ++updates_; misses_=0;
    if (!initialized_) fit_provisional(now);
    const bool fitted=fit(now);
    initialized_ = initialized_ || fitted;
  } else ++misses_;
  if (updates_>=config_.min_confirmed_updates) recovery_started_.reset();
  if (recovery_started_ && seconds(time,*recovery_started_)>0.5) {
    reset(); diagnostics_.reset_reason="reconfirmation_failed";
    return;
  }
  if (!current_ && seen_ && (seconds(time,*seen_) > 0.5 ||
      (!initialized_ && misses_ >= config_.max_consecutive_misses)) &&
      std::any_of(selection.observations.begin(),selection.observations.end(),
        [](const auto & observation) { return observation.eligible; })) {
    // Sustained disagreement (including an actual speed change) must eventually
    // release a confident old prior. Never refresh it from rejected observations.
    reset(); diagnostics_.reset_reason="persistent_inconsistency";
  }
}

double VehicleTarget::prediction_seconds(std::chrono::steady_clock::time_point time) const
{ return time_ ? seconds(time,*time_) : 0; }

VehicleTargetDiagnostics VehicleTarget::diagnostics(std::chrono::steady_clock::time_point time) const
{
  auto result=diagnostics_;
  result.center_world_m={x_[0],x_[1],x_[2]};
  result.phase_rad=wrap(x_[3]+x_[4]*prediction_seconds(time)); result.angular_velocity_rad_s=x_[4];
  result.radius_m=x_[5]; result.alternate_radius_m=x_[5]+x_[6]; result.alternate_height_m=x_[7];
  result.observation_age_s=seen_ ? seconds(time,*seen_) : 0;
  result.omega_std_rad_s=std::sqrt(std::max(0.0,p_(4,4)));
  result.initialized=initialized_;
  return result;
}

TargetTrackEstimate VehicleTarget::estimate(std::chrono::steady_clock::time_point time) const
{
  TargetTrackEstimate result;
  result.timestamp=time; result.confirmed_updates=updates_; result.consecutive_misses=misses_;
  if (!seen_ || time < *seen_ || seconds(time,*seen_) > config_.max_prediction_age_seconds) {
    result.status="follow_prediction_expired"; return result;
  }
  result.last_observation=observation_; result.has_last_measurement=true;
  result.has_current_measurement=current_;
  if (current_) result.current_observation=observation_;
  const bool ready=updates_>=config_.min_confirmed_updates;
  result.state=ready ? (current_ ? TargetTrackState::tracking : TargetTrackState::temporarily_lost) : TargetTrackState::detecting;
  result.prediction_valid=result.velocity_valid=ready;
  result.status=!ready ? "follow_collecting_geometry" : (current_ ? "follow_ekf_tracking" : "follow_ekf_coasting");
  if (observation_) {
    result.measured_yaw_rad=result.unwrapped_yaw_rad=observation_->pose.yaw_rad;
    result.measured_pitch_rad=observation_->pose.pitch_rad; result.measured_range_m=observation_->pose.range_m;
  }
  return result;
}

int VehicleTarget::choose_plate(double dt, const cv::Vec3d & muzzle, int previous) const
{
  // Until geometry is observed, only the associated plate is supported. This is
  // the same model and trajectory, not a separate static-target tracking mode.
  if (!initialized_) return diagnostics_.associated_plate;
  const double facing=std::atan2(muzzle[1]-x_[1],muzzle[0]-x_[0]);
  const auto error=[&](int plate) { return wrap(x_[3]+x_[4]*dt+plate*quarter-facing); };
  // Separate exit/entry gates at IMPACT time. Once outgoing, prefer the adjacent
  // incoming plate in the estimated rotation direction; no state is changed here.
  constexpr double exit_angle=0.95, enter_angle=0.75;
  if (previous>=0 && previous<4) {
    if (std::abs(error(previous))<=exit_angle) return previous;
    const int incoming=(previous+(x_[4]>=0 ? 3 : 1))%4;
    if (error(previous)*x_[4]>0 && std::abs(error(incoming))<enter_angle) return incoming;
  }
  int best=0;
  for (int plate=1;plate<4;++plate) if (std::abs(error(plate))<std::abs(error(best))) best=plate;
  return best;
}
}  // namespace srm::vision
