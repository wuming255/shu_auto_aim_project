#include "vision/rotating_target.hpp"
#include <cassert>
#include <cmath>
#include <iostream>

namespace
{
using namespace srm::vision;
constexpr double pi = 3.14159265358979323846;
using Clock = std::chrono::steady_clock;

TargetSelection observe(const PoseSolverConfig & pose, double time, const cv::Matx33d & g2w,
  double pixel_noise = 0)
{
  TargetSelection selection;
  const cv::Vec3d center(5.0,0.2,0.6);
  const auto points = ArmorPoseSolver::object_points(pose.feature_size_m->width, pose.feature_size_m->height);
  ArmorPoseSolver solver(pose);
  for (int plate = 0; plate < 4; ++plate) {
    const double theta = 2.8 + 3.4*time + plate*pi/2, tilt = 0.25;
    const double c = std::cos(theta), s = std::sin(theta), ct = std::cos(tilt), st = std::sin(tilt);
    const cv::Vec3d position = center + cv::Vec3d(0.28*c,0.28*s,0);
    if (c > -0.25) continue;  // 只提供朝向相机的板，切换发生在同一轨道中。
    const cv::Matx33d world_from_object(-s,-c*st,c*ct, c,-s*st,s*ct, 0,ct,st);
    const cv::Matx33d camera_rotation = pose.rotation_camera_to_gimbal.t()*g2w.t()*world_from_object;
    cv::Vec3d rvec;
    cv::Rodrigues(camera_rotation,rvec);
    const cv::Vec3d tvec = pose.rotation_camera_to_gimbal.t()*
      (g2w.t()*position-pose.translation_camera_to_gimbal);
    std::vector<cv::Point2f> pixels;
    cv::projectPoints(std::vector<cv::Point3f>(points.begin(),points.end()),rvec,tvec,
      pose.camera_matrix,pose.distortion_coefficients,pixels);
    for (std::size_t i = 0; i < pixels.size(); ++i) {
      pixels[i].x += pixel_noise*std::sin(7*time+i+plate);
      pixels[i].y += pixel_noise*std::cos(11*time+2*i+plate);
    }
    TargetObservation observation;
    observation.eligible = true;
    observation.detection_index = static_cast<std::size_t>(plate);
    std::copy(pixels.begin(),pixels.end(),observation.armor.points.begin());
    observation.pose_hypotheses = solver.solve_candidates(observation.armor.points);
    assert(!observation.pose_hypotheses.empty());
    observation.pose = observation.pose_hypotheses.front();
    selection.observations.push_back(observation);
  }
  if (!selection.observations.empty()) selection.selected_index = 0;
  return selection;
}

void test_four_plates_camera_motion_and_loss()
{
  PoseSolverConfig pose;
  pose.camera_matrix = {1800,0,720, 0,1800,540, 0,0,1};
  pose.feature_size_m = cv::Size2d(0.135,0.055); // 明确为合成尺寸，非现场测量。
  RotatingTargetConfig config;
  config.enabled = true;
  config.mounting_tilt_rad = 0.25;
  config.max_angular_speed_rad_s = 10.0; // 25 Hz合成输入满足四甲相位采样条件。
  RotatingTarget tracker(config,pose);
  RotationEstimate estimate;
  int switches = 0, previous = -1;
  double previous_phase = 0;
  bool previous_reliable = false;
  for (int frame = 0; frame < 100; ++frame) {
    const double time = frame*0.04;
    const auto orientation = gimbal_to_world(0.2*std::sin(time),0.08*std::cos(time),0.03);
    const bool occluded = frame >= 35 && frame <= 37;
    estimate = tracker.update(occluded ? TargetSelection{} : observe(pose,time,orientation),
      Clock::time_point{}+std::chrono::milliseconds(frame*40),orientation);
    if (frame == 34 || frame == 99) {
      std::cerr << "frame=" << frame << " status=" << estimate.status << " radius=" << estimate.radius_m
        << " omega=" << estimate.angular_velocity_rad_s << " rms=" << estimate.position_rms_m << '\n';
      assert(estimate.reliable);
      assert(cv::norm(estimate.center_world_m-cv::Vec3d(5,0.2,0.6)) < 0.01);
      assert(std::abs(estimate.radius_m-0.28) < 0.01);
      assert(std::abs(estimate.angular_velocity_rad_s-3.4) < 0.03);
    }
    if (occluded) {
      assert(!estimate.current_visible && !estimate.reliable);
      assert(estimate.prediction_valid);
    }
    if (frame > 15 && !occluded && estimate.current_observation) {
      const auto id = static_cast<int>(estimate.current_observation->detection_index);
      if (previous >= 0 && id != previous) ++switches;
      previous = id;
      if (estimate.reliable && previous_reliable) {
        assert(std::abs(estimate.phase_rad-previous_phase) < 0.2);
      }
    }
    previous_phase = estimate.phase_rad;
    previous_reliable = estimate.reliable;
  }
  assert(switches >= 3);
  const auto lost = tracker.update({},Clock::time_point{}+std::chrono::seconds(5),cv::Matx33d::eye());
  assert(!lost.initialized && !lost.prediction_valid && !lost.reliable);
  const auto reacquired = tracker.update(observe(pose,5.04,cv::Matx33d::eye()),
    Clock::time_point{}+std::chrono::milliseconds(5040),cv::Matx33d::eye());
  assert(!reacquired.reliable);
  assert(reacquired.confirmed_updates == 1);
  tracker.reset();
  const auto unbound = tracker.update(observe(pose,0,cv::Matx33d::eye()),Clock::time_point{});
  assert(!unbound.stable_frame_valid && !unbound.reliable);
}

void test_loss_cannot_promote_an_unconfirmed_fit()
{
  PoseSolverConfig pose;
  pose.camera_matrix = {1800,0,720, 0,1800,540, 0,0,1};
  pose.feature_size_m = cv::Size2d(0.135,0.055);
  RotatingTargetConfig config;
  config.assume_static_camera = true;
  config.mounting_tilt_rad = 0.25;
  config.max_angular_speed_rad_s = 10;
  RotatingTarget tracker(config,pose);
  bool exercised = false;
  for (int frame = 0; frame < 8; ++frame) {
    const auto time = Clock::time_point{}+std::chrono::milliseconds(frame*40);
    const auto estimate = tracker.update(observe(pose,frame*0.04,cv::Matx33d::eye()),time);
    if (estimate.initialized && !estimate.reliable) {
      const auto lost = tracker.update({},time+std::chrono::milliseconds(40));
      assert(!lost.prediction_valid);
      exercised = true;
      break;
    }
  }
  assert(exercised);
}

void test_unknown_tilt_stays_diagnostic()
{
  PoseSolverConfig pose;
  pose.camera_matrix = {1800,0,720, 0,1800,540, 0,0,1};
  pose.feature_size_m = cv::Size2d(0.135,0.055);
  RotatingTargetConfig config;
  config.assume_static_camera = true;
  config.max_angular_speed_rad_s = 10;
  RotatingTarget tracker(config,pose);
  RotationEstimate estimate;
  for (int frame = 0; frame < 35; ++frame) {
    estimate = tracker.update(observe(pose,frame*0.04,cv::Matx33d::eye()),
      Clock::time_point{}+std::chrono::milliseconds(frame*40));
  }
  assert(estimate.initialized);
  assert(!estimate.reliable && !estimate.prediction_valid);
  assert(estimate.status == "mounting_tilt_unconfirmed");
  const auto lost = tracker.update({},Clock::time_point{}+std::chrono::milliseconds(1400));
  assert(!lost.prediction_valid);
}

void test_all_observations_and_rejected_measurements()
{
  PoseSolverConfig pose;
  pose.camera_matrix = {1800,0,720, 0,1800,540, 0,0,1};
  pose.feature_size_m = cv::Size2d(0.135,0.055);
  RotatingTargetConfig config;
  config.assume_static_camera = true;
  config.mounting_tilt_rad = 0.25;
  config.max_angular_speed_rad_s = 10;
  config.min_updates = 3;
  RotatingTarget all(config,pose), selected_only(config,pose);
  RotationEstimate combined, single;
  for (int frame = 0; frame < 2; ++frame) {
    auto selection = observe(pose,frame*0.04,cv::Matx33d::eye());
    combined = all.update(selection,Clock::time_point{}+std::chrono::milliseconds(frame*40));
    selection.observations.resize(1);
    single = selected_only.update(selection,Clock::time_point{}+std::chrono::milliseconds(frame*40));
  }
  assert(combined.initialized && !single.initialized);
  for (int frame = 2; frame <= 34; ++frame) {
    combined = all.update(observe(pose,frame*0.04,cv::Matx33d::eye()),
      Clock::time_point{}+std::chrono::milliseconds(frame*40));
  }
  assert(combined.reliable);
  auto outliers = observe(pose,1.40,cv::Matx33d::eye());
  for (auto & observation : outliers.observations) {
    observation.pose.translation_gimbal_m[0] += 1;
    for (auto & hypothesis : observation.pose_hypotheses) hypothesis.translation_gimbal_m[0] += 1;
  }
  const auto rejected = all.update(outliers,Clock::time_point{}+std::chrono::milliseconds(1400));
  assert(!rejected.current_observation && !rejected.current_visible && rejected.prediction_valid);
  const auto recovered = all.update(observe(pose,1.44,cv::Matx33d::eye()),
    Clock::time_point{}+std::chrono::milliseconds(1440));
  // 80 ms * 10 rad/s 已超过四甲的 pi/4 展相位界限，重捕获必须重新收敛。
  assert(recovered.current_observation && recovered.confirmed_updates == 1);
  assert(!recovered.reliable && !recovered.prediction_valid);
  const auto invalid = all.update({},Clock::time_point{}+std::chrono::milliseconds(1640));
  assert(!invalid.current_observation && !invalid.prediction_valid);
  const auto long_loss = all.update({},Clock::time_point{}+std::chrono::milliseconds(2000));
  assert(!long_loss.initialized && !long_loss.reliable);
  all.update(observe(pose,2.04,cv::Matx33d::eye()),Clock::time_point{}+std::chrono::milliseconds(2040));
  const auto backwards = all.update(observe(pose,2.0,cv::Matx33d::eye()),
    Clock::time_point{}+std::chrono::milliseconds(2000));
  assert(backwards.confirmed_updates == 1 && !backwards.prediction_valid);
  const auto bad_rotation = all.update({},Clock::time_point{}+std::chrono::milliseconds(2080),
    cv::Matx33d::zeros());
  assert(!bad_rotation.initialized && bad_rotation.status == "invalid_reference_rotation");
}

void test_noisy_reversal_reacquires()
{
  PoseSolverConfig pose;
  pose.camera_matrix = {1800,0,720, 0,1800,540, 0,0,1};
  pose.feature_size_m = cv::Size2d(0.135,0.055);
  RotatingTargetConfig config;
  config.assume_static_camera = true;
  config.mounting_tilt_rad = 0.25;
  config.max_angular_speed_rad_s = 10;
  RotatingTarget tracker(config,pose);
  RotationEstimate estimate;
  bool invalidated = false;
  for (int frame = 0; frame < 160; ++frame) {
    const double time = frame*0.04;
    estimate = tracker.update(observe(pose,time <= 2 ? time : 4-time,cv::Matx33d::eye(),0.02),
      Clock::time_point{}+std::chrono::milliseconds(frame*40));
    if (time > 2 && time < 3.5 && !estimate.reliable) invalidated = true;
  }
  assert(invalidated && estimate.reliable);
  assert(std::abs(estimate.angular_velocity_rad_s+3.4) < 0.1);
  assert(std::abs(estimate.radius_m-0.28) < 0.02);
}
}
int main()
{
  test_four_plates_camera_motion_and_loss();
  test_loss_cannot_promote_an_unconfirmed_fit();
  test_unknown_tilt_stays_diagnostic();
  test_all_observations_and_rejected_measurements();
  test_noisy_reversal_reacquires();
}
