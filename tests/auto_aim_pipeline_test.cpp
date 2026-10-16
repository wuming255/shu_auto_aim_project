#include "control/auto_aim_pipeline.hpp"

#include <algorithm>
#include <array>
#include <cassert>
#include <chrono>
#include <cmath>
#include <vector>

namespace
{
using srm::control::AutoAimPipeline;
using srm::control::AutoAimPipelineConfig;
using srm::vision::ArmorObservation;
using srm::vision::ArmorPoseSolver;
using srm::vision::LightColor;
using srm::vision::PoseSolverConfig;
using srm::vision::TargetTrackState;

using TimePoint = std::chrono::steady_clock::time_point;

PoseSolverConfig make_pose_config()
{
  PoseSolverConfig config;
  config.camera_matrix = cv::Matx33d(800.0, 0.0, 320.0, 0.0, 800.0, 240.0, 0.0, 0.0, 1.0);
  config.distortion_coefficients = {0.0, 0.0, 0.0, 0.0, 0.0};
  config.max_reprojection_rms_px = 0.5;
  config.translation_camera_to_gimbal = cv::Vec3d(0.0, 0.0, 0.0);
  return config;
}

cv::Vec3d gimbal_position(double yaw, double pitch, double range)
{
  const double horizontal = range * std::cos(pitch);
  return {
    horizontal * std::cos(yaw), horizontal * std::sin(yaw), range * std::sin(pitch)};
}

ArmorObservation make_armor(
  const PoseSolverConfig & config, const cv::Vec3d & gimbal_position_m, std::size_t id)
{
  const auto object_points = ArmorPoseSolver::object_points();
  const std::vector<cv::Point3f> object_vector(object_points.begin(), object_points.end());
  const cv::Vec3d camera_translation =
    config.rotation_camera_to_gimbal.t() *
    (gimbal_position_m - config.translation_camera_to_gimbal);

  std::vector<cv::Point2f> projected;
  cv::projectPoints(
    object_vector, cv::Vec3d(0.0, 0.0, 0.0), camera_translation,
    cv::Mat(config.camera_matrix), cv::Mat(config.distortion_coefficients), projected);

  ArmorObservation armor;
  armor.color = LightColor::blue;
  armor.points = {projected[0], projected[1], projected[2], projected[3]};
  armor.center = (armor.points[0] + armor.points[1] + armor.points[2] + armor.points[3]) / 4.0F;
  armor.roi = cv::Rect(
    static_cast<int>(std::floor(armor.center.x)), static_cast<int>(std::floor(armor.center.y)), 1, 1);
  armor.left.id = id * 2;
  armor.right.id = id * 2 + 1;
  return armor;
}

srm::vision::DetectionResult make_detection(
  const PoseSolverConfig & config, double target_yaw, double distractor_yaw,
  bool include_distractor)
{
  srm::vision::DetectionResult detections;
  detections.armors.push_back(make_armor(config, gimbal_position(target_yaw, 0.0, 5.0), 1));
  if (include_distractor) {
    detections.armors.push_back(
      make_armor(config, gimbal_position(distractor_yaw, 0.0, 5.0), 2));
  }
  return detections;
}

AutoAimPipelineConfig make_pipeline_config()
{
  AutoAimPipelineConfig config;
  config.pose_solver = make_pose_config();
  config.tracker.min_confirmed_updates = 3;
  config.tracker.max_consecutive_misses = 2;
  config.tracker.min_dt_seconds = 1e-6;
  config.tracker.max_dt_seconds = 0.2;
  config.tracker.prediction_horizon_seconds = 0.02;
  config.tracker.max_prediction_age_seconds = 0.25;
  config.tracker.velocity_smoothing_factor = 1.0;
  config.tracker.max_abs_angular_velocity_rad_s = 10.0;
  config.tracker.use_prediction_association = true;
  config.tracker.max_association_angular_error_rad = 0.2;
  config.tracker.max_association_range_error_m = 0.5;
  config.aim_compensator.gravity_m_s2 = 9.80665;
  config.aim_compensator.min_bullet_speed_m_s = 1.0;
  config.aim_compensator.max_bullet_speed_m_s = 100.0;
  config.aim_compensator.max_target_range_m = 20.0;
  config.aim_compensator.max_flight_time_s = 2.0;
  config.aim_compensator.max_iterations = 6;
  return config;
}

void test_complete_synthetic_sequence()
{
  const auto config = make_pipeline_config();
  AutoAimPipeline pipeline(config);
  const TimePoint t0{};

  const auto first = pipeline.process(
    make_detection(config.pose_solver, 0.0, 0.0, false), t0, 0, 20.0);
  assert(first.frame_sequence == 0);
  assert(first.selection.has_target());
  assert(first.tracking.state == TargetTrackState::detecting);
  assert(!first.aim.valid);

  const auto second = pipeline.process(
    make_detection(config.pose_solver, 0.05, 0.4, true),
    t0 + std::chrono::milliseconds(100), 1, 20.0);
  assert(second.selection.observations.size() == 2);
  assert(second.tracking.state == TargetTrackState::detecting);
  assert(!second.aim.valid);

  // 单帧选择器会优先看到接近视轴的干扰候选；已有速度后，跟踪器应关联到连续目标。
  const auto third = pipeline.process(
    make_detection(config.pose_solver, 0.10, 0.0, true),
    t0 + std::chrono::milliseconds(200), 2, 20.0);
  assert(third.selection.has_target());
  assert(third.tracking.state == TargetTrackState::tracking);
  assert(third.tracking.has_current_measurement);
  assert(third.tracking.last_observation.has_value());
  assert(third.tracking.last_observation->detection_index == 0);
  assert(third.aim.valid);
  assert(third.aim.flight_time_s > 0.0);
  assert(third.aim.pitch_rad > third.aim.line_of_sight_pitch_rad);

  const auto fourth = pipeline.process(
    make_detection(config.pose_solver, 0.15, 0.0, true),
    t0 + std::chrono::milliseconds(300), 3, 20.0);
  assert(fourth.tracking.state == TargetTrackState::tracking);
  assert(fourth.aim.valid);
  assert(fourth.aim.yaw_rad > third.aim.yaw_rad);

  const srm::vision::DetectionResult empty;
  const auto missed = pipeline.process(
    empty, t0 + std::chrono::milliseconds(400), 4, 20.0);
  assert(missed.tracking.state == TargetTrackState::temporarily_lost);
  assert(!missed.aim.valid);

  const auto recovered = pipeline.process(
    make_detection(config.pose_solver, 0.25, 0.0, true),
    t0 + std::chrono::milliseconds(500), 5, 20.0);
  assert(recovered.tracking.state == TargetTrackState::tracking);
  assert(recovered.tracking.has_current_measurement);
  assert(recovered.aim.valid);
}

void test_bad_bullet_speed_does_not_break_tracking()
{
  const auto config = make_pipeline_config();
  AutoAimPipeline pipeline(config);
  const TimePoint t0{};

  (void)pipeline.process(
    make_detection(config.pose_solver, 0.0, 0.0, false), t0, 0, 20.0);
  (void)pipeline.process(
    make_detection(config.pose_solver, 0.01, 0.0, false),
    t0 + std::chrono::milliseconds(100), 1, 20.0);
  const auto result = pipeline.process(
    make_detection(config.pose_solver, 0.02, 0.0, false),
    t0 + std::chrono::milliseconds(200), 2, 0.0);

  assert(result.tracking.state == TargetTrackState::tracking);
  assert(!result.aim.valid);
  assert(!result.aim.error.empty());
}

void test_reset_clears_cross_frame_state()
{
  const auto config = make_pipeline_config();
  AutoAimPipeline pipeline(config);
  const TimePoint t0{};

  (void)pipeline.process(
    make_detection(config.pose_solver, 0.0, 0.0, false), t0, 0, 20.0);
  (void)pipeline.process(
    make_detection(config.pose_solver, 0.05, 0.0, false),
    t0 + std::chrono::milliseconds(100), 1, 20.0);
  pipeline.reset();

  const auto result = pipeline.process(
    make_detection(config.pose_solver, 0.10, 0.0, false),
    t0 + std::chrono::milliseconds(200), 2, 20.0);
  assert(result.tracking.state == TargetTrackState::detecting);
  assert(result.tracking.confirmed_updates == 1);
  assert(!result.aim.valid);
}

void test_rotation_pipeline_uses_current_association_and_resets()
{
  auto config = make_pipeline_config();
  config.pose_solver.camera_matrix = {1800,0,720, 0,1800,540, 0,0,1};
  config.pose_solver.feature_size_m = cv::Size2d(0.135,0.055);
  config.rotation.enabled = true;
  config.rotation.assume_static_camera = true;
  config.rotation.mounting_tilt_rad = 0.25;
  config.rotation.max_angular_speed_rad_s = 10;
  AutoAimPipeline pipeline(config);
  srm::control::AutoAimFrameResult result;
  for (int frame = 0; frame < 35; ++frame) {
    srm::vision::DetectionResult detections;
    const auto points = ArmorPoseSolver::object_points(0.135,0.055);
    for (int plate = 0; plate < 4; ++plate) {
      const double theta = 2.8+3.4*frame*0.04+plate*CV_PI/2;
      const double c = std::cos(theta), s = std::sin(theta);
      if (c > -0.25) continue;
      const double ct = std::cos(0.25), st = std::sin(0.25);
      const cv::Matx33d world_rotation(-s,-c*st,c*ct, c,-s*st,s*ct, 0,ct,st);
      cv::Vec3d rvec;
      cv::Rodrigues(config.pose_solver.rotation_camera_to_gimbal.t()*world_rotation,rvec);
      const cv::Vec3d position = config.pose_solver.rotation_camera_to_gimbal.t()*
        cv::Vec3d(5+0.28*c,0.2+0.28*s,0.6);
      std::vector<cv::Point2f> pixels;
      cv::projectPoints(std::vector<cv::Point3f>(points.begin(),points.end()),rvec,position,
        config.pose_solver.camera_matrix,config.pose_solver.distortion_coefficients,pixels);
      ArmorObservation armor;
      std::copy(pixels.begin(),pixels.end(),armor.points.begin());
      detections.armors.push_back(armor);
    }
    result = pipeline.process(detections,TimePoint{}+std::chrono::milliseconds(frame*40),frame,20);
  }
  assert(result.rotation.reliable && result.tracking.prediction_valid);
  assert(result.tracking.state == TargetTrackState::tracking);
  assert(result.tracking.current_observation && result.rotation.current_observation);
  assert(result.tracking.current_observation->detection_index == result.rotation.current_observation->detection_index);
  assert(result.aim.valid);
  assert(result.guard_command_valid);
  const auto lost = pipeline.process({},TimePoint{}+std::chrono::milliseconds(1400),35,20);
  assert(lost.tracking.state == TargetTrackState::temporarily_lost);
  assert(!lost.tracking.has_current_measurement && !lost.tracking.current_observation);
  assert(lost.tracking.last_observation && lost.tracking.prediction_valid);
  assert(lost.aim.valid && lost.guard_command_valid);
  assert(!lost.guard_target_visible && !lost.fire_window_valid);
  pipeline.reset();
  const auto reset = pipeline.process({},TimePoint{}+std::chrono::milliseconds(1440),36,20);
  assert(reset.tracking.state == TargetTrackState::lost);
  assert(!reset.tracking.last_observation && !reset.tracking.prediction_valid);
}

}  // namespace

int main()
{
  test_complete_synthetic_sequence();
  test_bad_bullet_speed_does_not_break_tracking();
  test_reset_clears_cross_frame_state();
  test_rotation_pipeline_uses_current_association_and_resets();
  return 0;
}
