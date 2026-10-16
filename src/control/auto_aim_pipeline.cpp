#include "control/auto_aim_pipeline.hpp"

#include <utility>
#include <cmath>

namespace srm::control
{

AutoAimPipeline::AutoAimPipeline(AutoAimPipelineConfig config)
: config_(std::move(config)),
  pose_solver_(config_.pose_solver),
  target_selector_(config_.target_selector),
  target_tracker_(config_.tracker),
  aim_compensator_(config_.aim_compensator),
  rotating_target_(config_.rotation,config_.pose_solver)
{
}

AutoAimFrameResult AutoAimPipeline::process(
  const vision::DetectionResult & detections,
  std::chrono::steady_clock::time_point timestamp,
  std::uint64_t frame_sequence, double bullet_speed_m_s,
  std::optional<cv::Matx33d> rotation_gimbal_to_world)
{
  AutoAimFrameResult result;
  result.frame_sequence = frame_sequence;
  result.timestamp = timestamp;
  result.selection = target_selector_.select(
    detections, pose_solver_, timestamp, frame_sequence);
  if (config_.rotation.enabled) {
    result.rotation = rotating_target_.update(result.selection,timestamp,rotation_gimbal_to_world);
    const auto & rotation = result.rotation;
    auto & track = result.tracking;
    track.timestamp = timestamp;
    track.current_observation = rotation.current_observation;
    track.has_current_measurement = rotation.current_visible;
    if (rotation.current_observation) rotation_last_observation_ = rotation.current_observation;
    if (!rotation.initialized && rotation.confirmed_updates == 0) rotation_last_observation_.reset();
    track.last_observation = rotation_last_observation_;
    track.has_last_measurement = rotation_last_observation_.has_value();
    track.confirmed_updates = rotation.confirmed_updates;
    track.prediction_valid = rotation.prediction_valid;
    track.velocity_valid = rotation.reliable;
    track.state = rotation.current_visible ? (rotation.reliable ? vision::TargetTrackState::tracking :
      vision::TargetTrackState::detecting) : (rotation.initialized ? vision::TargetTrackState::temporarily_lost :
      vision::TargetTrackState::lost);
    track.status = rotation.status;
    if (rotation.current_observation) {
      const auto & pose = rotation.current_observation->pose;
      track.measured_yaw_rad = pose.yaw_rad; track.unwrapped_yaw_rad = pose.yaw_rad;
      track.measured_pitch_rad = pose.pitch_rad; track.measured_range_m = pose.range_m;
    }
    if (rotation.initialized) {
      const auto position = rotation.rotation_gimbal_to_world.t()*rotation.armor_position(0,rotation.associated_plate);
      const auto radial = rotation.armor_position(0,rotation.associated_plate)-rotation.center_world_m;
      const auto velocity = rotation.rotation_gimbal_to_world.t()*cv::Vec3d(
        -rotation.angular_velocity_rad_s*radial[1],rotation.angular_velocity_rad_s*radial[0],0);
      const double horizontal = std::hypot(position[0],position[1]), range = cv::norm(position);
      track.predicted_yaw_rad = std::atan2(position[1],position[0]);
      track.predicted_pitch_rad = std::atan2(position[2],horizontal); track.predicted_range_m = range;
      if (horizontal > 1e-6 && range > 1e-6) {
        track.yaw_velocity_rad_s = (position[0]*velocity[1]-position[1]*velocity[0])/(horizontal*horizontal);
        track.pitch_velocity_rad_s = (horizontal*velocity[2]-position[2]*
          (position[0]*velocity[0]+position[1]*velocity[1])/horizontal)/(range*range);
        track.range_velocity_m_s = position.dot(velocity)/range;
      }
    }
  } else {
    result.rotation.status = "disabled";
    result.tracking = target_tracker_.update(result.selection, timestamp);
  }
  result.aim = aim_compensator_.solve(result.tracking, bullet_speed_m_s);
  if (config_.rotation.enabled) {
    // 四甲不能套用旧视线匀速外推；固定守点/到点弹道由下一层专门求解。
    result.aim.valid = false;
    result.aim.error = "fixed_guard_not_configured";
  }
  return result;
}

void AutoAimPipeline::reset() noexcept
{
  target_tracker_.reset();
  rotating_target_.reset();
  rotation_last_observation_.reset();
}

const AutoAimPipelineConfig & AutoAimPipeline::config() const noexcept
{
  return config_;
}

const vision::ArmorPoseSolver & AutoAimPipeline::pose_solver() const noexcept
{
  return pose_solver_;
}

const vision::TargetSelector & AutoAimPipeline::target_selector() const noexcept
{
  return target_selector_;
}

const vision::TargetTracker & AutoAimPipeline::target_tracker() const noexcept
{
  return target_tracker_;
}

const AimCompensator & AutoAimPipeline::aim_compensator() const noexcept
{
  return aim_compensator_;
}

}  // namespace srm::control
