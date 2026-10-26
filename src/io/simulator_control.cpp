#include "io/simulator_control.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace srm::io
{
namespace
{
constexpr double to_rad = 3.14159265358979323846 / 180.0;
const cv::Matx33d optical_to_flu(0, 0, 1, -1, 0, 0, 0, -1, 0);

std::int64_t stamp_ns(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<std::int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
}

bool fresh(std::int64_t now, std::int64_t stamp, double max_age)
{
  return stamp > 0 && now >= stamp && (now - stamp) * 1e-9 <= max_age;
}

cv::Vec3d vector(const geometry_msgs::msg::Vector3 & value)
{
  const cv::Vec3d result(value.x, value.y, value.z);
  if (!cv::checkRange(cv::Mat(result))) throw std::invalid_argument("nonfinite TF translation");
  return result;
}

cv::Matx33d rotation(const geometry_msgs::msg::Quaternion & q)
{
  const double norm = std::sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
  if (!std::isfinite(norm) || std::abs(norm - 1.0) > 0.01) {
    throw std::invalid_argument("invalid TF rotation");
  }
  const double x=q.x/norm, y=q.y/norm, z=q.z/norm, w=q.w/norm;
  return {1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w),
          2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w),
          2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)};
}

cv::Matx33d raw_orientation(const rm_interfaces::msg::VisionReceiveData & feedback)
{
  // Bevy YXZ -> FLU ZYX: Bevy +Z roll maps to FLU -X.
  return vision::gimbal_to_world(feedback.yaw*to_rad, -feedback.pitch*to_rad, -feedback.roll*to_rad);
}

protocol::GimbalState angles(const cv::Matx33d & orientation)
{
  protocol::GimbalState state;
  state.yaw = static_cast<float>(std::atan2(orientation(1,0), orientation(0,0))/to_rad);
  state.pitch = static_cast<float>(-std::asin(std::clamp(orientation(2,0), -1.0, 1.0))/to_rad);
  state.roll = static_cast<float>(std::atan2(orientation(2,1), orientation(2,2))/to_rad);
  return state;
}

control::SafetyControllerConfig safety_config(const SimulatorControlConfig & config)
{
  if (!std::isfinite(config.frame_max_age_sec) || config.frame_max_age_sec < 0.001 ||
      config.frame_max_age_sec > 5 || !std::isfinite(config.feedback_max_age_sec) ||
      config.feedback_max_age_sec < 0.001 || config.feedback_max_age_sec > 5) {
    throw std::invalid_argument("simulator freshness limits must be in [0.001, 5] seconds");
  }
  control::SafetyControllerConfig result;
  result.max_frame_age = std::chrono::milliseconds(static_cast<int>(config.frame_max_age_sec*1000));
  result.max_transport_state_age = std::chrono::milliseconds(static_cast<int>(config.feedback_max_age_sec*1000));
  result.allow_visual_fire = config.allow_visual_fire;
  result.require_gimbal_feedback_for_fire = true;
  result.max_gimbal_error_deg = 0.7;
  return result;
}
}  // namespace

SimulatorControl::SimulatorControl(SimulatorControlConfig config)
: config_(std::move(config)), safety_(safety_config(config_))
{
  for (const double value : {config_.frame_max_age_sec, config_.feedback_max_age_sec,
       config_.fire_period_sec, config_.pitch_limit_deg, config_.small_feature_width_m,
       config_.large_feature_width_m, config_.feature_height_m}) {
    if (!std::isfinite(value) || value <= 0) throw std::invalid_argument("invalid simulator control parameter");
  }
  if ((config_.aim_mode != "follow" && config_.aim_mode != "guard") ||
      config_.frame_max_age_sec > 5 || config_.feedback_max_age_sec > 5 ||
      config_.fire_period_sec < 0.12 || config_.pitch_limit_deg >= 90 ||
      !std::isfinite(config_.additional_latency_sec) || config_.additional_latency_sec < 0 ||
      !std::isfinite(config_.mounting_tilt_rad)) {
    throw std::invalid_argument("invalid simulator aim mode, age, latency, pitch or fire period");
  }
}

void SimulatorControl::reset_pipelines()
{
  for (auto & pipeline : pipelines_) pipeline.reset();
  invalidate_frame();
  selected_pose_.reset();
}

void SimulatorControl::set_camera(const vision::CameraCalibration & camera)
{
  if (!camera_ || cv::norm(cv::Mat(camera.camera_matrix-camera_->camera_matrix)) > 1e-8 ||
      camera.distortion_coefficients != camera_->distortion_coefficients ||
      camera.geometry.size != camera_->geometry.size) reset_pipelines();
  camera_ = camera;
}

void SimulatorControl::clear_camera()
{
  camera_.reset();
  reset_pipelines();
}

void SimulatorControl::receive_feedback(const rm_interfaces::msg::VisionReceiveData & message)
{
  if (stamp_ns(message.header.stamp) <= 0 || !std::isfinite(message.yaw) ||
      !std::isfinite(message.pitch) || !std::isfinite(message.roll)) {
    ++rejected_feedback_;
    feedback_.clear();
    invalidate_frame();
    return;
  }
  if (!feedback_.empty() && stamp_ns(message.header.stamp) <= stamp_ns(feedback_.back().header.stamp)) {
    if (stamp_ns(message.header.stamp) == stamp_ns(feedback_.back().header.stamp)) return;
    feedback_.clear();
    geometry_.clear();
    reset_pipelines();
  }
  feedback_.push_back(message);
  if (feedback_.size() > 256) feedback_.pop_front();
}

void SimulatorControl::receive_transforms(const tf2_msgs::msg::TFMessage & message)
{
  Geometry geometry;
  unsigned found = 0;
  try {
    for (const auto & item : message.transforms) {
      if (item.header.frame_id == "odom" && item.child_frame_id == "gimbal_link") {
        geometry.stamp_ns = stamp_ns(item.header.stamp);
        geometry.barrel_to_world = rotation(item.transform.rotation);
        found |= 1;
      } else if (item.header.frame_id == "gimbal_link" && item.child_frame_id == "camera_link") {
        geometry.camera_local = vector(item.transform.translation);
        found |= 2;
      } else if (item.header.frame_id == "gimbal_link" && item.child_frame_id == "muzzle") {
        geometry.muzzle_local = vector(item.transform.translation);
        found |= 4;
      } else if (item.header.frame_id == "map" && item.child_frame_id == "odom") {
        geometry.origin_world = vector(item.transform.translation);
        found |= 8;
      }
    }
    if (found != 15 || geometry.stamp_ns <= 0 ||
        cv::norm(geometry.camera_local) > 1.0 || cv::norm(geometry.muzzle_local) > 1.0) {
      throw std::invalid_argument("missing simulator mount TF or camera outside robot view");
    }
  } catch (const std::exception &) {
    ++rejected_transforms_;
    geometry_.clear();
    invalidate_frame();
    return;
  }
  if (!geometry_.empty() && geometry.stamp_ns <= geometry_.back().stamp_ns) {
    if (geometry.stamp_ns == geometry_.back().stamp_ns) return;
    geometry_.clear();
    reset_pipelines();
  }
  geometry_.push_back(geometry);
  if (geometry_.size() > 256) geometry_.pop_front();
}

std::optional<rm_interfaces::msg::VisionReceiveData> SimulatorControl::feedback_at(std::int64_t time) const
{
  for (auto iterator = feedback_.rbegin(); iterator != feedback_.rend(); ++iterator) {
    if (stamp_ns(iterator->header.stamp) <= time) {
      if (fresh(time, stamp_ns(iterator->header.stamp), config_.feedback_max_age_sec)) return *iterator;
      break;
    }
  }
  return std::nullopt;
}

void SimulatorControl::prepare_pipelines(
  const Geometry & geometry, const rm_interfaces::msg::VisionReceiveData & feedback)
{
  const cv::Matx33d mount = raw_orientation(feedback).t()*geometry.barrel_to_world;
  // The current Bevy publisher aligns gimbal_link to the barrel, but publishes local
  // camera/muzzle translations in the raw gimbal basis. Correct that mixed-basis TF here.
  const cv::Vec3d offset = mount.t()*(geometry.camera_local-geometry.muzzle_local);
  if (!mount_ || cv::norm(cv::Mat(mount-*mount_)) > 0.005 || !camera_from_muzzle_ ||
      cv::norm(offset-*camera_from_muzzle_) > 0.002) reset_pipelines();
  if (config_.aim_mode == "guard" && guard_origin_ && cv::norm(geometry.origin_world-*guard_origin_) > 0.02) {
    reset_pipelines();
    guard_origin_ = geometry.origin_world;
  }
  if (!guard_origin_) guard_origin_ = geometry.origin_world;
  mount_ = mount;
  camera_from_muzzle_ = offset;
  for (std::size_t type = 0; type < pipelines_.size(); ++type) {
    if (pipelines_[type]) continue;
    control::AutoAimPipelineConfig config;
    config.pose_solver.camera_matrix = camera_->camera_matrix;
    config.pose_solver.distortion_coefficients = camera_->distortion_coefficients;
    config.pose_solver.rotation_camera_to_gimbal = optical_to_flu;
    config.pose_solver.translation_camera_to_gimbal = offset;
    config.pose_solver.feature_size_m = cv::Size2d(type ? config_.large_feature_width_m :
      config_.small_feature_width_m, config_.feature_height_m);
    config.follow_target = config_.aim_mode == "follow";
    config.rotation.enabled = config_.aim_mode == "guard";
    config.rotation.feedback_frame_confirmed = true;
    config.rotation.mounting_tilt_rad = config_.mounting_tilt_rad;
    config.aim_compensator.additional_latency_seconds = config_.additional_latency_sec;
    pipelines_[type] = std::make_unique<control::AutoAimPipeline>(config);
  }
}

void SimulatorControl::invalidate_frame()
{
  result_.reset();
  pulse_until_ns_ = 0;
}

void SimulatorControl::process(
  const Frame & frame, const vision::DetectionResult & detection, std::int64_t source_stamp_ns)
{
  const auto now_ros = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
  if (!camera_ || !fresh(now_ros, source_stamp_ns, config_.frame_max_age_sec)) {
    status_ = camera_ ? "image_stamp_stale" : "waiting_for_camera_info";
    invalidate_frame();
    return;
  }
  const Geometry * geometry = nullptr;
  std::optional<rm_interfaces::msg::VisionReceiveData> feedback;
  for (auto iterator = geometry_.rbegin(); iterator != geometry_.rend(); ++iterator) {
    if (iterator->stamp_ns <= source_stamp_ns) {
      if (!fresh(source_stamp_ns, iterator->stamp_ns, config_.feedback_max_age_sec)) break;
      const auto candidate = feedback_at(iterator->stamp_ns);
      // Both topics originate in the same Bevy update and carry the same stamp.
      // Never infer an installation rotation from different gimbal poses.
      if (candidate && stamp_ns(candidate->header.stamp) == iterator->stamp_ns) {
        geometry = &*iterator;
        feedback = candidate;
        break;
      }
    }
  }
  if (!feedback || feedback->bullet_speed == 0 || feedback->bullet_speed > 100) {
    status_ = "waiting_for_fresh_feedback_tf_and_speed";
    invalidate_frame();
    return;
  }
  if (source_stamp_ns <= source_stamp_ns_ || source_stamp_ns-source_stamp_ns_ > 500000000LL) {
    reset_pipelines();
  }
  prepare_pipelines(*geometry, *feedback);
  source_stamp_ns_ = source_stamp_ns;
  frame_time_ = std::chrono::steady_clock::now()-std::chrono::nanoseconds(now_ros-source_stamp_ns);
  std::array<vision::DetectionResult, 2> detections;
  for (const auto & armor : detection.armors) {
    if (armor.color != config_.enemy_color) continue;
    const bool large = !armor.classification.label.empty() ? armor.classification.label == "1" : armor.ratio >= 3.2;
    detections[large ? 1 : 0].armors.push_back(armor);
  }
  std::array<control::AutoAimFrameResult, 2> results;
  const auto motion_time = std::chrono::steady_clock::time_point(std::chrono::nanoseconds(source_stamp_ns));
  for (std::size_t type = 0; type < pipelines_.size(); ++type) {
    results[type] = pipelines_[type]->process(detections[type], motion_time, frame.sequence,
      feedback->bullet_speed, geometry->barrel_to_world);
  }
  const std::size_t other = 1-selected_type_;
  if ((!results[selected_type_].tracking.has_current_measurement && results[other].tracking.has_current_measurement) ||
      (!results[selected_type_].aim.valid && results[other].aim.valid)) selected_type_ = other;
  result_ = std::move(results[selected_type_]);
  if (result_->follow_target) result_->rotation.status = "follow_current_observation";
  selected_pose_ = pipelines_[selected_type_]->config().pose_solver;
  ++pipeline_frames_;
  status_ = result_->aim.valid ? "aim_ready" : result_->aim.error;
}

rm_interfaces::msg::VisionSendData SimulatorControl::cancellation(std::int64_t now_ros_ns)
{
  rm_interfaces::msg::VisionSendData message;
  message.header.stamp.sec = static_cast<std::int32_t>(now_ros_ns/1000000000LL);
  message.header.stamp.nanosec = static_cast<std::uint32_t>(now_ros_ns%1000000000LL);
  message.header.frame_id = "map";
  message.target_distance = -1.0F;
  return message;
}

rm_interfaces::msg::VisionSendData SimulatorControl::command(std::int64_t now_ros_ns)
{
  auto message = cancellation(now_ros_ns);
  const auto now = std::chrono::steady_clock::now();
  control::SafetyInput input;
  input.now = now;
  input.frame_available = result_.has_value();
  input.frame_timestamp = frame_time_;
  input.pipeline_result = result_ ? &*result_ : nullptr;
  input.transport_running = true;
  input.transport_connected = !feedback_.empty();
  input.processing_ok = result_.has_value() && !geometry_.empty() &&
    fresh(now_ros_ns, geometry_.back().stamp_ns, config_.feedback_max_age_sec);
  if (!feedback_.empty() && mount_) {
    const auto & feedback = feedback_.back();
    protocol::IncomingState state;
    state.gimbal = angles(raw_orientation(feedback) * *mount_);
    state.shoot.bullet_speed = feedback.bullet_speed;
    input.transport_state = state;
    const auto age = now_ros_ns-stamp_ns(feedback.header.stamp);
    input.transport_state_timestamp = now-std::chrono::nanoseconds(age);
    input.processing_ok = input.processing_ok && feedback.bullet_speed > 0 && feedback.bullet_speed <= 100;
  }
  input.visual_fire_intent = result_ && result_->tracking.has_current_measurement &&
    (result_->follow_target || (result_->guard_target_visible && result_->fire_window_valid));
  last_safety_ = safety_.decide(input);
  if (!last_safety_.aim_command_valid || !mount_ || !result_) {
    pulse_until_ns_ = 0;
    return message;
  }
  const auto target = angles(vision::gimbal_to_world(last_safety_.command.gimbal.yaw*to_rad,
    -last_safety_.command.gimbal.pitch*to_rad, 0.0)*mount_->t());
  if (std::abs(target.pitch) > config_.pitch_limit_deg || !std::isfinite(result_->aim.target_range_m) ||
      result_->aim.target_range_m <= 0) {
    status_ = "target_outside_simulator_gimbal_limits";
    pulse_until_ns_ = 0;
    return message;
  }
  message.target_state = 1;
  message.target_type = 0;
  message.yaw = feedback_.back().yaw + static_cast<float>(std::remainder(target.yaw-feedback_.back().yaw, 360.0));
  message.pitch = target.pitch;
  message.delta_yaw = static_cast<float>(std::remainder(message.yaw-feedback_.back().yaw, 360.0));
  message.delta_pitch = message.pitch-feedback_.back().pitch;
  message.target_distance = static_cast<float>(result_->aim.target_range_m);
  if (last_safety_.fire_allowed) {
    if (now_ros_ns >= next_fire_ns_) {
      pulse_until_ns_ = now_ros_ns+40000000LL;
      next_fire_ns_ = now_ros_ns+static_cast<std::int64_t>(config_.fire_period_sec*1e9);
    }
    if (now_ros_ns < pulse_until_ns_) message.target_state = 2;
  } else pulse_until_ns_ = 0;
  return message;
}
}  // namespace srm::io
