#include "app/runtime_csv_logger.hpp"

#include <chrono>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace srm::app
{

RuntimeCsvLogger::RuntimeCsvLogger(std::string path) : path_(std::move(path))
{
  if (path_.empty()) return;
  stream_.open(path_, std::ios::out | std::ios::trunc);
  if (!stream_.is_open()) {
    throw std::runtime_error("failed to open runtime log: " + path_);
  }
  write_header();
}

void RuntimeCsvLogger::log(
  const control::RuntimeStepResult & result, const RuntimeTransportSnapshot & transport)
{
  if (!enabled()) return;

  stream_ << (result.frame.has_value() ? std::to_string(result.frame->sequence) : "") << ','
          << (result.frame_read ? 1 : 0) << ','
          << (result.end_of_stream ? 1 : 0) << ',';

  if (result.frame.has_value()) {
    const auto timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
      result.frame->timestamp.time_since_epoch()).count();
    stream_ << timestamp_us << ',' << std::setprecision(17) << result.frame->source_time_seconds;
  } else {
    stream_ << ',';
  }
  stream_ << ',' << result.step_duration.count() << ','
          << transport.sent_frame_count << ','
          << transport.received_frame_count << ','
          << transport.communication_error_count << ',';

  if (result.pipeline.has_value()) {
    const auto & pipeline = result.pipeline.value();
    stream_ << pipeline.selection.observations.size() << ','
            << csv_field(vision::to_string(pipeline.tracking.state)) << ','
            << (pipeline.aim.valid ? 1 : 0) << ',' << std::setprecision(17)
            << pipeline.aim.yaw_deg << ',' << pipeline.aim.pitch_deg << ','
            << pipeline.aim.bullet_speed_m_s;
  } else {
    stream_ << ",,,,,";
  }
  stream_ << ','
          << csv_field(control::to_string(result.safety.state)) << ','
          << (result.safety.aim_command_valid ? 1 : 0) << ','
          << (result.safety.fire_allowed ? 1 : 0) << ','
          << result.safety.command.fire.fire_flag << ','
          << csv_field(join_reasons(result.safety)) << ','
          << csv_field(result.error) << ',';
  if (result.frame) {
    const auto & frame = *result.frame;
    if (frame.motion_timestamp) {
      stream_ << std::chrono::duration_cast<std::chrono::microseconds>(
        frame.motion_timestamp->time_since_epoch()).count();
    }
    stream_ << ',' << csv_field(frame.motion_time_source ? frame.motion_time_source : "")
            << ',' << (frame.timeline_reset ? 1 : 0);
  } else {
    stream_ << ",,";
  }
  if (result.pipeline) {
    const auto & pipeline = *result.pipeline;
    const auto & track = pipeline.tracking;
    stream_ << ',' << csv_field(track.status);
    if (track.has_current_measurement && track.current_observation) {
      const auto & observation = *track.current_observation;
      stream_ << ',' << observation.detection_index
              << ',' << csv_field(observation.armor.classification.label)
              << ',' << observation.armor.classification.confidence;
      for (const auto & point : observation.armor.points) stream_ << ',' << point.x << ',' << point.y;
    } else {
      for (int column = 0; column < 11; ++column) stream_ << ',';
    }
    const auto & rotation = pipeline.rotation;
    stream_ << ',' << csv_field(rotation.status) << ',' << rotation.initialized
            << ',' << rotation.reliable << ',' << rotation.prediction_valid
            << ',' << rotation.stable_frame_valid << ',' << rotation.pose_ambiguous;
    if (rotation.initialized) {
      stream_ << ',' << rotation.center_world_m[0] << ',' << rotation.center_world_m[1]
              << ',' << rotation.center_world_m[2] << ',' << rotation.radius_m
              << ',' << rotation.phase_rad << ',' << rotation.angular_velocity_rad_s
              << ',' << rotation.position_rms_m << ',' << rotation.phase_rms_rad;
    } else {
      for (int column = 0; column < 8; ++column) stream_ << ',';
    }
    stream_ << ',' << rotation.observation_age_seconds << ',';
    if (rotation.initialized || rotation.current_visible) stream_ << rotation.associated_plate;
    stream_ << ',' << (pipeline.guard_command_valid ? 1 : 0)
            << ',' << (pipeline.guard_target_visible ? 1 : 0)
            << ',' << (pipeline.fire_window_valid ? 1 : 0)
            << ',' << pipeline.fixed_guard.phase_error_rad;
  } else {
    for (int column = 0; column < 32; ++column) stream_ << ',';
  }
  stream_ << ',' << result.safety.command.gimbal.yaw << ',' << result.safety.command.gimbal.pitch;
  if (result.feedback) {
    const auto & feedback = result.feedback->state;
    stream_ << ',' << feedback.gimbal.yaw << ',' << feedback.gimbal.pitch << ',' << feedback.gimbal.roll
            << ',' << feedback.shoot.bullet_speed << ',';
    if (result.decision_timestamp) stream_ << std::chrono::duration<double,std::milli>(
      *result.decision_timestamp - result.feedback->timestamp).count();
  } else {
    stream_ << ",,,,,";
  }
  stream_ << ',' << (result.bullet_speed_overridden ? "fixed" : "referee") << ','
          << csv_field(result.pipeline ? result.pipeline->aim.error : "") << '\n';
  stream_.flush();
  if (!stream_) throw std::runtime_error("failed while writing runtime log: " + path_);
  ++rows_written_;
}

bool RuntimeCsvLogger::enabled() const noexcept
{
  return stream_.is_open();
}

std::size_t RuntimeCsvLogger::rows_written() const noexcept
{
  return rows_written_;
}

const std::string & RuntimeCsvLogger::path() const noexcept
{
  return path_;
}

std::string RuntimeCsvLogger::csv_field(const std::string & value)
{
  bool needs_quotes = false;
  for (const char character : value) {
    if (character == ',' || character == '"' || character == '\n' || character == '\r') {
      needs_quotes = true;
      break;
    }
  }
  if (!needs_quotes) return value;

  std::string escaped = "\"";
  for (const char character : value) {
    if (character == '"') escaped += "\"\"";
    else escaped += character;
  }
  escaped += '"';
  return escaped;
}

std::string RuntimeCsvLogger::join_reasons(const control::SafetyOutput & safety)
{
  std::ostringstream stream;
  for (std::size_t index = 0; index < safety.reasons.size(); ++index) {
    if (index != 0) stream << '|';
    stream << control::to_string(safety.reasons[index]);
  }
  return stream.str();
}

void RuntimeCsvLogger::write_header()
{
  stream_ << "frame_sequence,frame_read,end_of_stream,frame_time_us,source_time_seconds,"
             "step_duration_us,transport_sent_frame_count,transport_received_frame_count,"
             "transport_communication_error_count,candidate_count,track_state,aim_valid,yaw_deg,"
             "pitch_deg,bullet_speed_m_s,safety_state,aim_command_valid,fire_allowed,fire_flag,"
             "safety_reasons,error,motion_time_us,motion_time_source,timeline_reset,"
             "track_status,current_detection_index,current_label,current_confidence,"
             "current_tl_x,current_tl_y,current_tr_x,current_tr_y,"
             "current_br_x,current_br_y,current_bl_x,current_bl_y,"
             "rotation_status,rotation_initialized,rotation_reliable,rotation_prediction_valid,"
             "rotation_stable_frame,rotation_pose_ambiguous,"
             "rotation_center_x_m,rotation_center_y_m,rotation_center_z_m,rotation_radius_m,"
             "rotation_phase_rad,rotation_omega_rad_s,rotation_position_rms_m,rotation_phase_rms_rad,"
              "rotation_observation_age_s,rotation_associated_plate,guard_command_valid,"
               "guard_target_visible,fire_window_valid,guard_phase_error_rad,"
               "command_yaw_deg,command_pitch_deg,feedback_yaw_deg,feedback_pitch_deg,feedback_roll_deg,"
               "feedback_bullet_speed_m_s,feedback_age_ms,bullet_speed_source,aim_error\n";
  stream_.flush();
  if (!stream_) throw std::runtime_error("failed to write runtime log header: " + path_);
}

}  // namespace srm::app
