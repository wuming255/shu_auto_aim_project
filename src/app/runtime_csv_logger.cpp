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
  }
  stream_ << ','
          << csv_field(control::to_string(result.safety.state)) << ','
          << (result.safety.aim_command_valid ? 1 : 0) << ','
          << (result.safety.fire_allowed ? 1 : 0) << ','
          << result.safety.command.fire.fire_flag << ','
          << csv_field(join_reasons(result.safety)) << ','
          << csv_field(result.error) << '\n';
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
             "safety_reasons,error\n";
  stream_.flush();
  if (!stream_) throw std::runtime_error("failed to write runtime log header: " + path_);
}

}  // namespace srm::app
