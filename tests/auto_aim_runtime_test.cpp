#include "control/auto_aim_runtime.hpp"

#include <algorithm>
#include <atomic>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <poll.h>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core.hpp>
#include <unistd.h>

namespace
{
using namespace std::chrono_literals;
using srm::control::AutoAimPipelineConfig;
using srm::control::AutoAimRuntime;
using srm::control::RuntimeStepResult;
using srm::control::SafetyController;
using srm::control::SafetyControllerConfig;
using srm::io::Frame;
using srm::io::FrameSource;
using srm::io::FrameSourceConfig;
using srm::io::FrameSourceType;
using srm::io::SelfAimTransport;
using srm::io::TransportConfig;
using srm::protocol::IncomingState;
using srm::vision::ArmorObservation;
using srm::vision::ArmorPoseSolver;
using srm::vision::DetectionResult;
using srm::vision::LightColor;
using srm::vision::PoseSolverConfig;
using srm::vision::TargetTrackState;

class PtyPair final
{
public:
  PtyPair()
  {
    master_ = ::posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (master_ < 0) throw std::runtime_error("posix_openpt failed");
    if (::grantpt(master_) != 0 || ::unlockpt(master_) != 0) {
      ::close(master_);
      throw std::runtime_error("grantpt/unlockpt failed");
    }

    char slave_name[256]{};
    if (::ptsname_r(master_, slave_name, sizeof(slave_name)) != 0) {
      ::close(master_);
      throw std::runtime_error("ptsname_r failed");
    }
    slave_path_ = slave_name;
  }

  ~PtyPair()
  {
    if (master_ >= 0) ::close(master_);
  }

  PtyPair(const PtyPair &) = delete;
  PtyPair & operator=(const PtyPair &) = delete;

  const std::string & slave_path() const { return slave_path_; }

  void write_all(const std::vector<std::uint8_t> & data)
  {
    std::size_t offset = 0;
    while (offset < data.size()) {
      const auto written = ::write(master_, data.data() + offset, data.size() - offset);
      if (written < 0) throw std::runtime_error("pty write failed");
      offset += static_cast<std::size_t>(written);
    }
  }

  bool read_exact_until(
    std::vector<std::uint8_t> & result, std::size_t size,
    std::chrono::steady_clock::time_point deadline)
  {
    result.clear();
    result.reserve(size);
    while (result.size() < size) {
      const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
        deadline - std::chrono::steady_clock::now());
      if (remaining <= 0ms) return false;

      pollfd descriptor{master_, POLLIN, 0};
      const int ready = ::poll(&descriptor, 1, std::max(1, static_cast<int>(remaining.count())));
      if (ready < 0) throw std::runtime_error("poll failed");
      if (ready == 0) return false;
      if ((descriptor.revents & POLLIN) == 0) return false;

      std::uint8_t buffer[64]{};
      const auto wanted = std::min<std::size_t>(sizeof(buffer), size - result.size());
      const auto count = ::read(master_, buffer, wanted);
      if (count < 0) throw std::runtime_error("pty read failed");
      if (count == 0) return false;
      result.insert(result.end(), buffer, buffer + count);
    }
    return true;
  }

private:
  int master_ = -1;
  std::string slave_path_;
};

void write_u16(std::vector<std::uint8_t> & data, std::size_t offset, std::uint16_t value)
{
  data[offset] = static_cast<std::uint8_t>(value & 0xffU);
  data[offset + 1] = static_cast<std::uint8_t>((value >> 8U) & 0xffU);
}

void write_i16(std::vector<std::uint8_t> & data, std::size_t offset, std::int16_t value)
{
  write_u16(data, offset, static_cast<std::uint16_t>(value));
}

void write_i32(std::vector<std::uint8_t> & data, std::size_t offset, std::int32_t value)
{
  const auto bits = static_cast<std::uint32_t>(value);
  data[offset] = static_cast<std::uint8_t>(bits & 0xffU);
  data[offset + 1] = static_cast<std::uint8_t>((bits >> 8U) & 0xffU);
  data[offset + 2] = static_cast<std::uint8_t>((bits >> 16U) & 0xffU);
  data[offset + 3] = static_cast<std::uint8_t>((bits >> 24U) & 0xffU);
}

void write_f32(std::vector<std::uint8_t> & data, std::size_t offset, float value)
{
  std::uint32_t bits{};
  std::memcpy(&bits, &value, sizeof(bits));
  write_i32(data, offset, static_cast<std::int32_t>(bits));
}

float read_f32(const std::vector<std::uint8_t> & data, std::size_t offset)
{
  std::uint32_t bits = static_cast<std::uint32_t>(data[offset]) |
                       (static_cast<std::uint32_t>(data[offset + 1]) << 8U) |
                       (static_cast<std::uint32_t>(data[offset + 2]) << 16U) |
                       (static_cast<std::uint32_t>(data[offset + 3]) << 24U);
  float value{};
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

std::int32_t read_i32(const std::vector<std::uint8_t> & data, std::size_t offset)
{
  const auto bits = static_cast<std::uint32_t>(data[offset]) |
                    (static_cast<std::uint32_t>(data[offset + 1]) << 8U) |
                    (static_cast<std::uint32_t>(data[offset + 2]) << 16U) |
                    (static_cast<std::uint32_t>(data[offset + 3]) << 24U);
  return static_cast<std::int32_t>(bits);
}

std::vector<std::uint8_t> make_incoming_frame(float bullet_speed, float yaw = 0, float pitch = 0)
{
  std::vector<std::uint8_t> frame(srm::protocol::kIncomingFrameLength, 0);
  write_u16(frame, 0, 28);
  write_i16(frame, 2, 1);
  write_f32(frame, 4, yaw);
  write_f32(frame, 8, pitch);
  write_f32(frame, 12, 0.0F);
  write_i32(frame, 16, 0);
  write_i32(frame, 20, 0);
  write_i16(frame, 24, 2);
  write_f32(frame, 26, bullet_speed);
  return frame;
}

PoseSolverConfig make_pose_config()
{
  PoseSolverConfig config;
  config.camera_matrix = cv::Matx33d(800.0, 0.0, 320.0, 0.0, 800.0, 240.0, 0.0, 0.0, 1.0);
  config.distortion_coefficients = {0.0, 0.0, 0.0, 0.0, 0.0};
  config.max_reprojection_rms_px = 0.5;
  return config;
}

cv::Vec3d gimbal_position(double yaw, double pitch, double range)
{
  const double horizontal = range * std::cos(pitch);
  return {
    horizontal * std::cos(yaw), horizontal * std::sin(yaw), range * std::sin(pitch)};
}

ArmorObservation make_armor(const PoseSolverConfig & config, double yaw = 0.12)
{
  const auto object_points = ArmorPoseSolver::object_points();
  const std::vector<cv::Point3f> object_vector(object_points.begin(), object_points.end());
  const cv::Vec3d target = gimbal_position(yaw, 0.0, 5.0);
  const cv::Vec3d camera_translation = config.rotation_camera_to_gimbal.t() * target;

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
  return armor;
}

AutoAimPipelineConfig make_pipeline_config()
{
  AutoAimPipelineConfig config;
  config.pose_solver = make_pose_config();
  config.tracker.min_confirmed_updates = 1;
  config.tracker.min_dt_seconds = 1e-7;
  config.tracker.max_dt_seconds = 0.2;
  config.tracker.max_prediction_age_seconds = 0.25;
  config.tracker.max_abs_angular_velocity_rad_s = 100.0;
  config.aim_compensator.max_target_range_m = 20.0;
  config.aim_compensator.max_flight_time_s = 2.0;
  return config;
}

TransportConfig make_transport_config(const std::string & path)
{
  TransportConfig config;
  config.device = path;
  config.transmit_period = 5ms;
  config.io_timeout = 15ms;
  config.reconnect_interval = 10ms;
  return config;
}

bool is_non_firing_frame(const std::vector<std::uint8_t> & frame)
{
  return frame.size() == srm::protocol::kOutgoingFrameLength &&
         read_i32(frame, 14) == 0;
}

bool wait_for_matching_command(
  PtyPair & pty, float expected_yaw, float expected_pitch,
  std::chrono::milliseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  std::vector<std::uint8_t> frame;
  while (std::chrono::steady_clock::now() < deadline) {
    if (!pty.read_exact_until(frame, srm::protocol::kOutgoingFrameLength, deadline)) return false;
    if (is_non_firing_frame(frame) &&
        std::fabs(read_f32(frame, 4) - expected_yaw) < 1e-4F &&
        std::fabs(read_f32(frame, 8) - expected_pitch) < 1e-4F) {
      return true;
    }
  }
  return false;
}

void test_runtime_closed_loop_and_safety_fallback()
{
  const auto image_path =
    std::string("/tmp/shu_auto_aim_runtime_") + std::to_string(static_cast<long long>(::getpid())) +
    ".png";
  const cv::Mat image(16, 16, CV_8UC3, cv::Scalar(0, 0, 0));
  assert(cv::imwrite(image_path, image));

  PtyPair pty;
  SelfAimTransport transport(make_transport_config(pty.slave_path()));
  transport.start();
  assert(transport.wait_until_connected(500ms));

  std::vector<std::uint8_t> frame;
  assert(pty.read_exact_until(
    frame, srm::protocol::kOutgoingFrameLength,
    std::chrono::steady_clock::now() + 1s));
  assert(is_non_firing_frame(frame));

  // 未发射时的正常零值应使用默认22 m/s，仍能输出并发送瞄准命令。
  pty.write_all(make_incoming_frame(0.0F));
  const auto state_deadline = std::chrono::steady_clock::now() + 1s;
  while (transport.received_frame_count() == 0 && std::chrono::steady_clock::now() < state_deadline) {
    std::this_thread::sleep_for(1ms);
  }
  assert(transport.received_frame_count() >= 1);

  FrameSourceConfig source_config;
  source_config.type = FrameSourceType::image_file;
  source_config.uri = image_path;
  source_config.loop = true;
  FrameSource source(source_config);

  AutoAimPipelineConfig pipeline_config = make_pipeline_config();
  SafetyControllerConfig safety_config;
  safety_config.max_frame_age = 100ms;
  safety_config.max_transport_state_age = 40ms;

  std::size_t detector_calls = 0;
  AutoAimRuntime runtime(
    source, transport, srm::control::AutoAimPipeline(pipeline_config),
    SafetyController(safety_config),
    [&detector_calls, &pipeline_config](const Frame &) {
      ++detector_calls;
      DetectionResult result;
      if (detector_calls != 2) {
        result.armors.push_back(make_armor(pipeline_config.pose_solver));
      }
      return result;
    });

  const RuntimeStepResult aimed = runtime.step();
  assert(aimed.frame_read);
  assert(aimed.pipeline.has_value());
  assert(aimed.pipeline->tracking.state == TargetTrackState::tracking);
  assert(aimed.pipeline->aim.valid);
  assert(aimed.pipeline->aim.bullet_speed_m_s == 22.0);
  assert(aimed.feedback && aimed.feedback->state.shoot.bullet_speed == 0.0F);
  assert(!aimed.bullet_speed_overridden);
  assert(aimed.safety.state == srm::control::SafetyState::aiming);
  assert(aimed.safety.aim_command_valid);
  assert(aimed.safety.command.fire.fire_flag == 0);
  assert(detector_calls == 1);
  assert(wait_for_matching_command(
    pty, aimed.safety.command.gimbal.yaw, aimed.safety.command.gimbal.pitch, 1s));

  const RuntimeStepResult missing_target = runtime.step();
  assert(missing_target.frame_read);
  assert(missing_target.pipeline.has_value());
  assert(missing_target.pipeline->tracking.state == TargetTrackState::temporarily_lost);
  assert(missing_target.safety.state == srm::control::SafetyState::safe);
  assert(!missing_target.safety.aim_command_valid);
  assert(missing_target.safety.command.fire.fire_flag == 0);

  std::this_thread::sleep_for(60ms);
  const RuntimeStepResult stale_state = runtime.step();
  assert(stale_state.frame_read);
  assert(stale_state.pipeline.has_value());
  assert(stale_state.pipeline->aim.valid);
  assert(stale_state.safety.state == srm::control::SafetyState::safe);
  assert(!stale_state.safety.aim_command_valid);
  assert(stale_state.safety.command.fire.fire_flag == 0);
  bool saw_stale_reason = false;
  for (const auto reason : stale_state.safety.reasons) {
    if (reason == srm::control::SafetyReason::transport_state_stale) saw_stale_reason = true;
  }
  assert(saw_stale_reason);

  // 发射后收到有效实测值，立即用于解算，不继续写死22。
  const auto before = transport.received_frame_count();
  pty.write_all(make_incoming_frame(30.0F));
  const auto measured_deadline = std::chrono::steady_clock::now() + 1s;
  while (transport.received_frame_count() == before && std::chrono::steady_clock::now() < measured_deadline) {
    std::this_thread::sleep_for(1ms);
  }
  assert(transport.received_frame_count() > before);
  const auto measured = runtime.step();
  assert(measured.pipeline && measured.pipeline->aim.bullet_speed_m_s == 30.0);
  assert(measured.feedback && measured.feedback->state.shoot.bullet_speed == 30.0F);
  assert(measured.safety.aim_command_valid && measured.safety.command.fire.fire_flag == 0);
  assert(wait_for_matching_command(
    pty, measured.safety.command.gimbal.yaw, measured.safety.command.gimbal.pitch, 1s));

  transport.stop();
  assert(!transport.running());
  assert(transport.command().fire.fire_flag == 0);
  ::unlink(image_path.c_str());
}

void test_fixed_speed_follow_sends_absolute_non_firing_command()
{
  const auto path = "/tmp/shu_follow_" + std::to_string(::getpid()) + ".png";
  assert(cv::imwrite(path, cv::Mat::zeros(16,16,CV_8UC3)));
  PtyPair pty;
  SelfAimTransport transport(make_transport_config(pty.slave_path()));
  transport.start();
  assert(transport.wait_until_connected(500ms));
  FrameSourceConfig source_config;
  source_config.type = FrameSourceType::image_file;
  source_config.uri = path;
  source_config.loop = true;
  FrameSource source(source_config);
  auto config = make_pipeline_config();
  config.follow_target = true;
  config.rotation.enabled = true;
  config.rotation.feedback_frame_confirmed = true;
  SafetyControllerConfig safety;
  safety.fixed_bullet_speed_m_s = 22;
  bool visible = true;
  AutoAimRuntime runtime(source, transport, srm::control::AutoAimPipeline(config),
    SafetyController(safety), [&](const Frame &) {
      DetectionResult detections;
      if (visible) detections.armors.push_back(make_armor(config.pose_solver, 0));
      return detections;
    });
  for (const float raw_speed : {0.0F, std::numeric_limits<float>::quiet_NaN(), 30.0F}) {
    const auto before = transport.received_frame_count();
    pty.write_all(make_incoming_frame(raw_speed, 350, -10));
    const auto deadline = std::chrono::steady_clock::now() + 1s;
    while (transport.received_frame_count() == before && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(1ms);
    }
    assert(transport.received_frame_count() > before);
    const auto aimed = runtime.step();
    assert(aimed.pipeline && aimed.pipeline->aim.bullet_speed_m_s == 22);
    assert(aimed.feedback);
    assert(aimed.decision_timestamp && *aimed.decision_timestamp >= aimed.feedback->timestamp);
    if (std::isnan(raw_speed)) assert(std::isnan(aimed.feedback->state.shoot.bullet_speed));
    else assert(aimed.feedback->state.shoot.bullet_speed == raw_speed);
    assert(aimed.safety.aim_command_valid);
    assert(std::abs(aimed.safety.command.gimbal.yaw - 350) < 0.001);
    assert(aimed.safety.command.gimbal.pitch < -12.8 && aimed.safety.command.gimbal.pitch > -13.0);
    assert(aimed.safety.command.fire.fire_flag == 0);
    assert(wait_for_matching_command(pty, aimed.safety.command.gimbal.yaw,
      aimed.safety.command.gimbal.pitch, 1s));
  }
  visible = false;
  const auto lost = runtime.step();
  assert(!lost.safety.aim_command_valid && lost.safety.command.fire.fire_flag == 0);
  // 当前反馈新鲜时，丢目标保持当前姿态，避免反复跳回零位。
  assert(std::abs(lost.safety.command.gimbal.yaw - 350) < 0.001);
  assert(std::abs(lost.safety.command.gimbal.pitch + 10) < 0.001);
  transport.stop();
  ::unlink(path.c_str());
}

void test_runtime_end_of_stream_is_safe_and_stops_transport()
{
  const auto image_path =
    std::string("/tmp/shu_auto_aim_runtime_eof_") +
    std::to_string(static_cast<long long>(::getpid())) + ".png";
  const cv::Mat image(8, 8, CV_8UC3, cv::Scalar(0, 0, 0));
  assert(cv::imwrite(image_path, image));

  PtyPair pty;
  SelfAimTransport transport(make_transport_config(pty.slave_path()));
  FrameSourceConfig source_config;
  source_config.type = FrameSourceType::image_file;
  source_config.uri = image_path;
  FrameSource source(source_config);

  std::atomic<bool> stop_requested{false};
  AutoAimRuntime runtime(
    source, transport, srm::control::AutoAimPipeline(make_pipeline_config()), SafetyController(),
    [](const Frame &) { return DetectionResult{}; });
  runtime.run(stop_requested);

  assert(!transport.running());
  assert(!transport.connected());
  assert(transport.command().fire.fire_flag == 0);
  ::unlink(image_path.c_str());
}

void test_video_motion_time_does_not_follow_processing_delay()
{
  const auto path = std::string("/tmp/shu_auto_aim_motion_time_") +
    std::to_string(static_cast<long long>(::getpid())) + ".avi";
  cv::VideoWriter writer(path, cv::VideoWriter::fourcc('M','J','P','G'), 20.0, {32, 24});
  assert(writer.isOpened());
  for (int i = 0; i < 3; ++i) writer.write(cv::Mat(24, 32, CV_8UC3, cv::Scalar(0,0,0)));
  writer.release();
  FrameSourceConfig config;
  config.type = FrameSourceType::video_file;
  config.uri = path;
  config.loop = true;
  FrameSource source(config);
  SelfAimTransport transport(make_transport_config("/nonexistent/offline"));
  AutoAimRuntime runtime(source, transport,
    srm::control::AutoAimPipeline(make_pipeline_config()), SafetyController(),
    [](const Frame & frame) {
      DetectionResult result;
      result.armors.push_back(make_armor(make_pose_config(), 0.12 + 0.01*(frame.sequence%3)));
      return result;
    });
  const auto first = runtime.step();
  std::this_thread::sleep_for(300ms);  // 超过跟踪器允许间隔，模拟暂停。
  const auto second = runtime.step();
  assert(first.pipeline && second.pipeline);
  const double dt = std::chrono::duration<double>(
    second.pipeline->timestamp - first.pipeline->timestamp).count();
  assert(std::abs(dt - 0.05) < 1e-6);
  assert(!second.pipeline->tracking.reset_due_to_time_gap);
  assert(second.pipeline->tracking.has_current_measurement);
  assert(second.pipeline->tracking.velocity_valid);
  assert(std::abs(second.pipeline->tracking.yaw_velocity_rad_s - 0.2) < 0.001);
  assert(second.frame->timestamp > first.frame->timestamp + 250ms);
  (void)runtime.step();
  const auto looped = runtime.step();
  assert(looped.pipeline && looped.frame->sequence == 3);
  assert(looped.frame->timeline_reset);
  assert(looped.pipeline->tracking.confirmed_updates == 1);
  assert(!looped.pipeline->tracking.velocity_valid);
  assert(looped.safety.command.fire.fire_flag == 0);
  ::unlink(path.c_str());
}

}  // namespace

int main()
{
  test_fixed_speed_follow_sends_absolute_non_firing_command();
  test_runtime_closed_loop_and_safety_fallback();
  test_runtime_end_of_stream_is_safe_and_stops_transport();
  test_video_motion_time_does_not_follow_processing_delay();
  return 0;
}
