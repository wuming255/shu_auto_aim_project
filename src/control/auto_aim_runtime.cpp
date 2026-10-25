#include "control/auto_aim_runtime.hpp"

#include <cmath>
#include <exception>
#include <stdexcept>
#include <utility>

namespace srm::control
{
namespace
{
struct DurationTimer
{
  std::chrono::microseconds & result;
  std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
  ~DurationTimer() { result = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - start); }
};
}
// 这是构造函数的定义
AutoAimRuntime::AutoAimRuntime(
  io::FrameSource & frame_source, io::SelfAimTransport & transport,
  AutoAimPipeline pipeline, SafetyController safety_controller, DetectionCallback detector)
: frame_source_(frame_source),        // 冒号表示初始化成员列表，括号内是初始化传入的参数，括号外是成员变量的名字
  transport_(transport),
  pipeline_(std::move(pipeline)),
  safety_controller_(std::move(safety_controller)),
  detector_(std::move(detector))
{
  if (!detector_) throw std::invalid_argument("auto aim runtime detector callback must be set");
}

RuntimeStepResult AutoAimRuntime::step()
{
  const auto step_start = std::chrono::steady_clock::now();   // 获取当前时间点
  RuntimeStepResult result;
  result.bullet_speed_overridden = safety_controller_.config().fixed_bullet_speed_m_s.has_value();
  io::Frame frame;
  bool read_ok = false;
  {
    DurationTimer timer{result.acquisition_duration};
    read_ok = frame_source_.read(frame);
  }
  if (!read_ok) {     // 读取一帧图像
    result.end_of_stream = frame_source_.end_of_stream();
    result.error = result.end_of_stream ? "frame source reached end of stream"
                                        : "frame source read failed";
    pipeline_.reset();          // 重置瞄准算法模块，清除跟踪状态

    const auto decision_now = std::chrono::steady_clock::now();            // 这里获取时间传入下面
    result.decision_timestamp = decision_now;
    auto input = make_base_safety_input(decision_now, std::nullopt);      
    input.processing_ok = false;                                           // 设置input参数
    result.safety = safety_controller_.decide(input);                      // 传入处理并作为新的参数输入
    transport_.set_command(result.safety.command);
    result.step_duration = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::steady_clock::now() - step_start);
    return result;                                                          // 返回本次读取的各项结果
  }

  result.frame_read = true;
  result.frame = frame;                                                     // 依旧设参数
  const auto snapshot = transport_.state_snapshot();
  result.feedback = snapshot;
  const auto state = snapshot ? std::optional<protocol::IncomingState>(snapshot->state) : std::nullopt;
  const auto state_timestamp = snapshot ? std::optional<std::chrono::steady_clock::time_point>(snapshot->timestamp) : std::nullopt;
  const double bullet_speed = safety_controller_.effective_bullet_speed(
    state ? static_cast<double>(state->shoot.bullet_speed) : 0.0);

  try {
    if (frame.timeline_reset) pipeline_.reset();
    if (!frame.motion_timestamp) throw std::runtime_error("frame has no valid motion timestamp");
    vision::DetectionResult detections;
    {
      DurationTimer timer{result.detection_duration};
      detections = detector_(frame);
    }
    DurationTimer timer{result.pipeline_duration};
    std::optional<cv::Matx33d> orientation;
    const bool realtime = frame_source_.type() == io::FrameSourceType::mvs_camera ||
                          frame_source_.type() == io::FrameSourceType::opencv_camera;
    if ((realtime || pipeline_.config().follow_target) &&
        pipeline_.config().rotation.feedback_frame_confirmed) {
      const auto pose_state = transport_.state_at(frame.timestamp,std::chrono::milliseconds(50));
      if (pose_state) {
        constexpr double to_rad = 3.14159265358979323846/180.0;
        const auto & gimbal = pose_state->state.gimbal;
        if (std::isfinite(gimbal.yaw) && std::isfinite(gimbal.pitch) && std::isfinite(gimbal.roll)) {
          // 已确认同济约定：外部 pitch 向下为正，数学旋转函数 pitch 向上为正。
          orientation = vision::gimbal_to_world(gimbal.yaw*to_rad,-gimbal.pitch*to_rad,gimbal.roll*to_rad);
        }
      }
    }
    result.pipeline = pipeline_.process(                                    // 尝试加载处理
      detections, *frame.motion_timestamp, frame.sequence, bullet_speed,orientation);
  } catch (const std::exception & error) {
    result.error = error.what();
    pipeline_.reset();
  } catch (...) {
    result.error = "unknown frame processing error";
    pipeline_.reset();
  }

  // FrameSource 在 read() 内生成单调时间戳；决策时刻必须在 read() 之后
  // 获取，否则极快输入可能被误判为“来自未来”的帧。
  const auto decision_now = std::chrono::steady_clock::now();                           // 获取时间
  result.decision_timestamp = decision_now;
  SafetyInput input = make_base_safety_input(decision_now, result.frame);               
  input.transport_state = state;
  input.transport_state_timestamp = state_timestamp;
  input.processing_ok = result.pipeline.has_value() && result.error.empty();
  input.pipeline_result = result.pipeline.has_value() ? &result.pipeline.value() : nullptr;     // 依旧设置参数
  input.visual_fire_intent = result.pipeline.has_value() && result.pipeline->tracking.state ==
    vision::TargetTrackState::tracking && result.pipeline->aim.valid &&
    (result.pipeline->follow_target ||
     (result.pipeline->guard_command_valid && result.pipeline->guard_target_visible &&
      result.pipeline->fire_window_valid));
  result.safety = safety_controller_.decide(input);                                             // 继续设参数
  transport_.set_command(result.safety.command);                        // 进行通信
  result.step_duration = std::chrono::duration_cast<std::chrono::microseconds>(
    std::chrono::steady_clock::now() - step_start);
  return result;
}

void AutoAimRuntime::run(std::atomic<bool> & stop_requested)        // 循环运行step()和通信（其实main里面有这个循环设置）
{
  transport_.start();
  while (!stop_requested.load()) {
    const auto result = step();
    if (result.end_of_stream) break;
    if (!result.frame_read && !transport_.running()) break;
  }
  (void)transport_.send_command_now(safe_command());
  transport_.stop();
}

AutoAimPipeline & AutoAimRuntime::pipeline() noexcept     
{
  return pipeline_;
}

const AutoAimPipeline & AutoAimRuntime::pipeline() const noexcept
{
  return pipeline_;
}

SafetyController & AutoAimRuntime::safety_controller() noexcept
{
  return safety_controller_;
}

const SafetyController & AutoAimRuntime::safety_controller() const noexcept
{
  return safety_controller_;
}

SafetyInput AutoAimRuntime::make_base_safety_input(           // 设置安全控制器的输入
  std::chrono::steady_clock::time_point now,
  const std::optional<io::Frame> & frame) const
{
  SafetyInput input;
  input.now = now;
  input.frame_available = frame.has_value();
  if (frame.has_value()) input.frame_timestamp = frame->timestamp;
  input.transport_running = transport_.running();
  input.transport_connected = transport_.connected();
  const auto snapshot = transport_.state_snapshot();
  if (snapshot) {
    input.transport_state = snapshot->state;
    input.transport_state_timestamp = snapshot->timestamp;
  }
  return input;
}

protocol::Command AutoAimRuntime::safe_command() noexcept
{
  protocol::Command command{};
  command.fire.fire_flag = 0;
  return command;
}

}  // namespace srm::control
