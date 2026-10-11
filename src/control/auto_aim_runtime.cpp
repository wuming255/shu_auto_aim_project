#include "control/auto_aim_runtime.hpp"

#include <cmath>
#include <exception>
#include <limits>
#include <stdexcept>
#include <utility>

namespace srm::control
{
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
  io::Frame frame;
  if (!frame_source_.read(frame)) {     // 读取一帧图像
    result.end_of_stream = frame_source_.end_of_stream();
    result.error = result.end_of_stream ? "frame source reached end of stream"
                                        : "frame source read failed";
    pipeline_.reset();          // 重置瞄准算法模块，清除跟踪状态

    const auto decision_now = std::chrono::steady_clock::now();            // 这里获取时间传入下面
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
  const auto state = transport_.latest_state();
  const auto state_timestamp = transport_.latest_state_timestamp();         // 获取时间戳
  const double bullet_speed = state.has_value()     // 获取弹速
                                ? static_cast<double>(state->shoot.bullet_speed)
                                : std::numeric_limits<double>::quiet_NaN();

  try {
    const auto detections = detector_(frame);                               // 尝试识别
    result.pipeline = pipeline_.process(                                    // 尝试加载处理
      detections, frame.timestamp, frame.sequence, bullet_speed);
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
  SafetyInput input = make_base_safety_input(decision_now, result.frame);               
  input.transport_state = state;
  input.transport_state_timestamp = state_timestamp;
  input.processing_ok = result.pipeline.has_value() && result.error.empty();
  input.pipeline_result = result.pipeline.has_value() ? &result.pipeline.value() : nullptr;     // 依旧设置参数
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
  input.transport_state = transport_.latest_state();
  input.transport_state_timestamp = transport_.latest_state_timestamp();
  return input;
}

protocol::Command AutoAimRuntime::safe_command() noexcept
{
  protocol::Command command{};
  command.fire.fire_flag = 0;
  return command;
}

}  // namespace srm::control
