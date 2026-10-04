#ifndef SHU_AUTO_AIM_AUTO_AIM_RUNTIME_HPP
#define SHU_AUTO_AIM_AUTO_AIM_RUNTIME_HPP

#include <atomic>
#include <chrono>
#include <functional>
#include <optional>
#include <string>

#include "control/auto_aim_pipeline.hpp"
#include "control/safety_controller.hpp"
#include "io/frame_source.hpp"
#include "io/self_aim_transport.hpp"

namespace srm::control
{

using DetectionCallback = std::function<vision::DetectionResult(const io::Frame & frame)>;

struct RuntimeStepResult
{
  bool frame_read = false;
  bool end_of_stream = false;
  std::string error;
  // 本次 step 从读取帧开始到安全命令写入 transport 的单调耗时。
  std::chrono::microseconds step_duration{0};

  std::optional<io::Frame> frame;
  std::optional<AutoAimFrameResult> pipeline;
  SafetyOutput safety;
};

class AutoAimRuntime final
{
public:
  AutoAimRuntime(
    io::FrameSource & frame_source, io::SelfAimTransport & transport,
    AutoAimPipeline pipeline, SafetyController safety_controller,
    DetectionCallback detector);

  AutoAimRuntime(const AutoAimRuntime &) = delete;
  AutoAimRuntime & operator=(const AutoAimRuntime &) = delete;

  // 执行一次有限步骤，并把安全决策得到的命令写入 transport。
  RuntimeStepResult step();

  // 启动 transport 后持续处理，直到 stop_requested 或文件输入结束。
  // 退出前发送一次 fire_flag=0 的安全命令并停止 transport。
  void run(std::atomic<bool> & stop_requested);

  AutoAimPipeline & pipeline() noexcept;
  const AutoAimPipeline & pipeline() const noexcept;
  SafetyController & safety_controller() noexcept;
  const SafetyController & safety_controller() const noexcept;

private:
  SafetyInput make_base_safety_input(
    std::chrono::steady_clock::time_point now,
    const std::optional<io::Frame> & frame) const;
  static protocol::Command safe_command() noexcept;

  io::FrameSource & frame_source_;
  io::SelfAimTransport & transport_;
  AutoAimPipeline pipeline_;
  SafetyController safety_controller_;
  DetectionCallback detector_;
};

}  // namespace srm::control

#endif  // SHU_AUTO_AIM_AUTO_AIM_RUNTIME_HPP
