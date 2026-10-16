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

struct RuntimeStepResult               // 处理数据汇总struct
{
  bool frame_read = false;             // 是否成功读到一帧图像
  bool end_of_stream = false;          // 是否到达输入源末尾
  std::string error;                   // 错误信息记录
  std::chrono::microseconds step_duration{0};   // 本次 step 从读取帧开始到安全命令写入 transport 的单调耗时。
  std::chrono::microseconds acquisition_duration{0};
  std::chrono::microseconds detection_duration{0};
  std::chrono::microseconds pipeline_duration{0};

  std::optional<io::Frame> frame;
  std::optional<AutoAimFrameResult> pipeline;
  SafetyOutput safety;            // 安全判断结果
  std::optional<io::TimedIncomingState> feedback;  // 原始反馈，弹速覆盖不改变此值。
  std::optional<std::chrono::steady_clock::time_point> decision_timestamp;
  bool bullet_speed_overridden = false;
};

class AutoAimRuntime final
{
public:
  AutoAimRuntime(
    io::FrameSource & frame_source, io::SelfAimTransport & transport,
    AutoAimPipeline pipeline, SafetyController safety_controller,
    DetectionCallback detector);

  AutoAimRuntime(const AutoAimRuntime &) = delete;                    // 禁止拷贝构造函数（即禁止这个类被拷贝）
  AutoAimRuntime & operator=(const AutoAimRuntime &) = delete;        // 禁止拷贝赋值运算符（即禁止这个类被拷贝）

  // 执行一次有限步骤，并把安全决策得到的命令写入 transport。
  RuntimeStepResult step();

  // 启动 transport 后持续处理，直到 stop_requested 或文件输入结束。
  // 退出前发送一次 fire_flag=0 的安全命令并停止 transport。
  void run(std::atomic<bool> & stop_requested);

  AutoAimPipeline & pipeline() noexcept;                          // 这些是把pipeline和safetycontroller交给调用者访问以内部操作的函数
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
