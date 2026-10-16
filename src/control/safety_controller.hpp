#ifndef SHU_AUTO_AIM_SAFETY_CONTROLLER_HPP
#define SHU_AUTO_AIM_SAFETY_CONTROLLER_HPP

#include <chrono>
#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include "control/auto_aim_pipeline.hpp"
#include "protocol/self_aim_protocol.hpp"

namespace srm::control
{

enum class SafetyState
{
  safe,
  aiming,
  fire_ready,
};

const char * to_string(SafetyState state) noexcept;

enum class SafetyReason
{
  none,
  transport_not_running,
  transport_disconnected,
  transport_state_missing,
  transport_state_stale,
  frame_missing,
  frame_stale,
  processing_failed,
  target_unavailable,
  aim_invalid,
  bullet_speed_invalid,
  fire_disabled,
  target_not_visible,
  fire_window_invalid,
  gimbal_feedback_missing,
  gimbal_feedback_stale,
  gimbal_not_settled,
};

const char * to_string(SafetyReason reason) noexcept;

struct SafetyControllerConfig
{
  std::chrono::milliseconds max_frame_age{100};
  std::chrono::milliseconds max_transport_state_age{250};

  // 仅用于安全门检查；实际数学补偿仍由 AimCompensator 配置控制。
  double min_bullet_speed_m_s = 1.0;
  double max_bullet_speed_m_s = 100.0;
  // 临时非开火联调覆盖；同一值用于 runtime 解算和本安全门，原始反馈不改写。
  std::optional<double> fixed_bullet_speed_m_s;

  // 默认关闭视觉开火。即使打开，也仍需调用方显式提供 visual_fire_intent。
  bool allow_visual_fire = false;

  // 只有明确启用时，才用正式状态帧中的 yaw/pitch 反馈作为开火到位门。
  bool require_gimbal_feedback_for_fire = false;
  double max_gimbal_error_deg = 0.5;
  std::chrono::milliseconds min_gimbal_settle_time{50};
};

struct SafetyInput
{
  std::chrono::steady_clock::time_point now{};

  bool frame_available = false;
  std::chrono::steady_clock::time_point frame_timestamp{};

  // 只在本次 step 生命周期内使用，不由 SafetyController 保存指针。
  const AutoAimFrameResult * pipeline_result = nullptr;
  bool processing_ok = true;

  bool transport_running = false;
  bool transport_connected = false;
  std::optional<protocol::IncomingState> transport_state;
  std::optional<std::chrono::steady_clock::time_point> transport_state_timestamp;

  // 后续决策层可提供视觉开火意图；当前默认配置仍会强制 fire_flag=0。
  bool visual_fire_intent = false;
};

struct SafetyOutput
{
  SafetyState state = SafetyState::safe;
  protocol::Command command{};
  bool aim_command_valid = false;
  bool fire_allowed = false;
  std::vector<SafetyReason> reasons;
  std::string summary;
};

class SafetyController final
{
public:
  explicit SafetyController(SafetyControllerConfig config = {});

  SafetyOutput decide(const SafetyInput & input) const;

  const SafetyControllerConfig & config() const noexcept;

private:
  static bool finite(double value) noexcept;
  static bool timestamp_fresh(
    std::chrono::steady_clock::time_point now,
    std::chrono::steady_clock::time_point timestamp,
    std::chrono::milliseconds max_age) noexcept;
  static void validate_config(const SafetyControllerConfig & config);
  static void add_reason(SafetyOutput & output, SafetyReason reason);
  static bool has_reason(const SafetyOutput & output, SafetyReason reason) noexcept;

  SafetyControllerConfig config_;
  mutable std::optional<std::chrono::steady_clock::time_point> gimbal_settle_start_;
};

}  // namespace srm::control

#endif  // SHU_AUTO_AIM_SAFETY_CONTROLLER_HPP
