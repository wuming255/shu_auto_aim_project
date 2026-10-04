#ifndef SHU_AUTO_AIM_TARGET_TRACKER_HPP
#define SHU_AUTO_AIM_TARGET_TRACKER_HPP

#include <chrono>
#include <cstddef>
#include <limits>
#include <optional>
#include <string>

#include "vision/target_selector.hpp"

namespace srm::vision
{

enum class TargetTrackState
{
  lost,
  detecting,
  tracking,
  temporarily_lost,
};

const char * to_string(TargetTrackState state) noexcept;

struct TargetTrackerConfig
{
  // 连续有效观测达到该数量后才进入 tracking；1 表示首个有效观测即可跟踪。
  std::size_t min_confirmed_updates = 3;
  // tracking 状态下允许连续无有效观测的帧数，超过后回到 lost。
  std::size_t max_consecutive_misses = 3;

  // 时间戳异常保护。真实帧率和调度延迟未知，现场适配时必须配置和验证。
  double min_dt_seconds = 1e-4;
  double max_dt_seconds = 0.25;

  // 预测输出的固定前视时间，以及丢帧时允许使用历史状态的最长时间。
  // 这些值不是现场标定结果，只是可被配置覆盖的离线默认值。
  double prediction_horizon_seconds = 0.05;
  double max_prediction_age_seconds = 0.15;

  // 速度指数平滑系数：1 表示完全使用最新差分，0 表示保留旧速度。
  double velocity_smoothing_factor = 1.0;
  // 无穷大表示不按速度做额外拒绝；仍会拒绝非有限速度。
  double max_abs_angular_velocity_rad_s = std::numeric_limits<double>::infinity();
};

struct TargetTrackEstimate
{
  TargetTrackState state = TargetTrackState::lost;
  bool has_current_measurement = false;
  bool has_last_measurement = false;
  bool velocity_valid = false;
  bool prediction_valid = false;
  bool reset_due_to_time_gap = false;

  std::size_t confirmed_updates = 0;
  std::size_t consecutive_misses = 0;
  std::chrono::steady_clock::time_point timestamp{};

  // current_observation 只在本次 update 有有效目标时有值；
  // last_observation 在暂时丢失状态下保留最近一次有效观测。
  std::optional<TargetObservation> current_observation;
  std::optional<TargetObservation> last_observation;

  // measured_yaw_rad 保留最近一次观测的规范化角度；
  // unwrapped_yaw_rad 用于跨越 -pi/pi 时连续计算角速度。
  double measured_yaw_rad = 0.0;
  double measured_pitch_rad = 0.0;
  double measured_range_m = 0.0;
  double unwrapped_yaw_rad = 0.0;
  double yaw_velocity_rad_s = 0.0;
  double pitch_velocity_rad_s = 0.0;
  double range_velocity_m_s = 0.0;

  // 预测角度使用连续 yaw，交给后续瞄准层时再按其坐标/范围要求规范化。
  double predicted_yaw_rad = 0.0;
  double predicted_pitch_rad = 0.0;
  double predicted_range_m = 0.0;

  std::string status;
};

class TargetTracker final
{
public:
  explicit TargetTracker(TargetTrackerConfig config = {});

  // 使用本帧 TargetSelection 的已选候选更新跟踪状态。
  // 没有 selected target 时仍必须传入当前帧 timestamp，以便正确处理丢帧和超时。
  TargetTrackEstimate update(
    const TargetSelection & selection, std::chrono::steady_clock::time_point timestamp);

  void reset() noexcept;

  const TargetTrackerConfig & config() const noexcept;
  TargetTrackState state() const noexcept;

  // 将角度规范化到 [-pi, pi] 附近，并按最短角差进行连续展开。
  static double wrap_angle(double angle_rad) noexcept;
  static double unwrap_angle(double previous_unwrapped_rad, double current_wrapped_rad) noexcept;

private:
  static void validate_config(const TargetTrackerConfig & config);
  static bool valid_measurement(const TargetObservation & observation) noexcept;
  static double elapsed_seconds(
    std::chrono::steady_clock::time_point newer,
    std::chrono::steady_clock::time_point older) noexcept;

  void clear_track() noexcept;
  void start_track(const TargetObservation & observation, std::chrono::steady_clock::time_point t);
  TargetTrackEstimate make_estimate(
    std::chrono::steady_clock::time_point timestamp, bool current_measurement,
    bool reset_due_to_time_gap, std::string status) const;
  void fill_prediction(TargetTrackEstimate & estimate, double age_seconds) const;

  TargetTrackerConfig config_;
  TargetTrackState state_ = TargetTrackState::lost;
  bool has_update_timestamp_ = false;
  bool has_measurement_ = false;
  bool velocity_initialized_ = false;

  std::chrono::steady_clock::time_point last_update_timestamp_{};
  std::chrono::steady_clock::time_point last_measurement_timestamp_{};
  std::optional<TargetObservation> last_observation_;

  std::size_t confirmed_updates_ = 0;
  std::size_t consecutive_misses_ = 0;

  double measured_yaw_rad_ = 0.0;
  double measured_pitch_rad_ = 0.0;
  double measured_range_m_ = 0.0;
  double unwrapped_yaw_rad_ = 0.0;
  double yaw_velocity_rad_s_ = 0.0;
  double pitch_velocity_rad_s_ = 0.0;
  double range_velocity_m_s_ = 0.0;
  bool velocity_valid_ = false;
};

}  // namespace srm::vision

#endif  // SHU_AUTO_AIM_TARGET_TRACKER_HPP
