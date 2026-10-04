#ifndef SHU_AUTO_AIM_AIM_COMPENSATOR_HPP
#define SHU_AUTO_AIM_AIM_COMPENSATOR_HPP

#include <cstddef>
#include <limits>
#include <string>

#include "vision/target_tracker.hpp"

namespace srm::control
{

struct AimCompensatorConfig
{
  // 当前基础模型采用无空气阻力、恒定重力的点目标模型；参数均可替换。
  double gravity_m_s2 = 9.80665;

  // bullet_speed 来自正式 CDC 下位机回传的 ShootState::bullet_speed，单位 m/s。
  double min_bullet_speed_m_s = 1.0;
  double max_bullet_speed_m_s = 100.0;

  // 当前仅允许在该范围内进行纯数学解算，避免异常距离进入弹道公式。
  double max_target_range_m = 100.0;
  double max_flight_time_s = 2.0;

  // 该延迟表示跟踪器已有 prediction_horizon 之外、尚未建模的额外延迟。
  // 不应把跟踪器已经使用的前视时间再次填入这里。
  double additional_latency_seconds = 0.0;

  // 迭代求解“目标运动 + 弹丸飞行时间”的耦合关系。
  std::size_t max_iterations = 4;
  double convergence_tolerance_seconds = 1e-5;

  // false 时只输出视线角和几何飞行时间，用于隔离测试和对比。
  bool enable_gravity_compensation = true;
};

struct AimSolution
{
  bool valid = false;
  std::string error;

  // yaw 输出规范化到 [-pi, pi]；pitch 为含重力补偿后的目标角，单位为 rad。
  double yaw_rad = 0.0;
  double pitch_rad = 0.0;
  double yaw_deg = 0.0;
  double pitch_deg = 0.0;

  double line_of_sight_pitch_rad = 0.0;
  double ballistic_pitch_offset_rad = 0.0;
  double horizontal_distance_m = 0.0;
  double vertical_distance_m = 0.0;
  double target_range_m = 0.0;
  double bullet_speed_m_s = 0.0;
  double flight_time_s = 0.0;
  double total_prediction_lead_s = 0.0;
};

class AimCompensator final
{
public:
  explicit AimCompensator(AimCompensatorConfig config = {});

  // 使用已确认 tracking 状态的目标估计和正式协议回传的 bullet_speed 解算目标角。
  // 本模块只返回数学结果，不生成协议帧，不产生 fire_flag。
  AimSolution solve(
    const vision::TargetTrackEstimate & target, double bullet_speed_m_s) const;

  const AimCompensatorConfig & config() const noexcept;

private:
  struct BallisticResult
  {
    bool valid = false;
    std::string error;
    double pitch_rad = 0.0;
    double flight_time_s = 0.0;
  };

  static bool finite(double value) noexcept;
  static void validate_config(const AimCompensatorConfig & config);
  static BallisticResult solve_ballistic(
    double horizontal_distance_m, double vertical_distance_m, double bullet_speed_m_s,
    double gravity_m_s2, bool enable_gravity_compensation);

  AimCompensatorConfig config_;
};

}  // namespace srm::control

#endif  // SHU_AUTO_AIM_AIM_COMPENSATOR_HPP
