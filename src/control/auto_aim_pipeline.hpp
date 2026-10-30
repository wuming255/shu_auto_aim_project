#ifndef SHU_AUTO_AIM_AUTO_AIM_PIPELINE_HPP
#define SHU_AUTO_AIM_AUTO_AIM_PIPELINE_HPP

#include <chrono>
#include <cstdint>

#include "control/aim_compensator.hpp"
#include "control/fixed_guard.hpp"
#include "vision/pose_solver.hpp"
#include "vision/target_selector.hpp"
#include "vision/target_tracker.hpp"
#include "vision/rotating_target.hpp"
#include "vision/vehicle_target.hpp"

namespace srm::control
{

// 阶段六的软件垂直组合配置。通信设备、相机采集和开火安全门不属于本类职责。
struct AutoAimPipelineConfig
{
  vision::PoseSolverConfig pose_solver;
  vision::TargetSelectorConfig target_selector;
  vision::TargetTrackerConfig tracker;
  AimCompensatorConfig aim_compensator;
  vision::RotatingTargetConfig rotation;
  FixedGuardConfig fixed_guard;
  bool follow_target = false;  // 当前观测跟随；仿真可另启用整车预测。
  bool follow_vehicle_model = false;  // 仿真专用：惯性系四甲EKF、命中时刻预测。
};

struct AutoAimFrameResult
{
  std::uint64_t frame_sequence = 0;
  std::chrono::steady_clock::time_point timestamp{};

  vision::TargetSelection selection;
  vision::TargetTrackEstimate tracking;
  AimSolution aim;
  vision::RotationEstimate rotation;
  FixedGuardResult fixed_guard;
  bool guard_command_valid = false;
  bool guard_target_visible = false;
  bool fire_window_valid = false;
  bool follow_target = false;  // aim 为世界系绝对角，数学 pitch 向上为正。
  vision::VehicleTargetDiagnostics follow_model;
  int follow_plate = -1;
  std::string follow_selection_reason;
  double follow_yaw_step_rad = 0;
};

class AutoAimPipeline final
{
public:
  explicit AutoAimPipeline(AutoAimPipelineConfig config = {});

  // 处理一帧检测结果和同一时刻可用的下位机 bullet_speed。
  // 该接口只返回选择/跟踪/数学瞄准结果，不产生 CDC 命令和 fire_flag。
  AutoAimFrameResult process(
    const vision::DetectionResult & detections,
    std::chrono::steady_clock::time_point timestamp,
    std::uint64_t frame_sequence, double bullet_speed_m_s,
    std::optional<cv::Matx33d> rotation_gimbal_to_world = std::nullopt,
    const cv::Vec3d & muzzle_world = {});

  // 同一时间轴上的命令时刻重新预测；不修改滤波器、不刷新观测时间。
  void predict_follow(AutoAimFrameResult & result,
    std::chrono::steady_clock::time_point time, const cv::Vec3d & muzzle_world,
    double bullet_speed_m_s);

  void reset() noexcept;

  const AutoAimPipelineConfig & config() const noexcept;
  const vision::ArmorPoseSolver & pose_solver() const noexcept;
  const vision::TargetSelector & target_selector() const noexcept;
  const vision::TargetTracker & target_tracker() const noexcept;
  const AimCompensator & aim_compensator() const noexcept;

private:
  // Preview and ballistic iterations are pure with respect to the command lock.
  void forecast_follow(AutoAimFrameResult & result,
    std::chrono::steady_clock::time_point time, const cv::Vec3d & muzzle_world,
    double bullet_speed_m_s) const;
  AutoAimPipelineConfig config_;
  vision::ArmorPoseSolver pose_solver_;
  vision::TargetSelector target_selector_;
  vision::TargetTracker target_tracker_;
  AimCompensator aim_compensator_;
  vision::RotatingTarget rotating_target_;
  FixedGuardController fixed_guard_;
  vision::VehicleTarget vehicle_target_;
  int follow_plate_ = -1;
  std::size_t follow_generation_ = 0;
  std::optional<double> follow_command_yaw_;
  std::optional<vision::TargetObservation> rotation_last_observation_;
};

}  // namespace srm::control

#endif  // SHU_AUTO_AIM_AUTO_AIM_PIPELINE_HPP
