#ifndef SHU_AUTO_AIM_AUTO_AIM_PIPELINE_HPP
#define SHU_AUTO_AIM_AUTO_AIM_PIPELINE_HPP

#include <chrono>
#include <cstdint>

#include "control/aim_compensator.hpp"
#include "vision/pose_solver.hpp"
#include "vision/target_selector.hpp"
#include "vision/target_tracker.hpp"

namespace srm::control
{

// 阶段六的软件垂直组合配置。通信设备、相机采集和开火安全门不属于本类职责。
struct AutoAimPipelineConfig
{
  vision::PoseSolverConfig pose_solver;
  vision::TargetSelectorConfig target_selector;
  vision::TargetTrackerConfig tracker;
  AimCompensatorConfig aim_compensator;
};

struct AutoAimFrameResult
{
  std::uint64_t frame_sequence = 0;
  std::chrono::steady_clock::time_point timestamp{};

  vision::TargetSelection selection;
  vision::TargetTrackEstimate tracking;
  AimSolution aim;
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
    std::uint64_t frame_sequence, double bullet_speed_m_s);

  void reset() noexcept;

  const AutoAimPipelineConfig & config() const noexcept;
  const vision::ArmorPoseSolver & pose_solver() const noexcept;
  const vision::TargetSelector & target_selector() const noexcept;
  const vision::TargetTracker & target_tracker() const noexcept;
  const AimCompensator & aim_compensator() const noexcept;

private:
  AutoAimPipelineConfig config_;
  vision::ArmorPoseSolver pose_solver_;
  vision::TargetSelector target_selector_;
  vision::TargetTracker target_tracker_;
  AimCompensator aim_compensator_;
};

}  // namespace srm::control

#endif  // SHU_AUTO_AIM_AUTO_AIM_PIPELINE_HPP
