#include "control/auto_aim_pipeline.hpp"

#include <utility>

namespace srm::control
{

AutoAimPipeline::AutoAimPipeline(AutoAimPipelineConfig config)
: config_(std::move(config)),
  pose_solver_(config_.pose_solver),
  target_selector_(config_.target_selector),
  target_tracker_(config_.tracker),
  aim_compensator_(config_.aim_compensator)
{
}

AutoAimFrameResult AutoAimPipeline::process(
  const vision::DetectionResult & detections,
  std::chrono::steady_clock::time_point timestamp,
  std::uint64_t frame_sequence, double bullet_speed_m_s)
{
  AutoAimFrameResult result;
  result.frame_sequence = frame_sequence;
  result.timestamp = timestamp;
  result.selection = target_selector_.select(
    detections, pose_solver_, timestamp, frame_sequence);
  result.tracking = target_tracker_.update(result.selection, timestamp);
  result.aim = aim_compensator_.solve(result.tracking, bullet_speed_m_s);
  return result;
}

void AutoAimPipeline::reset() noexcept
{
  target_tracker_.reset();
}

const AutoAimPipelineConfig & AutoAimPipeline::config() const noexcept
{
  return config_;
}

const vision::ArmorPoseSolver & AutoAimPipeline::pose_solver() const noexcept
{
  return pose_solver_;
}

const vision::TargetSelector & AutoAimPipeline::target_selector() const noexcept
{
  return target_selector_;
}

const vision::TargetTracker & AutoAimPipeline::target_tracker() const noexcept
{
  return target_tracker_;
}

const AimCompensator & AutoAimPipeline::aim_compensator() const noexcept
{
  return aim_compensator_;
}

}  // namespace srm::control
