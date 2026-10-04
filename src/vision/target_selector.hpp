#ifndef SHU_AUTO_AIM_TARGET_SELECTOR_HPP
#define SHU_AUTO_AIM_TARGET_SELECTOR_HPP

#include <chrono>
#include <cstdint>
#include <cstddef>
#include <limits>
#include <optional>
#include <vector>

#include "vision/pose_solver.hpp"

namespace srm::vision
{

struct TargetSelectorConfig
{
  // 当前传统检测器只输出颜色和几何候选，不包含数字分类结果。
  // 默认不按颜色过滤；需要时由上层根据赛务/下位机配置显式启用。
  bool filter_color = false;
  LightColor required_color = LightColor::unknown;

  // 质量门限。无穷大表示不额外限制，PnP 自身仍会执行有效性检查。
  double max_aim_error_rad = std::numeric_limits<double>::infinity();
  double max_range_m = std::numeric_limits<double>::infinity();
  double max_reprojection_rms_px = std::numeric_limits<double>::infinity();
};

// 一次图像帧中，一个装甲板检测候选及其静态几何解算结果。
// 该结构不包含跨帧状态；角速度、预测位置和目标身份由后续跟踪层负责。
struct TargetObservation
{
  std::uint64_t frame_sequence = 0;
  std::chrono::steady_clock::time_point timestamp{};
  std::size_t detection_index = 0;

  ArmorObservation armor;
  PoseSolution pose;

  // 当前选择策略使用云台目标角到零位的二维角距离作为“靠近视轴”指标。
  // 后续接入云台实时姿态或跟踪器时，可在不改变基础观测字段的情况下替换排序策略。
  double aim_error_rad = 0.0;
};

struct TargetSelection
{
  // 保留本帧所有候选（包括解算失败者），便于离线诊断和后续调参。
  // selected_index 是 observations 的下标，不是原始 DetectionResult::armors 的下标。
  std::vector<TargetObservation> observations;
  std::optional<std::size_t> selected_index;

  bool has_target() const noexcept;
  const TargetObservation * selected() const noexcept;
  TargetObservation * selected() noexcept;
};

class TargetSelector final
{
public:
  explicit TargetSelector(TargetSelectorConfig config = {});

  // 对每个检测候选执行 PnP，并从有效候选中选择最接近当前云台零位的一个。
  // 单个候选数据异常不会使整帧失败，而是保留为 pose.valid=false 的诊断记录。
  TargetSelection select(
    const DetectionResult & detections, const ArmorPoseSolver & pose_solver,
    std::chrono::steady_clock::time_point timestamp, std::uint64_t frame_sequence = 0) const;

  const TargetSelectorConfig & config() const noexcept;

private:
  bool eligible(const TargetObservation & observation) const noexcept;
  static double aim_error(const PoseSolution & pose) noexcept;
  static void validate_config(const TargetSelectorConfig & config);

  TargetSelectorConfig config_;
};

}  // namespace srm::vision

#endif  // SHU_AUTO_AIM_TARGET_SELECTOR_HPP
