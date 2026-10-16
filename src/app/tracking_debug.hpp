#ifndef SHU_AUTO_AIM_TRACKING_DEBUG_HPP
#define SHU_AUTO_AIM_TRACKING_DEBUG_HPP

#include "control/auto_aim_pipeline.hpp"

namespace srm::app
{
// 保存和窗口复用同一叠加：紫色为本帧实际关联，青色为有效模型预测。
cv::Mat draw_tracking_debug(const cv::Mat & drawing,
  const control::AutoAimFrameResult & result, const vision::PoseSolverConfig & pose);
}
#endif
