#ifndef SHU_AUTO_AIM_DEBUG_VIEWER_HPP
#define SHU_AUTO_AIM_DEBUG_VIEWER_HPP

#include <chrono>
#include <functional>
#include <opencv2/core.hpp>

#include "io/frame_source.hpp"
#include "vision/traditional_detector.hpp"

namespace srm::app
{

// 应用层的可选 dry-run 窗口；相机只处理 GUI 事件，文件回放可暂停/限速。
class DebugViewer final
{
public:
  static bool available() noexcept;
  static void check_environment();

  explicit DebugViewer(double playback_fps, bool start_paused, bool realtime = false);
  ~DebugViewer();

  DebugViewer(const DebugViewer &) = delete;
  DebugViewer & operator=(const DebugViewer &) = delete;

  // detection 为空时显示当前原图和空二值图，避免沿用上一帧结果。
  // 返回 false 表示窗口关闭、退出按键或外部停止请求。
  bool show_and_wait(
    const cv::Mat & drawing, const io::Frame & frame,
    const vision::DetectionResult * detection, const std::function<bool()> & stop_requested);

private:
  void close() noexcept;
  void show_drawing(
    const cv::Mat & drawing, const io::Frame & frame, const vision::DetectionResult * detection);
  bool windows_open() const;

  std::chrono::steady_clock::duration frame_period_{};
  bool paused_ = false;
  bool realtime_ = false;
  bool windows_created_ = false;
  bool sized_ = false;
};

}  // namespace srm::app

#endif  // SHU_AUTO_AIM_DEBUG_VIEWER_HPP
