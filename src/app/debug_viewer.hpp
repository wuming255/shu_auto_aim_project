#ifndef SHU_AUTO_AIM_DEBUG_VIEWER_HPP
#define SHU_AUTO_AIM_DEBUG_VIEWER_HPP

#include <chrono>
#include <functional>
#include <opencv2/core.hpp>

#include "io/frame_source.hpp"
#include "vision/traditional_detector.hpp"

namespace srm::app
{

// 应用层的可选回放窗口；检测、读帧和调试图保存仍不依赖 GUI。
class DebugViewer final
{
public:
  static bool available() noexcept;

  explicit DebugViewer(double playback_fps, bool start_paused);
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
  bool windows_created_ = false;
  bool sized_ = false;
};

}  // namespace srm::app

#endif  // SHU_AUTO_AIM_DEBUG_VIEWER_HPP
