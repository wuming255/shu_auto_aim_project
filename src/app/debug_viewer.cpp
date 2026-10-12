#include "app/debug_viewer.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <sstream>
#include <stdexcept>

#ifdef SRM_HAVE_HIGHGUI
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#endif

namespace srm::app
{

#ifdef SRM_HAVE_HIGHGUI
namespace
{
constexpr const char * kDetectionWindow = "SRM detection";
constexpr const char * kMaskWindow = "SRM brightness mask";
}  // namespace
#endif

bool DebugViewer::available() noexcept
{
#ifdef SRM_HAVE_HIGHGUI
  return true;
#else
  return false;
#endif
}

DebugViewer::DebugViewer(double playback_fps, bool start_paused) : paused_(start_paused)
{
  if (!available()) {
    throw std::runtime_error(
      "debug windows are unavailable in this build; use --no-show-debug or rebuild "
      "with SRM_ENABLE_GUI=ON and OpenCV HighGUI");
  }
  if (!std::isfinite(playback_fps) || playback_fps <= 0.0) {
    throw std::invalid_argument("debug viewer requires a finite positive playback rate");
  }
  frame_period_ = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(1.0 / std::clamp(playback_fps, 0.1, 1000.0)));

#ifdef SRM_HAVE_HIGHGUI
#if defined(__linux__)
  const auto display = std::getenv("DISPLAY");
  const auto wayland = std::getenv("WAYLAND_DISPLAY");
  if ((display == nullptr || *display == '\0') && (wayland == nullptr || *wayland == '\0')) {
    throw std::runtime_error(
      "--show-debug requires a desktop display; use --no-show-debug for headless replay");
  }
#endif
  try {
    cv::namedWindow(kDetectionWindow, cv::WINDOW_NORMAL);
    windows_created_ = true;
    cv::namedWindow(kMaskWindow, cv::WINDOW_NORMAL);
  } catch (const cv::Exception &) {
    close();
    throw std::runtime_error(
      "failed to create debug windows; check the OpenCV GUI backend and desktop display, "
      "or use --no-show-debug");
  }
#endif
}

DebugViewer::~DebugViewer()
{
  close();
}

void DebugViewer::close() noexcept
{
#ifdef SRM_HAVE_HIGHGUI
  if (!windows_created_) return;
  for (const auto window : {kDetectionWindow, kMaskWindow}) {
    try {
      cv::destroyWindow(window);
    } catch (const cv::Exception &) {
      // 关闭按钮可能已经销毁窗口；清理不能遮盖原有退出或异常。
    }
  }
#endif
  windows_created_ = false;
}

void DebugViewer::show_drawing(
  const cv::Mat & drawing, const io::Frame & frame, const vision::DetectionResult * detection)
{
#ifdef SRM_HAVE_HIGHGUI
  // 在图像外加信息栏，保留灯条和四角点的可见性。
  cv::Mat display;
  cv::copyMakeBorder(
    drawing, display, 64, 0, 0, std::max(0, 640 - drawing.cols), cv::BORDER_CONSTANT,
    cv::Scalar(0, 0, 0));
  std::ostringstream info;
  info << "frame=" << frame.sequence << " time=" << std::fixed << std::setprecision(3)
       << frame.source_time_seconds << "s ";
  if (detection != nullptr) {
    info << "lights=" << detection->lightbars.size() << " armors=" << detection->armors.size();
  } else {
    info << "detection unavailable";
  }
  cv::putText(
    display, info.str(), {10, 23}, cv::FONT_HERSHEY_SIMPLEX, 0.55, {255, 255, 255}, 1,
    cv::LINE_AA);
  cv::putText(
    display, std::string(paused_ ? "PAUSED" : "PLAYING") +
               " | Space: pause/resume  n: next  q/Esc: quit",
    {10, 49}, cv::FONT_HERSHEY_SIMPLEX, 0.5, {0, 255, 255}, 1, cv::LINE_AA);
  cv::imshow(kDetectionWindow, display);
  if (!sized_) {
    const double scale = std::min({1.0, 960.0 / display.cols, 720.0 / display.rows});
    cv::resizeWindow(
      kDetectionWindow, static_cast<int>(display.cols * scale),
      static_cast<int>(display.rows * scale));
  }
#else
  (void)drawing;
  (void)frame;
  (void)detection;
#endif
}

bool DebugViewer::windows_open() const
{
#ifdef SRM_HAVE_HIGHGUI
  try {
    for (const auto window : {kDetectionWindow, kMaskWindow}) {
      const double visible = cv::getWindowProperty(window, cv::WND_PROP_VISIBLE);
      // GTK3 等后端不支持 VISIBLE，返回 -1；AUTOSIZE 仍能区分存在/已关闭的窗口。
      if (visible >= 0.0) {
        if (visible < 1.0) return false;
      } else if (cv::getWindowProperty(window, cv::WND_PROP_AUTOSIZE) < 0.0) {
        return false;
      }
    }
    return true;
  } catch (const cv::Exception &) {
    return false;
  }
#else
  return false;
#endif
}

bool DebugViewer::show_and_wait(
  const cv::Mat & drawing, const io::Frame & frame,
  const vision::DetectionResult * detection, const std::function<bool()> & stop_requested)
{
#ifdef SRM_HAVE_HIGHGUI
  show_drawing(drawing, frame, detection);
  const cv::Mat mask = detection != nullptr && !detection->brightness_mask.empty()
                         ? detection->brightness_mask
                         : cv::Mat::zeros(frame.bgr.size(), CV_8UC1);
  cv::imshow(kMaskWindow, mask);
  if (!sized_) {
    const double scale = std::min({1.0, 960.0 / mask.cols, 720.0 / mask.rows});
    cv::resizeWindow(
      kMaskWindow, static_cast<int>(mask.cols * scale), static_cast<int>(mask.rows * scale));
    sized_ = true;
  }

  // 计入本帧检测/保存耗时；处理慢于目标帧率时逐帧显示，不跳帧追赶。
  auto deadline = frame.timestamp + frame_period_;
  while (!stop_requested()) {
    const int key = cv::waitKey(10);
    if (!windows_open()) return false;
    if (key >= 0) {
      const int character = key & 0xff;
      if (character == 'q' || character == 'Q' || character == 27) return false;
      if (character == 'n' || character == 'N') {
        paused_ = true;
        return true;
      }
      if (character == ' ') {
        paused_ = !paused_;
        deadline = std::chrono::steady_clock::now() + frame_period_;
        show_drawing(drawing, frame, detection);
      }
    }
    if (!paused_ && std::chrono::steady_clock::now() >= deadline) return true;
  }
#else
  (void)drawing;
  (void)frame;
  (void)detection;
  (void)stop_requested;
#endif
  return false;
}

}  // namespace srm::app
