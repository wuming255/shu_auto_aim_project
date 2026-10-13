#ifndef SHU_AUTO_AIM_VIDEO_RECORDER_HPP
#define SHU_AUTO_AIM_VIDEO_RECORDER_HPP

#include <cstddef>
#include <string>
#include <opencv2/videoio.hpp>

namespace srm::app
{

// 同步 dry-run 录像。先写同目录临时 AVI，封口并完整回读后无覆盖发布。
// 原始 BGR 图像，不包含检测叠加、窗口信息栏或虚构姿态数据。
class VideoRecorder final
{
public:
  VideoRecorder(std::string path, double fps, std::string codec = "MJPG");
  ~VideoRecorder();
  VideoRecorder(const VideoRecorder &) = delete;
  VideoRecorder & operator=(const VideoRecorder &) = delete;

  void write(const cv::Mat & bgr);
  void close();  // 必须显式调用以获知封口/校验/发布错误；析构只做兜底。
  std::size_t frames_written() const noexcept { return frames_; }
  const std::string & path() const noexcept { return path_; }
  double fps() const noexcept { return fps_; }
  static double choose_fps(double requested, double nominal);

private:
  std::string path_;
  std::string temporary_path_;
  double fps_;
  std::string codec_;
  cv::Size size_;
  cv::VideoWriter writer_;
  int temporary_fd_ = -1;
  std::size_t frames_ = 0;
  bool failed_ = false;
  bool closed_ = false;
};

}  // namespace srm::app
#endif
