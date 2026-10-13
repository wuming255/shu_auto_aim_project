#include "app/video_recorder.hpp"

#include <cerrno>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <utility>
#include <vector>
#include <sys/stat.h>
#include <unistd.h>

namespace srm::app
{
namespace
{
bool valid_fps(double fps) { return std::isfinite(fps) && fps >= 0.1 && fps <= 1000.0; }
}

double VideoRecorder::choose_fps(double requested, double nominal)
{
  if (requested != 0.0) {
    if (!valid_fps(requested)) throw std::invalid_argument("invalid recording FPS");
    return requested;
  }
  return valid_fps(nominal) ? nominal : 30.0;
}

VideoRecorder::VideoRecorder(std::string path, double fps, std::string codec)
  : path_(std::move(path)), fps_(fps), codec_(std::move(codec))
{
  if (!valid_fps(fps_) || codec_.size() != 4) throw std::invalid_argument("invalid recording FPS/codec");
  const std::filesystem::path output(path_);
  if (output.extension() != ".avi") throw std::invalid_argument("recording path must end in .avi");
  struct stat status{};
  if (::lstat(path_.c_str(), &status) == 0) {
    throw std::runtime_error("recording path already exists (no overwrite): " + path_);
  }
  if (errno != ENOENT) throw std::runtime_error("cannot inspect recording path: " + path_ + ": " + std::strerror(errno));
  if (!output.parent_path().empty()) std::filesystem::create_directories(output.parent_path());
}

VideoRecorder::~VideoRecorder()
{
  try { close(); }
  catch (const std::exception & error) { std::cerr << "recording cleanup: " << error.what() << '\n'; }
}

void VideoRecorder::write(const cv::Mat & bgr)
{
  if (closed_ || failed_) throw std::runtime_error("recording is closed or failed");
  try {
    // 某些视频后端会悄悄裁掉奇数边；明确拒绝，不能称为完整原始帧。
    if (bgr.empty() || bgr.type() != CV_8UC3 || bgr.cols % 2 || bgr.rows % 2) {
      throw std::runtime_error("recording requires nonempty BGR8 frames with even width/height");
    }
    if (temporary_path_.empty()) {
      std::string pattern = path_ + ".partial-XXXXXX.avi";
      std::vector<char> name(pattern.begin(), pattern.end());
      name.push_back('\0');
      temporary_fd_ = ::mkstemps(name.data(), 4);
      if (temporary_fd_ < 0) throw std::runtime_error("cannot create recording file: " + std::string(std::strerror(errno)));
      temporary_path_ = name.data();
      size_ = bgr.size();
      const auto code = cv::VideoWriter::fourcc(codec_[0], codec_[1], codec_[2], codec_[3]);
      // 明确使用视频编码后端，避免图像序列或其他后端的隐式回退。
      if (!writer_.open(temporary_path_, cv::CAP_FFMPEG, code, fps_, size_, true)) {
        throw std::runtime_error("recording encoder unavailable or output cannot be opened: " + codec_ +
                                 " (requires OpenCV FFmpeg video writer)");
      }
    }
    if (bgr.size() != size_) throw std::runtime_error("recording frame dimensions changed");
    writer_.write(bgr);
    if (!writer_.isOpened()) throw std::runtime_error("recording writer closed unexpectedly");
    ++frames_;
  } catch (const std::exception & error) {
    failed_ = true;
    throw std::runtime_error(std::string(error.what()) + "; recording=" + path_ +
                             "; incomplete file=" + temporary_path_);
  }
}

void VideoRecorder::close()
{
  if (closed_) return;
  closed_ = true;
  try {
    writer_.release();  // 写完视频索引/尾部后才做持久化与回读。
    if (temporary_fd_ >= 0) {
      const int synced = ::fsync(temporary_fd_);
      const int sync_error = errno;
      const int closed = ::close(temporary_fd_);
      temporary_fd_ = -1;
      if (synced != 0 || closed != 0) {
        throw std::runtime_error("recording flush failed: " + std::string(std::strerror(synced != 0 ? sync_error : errno)));
      }
    }
    if (failed_) throw std::runtime_error("recording write failed");
    if (frames_ == 0) return;  // 无帧不发布空视频。

    // VideoWriter::write 返回 void，FFmpeg 的部分 I/O 错误只写 stderr。
    // 完整解码能发现静默短写、缺失尾部或少帧；不能仅凭 isOpened 声称成功。
    cv::VideoCapture verify(temporary_path_, cv::CAP_FFMPEG);
    if (!verify.isOpened()) throw std::runtime_error("recording verification failed: cannot reopen video");
    cv::Mat decoded;
    std::size_t count = 0;
    while (verify.read(decoded)) {
      if (decoded.size() != size_ || ++count > frames_) {
        throw std::runtime_error("recording verification failed: size/frame count mismatch");
      }
    }
    verify.release();
    if (count != frames_) throw std::runtime_error("recording verification failed: expected " +
      std::to_string(frames_) + " frames, decoded " + std::to_string(count));

    // 同一目录硬链接原子发布：即使录制期间出现同名文件，也绝不覆盖。
    if (::link(temporary_path_.c_str(), path_.c_str()) != 0) {
      throw std::runtime_error("cannot publish recording (no overwrite): " + std::string(std::strerror(errno)));
    }
    if (::unlink(temporary_path_.c_str()) != 0) {
      std::cerr << "recording published; could not remove temporary link: " << temporary_path_ << '\n';
    }
    temporary_path_.clear();
  } catch (const std::exception & error) {
    try { writer_.release(); } catch (...) {}
    if (temporary_fd_ >= 0) { (void)::close(temporary_fd_); temporary_fd_ = -1; }
    throw std::runtime_error(std::string(error.what()) + "; incomplete file retained at " + temporary_path_);
  }
}

}  // namespace srm::app
