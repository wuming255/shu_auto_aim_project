#ifndef SHU_AUTO_AIM_MVS_CAMERA_HPP
#define SHU_AUTO_AIM_MVS_CAMERA_HPP

#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <opencv2/core.hpp>

namespace srm::io
{

struct MvsCameraConfig
{
  // 不指定序列号/索引时，仅在发现一个物理 USB 相机时自动选择。
  std::string serial_number;
  int device_index = -1;
  std::string sdk_library;
  std::optional<double> exposure_us;
  std::optional<double> gain;
  std::string pixel_format;  // GenICam symbolic，例如 BayerRG8；空值保留相机当前值。
  unsigned int timeout_ms = 1000;
};

void validate_mvs_config(const MvsCameraConfig & config);

// 仅描述编译支持；运行库按需加载，离线输入不调用 SDK。
bool mvs_compiled() noexcept;
std::string list_mvs_cameras(const std::string & sdk_library = {});

class MvsCamera final
{
public:
  MvsCamera(const MvsCameraConfig & config, int width, int height, double fps,
            bool start_grabbing = true);
  ~MvsCamera();
  MvsCamera(const MvsCamera &) = delete;
  MvsCamera & operator=(const MvsCamera &) = delete;

  bool read(cv::Mat & bgr, std::chrono::steady_clock::time_point & timestamp);
  const std::string & report() const noexcept;
  const std::string & last_error() const noexcept;
  double nominal_fps() const noexcept;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace srm::io
#endif
