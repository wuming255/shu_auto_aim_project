#include "io/ros_image.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <opencv2/imgproc.hpp>

namespace srm::io
{
namespace
{
void validate_size(std::uint32_t width, std::uint32_t height)
{
  if (width == 0 || height == 0 || width > static_cast<std::uint32_t>(std::numeric_limits<int>::max()) ||
      height > static_cast<std::uint32_t>(std::numeric_limits<int>::max())) {
    throw std::invalid_argument("ROS image dimensions must be nonzero and fit OpenCV integers");
  }
}
}  // namespace

cv::Mat decode_ros_image(const sensor_msgs::msg::Image & image)
{
  validate_size(image.width, image.height);
  if (image.encoding != "rgb8" && image.encoding != "bgr8") {
    throw std::invalid_argument("simulator input supports rgb8/bgr8, got " + image.encoding);
  }
  const auto width = static_cast<std::size_t>(image.width);
  const auto height = static_cast<std::size_t>(image.height);
  if (width > std::numeric_limits<std::size_t>::max() / 3 || image.step < width * 3 ||
      image.step > std::numeric_limits<std::size_t>::max() / height ||
      image.data.size() < static_cast<std::size_t>(image.step) * height) {
    throw std::invalid_argument("ROS image stride/buffer is shorter than its declared layout");
  }
  cv::Mat result(static_cast<int>(image.height), static_cast<int>(image.width), CV_8UC3);
  for (std::size_t row = 0; row < height; ++row) {
    std::memcpy(result.ptr(static_cast<int>(row)), image.data.data() + row * image.step, width * 3);
  }
  if (image.encoding == "rgb8") cv::cvtColor(result, result, cv::COLOR_RGB2BGR);
  return result;
}

vision::CameraCalibration camera_from_ros_info(const sensor_msgs::msg::CameraInfo & info)
{
  validate_size(info.width, info.height);
  if (info.distortion_model != "plumb_bob" && info.distortion_model != "rational_polynomial" &&
      !(info.distortion_model.empty() && info.d.empty())) {
    throw std::invalid_argument("simulator CameraInfo must use an OpenCV pinhole distortion model");
  }
  for (const double value : info.k) {
    if (!std::isfinite(value)) throw std::invalid_argument("CameraInfo K is not finite");
  }
  if (std::abs(info.k[1]) > 1e-9 || std::abs(info.k[3]) > 1e-9) {
    throw std::invalid_argument("CameraInfo K must have zero skew");
  }
  vision::CameraCalibration camera;
  camera.geometry = {{static_cast<int>(info.width), static_cast<int>(info.height)}, 0};
  std::copy(info.k.begin(), info.k.end(), camera.camera_matrix.val);
  camera.distortion_coefficients.assign(info.d.begin(), info.d.end());
  vision::PoseSolverConfig config;
  config.camera_matrix = camera.camera_matrix;
  config.distortion_coefficients = camera.distortion_coefficients;
  (void)vision::ArmorPoseSolver(config);
  return camera;
}
}  // namespace srm::io
