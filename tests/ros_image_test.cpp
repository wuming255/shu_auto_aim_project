#include "io/ros_image.hpp"

#include <cassert>
#include <limits>
#include <stdexcept>

namespace
{
template<class Action>
void rejects(Action action)
{
  bool rejected = false;
  try { action(); } catch (const std::invalid_argument &) { rejected = true; }
  assert(rejected);
}

void test_owned_bgr_and_padded_rgb()
{
  sensor_msgs::msg::Image image;
  image.width = 2;
  image.height = 2;
  image.step = 8;
  image.encoding = "rgb8";
  image.data = {10, 20, 30, 40, 50, 60, 255, 255,
                70, 80, 90, 100, 110, 120, 255, 255};
  auto bgr = srm::io::decode_ros_image(image);
  assert(bgr.rows == 2 && bgr.cols == 2 && bgr.type() == CV_8UC3);
  assert(bgr.at<cv::Vec3b>(0, 0) == cv::Vec3b(30, 20, 10));
  assert(bgr.at<cv::Vec3b>(1, 1) == cv::Vec3b(120, 110, 100));
  image.encoding = "bgr8";
  bgr = srm::io::decode_ros_image(image);
  image.data[0] = 99;
  assert(bgr.at<cv::Vec3b>(0, 0) == cv::Vec3b(10, 20, 30));
  assert(bgr.at<cv::Vec3b>(1, 0) == cv::Vec3b(70, 80, 90));
}

void test_invalid_layouts()
{
  sensor_msgs::msg::Image image;
  rejects([&] { (void)srm::io::decode_ros_image(image); });
  image.width = 2;
  image.height = 2;
  image.step = 6;
  image.encoding = "rgb8";
  image.data.resize(11);
  rejects([&] { (void)srm::io::decode_ros_image(image); });
  image.data.resize(12);
  image.step = 5;
  rejects([&] { (void)srm::io::decode_ros_image(image); });
  image.step = 6;
  image.encoding = "mono8";
  rejects([&] { (void)srm::io::decode_ros_image(image); });
  image.encoding = "rgb8";
  image.width = std::numeric_limits<std::uint32_t>::max();
  rejects([&] { (void)srm::io::decode_ros_image(image); });
}

void test_virtual_camera_info()
{
  sensor_msgs::msg::CameraInfo info;
  info.width = 1440;
  info.height = 1080;
  info.distortion_model = "plumb_bob";
  info.k = {1303.675, 0., 720., 0., 1303.675, 540., 0., 0., 1.};
  info.d = {0., 0., 0., 0., 0.};
  const auto camera = srm::io::camera_from_ros_info(info);
  assert(camera.geometry.size == cv::Size(1440, 1080));
  assert(camera.geometry.orientation_deg == 0);
  assert(camera.camera_matrix(0, 0) == 1303.675);
  assert(camera.camera_matrix(1, 2) == 540.);
  assert(camera.distortion_coefficients.size() == 5);
  info.k[0] = 0.;
  rejects([&] { (void)srm::io::camera_from_ros_info(info); });
  info.k[0] = 1303.675;
  info.k[3] = std::numeric_limits<double>::quiet_NaN();
  rejects([&] { (void)srm::io::camera_from_ros_info(info); });
  info.k[3] = 0.;
  info.distortion_model = "equidistant";
  rejects([&] { (void)srm::io::camera_from_ros_info(info); });
  info.distortion_model = "plumb_bob";
  info.width = 0;
  rejects([&] { (void)srm::io::camera_from_ros_info(info); });
}
}  // namespace

int main()
{
  test_owned_bgr_and_padded_rgb();
  test_invalid_layouts();
  test_virtual_camera_info();
}
