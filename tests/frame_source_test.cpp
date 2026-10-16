#include "io/frame_source.hpp"

#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <thread>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <unistd.h>

namespace
{
using srm::io::Frame;
using srm::io::FrameSource;
using srm::io::FrameSourceConfig;
using srm::io::FrameSourceType;

std::string temporary_image_path()
{
  return "/tmp/shu_auto_aim_frame_source_" + std::to_string(static_cast<long long>(::getpid())) +
         ".png";
}

cv::Mat make_test_image()
{
  cv::Mat image(3, 4, CV_8UC3, cv::Scalar(0, 0, 0));
  image.at<cv::Vec3b>(0, 0) = cv::Vec3b(11, 22, 33);
  image.at<cv::Vec3b>(2, 3) = cv::Vec3b(101, 102, 103);
  return image;
}

void test_image_source_and_reset(const std::string & path)
{
  FrameSourceConfig config;
  config.type = FrameSourceType::image_file;
  config.uri = path;

  FrameSource source(config);
  assert(source.is_open());
  assert(!source.end_of_stream());
  assert(source.type() == FrameSourceType::image_file);
  assert(source.description() == "image:" + path);

  Frame frame;
  assert(source.read(frame));
  assert(frame.sequence == 0);
  assert(frame.bgr.rows == 3 && frame.bgr.cols == 4);
  assert(frame.bgr.type() == CV_8UC3);
  assert(frame.bgr.at<cv::Vec3b>(0, 0) == cv::Vec3b(11, 22, 33));
  assert(frame.bgr.at<cv::Vec3b>(2, 3) == cv::Vec3b(101, 102, 103));
  assert(frame.timestamp != std::chrono::steady_clock::time_point{});
  assert(frame.source_time_seconds == 0.0);

  assert(!source.read(frame));
  assert(source.end_of_stream());

  source.reset();
  assert(!source.end_of_stream());
  assert(source.read(frame));
  assert(frame.sequence == 0);
}

void test_looping_image_source(const std::string & path)
{
  FrameSourceConfig config;
  config.type = FrameSourceType::image_file;
  config.uri = path;
  config.loop = true;

  FrameSource source(config);
  Frame first;
  Frame second;
  Frame third;
  assert(source.read(first));
  assert(source.read(second));
  assert(source.read(third));
  assert(first.sequence == 0);
  assert(second.sequence == 1);
  assert(third.sequence == 2);
  assert(!source.end_of_stream());
  assert(first.bgr.at<cv::Vec3b>(0, 0) == second.bgr.at<cv::Vec3b>(0, 0));
}

void test_compatibility_read(const std::string & path)
{
  FrameSourceConfig config{FrameSourceType::image_file, path};
  FrameSource source(config);
  cv::Mat image;
  std::chrono::steady_clock::time_point timestamp;
  assert(source.read(image, timestamp));
  assert(!image.empty());
  assert(timestamp != std::chrono::steady_clock::time_point{});
}

void test_video_source_clock_and_loop(const std::string & image_path)
{
  const auto path = image_path + ".avi";
  cv::VideoWriter writer(path, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 20.0, {32, 24});
  assert(writer.isOpened());
  for (int i = 0; i < 3; ++i) writer.write(cv::Mat(24, 32, CV_8UC3, cv::Scalar(i*40, 0, 0)));
  writer.release();

  {
    FrameSourceConfig config{FrameSourceType::video_file, path};
    config.loop = true;
    FrameSource source(config);
    Frame first, second, third, wrapped, next;
    assert(source.read(first));
    assert(first.motion_timestamp.has_value());
    assert(first.motion_timestamp.value() == std::chrono::steady_clock::time_point{});
    assert(!first.timeline_reset);
    std::this_thread::sleep_for(std::chrono::milliseconds(120));
    assert(source.read(second));
    assert(second.motion_timestamp.has_value());
    const double dt = std::chrono::duration<double>(*second.motion_timestamp-*first.motion_timestamp).count();
    assert(std::abs(dt-0.05) < 1e-6);
    assert(second.timestamp-first.timestamp >= std::chrono::milliseconds(100));
    assert(std::string(second.motion_time_source) == "video_nominal_fps");
    assert(source.read(third));
    assert(source.read(wrapped));
    assert(wrapped.sequence == 3 && wrapped.timeline_reset);
    assert(wrapped.motion_timestamp == first.motion_timestamp);
    assert(wrapped.timestamp > third.timestamp);
    assert(source.read(next));
    assert(!next.timeline_reset && next.motion_timestamp == second.motion_timestamp);

    source.reset();
    assert(source.read(wrapped));
    assert(wrapped.sequence == 0 && wrapped.timeline_reset);
    assert(wrapped.motion_timestamp == first.motion_timestamp);
    assert(source.read(next));
    assert(!next.timeline_reset && next.motion_timestamp == second.motion_timestamp);
  }
  std::remove(path.c_str());
}

void test_invalid_configuration(const std::string & path)
{
  bool empty_uri_threw = false;
  try {
    FrameSourceConfig config;
    config.type = FrameSourceType::image_file;
    FrameSource source(config);
  } catch (const std::invalid_argument &) {
    empty_uri_threw = true;
  }
  assert(empty_uri_threw);

  bool missing_file_threw = false;
  try {
    FrameSourceConfig config;
    config.type = FrameSourceType::image_file;
    config.uri = path + ".missing";
    FrameSource source(config);
  } catch (const std::runtime_error &) {
    missing_file_threw = true;
  }
  assert(missing_file_threw);

  bool bad_fps_threw = false;
  try {
    FrameSourceConfig config;
    config.type = FrameSourceType::image_file;
    config.uri = path;
    config.fps = -1.0;
    FrameSource source(config);
  } catch (const std::invalid_argument &) {
    bad_fps_threw = true;
  }
  assert(bad_fps_threw);
}

void test_mvs_unavailable()
{
  FrameSourceConfig config;
  config.type = FrameSourceType::mvs_camera;
  config.mvs.sdk_library = "/nonexistent/srm_mvs_runtime.so";
  bool caught = false;
  try { FrameSource source(config); } catch (const std::runtime_error & error) {
    caught = true;
    const std::string message = error.what();
    assert(message.find(srm::io::mvs_compiled() ? "runtime unavailable" :
                       "support is unavailable in this build") != std::string::npos);
  }
  assert(caught);
}

}  // namespace

int main()
{
  const auto path = temporary_image_path();
  const auto image = make_test_image();
  assert(cv::imwrite(path, image));

  test_image_source_and_reset(path);
  test_looping_image_source(path);
  test_compatibility_read(path);
  test_video_source_clock_and_loop(path);
  test_invalid_configuration(path);
  test_mvs_unavailable();

  std::remove(path.c_str());
  return 0;
}
