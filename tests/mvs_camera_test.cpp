#include "io/frame_source.hpp"
#include <PixelType.h>
#include <cassert>
#include <cstdlib>
#include <functional>
#include <stdexcept>
#include <string>

extern "C" {
void srm_mvs_mock_reset(int count);
int srm_mvs_mock_stat(const char * key);
void srm_mvs_mock_pixel(unsigned int type);
}

namespace
{
using srm::io::FrameSource;
using srm::io::FrameSourceConfig;
using srm::io::FrameSourceType;
using srm::io::MvsCamera;

void expect_error(const std::function<void()> & action, const std::string & message)
{
  bool caught = false;
  try { action(); } catch (const std::runtime_error & error) {
    caught = true;
    assert(std::string(error.what()).find(message) != std::string::npos);
  }
  assert(caught);
}

void reset(int count = 1, const char * failure = "")
{
  srm_mvs_mock_reset(count);
  assert(::setenv("SRM_TEST_MVS_FAIL", failure, 1) == 0);
}

void test_selection_and_lifecycle(const FrameSourceConfig & config)
{
  reset(0);
  assert(srm::io::list_mvs_cameras(config.mvs.sdk_library).find("usb_device_count=0") != std::string::npos);
  expect_error([&] { FrameSource camera(config); }, "selection failed");
  assert(srm_mvs_mock_stat("create") == 0);
  reset(2);
  expect_error([&] { FrameSource camera(config); }, "selection failed");
  auto selected = config;
  selected.mvs.serial_number = "MOCK-B";
  {
    FrameSource camera(selected);
    assert(camera.description().find("serial=MOCK-B") != std::string::npos);
  }
  assert(srm_mvs_mock_stat("stop") == 1 && srm_mvs_mock_stat("close") == 1);
  assert(srm_mvs_mock_stat("destroy") == 1);
  assert(srm_mvs_mock_stat("initialize") == srm_mvs_mock_stat("finalize"));
  selected.mvs.serial_number = "missing";
  expect_error([&] { FrameSource camera(selected); }, "selection failed");
  selected.mvs.serial_number.clear(); selected.mvs.device_index = 1;
  { FrameSource camera(selected); assert(camera.description().find("serial=MOCK-B") != std::string::npos); }

  reset(1, "open");
  expect_error([&] { FrameSource camera(config); }, "MV_CC_OpenDevice");
  assert(srm_mvs_mock_stat("destroy") == 1 && srm_mvs_mock_stat("close") == 0);
  reset(1, "start");
  expect_error([&] { FrameSource camera(config); }, "MV_CC_StartGrabbing");
  assert(srm_mvs_mock_stat("close") == 1 && srm_mvs_mock_stat("destroy") == 1);
  assert(srm_mvs_mock_stat("stop") == 0);
}

void test_parameters(const FrameSourceConfig & config)
{
  reset();
  {
    MvsCamera info(config.mvs, 0, 0, 0, false);
    assert(info.report().find("ExposureTime RW current=1000 min=1 max=10000") != std::string::npos);
    assert(info.report().find("supported=RGB8Packed,BGR8Packed,Mono8") != std::string::npos);
    assert(srm_mvs_mock_stat("set_enum") == 0 && srm_mvs_mock_stat("set_float") == 0);
    assert(srm_mvs_mock_stat("start") == 0);
  }
  reset();
  auto custom = config;
  custom.width = 4; custom.height = 4; custom.fps = 60;
  custom.mvs.exposure_us = 2000; custom.mvs.gain = 0;
  custom.mvs.pixel_format = "Mono8";
  {
    FrameSource camera(custom);
    assert(camera.nominal_fps() == 60);
    assert(camera.description().find("ExposureTime RW current=2000") != std::string::npos);
    assert(srm_mvs_mock_stat("nodes") == 1);
    assert(srm_mvs_mock_stat("set_float") == 3 && srm_mvs_mock_stat("set_integer") == 2);
  }
  reset(); custom.width = 3;
  expect_error([&] { FrameSource camera(custom); }, "inc=2");
  assert(srm_mvs_mock_stat("set_integer") == 0 && srm_mvs_mock_stat("start") == 0);
  reset(); custom = config; custom.mvs.exposure_us = 10001;
  expect_error([&] { FrameSource camera(custom); }, "outside [");
  assert(srm_mvs_mock_stat("set_float") == 0 && srm_mvs_mock_stat("destroy") == 1);
  reset(1, "gain_ro"); custom = config; custom.mvs.gain = 0;
  expect_error([&] { FrameSource camera(custom); }, "not writable");
  reset(1, "readback"); custom = config; custom.mvs.gain = 0;
  expect_error([&] { FrameSource camera(custom); }, "readback mismatch");
  reset(); custom = config; custom.mvs.pixel_format = "unsupported";
  expect_error([&] { FrameSource camera(custom); }, "PixelFormat=unsupported");
}

void test_frames(const FrameSourceConfig & config)
{
  for (const auto pixel : {PixelType_Gvsp_RGB8_Packed, PixelType_Gvsp_BGR8_Packed, PixelType_Gvsp_Mono8}) {
    reset(); srm_mvs_mock_pixel(pixel);
    FrameSource camera(config);
    srm::io::Frame first, second;
    assert(camera.read(first) && camera.read(second));
    assert(first.bgr.type() == CV_8UC3 && first.bgr.rows == 2 && first.bgr.cols == 2);
    const auto color = first.bgr.at<cv::Vec3b>(0, 0);
    if (pixel == PixelType_Gvsp_RGB8_Packed) assert(color == cv::Vec3b(30, 20, 10));
    if (pixel == PixelType_Gvsp_BGR8_Packed) assert(color == cv::Vec3b(10, 20, 30));
    if (pixel == PixelType_Gvsp_Mono8) assert(color == cv::Vec3b(10, 10, 10));
    assert(first.sequence == 0 && second.sequence == 1 && second.timestamp > first.timestamp);
    assert(first.source_time_seconds == 0 && !camera.end_of_stream());
    assert(srm_mvs_mock_stat("free") == 2 && srm_mvs_mock_stat("timeout_ms") == 250);
    assert(srm_mvs_mock_stat("convert") == (pixel == PixelType_Gvsp_BGR8_Packed ? 0 : 2));
    camera.reset(); assert(camera.read(second) && second.sequence == 0);
  }
  for (const char * failure : {"read", "convert", "convert_length", "free", "metadata", "oversize", "lost_packet", "truncated"}) {
    reset(1, failure);
    if (std::string(failure) == "truncated") srm_mvs_mock_pixel(PixelType_Gvsp_BGR8_Packed);
    {
      FrameSource camera(config);
      srm::io::Frame frame;
      assert(!camera.read(frame));
      assert(!camera.end_of_stream() && !camera.last_error().empty() && frame.bgr.empty());
      assert(srm_mvs_mock_stat("free") == (std::string(failure) == "read" ? 0 : 1));
    }
    assert(srm_mvs_mock_stat("stop") == 1 && srm_mvs_mock_stat("close") == 1);
    assert(srm_mvs_mock_stat("destroy") == 1 && srm_mvs_mock_stat("finalize") == 1);
  }
}
}  // namespace

int main(int argc, const char * const argv[])
{
  assert(argc == 3 && srm::io::mvs_compiled());
  FrameSourceConfig config;
  config.type = FrameSourceType::mvs_camera;
  config.mvs.sdk_library = argv[1]; config.mvs.timeout_ms = 250;
  test_selection_and_lifecycle(config);
  test_parameters(config);
  test_frames(config);
  reset(1, "version");
  expect_error([&] { FrameSource camera(config); }, "4.1.2.2 is unsupported");
  assert(srm_mvs_mock_stat("initialize") == 0);
  reset();
  expect_error([&] { (void)srm::io::list_mvs_cameras(argv[2]); }, "missing SDK symbol");
  expect_error([&] { (void)srm::io::list_mvs_cameras("/nonexistent/srm_sdk.so"); }, "runtime unavailable");
  return 0;
}
