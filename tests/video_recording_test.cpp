#include "app/video_recorder.hpp"

#include <cassert>
#include <cmath>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <sys/resource.h>
#include <sys/wait.h>
#include <unistd.h>

namespace
{
using srm::app::VideoRecorder;
void require(bool value, const char * message) { if (!value) throw std::runtime_error(message); }
void expect_error(const std::function<void()> & operation)
{
  bool threw = false;
  try { operation(); } catch (const std::exception &) { threw = true; }
  require(threw, "expected recorder failure");
}

void unit_tests()
{
  char name[] = "/tmp/srm-recorder-XXXXXX";
  require(::mkdtemp(name) != nullptr, "mkdtemp failed");
  const std::filesystem::path directory(name);
  const auto path = [&](const char * file) { return (directory / file).string(); };
  const cv::Mat frame(48, 64, CV_8UC3, cv::Scalar(30, 70, 190));
  require(VideoRecorder::choose_fps(0, 25) == 25 && VideoRecorder::choose_fps(0, 0) == 30 &&
          VideoRecorder::choose_fps(60, 25) == 60, "record FPS choice failed");
  expect_error([&] { VideoRecorder bad(path("invalid.avi"), -1); });
  expect_error([&] { VideoRecorder bad(path("invalid.mp4"), 30); });
  { VideoRecorder empty(path("empty.avi"), 30); empty.close(); }
  require(!std::filesystem::exists(path("empty.avi")), "empty recording was published");
  {
    VideoRecorder recorder(path("good.avi"), 25);
    recorder.write(frame); recorder.write(frame);
    require(!std::filesystem::exists(path("good.avi")), "record published before finalization");
    recorder.close(); recorder.close();
    cv::VideoCapture capture(path("good.avi"));
    cv::Mat decoded;
    require(capture.read(decoded) && cv::norm(decoded, frame, cv::NORM_INF) < 8, "MJPG color mismatch");
    require(capture.read(decoded) && !capture.read(decoded), "MJPG frame count mismatch");
  }
  expect_error([&] { VideoRecorder duplicate(path("good.avi"), 30); });
  require(::symlink("missing-target", path("link.avi").c_str()) == 0, "symlink failed");
  expect_error([&] { VideoRecorder symlink(path("link.avi"), 30); });
  {
    VideoRecorder recorder(path("collision.avi"), 30);
    recorder.write(frame);
    { std::ofstream sentinel(path("collision.avi")); sentinel << "keep"; }
    expect_error([&] { recorder.close(); });
    std::ifstream sentinel(path("collision.avi")); std::string text; sentinel >> text;
    require(text == "keep", "finalization overwrote another file");
  }
  {
    VideoRecorder recorder(path("resize.avi"), 30);
    recorder.write(frame);
    expect_error([&] { recorder.write(cv::Mat(50, 64, CV_8UC3)); });
    expect_error([&] { recorder.close(); });
    require(!std::filesystem::exists(path("resize.avi")), "failed video was published");
  }
  {
    VideoRecorder recorder(path("odd.avi"), 30);
    expect_error([&] { recorder.write(cv::Mat(49, 63, CV_8UC3)); });
    expect_error([&] { recorder.close(); });
  }
  // Real file-size write failure: FFmpeg can log the failure without throwing.
  // Run in a child so process limits/signals do not affect the rest of CTest.
  const pid_t child = ::fork();
  require(child >= 0, "fork failed");
  if (child == 0) {
    std::signal(SIGXFSZ, SIG_IGN);
    const rlimit limit{1024, 1024};
    if (::setrlimit(RLIMIT_FSIZE, &limit) != 0) ::_exit(2);
    try {
      VideoRecorder recorder(path("limited.avi"), 30, "FFV1");
      cv::Mat noise(480, 640, CV_8UC3); cv::randu(noise, 0, 255);
      recorder.write(noise); recorder.close();
      ::_exit(3);
    } catch (const std::exception &) {
      ::_exit(std::filesystem::exists(path("limited.avi")) ? 4 : 0);
    }
  }
  int status = 0;
  require(::waitpid(child, &status, 0) == child && WIFEXITED(status) && WEXITSTATUS(status) == 0,
          "silent file-size write failure was not detected");
  std::filesystem::remove_all(directory);
}
}  // namespace

int main(int argc, char ** argv)
{
  try {
    if (argc == 1) { unit_tests(); return 0; }
    if (argc == 3 && std::string(argv[1]) == "--make-input") {
      cv::Mat input(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
      cv::rectangle(input, {240, 160, 8, 60}, {0, 0, 255}, cv::FILLED);
      cv::rectangle(input, {360, 160, 8, 60}, {0, 0, 255}, cv::FILLED);
      require(cv::imwrite(argv[2], input), "cannot create synthetic input"); return 0;
    }
    require(argc == 6, "expected VIDEO COUNT FPS RAW_IMAGE DEBUG_IMAGE");
    cv::VideoCapture video(argv[1]);
    require(video.isOpened(), "cannot reopen recording");
    require(std::abs(video.get(cv::CAP_PROP_FPS) - std::stod(argv[3])) < 0.1, "wrong video FPS");
    const auto raw = cv::imread(argv[4]);
    const auto debug = cv::imread(argv[5]);
    require(!raw.empty() && !debug.empty() && cv::norm(raw, debug, cv::NORM_INF) > 0,
            "fixture did not produce a detection overlay");
    cv::Mat frame;
    int count = 0;
    while (video.read(frame)) {
      require(frame.size() == raw.size() && cv::norm(frame, raw, cv::NORM_INF) == 0,
              "recorded image is not the raw lossless BGR input");
      ++count;
    }
    require(count == std::stoi(argv[2]), "wrong decoded frame count");
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n'; return 1;
  }
}
