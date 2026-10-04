#include "app/application_options.hpp"
#include "app/runtime_csv_logger.hpp"

#include <cassert>
#include <chrono>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <unistd.h>

namespace
{
using srm::app::ApplicationOptions;
using srm::app::RuntimeCsvLogger;
using srm::control::RuntimeStepResult;
using srm::control::SafetyReason;
using srm::control::SafetyState;
using srm::io::FrameSourceType;

ApplicationOptions parse(std::initializer_list<const char *> arguments)
{
  std::vector<const char *> argv;
  argv.push_back("test_app");
  argv.insert(argv.end(), arguments.begin(), arguments.end());
  return srm::app::parse_arguments(static_cast<int>(argv.size()), argv.data());
}

ApplicationOptions parse_dynamic(const std::vector<std::string> & arguments)
{
  std::vector<const char *> argv;
  argv.push_back("test_app");
  for (const auto & argument : arguments) argv.push_back(argument.c_str());
  return srm::app::parse_arguments(static_cast<int>(argv.size()), argv.data());
}

void test_command_line_options()
{
  const auto options = parse({
    "--input", "video", "--source", "input.mp4", "--device", "/dev/test",
    "--camera-index", "2", "--loop", "--dry-run", "--max-frames", "7",
    "--intrinsics", "800,801,320,240", "--log", "/tmp/run.csv",
    "--transmit-period-ms", "15", "--io-timeout-ms", "22", "--reconnect-ms", "33"});

  assert(options.frame_source.type == FrameSourceType::video_file);
  assert(options.frame_source.uri == "input.mp4");
  assert(options.frame_source.camera_index == 2);
  assert(options.frame_source.loop);
  assert(options.dry_run);
  assert(options.max_frames == 7);
  assert(options.transport.device == "/dev/test");
  assert(options.transport.transmit_period == std::chrono::milliseconds(15));
  assert(options.transport.io_timeout == std::chrono::milliseconds(22));
  assert(options.transport.reconnect_interval == std::chrono::milliseconds(33));
  assert(options.intrinsics_set);
  assert(options.fx == 800.0 && options.fy == 801.0);
  assert(options.cx == 320.0 && options.cy == 240.0);
  assert(options.log_path == "/tmp/run.csv");

  const auto camera_options = parse({"--input", "camera", "--camera-index", "1"});
  assert(camera_options.frame_source.type == FrameSourceType::opencv_camera);
  assert(camera_options.frame_source.camera_index == 1);
}

void test_command_line_validation()
{
  bool missing_source = false;
  try {
    (void)parse({"--input", "image"});
  } catch (const std::invalid_argument &) {
    missing_source = true;
  }
  assert(missing_source);

  bool bad_intrinsics = false;
  try {
    (void)parse({"--input", "image", "--source", "x.png", "--intrinsics", "0,1,2,3"});
  } catch (const std::invalid_argument &) {
    bad_intrinsics = true;
  }
  assert(bad_intrinsics);

  bool unknown_option = false;
  try {
    (void)parse({"--not-an-option"});
  } catch (const std::invalid_argument &) {
    unknown_option = true;
  }
  assert(unknown_option);

  const auto help = parse({"--help", "--input", "image"});
  assert(help.help);
}


void test_config_file_and_cli_override()
{
  const auto path =
    std::string("/tmp/shu_auto_aim_application_config_") +
    std::to_string(static_cast<long long>(::getpid())) + ".conf";
  {
    std::ofstream output(path);
    output << "# offline configuration\n"
           << "input.type=image\n"
           << "input.source=config.png\n"
           << "input.loop=true\n"
           << "input.camera_index=3\n"
           << "transport.device=/dev/config\n"
           << "transport.transmit_period_ms=17\n"
           << "transport.io_timeout_ms=23\n"
           << "transport.reconnect_ms=31\n"
           << "runtime.dry_run=true\n"
           << "runtime.log=config.csv\n"
           << "runtime.max_frames=4\n"
           << "camera.intrinsics=700,701,320,240\n";
  }

  const auto overridden = parse_dynamic({
    "--config", path, "--source", "cli.png", "--no-loop", "--live", "--max-frames", "8",
    "--device", "/dev/cli", "--intrinsics", "800,801,320,240"});
  assert(overridden.config_path == path);
  assert(overridden.frame_source.type == FrameSourceType::image_file);
  assert(overridden.frame_source.uri == "cli.png");
  assert(!overridden.frame_source.loop);
  assert(overridden.frame_source.camera_index == 3);
  assert(!overridden.dry_run);
  assert(overridden.max_frames == 8);
  assert(overridden.transport.device == "/dev/cli");
  assert(overridden.transport.transmit_period == std::chrono::milliseconds(17));
  assert(overridden.transport.io_timeout == std::chrono::milliseconds(23));
  assert(overridden.transport.reconnect_interval == std::chrono::milliseconds(31));
  assert(overridden.intrinsics_set);
  assert(overridden.fx == 800.0 && overridden.fy == 801.0);
  assert(overridden.log_path == "config.csv");

  const auto config_only = parse_dynamic({"--config", path});
  assert(config_only.dry_run);
  assert(config_only.frame_source.loop);
  assert(config_only.max_frames == 4);
  assert(config_only.fx == 700.0 && config_only.fy == 701.0);

  bool unknown_key = false;
  {
    std::ofstream output(path);
    output << "not_a_supported_key=value\n";
  }
  try {
    (void)parse_dynamic({"--config", path});
  } catch (const std::invalid_argument &) {
    unknown_key = true;
  }
  assert(unknown_key);
  ::unlink(path.c_str());
}

void test_runtime_csv_logger()
{
  const auto path =
    std::string("/tmp/shu_auto_aim_runtime_log_") +
    std::to_string(static_cast<long long>(::getpid())) + ".csv";
  RuntimeCsvLogger logger(path);
  assert(logger.enabled());
  assert(logger.rows_written() == 0);

  RuntimeStepResult result;
  srm::io::Frame frame;
  frame.sequence = 4;
  frame.timestamp = std::chrono::steady_clock::time_point{} + std::chrono::milliseconds(12);
  frame.source_time_seconds = 0.25;
  result.frame_read = true;
  result.frame = frame;
  result.pipeline.emplace();
  result.pipeline->selection.observations.resize(2);
  result.pipeline->tracking.state = srm::vision::TargetTrackState::tracking;
  result.pipeline->aim.valid = true;
  result.pipeline->aim.yaw_deg = 12.0;
  result.pipeline->aim.pitch_deg = -2.0;
  result.pipeline->aim.bullet_speed_m_s = 20.0;
  result.safety.state = SafetyState::aiming;
  result.safety.aim_command_valid = true;
  result.safety.command.gimbal = {12.0F, -2.0F};
  result.safety.reasons = {SafetyReason::fire_disabled};
  result.error = "comma,error";

  logger.log(result);
  assert(logger.rows_written() == 1);

  std::ifstream input(path);
  std::stringstream contents;
  contents << input.rdbuf();
  const auto text = contents.str();
  assert(text.find("frame_sequence,frame_read,end_of_stream") == 0);
  assert(text.find("4,1,0,") != std::string::npos);
  assert(text.find("tracking") != std::string::npos);
  assert(text.find("fire_disabled") != std::string::npos);
  assert(text.find("\"comma,error\"") != std::string::npos);

  RuntimeCsvLogger disabled;
  disabled.log(result);
  assert(!disabled.enabled());
  assert(disabled.rows_written() == 0);
  ::unlink(path.c_str());
}

}  // namespace

int main()
{
  test_command_line_options();
  test_command_line_validation();
  test_config_file_and_cli_override();
  test_runtime_csv_logger();
  return 0;
}
