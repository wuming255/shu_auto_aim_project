#include "app/application_options.hpp"
#include "app/runtime_csv_logger.hpp"
#include "app/runtime_statistics.hpp"

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
using srm::app::RuntimeStatistics;
using srm::app::RuntimeTransportSnapshot;
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
    "--debug-dir", "/tmp/debug", "--summary", "/tmp/summary.txt", "--debug-every", "3",
    "--show-debug", "--start-paused", "--playback-fps", "12.5",
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
  assert(options.debug_directory == "/tmp/debug");
  assert(options.summary_path == "/tmp/summary.txt");
  assert(options.debug_every == 3);
  assert(options.show_debug && options.start_paused);
  assert(options.playback_fps == 12.5);

  const auto camera_options = parse({"--input", "camera", "--camera-index", "1"});
  assert(camera_options.frame_source.type == FrameSourceType::opencv_camera);
  assert(camera_options.frame_source.camera_index == 1);
  assert(!camera_options.show_debug && !camera_options.start_paused);
  assert(camera_options.playback_fps == 0.0);
}

void test_debug_window_validation()
{
  // 交互不能暂停实时控制；无窗口时也不能静默忽略播放参数。
  const std::vector<std::vector<std::string>> invalid_arguments{
    {"--input", "video", "--source", "x.avi", "--show-debug"},
    {"--input", "camera", "--dry-run", "--show-debug"},
    {"--start-paused"},
    {"--playback-fps", "12"},
    {"--playback-fps", "-1"},
    {"--playback-fps", "0.01"},
    {"--playback-fps", "1001"},
    {"--playback-fps", "nan"},
    {"--playback-fps", "inf"}};
  for (const auto & arguments : invalid_arguments) {
    bool rejected = false;
    try {
      (void)parse_dynamic(arguments);
    } catch (const std::invalid_argument &) {
      rejected = true;
    }
    assert(rejected);
  }

  const auto image = parse({"--input", "image", "--source", "x.png", "--dry-run", "--show-debug"});
  assert(image.show_debug && image.playback_fps == 0.0);
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

  bool zero_debug_interval = false;
  try {
    (void)parse({"--debug-every", "0"});
  } catch (const std::invalid_argument &) {
    zero_debug_interval = true;
  }
  assert(zero_debug_interval);

  const auto help = parse({"--help", "--input", "image"});
  assert(help.help);
}

void test_mvs_options()
{
  const auto selected = parse({"--input", "mvs", "--dry-run", "--mvs-serial", "SERIAL",
    "--mvs-library", "/test/sdk.so", "--exposure-us", "2000", "--gain", "0",
    "--camera-width", "1440", "--camera-height", "1080", "--camera-fps", "60",
    "--pixel-format", "BayerRG8", "--camera-timeout-ms", "250"});
  assert(selected.frame_source.type == FrameSourceType::mvs_camera);
  assert(selected.frame_source.mvs.serial_number == "SERIAL");
  assert(selected.frame_source.mvs.device_index == -1);
  assert(selected.frame_source.mvs.sdk_library == "/test/sdk.so");
  assert(selected.frame_source.mvs.exposure_us == 2000);
  assert(selected.frame_source.mvs.gain == 0);
  assert(selected.frame_source.mvs.pixel_format == "BayerRG8");
  assert(selected.frame_source.mvs.timeout_ms == 250);
  assert(selected.frame_source.width == 1440 && selected.frame_source.height == 1080);
  assert(selected.frame_source.fps == 60);
  const auto defaults = parse({"--input", "mvs"});
  assert(!defaults.frame_source.mvs.exposure_us && !defaults.frame_source.mvs.gain);
  assert(defaults.frame_source.mvs.pixel_format.empty());
  assert(defaults.frame_source.width == 0 && defaults.frame_source.fps == 0);
  assert(parse({"--list-cameras"}).list_cameras);
  assert(parse({"--camera-info", "--mvs-index", "0"}).camera_info);

  const std::vector<std::vector<std::string>> invalid{
    {"--input", "mvs", "--dry-run", "--show-debug"},
    {"--mvs-serial", "A", "--mvs-index", "0"}, {"--mvs-index", "-2"},
    {"--camera-timeout-ms", "0"}, {"--camera-timeout-ms", "10001"},
    {"--exposure-us", "0"}, {"--gain", "-1"}, {"--exposure-us", "nan"},
    {"--camera-fps", "-1"}, {"--camera-fps", "inf"}, {"--camera-width", "-1"},
    {"--camera-height", "2147483648"}, {"--list-cameras", "--camera-info"}};
  for (const auto & args : invalid) {
    bool rejected = false;
    try { (void)parse_dynamic(args); } catch (const std::invalid_argument &) { rejected = true; }
    assert(rejected);
  }

  const auto path = "/tmp/shu_auto_aim_mvs_config_" + std::to_string(::getpid()) + ".conf";
  {
    std::ofstream out(path);
    out << "input.type=mvs\ninput.width=1440\ninput.height=1080\ninput.fps=30\n"
        << "camera.mvs_serial=SERIAL\ncamera.mvs_index=-1\ncamera.mvs_library=/test/sdk.so\n"
        << "camera.exposure_us=1000\ncamera.gain=4\ncamera.pixel_format=BayerRG8\n"
        << "camera.timeout_ms=700\nruntime.dry_run=true\n";
  }
  const auto from_file = parse_dynamic({"--config", path});
  assert(from_file.frame_source.mvs.exposure_us == 1000 && from_file.frame_source.mvs.gain == 4);
  assert(from_file.frame_source.mvs.timeout_ms == 700 && from_file.frame_source.fps == 30);
  const auto overridden = parse_dynamic({"--config", path, "--exposure-us", "default",
    "--gain", "default", "--pixel-format", "default", "--camera-fps", "0"});
  assert(!overridden.frame_source.mvs.exposure_us && !overridden.frame_source.mvs.gain);
  assert(overridden.frame_source.mvs.pixel_format.empty() && overridden.frame_source.fps == 0);
  ::unlink(path.c_str());
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
           << "runtime.debug_dir=config_debug\n"
           << "runtime.summary=config_summary.txt\n"
           << "runtime.debug_every=5\n"
           << "runtime.max_frames=4\n"
           << "runtime.show_debug=true\n"
           << "runtime.start_paused=true\n"
           << "runtime.playback_fps=15\n"
           << "camera.intrinsics=700,701,320,240\n";
  }

  const auto overridden = parse_dynamic({
    "--config", path, "--source", "cli.png", "--no-loop", "--live", "--max-frames", "8",
    "--device", "/dev/cli", "--intrinsics", "800,801,320,240",
    "--debug-dir", "cli_debug", "--summary", "cli_summary.txt", "--debug-every", "2",
    "--no-show-debug", "--no-start-paused", "--playback-fps", "0"});
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
  assert(overridden.debug_directory == "cli_debug");
  assert(overridden.summary_path == "cli_summary.txt");
  assert(overridden.debug_every == 2);
  assert(!overridden.show_debug && !overridden.start_paused);
  assert(overridden.playback_fps == 0.0);

  const auto config_only = parse_dynamic({"--config", path});
  assert(config_only.dry_run);
  assert(config_only.frame_source.loop);
  assert(config_only.max_frames == 4);
  assert(config_only.fx == 700.0 && config_only.fy == 701.0);
  assert(config_only.debug_directory == "config_debug");
  assert(config_only.summary_path == "config_summary.txt");
  assert(config_only.debug_every == 5);
  assert(config_only.show_debug && config_only.start_paused);
  assert(config_only.playback_fps == 15.0);

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
  result.step_duration = std::chrono::microseconds(123);
  result.safety.state = SafetyState::aiming;
  result.safety.aim_command_valid = true;
  result.safety.command.gimbal = {12.0F, -2.0F};
  result.safety.reasons = {SafetyReason::fire_disabled};
  result.error = "comma,error";

  const RuntimeTransportSnapshot transport{4, 5, 6};
  logger.log(result, transport);
  assert(logger.rows_written() == 1);

  std::ifstream input(path);
  std::stringstream contents;
  contents << input.rdbuf();
  const auto text = contents.str();
  assert(text.find("frame_sequence,frame_read,end_of_stream") == 0);
  assert(text.find("step_duration_us,transport_sent_frame_count") != std::string::npos);
  assert(text.find("4,1,0,") != std::string::npos);
  assert(text.find(",123,4,5,6,2,tracking") != std::string::npos);
  assert(text.find("tracking") != std::string::npos);
  assert(text.find("fire_disabled") != std::string::npos);
  assert(text.find("\"comma,error\"") != std::string::npos);

  RuntimeCsvLogger disabled;
  disabled.log(result);
  assert(!disabled.enabled());
  assert(disabled.rows_written() == 0);
  ::unlink(path.c_str());
}

void test_runtime_csv_missing_values()
{
  const auto path = std::string("/tmp/shu_auto_aim_runtime_log_missing_") +
                    std::to_string(static_cast<long long>(::getpid())) + ".csv";
  RuntimeCsvLogger logger(path);
  RuntimeStepResult end;
  end.end_of_stream = true;
  end.step_duration = std::chrono::microseconds(77);
  end.safety.state = SafetyState::safe;
  end.safety.reasons = {SafetyReason::frame_missing};
  end.error = "frame source reached end of stream";
  logger.log(end, RuntimeTransportSnapshot{4, 5, 6});

  RuntimeStepResult failure;
  failure.frame_read = true;
  failure.frame.emplace();
  failure.frame->sequence = 9;
  failure.frame->source_time_seconds = 0.25;
  failure.safety.state = SafetyState::safe;
  failure.safety.reasons = {SafetyReason::processing_failed};
  failure.error = "processing failed";
  logger.log(failure);
  assert(logger.rows_written() == 2);

  std::ifstream input(path);
  // 这些固定测试行不含转义字段，检查缺失值不会移动后续诊断列。
  const auto read_fields = [&input] {
    std::string line;
    assert(static_cast<bool>(std::getline(input, line)));
    std::istringstream row(line);
    std::vector<std::string> fields;
    for (std::string field; std::getline(row, field, ',');) fields.push_back(field);
    return fields;
  };
  const auto header = read_fields();
  const auto eof_fields = read_fields();
  const auto failure_fields = read_fields();
  assert(header.size() == 21);
  assert(eof_fields.size() == header.size());
  assert(failure_fields.size() == header.size());
  assert(eof_fields[0].empty() && eof_fields[1] == "0" && eof_fields[2] == "1");
  assert(eof_fields[3].empty() && eof_fields[4].empty());
  assert(eof_fields[5] == "77" && eof_fields[6] == "4" && eof_fields[7] == "5" &&
         eof_fields[8] == "6");
  assert(failure_fields[0] == "9" && failure_fields[1] == "1" && failure_fields[2] == "0");
  assert(failure_fields[4] == "0.25");
  for (const auto & fields : {eof_fields, failure_fields}) {
    for (std::size_t index = 9; index <= 14; ++index) assert(fields[index].empty());
    assert(fields[15] == "safe");
    assert(fields[16] == "0" && fields[17] == "0" && fields[18] == "0");
  }
  assert(eof_fields[19] == "frame_missing" && eof_fields[20] == end.error);
  assert(failure_fields[19] == "processing_failed" && failure_fields[20] == failure.error);
  ::unlink(path.c_str());
}

void test_runtime_statistics()
{
  RuntimeStatistics statistics;

  RuntimeStepResult aiming;
  aiming.frame_read = true;
  aiming.step_duration = std::chrono::microseconds(10);
  aiming.pipeline.emplace();
  aiming.pipeline->selection.observations.resize(2);
  aiming.safety.state = SafetyState::aiming;
  statistics.observe(aiming, RuntimeTransportSnapshot{1, 2, 3});

  RuntimeStepResult failure;
  failure.error = "processing failed";
  failure.step_duration = std::chrono::microseconds(20);
  failure.safety.state = SafetyState::safe;
  statistics.observe(failure, RuntimeTransportSnapshot{4, 5, 6});

  RuntimeStepResult end;
  end.end_of_stream = true;
  end.error = "frame source reached end of stream";
  end.step_duration = std::chrono::microseconds(30);
  end.safety.state = SafetyState::safe;
  statistics.observe(end, RuntimeTransportSnapshot{7, 8, 9});

  const auto & snapshot = statistics.snapshot();
  assert(snapshot.step_count == 3);
  assert(snapshot.frame_count == 1);
  assert(snapshot.end_of_stream_count == 1);
  assert(snapshot.error_count == 1);
  assert(snapshot.candidate_count == 2);
  assert(snapshot.safe_count == 2);
  assert(snapshot.aiming_count == 1);
  assert(snapshot.fire_ready_count == 0);
  assert(snapshot.total_step_duration_us == 60);
  assert(snapshot.max_step_duration_us == 30);
  assert(snapshot.transport.sent_frame_count == 7);
  assert(snapshot.transport.received_frame_count == 8);
  assert(snapshot.transport.communication_error_count == 9);

  const auto summary = statistics.summary_text();
  assert(summary.find("step_count=3\n") != std::string::npos);
  assert(summary.find("candidate_count=2\n") != std::string::npos);
  assert(summary.find("average_step_duration_us=20") != std::string::npos);
  assert(summary.find("transport_sent_frame_count=7\n") != std::string::npos);
}

}  // namespace

int main()
{
  test_command_line_options();
  test_debug_window_validation();
  test_command_line_validation();
  test_mvs_options();
  test_config_file_and_cli_override();
  test_runtime_csv_logger();
  test_runtime_csv_missing_values();
  test_runtime_statistics();
  return 0;
}
