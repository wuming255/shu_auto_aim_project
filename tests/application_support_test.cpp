#include "app/application_options.hpp"
#include "app/runtime_csv_logger.hpp"
#include "app/runtime_statistics.hpp"
#include "app/playback_timing.hpp"
#include "app/tracking_debug.hpp"

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
     "--transmit-period-ms", "15", "--io-timeout-ms", "22", "--reconnect-ms", "33",
     "--command-max-age-ms", "44"});

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
  assert(options.transport.command_max_age == std::chrono::milliseconds(44));
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
    {"--input", "video", "--source", "x.avi", "--live", "--show-debug", "--start-paused"},
    {"--input", "image", "--source", "x.png", "--live", "--show-debug", "--playback-fps", "30"},
    {"--input", "mvs", "--live", "--show-debug", "--start-paused"},
    {"--input", "camera", "--live", "--show-debug", "--playback-fps", "30"},
    {"--input", "mvs", "--dry-run", "--show-debug", "--start-paused"},
    {"--input", "camera", "--dry-run", "--show-debug", "--playback-fps", "30"},
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
  for (const auto * type : {"mvs", "camera"}) {
    const auto realtime = parse({"--input", type, "--dry-run", "--show-debug"});
    assert(realtime.show_debug && !realtime.start_paused && realtime.playback_fps == 0.0);
  }
  for (const auto * type : {"mvs", "camera", "image", "video"}) {
    const auto live = parse({"--input", type, "--source", "input", "--live", "--show-debug"});
    assert(live.show_debug && !live.dry_run && !live.start_paused && live.playback_fps == 0.0);
  }
  const auto no_window = parse({"--input", "mvs", "--live", "--show-debug", "--no-show-debug"});
  assert(!no_window.show_debug && !no_window.dry_run);
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

void test_recording_options()
{
  const auto options = parse({"--input", "mvs", "--dry-run", "--record", "raw.avi",
                             "--record-fps", "25", "--record-codec", "FFV1", "--show-debug"});
  assert(options.record_path == "raw.avi" && options.record_fps == 25 && options.record_codec == "FFV1");
  assert(parse({"--record", "raw.avi", "--no-record"}).record_path.empty());
  const std::vector<std::vector<std::string>> invalid{
    {"--record", "raw.avi", "--live"}, {"--record-fps", "-1"}, {"--record-fps", "nan"},
    {"--record-fps", "0.01"}, {"--record-fps", "1001"}, {"--record-codec", "MJPEG"},
    {"--record-codec", "?bad"}, {"--record"},
    {"--dry-run", "--record", "./alias.avi", "--summary", "alias.avi"}};
  for (const auto & arguments : invalid) {
    bool rejected = false;
    try { (void)parse_dynamic(arguments); } catch (const std::invalid_argument &) { rejected = true; }
    assert(rejected);
  }
  const auto path = "/tmp/shu_auto_aim_record_config_" + std::to_string(::getpid()) + ".conf";
  {
    std::ofstream config(path);
    config << "input.type=mvs\nruntime.dry_run=true\nruntime.show_debug=true\n"
           << "runtime.record=raw.avi\nruntime.record_fps=60\nruntime.record_codec=MJPG\n";
  }
  const auto configured = parse_dynamic({"--config", path});
  assert(configured.show_debug && configured.record_path == "raw.avi" && configured.record_fps == 60);
  const auto overridden = parse_dynamic({"--config", path, "--no-record", "--record-fps", "0"});
  assert(overridden.record_path.empty() && overridden.record_fps == 0);
  ::unlink(path.c_str());
}

void test_center_pattern_options()
{
  assert(srm::app::usage("test_app").find("--no-center-pattern") != std::string::npos);
  assert(parse({}).require_center_pattern);
  assert(!parse({"--no-center-pattern"}).require_center_pattern);
  assert(parse({"--no-center-pattern", "--require-center-pattern"}).require_center_pattern);
  const auto path = "/tmp/shu_auto_aim_pattern_config_" + std::to_string(::getpid()) + ".conf";
  {
    std::ofstream out(path);
    out << "detector.require_center_pattern=false\n";
  }
  assert(!parse_dynamic({"--config", path}).require_center_pattern);
  assert(parse_dynamic({"--config", path, "--require-center-pattern"}).require_center_pattern);
  {
    std::ofstream out(path);
    out << "detector.require_center_pattern=invalid\n";
  }
  bool rejected = false;
  try { (void)parse_dynamic({"--config", path}); }
  catch (const std::invalid_argument &) { rejected = true; }
  assert(rejected);
  ::unlink(path.c_str());
}

void test_classifier_and_rotation_options()
{
  // 默认入口必须实际加载分类证据和独立旋转模型；现场未知量保持未确认。
  const auto defaults = parse({});
  assert(defaults.classifier.enabled && defaults.rotation.enabled);
  assert(!defaults.feature_size_m && !defaults.rotation.mounting_tilt_rad);
  assert(!defaults.rotation.assume_static_camera && !defaults.rotation.feedback_frame_confirmed);
  const auto path = "/tmp/shu_auto_aim_rotation_config_" + std::to_string(::getpid()) + ".conf";
  {
    std::ofstream out(path);
    out << "input.type=video\ninput.source=synthetic.avi\nruntime.dry_run=true\n"
        << "detector.classifier.enabled=true\ndetector.classifier.model=model.onnx\n"
        << "detector.classifier.labels=labels.txt\ndetector.classifier.min_confidence=0.8\n"
        << "detector.classifier.conflict_margin=0.06\n"
        << "pose.feature_width_m=0.135\npose.feature_height_m=0.055\n"
        << "rotation.enabled=true\nrotation.assume_static_camera=true\n"
        << "rotation.feedback_frame_confirmed=false\nrotation.mounting_tilt_rad=0.25\n"
        << "rotation.max_angular_speed_rad_s=10\nrotation.min_updates=10\n";
  }
  // 未接入配置时在此抛出 unknown configuration key。
  const auto configured = parse_dynamic({"--config", path});
  assert(configured.classifier.model_path == "model.onnx" && configured.classifier.label_path == "labels.txt");
  assert(configured.classifier.min_confidence == 0.8 && configured.classifier.conflict_margin == 0.06);
  assert(configured.feature_size_m == cv::Size2d(0.135,0.055));
  assert(configured.rotation.enabled && configured.rotation.assume_static_camera);
  assert(configured.rotation.mounting_tilt_rad == 0.25 && configured.rotation.min_updates == 10);
  assert(configured.rotation.max_angular_speed_rad_s == 10);
  const auto disabled = parse_dynamic({"--config", path, "--no-classifier", "--no-rotation"});
  assert(!disabled.classifier.enabled && !disabled.rotation.enabled);
  const std::vector<std::vector<std::string>> invalid{
    {"--input", "camera", "--dry-run", "--static-camera"},
    {"--input", "video", "--source", "x.avi", "--live", "--static-camera"},
    {"--feature-size", "0.14,0"}, {"--feature-size", "nan,0.05"},
    {"--classifier-confidence", "0.4"}, {"--rotation-tilt", "1.6"}};
  for (const auto & args : invalid) {
    bool rejected = false;
    try { (void)parse_dynamic(args); } catch (const std::invalid_argument &) { rejected = true; }
    assert(rejected);
  }
  ::unlink(path.c_str());
}

void test_brightness_options()
{
  using srm::vision::BrightnessMethod;
  const auto path = "/tmp/shu_brightness_config_" + std::to_string(::getpid()) + ".conf";
  {
    std::ofstream out(path);
    out << "detector.brightness_method=grayscale\ndetector.brightness_threshold=160\n";
  }
  const auto configured = parse_dynamic({"--config", path});
  assert(configured.brightness_method == BrightnessMethod::grayscale);
  assert(configured.brightness_threshold == 160);
  const auto overridden = parse_dynamic({"--config", path, "--brightness-method", "max_channel",
                                         "--brightness-threshold", "151.5"});
  assert(overridden.brightness_method == BrightnessMethod::max_channel);
  assert(overridden.brightness_threshold == 151.5);
  for (const auto * value : {"0", "255"}) {
    (void)parse_dynamic({"--brightness-method", "grayscale", "--brightness-threshold", value});
  }
  for (const auto & args : std::vector<std::vector<std::string>>{
         {"--brightness-method", "rgb"}, {"--brightness-method", ""},
         {"--brightness-threshold", "-1"}, {"--brightness-threshold", "256"},
         {"--brightness-threshold", "nan"}, {"--brightness-threshold", "inf"}}) {
    bool rejected = false;
    try { (void)parse_dynamic(args); }
    catch (const std::invalid_argument &) { rejected = true; }
    assert(rejected);
  }
  for (const auto * entry : {"detector.brightness_method=invalid\n", "detector.brightness_threshold=-1\n"}) {
    { std::ofstream out(path); out << entry; }
    bool rejected = false;
    try { (void)parse_dynamic({"--config", path}); }
    catch (const std::invalid_argument &) { rejected = true; }
    assert(rejected);
  }
  ::unlink(path.c_str());
}


void test_follow_and_fixed_speed_options()
{
  const auto path = "/tmp/shu_follow_config_" + std::to_string(::getpid()) + ".conf";
  {
    std::ofstream out(path);
    out << "aim.follow_target=true\naim.fixed_bullet_speed_m_s=22\n"
        << "rotation.feedback_frame_confirmed=true\n";
  }
  const auto configured = parse_dynamic({"--config",path});
  assert(configured.follow_target && configured.safety.fixed_bullet_speed_m_s == 22);
  const auto overridden = parse_dynamic({"--config",path,"--fixed-bullet-speed","referee","--no-follow-target"});
  assert(!overridden.follow_target && !overridden.safety.fixed_bullet_speed_m_s);
  assert(parse({"--follow-target","--fixed-bullet-speed","22"}).follow_target);
  for (const auto * speed : {"nan","0","-1","101"}) {
    bool rejected = false;
    try { (void)parse_dynamic({"--fixed-bullet-speed",speed}); }
    catch (const std::invalid_argument &) { rejected = true; }
    assert(rejected);
  }
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
           << "transport.command_max_age_ms=41\n"
           << "fixed_guard.relocalization_center_threshold_m=0.12\n"
           << "fixed_guard.relocalization_radius_threshold_m=0.06\n"
           << "fixed_guard.relocalization_yaw_threshold_rad=0.20\n"
           << "fixed_guard.hit_window_half_width_rad=0.09\n"
           << "fixed_guard.max_hold_seconds=0.20\n"
           << "safety.allow_visual_fire=true\n"
           << "safety.require_gimbal_feedback_for_fire=true\n"
           << "safety.max_gimbal_error_deg=0.7\n"
           << "safety.min_gimbal_settle_ms=60\n"
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
  assert(config_only.transport.command_max_age == std::chrono::milliseconds(41));
  assert(config_only.fixed_guard.relocalization_center_threshold_m == 0.12);
  assert(config_only.fixed_guard.relocalization_radius_threshold_m == 0.06);
  assert(config_only.fixed_guard.relocalization_yaw_threshold_rad == 0.20);
  assert(config_only.fixed_guard.hit_window_half_width_rad == 0.09);
  assert(config_only.fixed_guard.max_hold_seconds == 0.20);
  assert(config_only.safety.allow_visual_fire && config_only.safety.require_gimbal_feedback_for_fire);
  assert(config_only.safety.max_gimbal_error_deg == 0.7);
  assert(config_only.safety.min_gimbal_settle_time == std::chrono::milliseconds(60));
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
  frame.motion_timestamp = std::chrono::steady_clock::time_point{} + std::chrono::milliseconds(250);
  frame.motion_time_source = "video_nominal_fps";
  result.frame_read = true;
  result.frame = frame;
  result.pipeline.emplace();
  result.pipeline->selection.observations.resize(2);
  result.pipeline->tracking.state = srm::vision::TargetTrackState::tracking;
  result.pipeline->tracking.status = "tracking_four_plates";
  result.pipeline->tracking.has_current_measurement = true;
  result.pipeline->tracking.current_observation.emplace();
  auto & associated = *result.pipeline->tracking.current_observation;
  associated.detection_index = 1;
  associated.armor.classification = {"2",0.875,{}};
  associated.armor.points = {{{10,20},{30,20},{30,40},{10,40}}};
  result.pipeline->rotation.status = "tracking_four_plates";
  result.pipeline->rotation.initialized = true;
  result.pipeline->rotation.reliable = true;
  result.pipeline->rotation.prediction_valid = true;
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
  result.feedback = srm::io::TimedIncomingState{};
  // 最新反馈可在读图之后到达，年龄必须相对于决策时刻计算。
  result.feedback->timestamp = frame.timestamp + std::chrono::milliseconds(1);
  result.decision_timestamp = frame.timestamp + std::chrono::milliseconds(3);
  result.feedback->state.gimbal = {350,-10,0,1,3};
  result.feedback->state.shoot.bullet_speed = 0;
  result.bullet_speed_overridden = true;

  const RuntimeTransportSnapshot transport{4, 5, 6};
  logger.log(result, transport);
  assert(logger.rows_written() == 1);
  result.bullet_speed_overridden = false;
  result.pipeline->aim.bullet_speed_m_s = 22.0;
  logger.log(result, transport);
  result.feedback->state.shoot.bullet_speed = 30.0F;
  result.pipeline->aim.bullet_speed_m_s = 30.0;
  logger.log(result, transport);
  assert(logger.rows_written() == 3);

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
  assert(text.find("error,motion_time_us,motion_time_source,timeline_reset,track_status,") != std::string::npos);
  assert(text.find("\"comma,error\",250000,video_nominal_fps,0,tracking_four_plates,1,2,0.875,10,20,30,20,30,40,10,40,") != std::string::npos);
  assert(text.find(",tracking_four_plates,1,1,1,") != std::string::npos);
  assert(text.find(",12,-2,350,-10,0,0,2,fixed,\n") != std::string::npos);
  assert(text.find(",12,-2,350,-10,0,0,2,default,\n") != std::string::npos);
  assert(text.find(",12,-2,350,-10,0,30,2,referee,\n") != std::string::npos);

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
  failure.frame->motion_time_source = "video_position";
  failure.safety.state = SafetyState::safe;
  failure.safety.reasons = {SafetyReason::processing_failed};
  failure.error = "processing failed";
  logger.log(failure);

  // A valid source time of zero must remain distinguishable from a missing time.
  RuntimeStepResult restart;
  restart.frame_read = true;
  restart.frame.emplace();
  restart.frame->sequence = 10;
  restart.frame->timestamp = std::chrono::steady_clock::time_point{} + std::chrono::milliseconds(120);
  restart.frame->motion_timestamp = std::chrono::steady_clock::time_point{};
  restart.frame->motion_time_source = "video_nominal_fps";
  restart.frame->timeline_reset = true;
  logger.log(restart);

  RuntimeStepResult host_time;
  host_time.frame_read = true;
  host_time.frame.emplace();
  host_time.frame->timestamp = std::chrono::steady_clock::time_point{} + std::chrono::milliseconds(150);
  host_time.frame->motion_timestamp = host_time.frame->timestamp;
  host_time.pipeline.emplace();
  host_time.pipeline->tracking.status = "no_observation";
  host_time.pipeline->tracking.last_observation.emplace(); // 历史观测不得写入 current 列。
  logger.log(host_time);
  assert(logger.rows_written() == 4);

  std::ifstream input(path);
  // 这些固定测试行不含转义字段，检查缺失值不会移动后续诊断列。
  const auto read_fields = [&input] {
    std::string line;
    assert(static_cast<bool>(std::getline(input, line)));
    std::istringstream row(line);
    std::vector<std::string> fields;
    for (std::string field; std::getline(row, field, ',');) fields.push_back(field);
    if (!line.empty() && line.back() == ',') fields.emplace_back();
    return fields;
  };
  const auto header = read_fields();
  const auto eof_fields = read_fields();
  const auto failure_fields = read_fields();
  const auto restart_fields = read_fields();
  const auto host_fields = read_fields();
  assert(header.size() == 65);
  assert(eof_fields.size() == header.size());
  assert(failure_fields.size() == header.size());
  assert(restart_fields.size() == header.size());
  assert(host_fields.size() == header.size());
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
  assert(eof_fields[21].empty() && eof_fields[22].empty() && eof_fields[23].empty());
  assert(failure_fields[21].empty() && failure_fields[22] == "video_position" && failure_fields[23] == "0");
  assert(restart_fields[3] == "120000" && restart_fields[21] == "0");
  assert(restart_fields[22] == "video_nominal_fps" && restart_fields[23] == "1");
  assert(host_fields[3] == "150000" && host_fields[21] == "150000");
  assert(host_fields[22] == "host_arrival" && host_fields[23] == "0");
  assert(host_fields[24] == "no_observation");
  for (std::size_t index = 25; index <= 35; ++index) assert(host_fields[index].empty());
  for (std::size_t index = 42; index <= 49; ++index) assert(host_fields[index].empty());
  for (std::size_t index = 24; index < 56; ++index) {
    assert(eof_fields[index].empty() && failure_fields[index].empty());
  }
  ::unlink(path.c_str());
}

void test_tracking_drawing_uses_current_association()
{
  cv::Mat image = cv::Mat::zeros(240,640,CV_8UC3);
  srm::control::AutoAimFrameResult result;
  result.selection.observations.resize(2);
  result.selection.selected_index = 0;
  result.tracking.has_current_measurement = true;
  result.tracking.current_observation.emplace();
  auto & observation = *result.tracking.current_observation;
  observation.detection_index = 1;
  observation.armor.points = {{{30,120},{90,120},{90,170},{30,170}}};
  result.tracking.last_observation = observation;
  srm::vision::PoseSolverConfig pose;
  auto visible = srm::app::draw_tracking_debug(image,result,pose);
  assert(visible.at<cv::Vec3b>(145,30) == cv::Vec3b(255,0,255));
  assert(image.at<cv::Vec3b>(145,30) == cv::Vec3b(0,0,0));
  result.tracking.has_current_measurement = false;
  result.tracking.current_observation.reset();
  result.tracking.state = srm::vision::TargetTrackState::temporarily_lost;
  auto lost = srm::app::draw_tracking_debug(image,result,pose);
  assert(lost.at<cv::Vec3b>(145,30) == cv::Vec3b(0,0,0));
  auto & rotation = result.rotation;
  rotation.initialized = true;
  rotation.stable_frame_valid = true;
  rotation.prediction_valid = true;
  rotation.center_world_m = {5,0,0}; rotation.radius_m = 0.25;
  pose.camera_matrix = {800,0,320, 0,800,150, 0,0,1};
  const auto predicted = srm::app::draw_tracking_debug(image,result,pose);
  assert(predicted.at<cv::Vec3b>(150,320) == cv::Vec3b(255,255,0));
  rotation.prediction_valid = false;
  const auto invalid = srm::app::draw_tracking_debug(image,result,pose);
  assert(invalid.at<cv::Vec3b>(150,320) == cv::Vec3b(0,0,0));
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
  const auto now = std::chrono::steady_clock::now();
  assert(srm::app::playback_wait_ms(false, now - std::chrono::milliseconds(20), now) == 1);
  assert(srm::app::playback_wait_ms(false, now + std::chrono::milliseconds(3), now) == 3);
  assert(srm::app::playback_wait_ms(false, now + std::chrono::milliseconds(50), now) == 10);
  assert(srm::app::playback_wait_ms(true, now - std::chrono::milliseconds(20), now) == 10);
  test_command_line_options();
  test_debug_window_validation();
  test_command_line_validation();
  test_mvs_options();
  test_recording_options();
  test_center_pattern_options();
  test_classifier_and_rotation_options();
  test_brightness_options();
  test_config_file_and_cli_override();
  test_follow_and_fixed_speed_options();
  test_runtime_csv_logger();
  test_runtime_csv_missing_values();
  test_tracking_drawing_uses_current_association();
  test_runtime_statistics();
  return 0;
}
