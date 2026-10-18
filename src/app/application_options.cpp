#include "app/application_options.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

namespace srm::app
{

namespace   // 匿名命名空间，表示这里面的东西只在这个文件中可见
{

std::string trim(const std::string & text)    // 去掉字符串前后的空白字符
{
  const auto first = std::find_if_not(text.begin(), text.end(), [](unsigned char character) {
    return std::isspace(character) != 0;
  });         // 第三个参数是一个匿名函数，判断字符是否为空白字符
  if (first == text.end()) return {};

  const auto last = std::find_if_not(text.rbegin(), text.rend(), [](unsigned char character) {
    return std::isspace(character) != 0;
  }).base();        // base()将反向迭代器换成正向的
  return std::string(first, last);
}

double parse_double(const std::string & text, const char * option)  // 将字符串转换为double类型
{
  std::size_t consumed = 0;
  double value = 0.0;
  try {
    value = std::stod(text, &consumed);
  } catch (const std::exception &) {
    throw std::invalid_argument(std::string("invalid value for ") + option + ": " + text);
  }
  if (consumed != text.size() || !std::isfinite(value)) {
    throw std::invalid_argument(std::string("invalid value for ") + option + ": " + text);
  }
  return value;
}

long long parse_integer(const std::string & text, const char * option)  // 将字符串转换为整数类型
{
  std::size_t consumed = 0;
  long long value = 0;
  try {
    value = std::stoll(text, &consumed, 10);
  } catch (const std::exception &) {
    throw std::invalid_argument(std::string("invalid integer for ") + option + ": " + text);
  }
  if (consumed != text.size()) {
    throw std::invalid_argument(std::string("invalid integer for ") + option + ": " + text);
  }
  return value;
}

bool parse_bool(const std::string & text, const char * option)  // 将字符串转换为布尔类型
{
  if (text == "1" || text == "true" || text == "TRUE" || text == "yes" || text == "on") {
    return true;
  }
  if (text == "0" || text == "false" || text == "FALSE" || text == "no" || text == "off") {
    return false;
  }
  throw std::invalid_argument(std::string("invalid boolean for ") + option + ": " + text);
}

std::string require_value(int & index, int argc, const char * const argv[], const char * option)    // 读入参数值
{
  if (index + 1 >= argc) {
    throw std::invalid_argument(std::string("missing value for ") + option);
  }
  ++index;
  return argv[index];
}

io::FrameSourceType parse_input_type(const std::string & value, const char * option)  // 读取输入源类型
{
  if (value == "image") return io::FrameSourceType::image_file;
  if (value == "video") return io::FrameSourceType::video_file;
  if (value == "camera") return io::FrameSourceType::opencv_camera;
  if (value == "mvs") return io::FrameSourceType::mvs_camera;
  throw std::invalid_argument(std::string(option) + " must be image, video, camera or mvs");
}

void parse_intrinsics(ApplicationOptions & options, const std::string & text, const char * option)  // 读取相机内参
{
  std::stringstream stream(text);
  std::string field;
  double values[4]{};       // {}是初始化，里面没填就默认初始化为0
  for (int index = 0; index < 4; ++index) {
    if (!std::getline(stream, field, ',')) {
      throw std::invalid_argument(std::string(option) + " expects fx,fy,cx,cy");
    }
    values[index] = parse_double(trim(field), option);
  }
  if (std::getline(stream, field, ',')) {
    throw std::invalid_argument(std::string(option) + " expects exactly fx,fy,cx,cy");
  }
  if (values[0] <= 0.0 || values[1] <= 0.0) {
    throw std::invalid_argument(std::string(option) + " requires positive fx and fy");
  }
  options.intrinsics_set = true;
  options.fx = values[0];
  options.fy = values[1];
  options.cx = values[2];
  options.cy = values[3];
}

void set_camera_index(ApplicationOptions & options, const std::string & text, const char * option)  // 读取相机索引编号
{
  const auto value = parse_integer(text, option);
  if (value < 0 || value > std::numeric_limits<int>::max()) {
    throw std::invalid_argument(std::string(option) + " is outside the supported range");
  }
  options.frame_source.camera_index = static_cast<int>(value);
}

void set_positive_duration(
  std::chrono::milliseconds & destination, const std::string & text, const char * option)
{
  const auto value = parse_integer(text, option);
  if (value <= 0 || value > std::numeric_limits<int>::max()) {
    throw std::invalid_argument(std::string(option) + " must be positive");
  }
  destination = std::chrono::milliseconds(value);
}

void set_max_frames(ApplicationOptions & options, const std::string & text, const char * option)  // 读取最大处理帧数
{
  const auto value = parse_integer(text, option);
  if (value < 0) throw std::invalid_argument(std::string(option) + " must not be negative");
  options.max_frames = static_cast<std::size_t>(value);
}

void set_debug_every(ApplicationOptions & options, const std::string & text, const char * option)   //读取调试图保存时间间隔
{
  const auto value = parse_integer(text, option);
  if (value <= 0) throw std::invalid_argument(std::string(option) + " must be positive");
  options.debug_every = static_cast<std::size_t>(value);
}

void set_playback_fps(ApplicationOptions & options, const std::string & text, const char * option)
{
  const auto value = parse_double(text, option);
  if (value != 0.0 && (value < 0.1 || value > 1000.0)) {
    throw std::invalid_argument(std::string(option) + " must be 0 (auto) or within [0.1, 1000]");
  }
  options.playback_fps = value;
}

void set_camera_integer(int & destination, const std::string & text, const char * option, int minimum = 0)
{
  const auto value = parse_integer(text, option);
  if (value < minimum || value > std::numeric_limits<int>::max()) {
    throw std::invalid_argument(std::string(option) + " is outside the supported range");
  }
  destination = static_cast<int>(value);
}

void set_camera_float(std::optional<double> & destination, const std::string & text, const char * option)
{
  if (text == "default") destination.reset();
  else destination = parse_double(text, option);
}

bool apply_camera_entry(ApplicationOptions & options, const std::string & key,
                        const std::string & value, const char * context)
{
  auto & source = options.frame_source;
  if (key == "camera.rotation_deg" || key == "input.rotation_deg" || key == "input.orientation_deg") {
    const auto parsed = parse_integer(value, context);
    if (parsed < 0 || parsed > 270) throw std::invalid_argument("invalid rotation/orientation");
    io::validate_rotation(static_cast<int>(parsed));
    if (key == "camera.rotation_deg") source.camera_rotation_deg = static_cast<int>(parsed);
    else if (key == "input.rotation_deg") source.file_rotation_deg = static_cast<int>(parsed);
    else source.file_orientation_deg = static_cast<int>(parsed);
  }
  else if (key == "camera.calibration") options.calibration_path = value;
  else if (key == "camera.extrinsics") options.extrinsics_path = value;
  else if (key == "input.width") set_camera_integer(source.width, value, context);
  else if (key == "input.height") set_camera_integer(source.height, value, context);
  else if (key == "input.fps") source.fps = parse_double(value, context);
  else if (key == "camera.mvs_serial") source.mvs.serial_number = value;
  else if (key == "camera.mvs_index") set_camera_integer(source.mvs.device_index, value, context, -1);
  else if (key == "camera.mvs_library") source.mvs.sdk_library = value;
  else if (key == "camera.exposure_us") set_camera_float(source.mvs.exposure_us, value, context);
  else if (key == "camera.gain") set_camera_float(source.mvs.gain, value, context);
  else if (key == "camera.pixel_format") source.mvs.pixel_format = value == "default" ? "" : value;
  else if (key == "camera.timeout_ms") {
    const auto parsed = parse_integer(value, context);
    if (parsed < 1 || parsed > 10000) throw std::invalid_argument(std::string(context) + " must be within [1, 10000]");
    source.mvs.timeout_ms = static_cast<unsigned int>(parsed);
  } else return false;
  return true;
}

bool apply_vision_entry(ApplicationOptions & options, const std::string & key,
                        const std::string & value, const char * context)
{
  auto & classifier = options.classifier;
  auto & rotation = options.rotation;
  if (key == "aim.follow_target") options.follow_target = parse_bool(value, context);
  else if (key == "aim.fixed_bullet_speed_m_s") {
    if (value == "referee") options.safety.fixed_bullet_speed_m_s.reset();
    else options.safety.fixed_bullet_speed_m_s = parse_double(value, context);
  }
  else if (key == "detector.classifier.enabled") classifier.enabled = parse_bool(value, context);
  else if (key == "detector.classifier.model") classifier.model_path = value;
  else if (key == "detector.classifier.labels") classifier.label_path = value;
  else if (key == "detector.classifier.min_confidence") classifier.min_confidence = parse_double(value, context);
  else if (key == "detector.classifier.conflict_margin") classifier.conflict_margin = parse_double(value, context);
  else if (key == "detector.brightness_method") {
    if (value == "grayscale") options.brightness_method = vision::BrightnessMethod::grayscale;
    else if (value == "max_channel") options.brightness_method = vision::BrightnessMethod::max_channel;
    else throw std::invalid_argument(std::string(context) + " must be max_channel or grayscale");
  }
  else if (key == "detector.brightness_threshold") options.brightness_threshold = parse_double(value, context);
  else if (key == "pose.feature_width_m" || key == "pose.feature_height_m") {
    if (!options.feature_size_m) options.feature_size_m = cv::Size2d{};
    const auto parsed = parse_double(value, context);
    if (key == "pose.feature_width_m") options.feature_size_m->width = parsed;
    else options.feature_size_m->height = parsed;
  }
  else if (key == "rotation.enabled") rotation.enabled = parse_bool(value, context);
  else if (key == "rotation.assume_static_camera") rotation.assume_static_camera = parse_bool(value, context);
  else if (key == "rotation.feedback_frame_confirmed") rotation.feedback_frame_confirmed = parse_bool(value, context);
  else if (key == "rotation.mounting_tilt_rad") {
    if (value == "unknown") rotation.mounting_tilt_rad.reset();
    else rotation.mounting_tilt_rad = parse_double(value, context);
  }
  else if (key == "rotation.min_updates") {
    const auto parsed = parse_integer(value, context);
    if (parsed < 3 || parsed > 512) throw std::invalid_argument("rotation.min_updates must be within [3,512]");
    rotation.min_updates = static_cast<std::size_t>(parsed);
  } else {
    const std::pair<const char *, double *> limits[] = {
      {"rotation.min_radius_m", &rotation.min_radius_m},
      {"rotation.max_radius_m", &rotation.max_radius_m},
      {"rotation.fit_window_seconds", &rotation.fit_window_seconds},
      {"rotation.short_loss_seconds", &rotation.short_loss_seconds},
      {"rotation.long_loss_seconds", &rotation.long_loss_seconds},
      {"rotation.max_position_residual_m", &rotation.max_position_residual_m},
      {"rotation.max_phase_residual_rad", &rotation.max_phase_residual_rad},
      {"rotation.max_tilt_error_rad", &rotation.max_tilt_error_rad},
      {"rotation.max_angular_speed_rad_s", &rotation.max_angular_speed_rad_s},
      {"rotation.min_normal_span_rad", &rotation.min_normal_span_rad}};
    for (const auto & entry : limits) {
      if (key == entry.first) { *entry.second = parse_double(value, context); return true; }
    }
    if (key == "fixed_guard.relocalization_center_threshold_m") {
      options.fixed_guard.relocalization_center_threshold_m = parse_double(value, context);
      return true;
    }
    if (key == "fixed_guard.relocalization_radius_threshold_m") {
      options.fixed_guard.relocalization_radius_threshold_m = parse_double(value, context);
      return true;
    }
    if (key == "fixed_guard.relocalization_yaw_threshold_rad") {
      options.fixed_guard.relocalization_yaw_threshold_rad = parse_double(value, context);
      return true;
    }
    if (key == "fixed_guard.hit_window_half_width_rad") {
      options.fixed_guard.hit_window_half_width_rad = parse_double(value, context);
      return true;
    }
    if (key == "fixed_guard.max_hold_seconds") {
      options.fixed_guard.max_hold_seconds = parse_double(value, context);
      return true;
    }
    if (key == "safety.allow_visual_fire") {
      options.safety.allow_visual_fire = parse_bool(value, context);
      return true;
    }
    if (key == "safety.require_gimbal_feedback_for_fire") {
      options.safety.require_gimbal_feedback_for_fire = parse_bool(value, context);
      return true;
    }
    if (key == "safety.max_gimbal_error_deg") {
      options.safety.max_gimbal_error_deg = parse_double(value, context);
      return true;
    }
    if (key == "safety.min_gimbal_settle_ms") {
      set_positive_duration(options.safety.min_gimbal_settle_time, value, context);
      return true;
    }
    return false;
  }
  return true;
}

void apply_config_entry(
  ApplicationOptions & options, const std::string & key, const std::string & value,
  std::size_t line_number)
{
  const std::string context = "config line " + std::to_string(line_number) + " (" + key + ")";
  if (apply_camera_entry(options, key, value, context.c_str())) return;
  if (apply_vision_entry(options, key, value, context.c_str())) return;
  if (key == "input.type") {
    options.frame_source.type = parse_input_type(value, context.c_str());
  } else if (key == "input.source") {
    options.frame_source.uri = value;
  } else if (key == "input.camera_index") {
    set_camera_index(options, value, context.c_str());
  } else if (key == "input.loop") {
    options.frame_source.loop = parse_bool(value, context.c_str());
  } else if (key == "transport.device") {
    if (value.empty()) throw std::invalid_argument(context + " must not be empty");
    options.transport.device = value;
  } else if (key == "transport.transmit_period_ms") {
    set_positive_duration(options.transport.transmit_period, value, context.c_str());
  } else if (key == "transport.io_timeout_ms") {
    set_positive_duration(options.transport.io_timeout, value, context.c_str());
  } else if (key == "transport.reconnect_ms") {
    set_positive_duration(options.transport.reconnect_interval, value, context.c_str());
  } else if (key == "transport.command_max_age_ms") {
    set_positive_duration(options.transport.command_max_age, value, context.c_str());
  } else if (key == "runtime.dry_run") {
    options.dry_run = parse_bool(value, context.c_str());
  } else if (key == "runtime.log") {
    options.log_path = value;
  } else if (key == "runtime.debug_dir") {
    options.debug_directory = value;
  } else if (key == "runtime.summary") {
    options.summary_path = value;
  } else if (key == "runtime.max_frames") {
    set_max_frames(options, value, context.c_str());
  } else if (key == "runtime.debug_every") {
    set_debug_every(options, value, context.c_str());
  } else if (key == "runtime.show_debug") {
    options.show_debug = parse_bool(value, context.c_str());
  } else if (key == "runtime.start_paused") {
    options.start_paused = parse_bool(value, context.c_str());
  } else if (key == "runtime.playback_fps") {
    set_playback_fps(options, value, context.c_str());
  } else if (key == "runtime.record") {
    options.record_path = value;
  } else if (key == "runtime.record_fps") {
    options.record_fps = parse_double(value, context.c_str());
  } else if (key == "runtime.record_codec") {
    options.record_codec = value;
  } else if (key == "detector.require_center_pattern") {
    options.require_center_pattern = parse_bool(value, context.c_str());
  } else if (key == "camera.intrinsics") {
    parse_intrinsics(options, value, context.c_str());
  } else {
    throw std::invalid_argument("unknown configuration key at line " +
                                std::to_string(line_number) + ": " + key);
  }
}

void validate_options(const ApplicationOptions & options)     // 配置参数合法性检查
{
  if (options.help) return;

  if (options.list_cameras && options.camera_info) throw std::invalid_argument("choose --list-cameras or --camera-info");
  io::validate_mvs_config(options.frame_source.mvs);
  if (options.frame_source.width < 0 || options.frame_source.height < 0 ||
      options.frame_source.fps < 0 || !std::isfinite(options.frame_source.fps)) {
    throw std::invalid_argument("camera dimensions/fps must be finite and non-negative");
  }
  if (options.list_cameras || options.camera_info) return;
  if (!options.calibration_path.empty() && options.intrinsics_set) {
    throw std::invalid_argument("choose full --calibration or legacy --intrinsics; do not combine them");
  }
  if (!options.extrinsics_path.empty() && options.calibration_path.empty()) {
    throw std::invalid_argument("--extrinsics requires full --calibration with resolution/orientation");
  }

  const auto type = options.frame_source.type;
  if ((type == io::FrameSourceType::image_file || type == io::FrameSourceType::video_file) &&
      options.frame_source.uri.empty()) {
    throw std::invalid_argument("--source is required for image and video input");
  }
  if (options.frame_source.camera_index < 0) {
    throw std::invalid_argument("camera index must not be negative");
  }
  if (options.transport.device.empty()) {
    throw std::invalid_argument("transport device must not be empty");
  }
  if (options.transport.transmit_period <= std::chrono::milliseconds::zero() ||
      options.transport.io_timeout <= std::chrono::milliseconds::zero() ||
      options.transport.reconnect_interval <= std::chrono::milliseconds::zero() ||
      options.transport.command_max_age <= std::chrono::milliseconds::zero()) {
    throw std::invalid_argument("transport timeouts and period must be positive");
  }
  if (options.debug_every == 0) {
    throw std::invalid_argument("debug frame interval must be positive");
  }
  const bool realtime = type == io::FrameSourceType::opencv_camera || type == io::FrameSourceType::mvs_camera;
  if (options.rotation.assume_static_camera && (realtime || !options.dry_run)) {
    throw std::invalid_argument("static camera assumption requires dry-run image/video input");
  }
  if (options.feature_size_m && (options.feature_size_m->width <= 0 || options.feature_size_m->height <= 0)) {
    throw std::invalid_argument("feature width and height must both be positive measured dimensions");
  }
  const auto & classifier = options.classifier;
  if (!std::isfinite(options.brightness_threshold) || options.brightness_threshold < 0.0 ||
      options.brightness_threshold > 255.0) {
    throw std::invalid_argument("brightness threshold must be within [0, 255]");
  }
  if (!std::isfinite(classifier.min_confidence) || classifier.min_confidence <= 0.5 ||
      classifier.min_confidence > 1 || !std::isfinite(classifier.conflict_margin) ||
      classifier.conflict_margin < 0 || classifier.conflict_margin > 1 ||
      (classifier.enabled && (classifier.model_path.empty() || classifier.label_path.empty()))) {
    throw std::invalid_argument("invalid number classifier paths or thresholds");
  }
  // 复用模型的参数约束，不加载相机、网络或串口。
  (void)vision::RotatingTarget(options.rotation);
  (void)control::FixedGuardController(options.fixed_guard);
  (void)control::SafetyController(options.safety);
  if ((!options.dry_run || realtime) && (options.start_paused || options.playback_fps != 0.0)) {
    throw std::invalid_argument("live/camera preview does not support --start-paused or --playback-fps");
  }
  if (!options.show_debug && (options.start_paused || options.playback_fps != 0.0)) {
    throw std::invalid_argument("--start-paused and --playback-fps require --show-debug");
  }
  if (!options.record_path.empty() && !options.dry_run) {
    throw std::invalid_argument("--record requires --dry-run (synchronous debug recording)");
  }
  if (!options.record_path.empty()) {
    const auto recording = std::filesystem::weakly_canonical(std::filesystem::absolute(options.record_path));
    for (const auto & output : {options.log_path, options.summary_path}) {
      if (!output.empty() && recording == std::filesystem::weakly_canonical(std::filesystem::absolute(output))) {
        throw std::invalid_argument("recording path conflicts with log/summary path");
      }
    }
  }
  // Loading calibration and recording metadata must never let a log truncate the input data.
  for (const auto & protected_path : {options.frame_source.uri, options.calibration_path, options.extrinsics_path,
         options.classifier.model_path, options.classifier.label_path,
         options.frame_source.uri.empty() ? std::string{} : options.frame_source.uri + ".metadata.yml",
         options.record_path.empty() ? std::string{} : options.record_path + ".metadata.yml"}) {
    if (protected_path.empty()) continue;
    for (const auto & output : {options.log_path, options.summary_path}) {
      if (output.empty()) continue;
      std::error_code error;
      const bool same_file = std::filesystem::equivalent(protected_path, output, error);
      if (same_file || std::filesystem::weakly_canonical(std::filesystem::absolute(protected_path)) ==
                       std::filesystem::weakly_canonical(std::filesystem::absolute(output))) {
        throw std::invalid_argument("log/summary path conflicts with input/calibration/metadata");
      }
    }
  }
  if (!std::isfinite(options.record_fps) || options.record_fps < 0.0 ||
      options.record_fps > 1000.0 || (options.record_fps > 0.0 && options.record_fps < 0.1)) {
    throw std::invalid_argument("--record-fps must be 0 (auto) or within [0.1, 1000]");
  }
  if (options.record_codec.size() != 4 ||
      !std::all_of(options.record_codec.begin(), options.record_codec.end(),
                   [](unsigned char c) { return std::isalnum(c); })) {
    throw std::invalid_argument("--record-codec must be a four-character codec such as MJPG or FFV1");
  }
}

ApplicationOptions default_options()    //初始化配置数据类
{
  ApplicationOptions options;
  options.frame_source.type = io::FrameSourceType::opencv_camera;
  options.transport.device = "/dev/ttyACM0";
  options.classifier.enabled = true;
  options.classifier.model_path = std::string(SRM_DEFAULT_NUMBER_MODEL_DIR) + "/mlp.onnx";
  options.classifier.label_path = std::string(SRM_DEFAULT_NUMBER_MODEL_DIR) + "/label.txt";
  options.rotation.enabled = true;
  return options;
}

}  // namespace的后括号

void load_config_file(const std::string & path, ApplicationOptions & options)   // 读取配置文件，将配置文件中的参数加载到配置数据类中
{
  if (path.empty()) throw std::invalid_argument("configuration path must not be empty");

  std::ifstream input(path);
  if (!input.is_open()) {
    throw std::invalid_argument("failed to open configuration file: " + path);
  }

  std::string line;
  std::size_t line_number = 0;
  while (std::getline(input, line)) {
    ++line_number;
    const auto trimmed = trim(line);
    if (trimmed.empty() || trimmed.front() == '#') continue;

    const auto separator = trimmed.find('=');
    if (separator == std::string::npos) {
      throw std::invalid_argument(
        "configuration line " + std::to_string(line_number) + " must contain '='");
    }
    const auto key = trim(trimmed.substr(0, separator));
    const auto value = trim(trimmed.substr(separator + 1));
    if (key.empty()) {
      throw std::invalid_argument("configuration line " + std::to_string(line_number) +
                                  " has an empty key");
    }
    apply_config_entry(options, key, value, line_number);
  }
  if (input.bad()) throw std::invalid_argument("failed while reading configuration file: " + path);
}

ApplicationOptions parse_arguments(int argc, const char * const argv[])   // 将命令行中提供的参数解析到配置数据类中
{
  if (argc <= 0 || argv == nullptr) throw std::invalid_argument("invalid argument vector");

  ApplicationOptions options = default_options();
  for (int index = 1; index < argc; ++index) {  // 读取配置文件路径，并加载配置文件中的参数到配置数据类中
    if (std::string(argv[index]) != "--config") continue;
    if (index + 1 >= argc) throw std::invalid_argument("missing value for --config");
    if (!options.config_path.empty()) {
      throw std::invalid_argument("--config may only be specified once");
    }
    options.config_path = argv[++index];
    if (options.config_path.empty()) throw std::invalid_argument("--config must not be empty");
  }
  if (!options.config_path.empty()) load_config_file(options.config_path, options);

  for (int index = 1; index < argc; ++index) {    // 根据命令行参数，一个一个设置参数
    const std::string argument = argv[index];
    if (argument == "--config") {
      ++index;
      continue;
    }
    if (argument == "--help" || argument == "-h") {
      options.help = true;
      continue;
    }
    if (argument == "--list-cameras" || argument == "--camera-info") {
      if (argument == "--list-cameras") options.list_cameras = true;
      else options.camera_info = true;
      continue;
    }
    const std::pair<const char *, const char *> camera_options[] = {
      {"--camera-width", "input.width"}, {"--camera-height", "input.height"},
      {"--camera-fps", "input.fps"}, {"--mvs-serial", "camera.mvs_serial"},
      {"--mvs-index", "camera.mvs_index"}, {"--mvs-library", "camera.mvs_library"},
      {"--camera-rotation", "camera.rotation_deg"}, {"--file-rotation", "input.rotation_deg"},
      {"--file-orientation", "input.orientation_deg"}, {"--calibration", "camera.calibration"},
      {"--extrinsics", "camera.extrinsics"},
      {"--exposure-us", "camera.exposure_us"}, {"--gain", "camera.gain"},
      {"--pixel-format", "camera.pixel_format"}, {"--camera-timeout-ms", "camera.timeout_ms"}};
    bool camera_option = false;
    for (const auto & entry : camera_options) {
      if (argument != entry.first) continue;
      (void)apply_camera_entry(options, entry.second,
        require_value(index, argc, argv, entry.first), entry.first);
      camera_option = true;
      break;
    }
    if (camera_option) continue;
    if (argument == "--classifier" || argument == "--no-classifier") {
      options.classifier.enabled = argument == "--classifier";
      continue;
    }
    if (argument == "--rotation" || argument == "--no-rotation") {
      options.rotation.enabled = argument == "--rotation";
      continue;
    }
    if (argument == "--follow-target" || argument == "--no-follow-target") {
      options.follow_target = argument == "--follow-target";
      continue;
    }
    if (argument == "--static-camera" || argument == "--no-static-camera") {
      options.rotation.assume_static_camera = argument == "--static-camera";
      continue;
    }
    if (argument == "--feature-size") {
      const auto value = require_value(index, argc, argv, "--feature-size");
      const auto comma = value.find(',');
      if (comma == std::string::npos || value.find(',',comma+1) != std::string::npos) {
        throw std::invalid_argument("--feature-size expects width_m,height_m");
      }
      options.feature_size_m = cv::Size2d(parse_double(value.substr(0,comma),"--feature-size"),
        parse_double(value.substr(comma+1),"--feature-size"));
      continue;
    }
    const std::pair<const char *, const char *> vision_options[] = {
      {"--brightness-method", "detector.brightness_method"},
      {"--brightness-threshold", "detector.brightness_threshold"},
      {"--classifier-model", "detector.classifier.model"},
      {"--classifier-labels", "detector.classifier.labels"},
      {"--classifier-confidence", "detector.classifier.min_confidence"},
      {"--fixed-bullet-speed", "aim.fixed_bullet_speed_m_s"},
      {"--rotation-tilt", "rotation.mounting_tilt_rad"}};
    bool vision_option = false;
    for (const auto & entry : vision_options) {
      if (argument != entry.first) continue;
      (void)apply_vision_entry(options,entry.second,require_value(index,argc,argv,entry.first),entry.first);
      vision_option = true;
      break;
    }
    if (vision_option) continue;
    if (argument == "--dry-run") {
      options.dry_run = true;
      continue;
    }
    if (argument == "--record") {
      options.record_path = require_value(index, argc, argv, "--record");
      continue;
    }
    if (argument == "--no-record") {
      options.record_path.clear();
      continue;
    }
    if (argument == "--record-fps") {
      options.record_fps = parse_double(require_value(index, argc, argv, "--record-fps"), "--record-fps");
      continue;
    }
    if (argument == "--record-codec") {
      options.record_codec = require_value(index, argc, argv, "--record-codec");
      continue;
    }
    if (argument == "--live") {
      options.dry_run = false;
      continue;
    }
    if (argument == "--loop") {
      options.frame_source.loop = true;
      continue;
    }
    if (argument == "--no-loop") {
      options.frame_source.loop = false;
      continue;
    }
    if (argument == "--input") {
      options.frame_source.type =
        parse_input_type(require_value(index, argc, argv, "--input"), "--input");
      continue;
    }
    if (argument == "--source") {
      options.frame_source.uri = require_value(index, argc, argv, "--source");
      continue;
    }
    if (argument == "--camera-index") {
      set_camera_index(
        options, require_value(index, argc, argv, "--camera-index"), "--camera-index");
      continue;
    }
    if (argument == "--device") {
      options.transport.device = require_value(index, argc, argv, "--device");
      if (options.transport.device.empty()) {
        throw std::invalid_argument("--device must not be empty");
      }
      continue;
    }
    if (argument == "--log") {
      options.log_path = require_value(index, argc, argv, "--log");
      continue;
    }
    if (argument == "--debug-dir") {
      options.debug_directory = require_value(index, argc, argv, "--debug-dir");
      continue;
    }
    if (argument == "--show-debug" || argument == "--no-show-debug") {
      options.show_debug = argument == "--show-debug";
      continue;
    }
    if (argument == "--require-center-pattern" || argument == "--no-center-pattern") {
      options.require_center_pattern = argument == "--require-center-pattern";
      continue;
    }
    if (argument == "--start-paused" || argument == "--no-start-paused") {
      options.start_paused = argument == "--start-paused";
      continue;
    }
    if (argument == "--playback-fps") {
      set_playback_fps(
        options, require_value(index, argc, argv, "--playback-fps"), "--playback-fps");
      continue;
    }
    if (argument == "--summary") {
      options.summary_path = require_value(index, argc, argv, "--summary");
      continue;
    }
    if (argument == "--debug-every") {
      set_debug_every(
        options, require_value(index, argc, argv, "--debug-every"), "--debug-every");
      continue;
    }
    if (argument == "--max-frames") {
      set_max_frames(
        options, require_value(index, argc, argv, "--max-frames"), "--max-frames");
      continue;
    }
    if (argument == "--intrinsics") {
      parse_intrinsics(
        options, require_value(index, argc, argv, "--intrinsics"), "--intrinsics");
      continue;
    }
    if (argument == "--transmit-period-ms") {
      set_positive_duration(
        options.transport.transmit_period,
        require_value(index, argc, argv, "--transmit-period-ms"), "--transmit-period-ms");
      continue;
    }
    if (argument == "--io-timeout-ms") {
      set_positive_duration(
        options.transport.io_timeout,
        require_value(index, argc, argv, "--io-timeout-ms"), "--io-timeout-ms");
      continue;
    }
    if (argument == "--reconnect-ms") {
      set_positive_duration(
        options.transport.reconnect_interval,
        require_value(index, argc, argv, "--reconnect-ms"), "--reconnect-ms");
      continue;
    }
    if (argument == "--command-max-age-ms") {
      set_positive_duration(
        options.transport.command_max_age,
        require_value(index, argc, argv, "--command-max-age-ms"), "--command-max-age-ms");
      continue;
    }

    throw std::invalid_argument("unknown option: " + argument);
  }

  validate_options(options);
  return options;
}

std::string usage(const char * program_name)
{
  const std::string program = program_name == nullptr ? "srm_auto_aim_app" : program_name;
  std::ostringstream stream;
  stream
    << "Usage: " << program << " [options]\n"
    << "  --config PATH               key=value configuration file\n"
    << "  --input image|video|camera|mvs  input mode (default: OpenCV camera)\n"
    << "  --source PATH               image/video path\n"
    << "  --camera-index N            OpenCV camera index (default: 0)\n"
    << "  --list-cameras              list MVS USB cameras and SDK version; exit\n"
    << "  --camera-info               query selected MVS camera nodes; exit (no grabbing)\n"
    << "  --mvs-serial SERIAL         select MVS USB camera by serial number\n"
    << "  --mvs-index N               MVS list index; -1=auto only if one camera\n"
    << "  --mvs-library PATH          MVS 4.7.x runtime .so (loaded on demand)\n"
    << "  --camera-width N            requested width; 0=keep current\n"
    << "  --camera-height N           requested height; 0=keep current\n"
    << "  --camera-fps FPS            requested acquisition rate; 0=keep current\n"
    << "  --exposure-us US|default    MVS manual exposure in microseconds; default=keep\n"
    << "  --gain VALUE|default        MVS manual gain (camera node units); default=keep\n"
    << "  --pixel-format NAME|default  MVS GenICam PixelFormat; default=keep\n"
    << "  --camera-timeout-ms N       MVS read timeout in [1,10000] (default: 1000)\n"
    << "  --device PATH               USB CDC device (default: /dev/ttyACM0)\n"
    << "  --intrinsics fx,fy,cx,cy    camera intrinsic parameters\n"
    << "  --calibration PATH          full OpenCV YAML: K, distortion, resolution, orientation\n"
    << "  --extrinsics PATH           explicit camera-to-gimbal R/t and coordinate convention\n"
    << "  --camera-rotation DEG       rotate live camera only: 0/90/180/270 clockwise\n"
    << "  --file-rotation DEG         additional rotation for image/video only (default: 0)\n"
    << "  --file-orientation DEG      stored orientation for legacy files without metadata\n"
    << "  --log PATH                  write CSV runtime log\n"
    << "  --summary PATH              write key=value replay summary\n"
    << "  --debug-dir PATH            save detection, association and rotation debug images\n"
    << "  --debug-every N             save every Nth frame (default: 1)\n"
    << "  --brightness-method METHOD  max_channel (default) or BGR grayscale\n"
    << "  --brightness-threshold N    brightness cutoff in [0,255] (default: 150)\n"
    << "  --require-center-pattern/--no-center-pattern  central bright marking check (default: on)\n"
    << "  --classifier/--no-classifier  ONNX content evidence (default: on; replaces bright marking gate)\n"
    << "  --classifier-model PATH     ONNX model (default: bundled armor_number/mlp.onnx)\n"
    << "  --classifier-labels PATH    label file including negative class\n"
    << "  --classifier-confidence P   acceptance threshold in (0.5,1] (default: 0.7)\n"
    << "  --rotation/--no-rotation    independent four-plate estimator (default: on)\n"
    << "  --follow-target/--no-follow-target  non-firing current-target follow; bypass rotation lock\n"
    << "  --fixed-bullet-speed MPS|referee  temporary speed override; forces fire_flag=0\n"
    << "                             referee uses feedback; zero/missing speed defaults to 22 m/s\n"
    << "  --feature-size W,H          measured light endpoint width,height in metres\n"
    << "  --static-camera/--no-static-camera  explicit fixed camera assumption (dry-run files only)\n"
    << "  --rotation-tilt RAD|unknown  measured mounting tilt; unknown=estimate with quality gates\n"
    << "  --show-debug/--no-show-debug  show debug windows in dry-run/live; live/camera has no pause/pacing\n"
    << "  --playback-fps FPS          dry-run file playback rate; 0=source or 30 (default)\n"
    << "                             nonzero FPS must be within [0.1, 1000]\n"
    << "  --start-paused/--no-start-paused  pause on the first displayed frame\n"
    << "                             keys: Space=pause, n=step, q/Esc=quit\n"
    << "  --record PATH               save raw BGR frames to a new .avi (dry-run only)\n"
    << "  --no-record                 disable recording configured in a file\n"
    << "  --record-fps FPS            video metadata rate; 0=source or 30, no capture pacing\n"
    << "  --record-codec FOURCC       FFmpeg encoder (default: MJPG; lossless: FFV1)\n"
    << "  --max-frames N              stop after N frames; 0 means no limit\n"
    << "  --loop/--no-loop            enable or disable image/video looping\n"
    << "  --dry-run/--live            disable or enable transport startup\n"
    << "  --transmit-period-ms N      CDC transmit period (default: 10)\n"
    << "  --io-timeout-ms N           serial IO timeout (default: 20)\n"
    << "  --reconnect-ms N            reconnect interval (default: 100)\n"
    << "  --command-max-age-ms N      expire stale commands to zero-safe output (default: 100)\n"
    << "  --help                      show this help\n";
  return stream.str();      // 把输入流中的内容转换为字符串返回
}

}  // namespace srm::app
