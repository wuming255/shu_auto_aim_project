#include "app/application_options.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>

namespace srm::app
{
namespace
{

std::string trim(const std::string & text)
{
  const auto first = std::find_if_not(text.begin(), text.end(), [](unsigned char character) {
    return std::isspace(character) != 0;
  });
  if (first == text.end()) return {};

  const auto last = std::find_if_not(text.rbegin(), text.rend(), [](unsigned char character) {
    return std::isspace(character) != 0;
  }).base();
  return std::string(first, last);
}

double parse_double(const std::string & text, const char * option)
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

long long parse_integer(const std::string & text, const char * option)
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

bool parse_bool(const std::string & text, const char * option)
{
  if (text == "1" || text == "true" || text == "TRUE" || text == "yes" || text == "on") {
    return true;
  }
  if (text == "0" || text == "false" || text == "FALSE" || text == "no" || text == "off") {
    return false;
  }
  throw std::invalid_argument(std::string("invalid boolean for ") + option + ": " + text);
}

std::string require_value(int & index, int argc, const char * const argv[], const char * option)
{
  if (index + 1 >= argc) {
    throw std::invalid_argument(std::string("missing value for ") + option);
  }
  ++index;
  return argv[index];
}

io::FrameSourceType parse_input_type(const std::string & value, const char * option)
{
  if (value == "image") return io::FrameSourceType::image_file;
  if (value == "video") return io::FrameSourceType::video_file;
  if (value == "camera") return io::FrameSourceType::opencv_camera;
  throw std::invalid_argument(std::string(option) + " must be image, video or camera");
}

void parse_intrinsics(ApplicationOptions & options, const std::string & text, const char * option)
{
  std::stringstream stream(text);
  std::string field;
  double values[4]{};
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

void set_camera_index(ApplicationOptions & options, const std::string & text, const char * option)
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

void set_max_frames(ApplicationOptions & options, const std::string & text, const char * option)
{
  const auto value = parse_integer(text, option);
  if (value < 0) throw std::invalid_argument(std::string(option) + " must not be negative");
  options.max_frames = static_cast<std::size_t>(value);
}

void set_debug_every(ApplicationOptions & options, const std::string & text, const char * option)
{
  const auto value = parse_integer(text, option);
  if (value <= 0) throw std::invalid_argument(std::string(option) + " must be positive");
  options.debug_every = static_cast<std::size_t>(value);
}

void apply_config_entry(
  ApplicationOptions & options, const std::string & key, const std::string & value,
  std::size_t line_number)
{
  const std::string context = "config line " + std::to_string(line_number) + " (" + key + ")";
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
  } else if (key == "camera.intrinsics") {
    parse_intrinsics(options, value, context.c_str());
  } else {
    throw std::invalid_argument("unknown configuration key at line " +
                                std::to_string(line_number) + ": " + key);
  }
}

void validate_options(const ApplicationOptions & options)
{
  if (options.help) return;

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
      options.transport.reconnect_interval <= std::chrono::milliseconds::zero()) {
    throw std::invalid_argument("transport timeouts and period must be positive");
  }
  if (options.debug_every == 0) {
    throw std::invalid_argument("debug frame interval must be positive");
  }
}

ApplicationOptions default_options()
{
  ApplicationOptions options;
  options.frame_source.type = io::FrameSourceType::opencv_camera;
  options.transport.device = "/dev/ttyACM0";
  return options;
}

}  // namespace

void load_config_file(const std::string & path, ApplicationOptions & options)
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

ApplicationOptions parse_arguments(int argc, const char * const argv[])
{
  if (argc <= 0 || argv == nullptr) throw std::invalid_argument("invalid argument vector");

  ApplicationOptions options = default_options();
  for (int index = 1; index < argc; ++index) {
    if (std::string(argv[index]) != "--config") continue;
    if (index + 1 >= argc) throw std::invalid_argument("missing value for --config");
    if (!options.config_path.empty()) {
      throw std::invalid_argument("--config may only be specified once");
    }
    options.config_path = argv[++index];
    if (options.config_path.empty()) throw std::invalid_argument("--config must not be empty");
  }
  if (!options.config_path.empty()) load_config_file(options.config_path, options);

  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    if (argument == "--config") {
      ++index;
      continue;
    }
    if (argument == "--help" || argument == "-h") {
      options.help = true;
      continue;
    }
    if (argument == "--dry-run") {
      options.dry_run = true;
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
    << "  --input image|video|camera  input mode (default: camera)\n"
    << "  --source PATH               image/video path\n"
    << "  --camera-index N            OpenCV camera index (default: 0)\n"
    << "  --device PATH               USB CDC device (default: /dev/ttyACM0)\n"
    << "  --intrinsics fx,fy,cx,cy    camera intrinsic parameters\n"
    << "  --log PATH                  write CSV runtime log\n"
    << "  --summary PATH              write key=value replay summary\n"
    << "  --debug-dir PATH            save detector debug images\n"
    << "  --debug-every N             save every Nth frame (default: 1)\n"
    << "  --max-frames N              stop after N frames; 0 means no limit\n"
    << "  --loop/--no-loop            enable or disable image/video looping\n"
    << "  --dry-run/--live            disable or enable transport startup\n"
    << "  --transmit-period-ms N      CDC transmit period (default: 10)\n"
    << "  --io-timeout-ms N           serial IO timeout (default: 20)\n"
    << "  --reconnect-ms N            reconnect interval (default: 100)\n"
    << "  --help                      show this help\n";
  return stream.str();
}

}  // namespace srm::app
