#include "app/application_options.hpp"

#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string_view>

namespace srm::app
{
namespace
{

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

std::string require_value(int & index, int argc, const char * const argv[], const char * option)
{
  if (index + 1 >= argc) {
    throw std::invalid_argument(std::string("missing value for ") + option);
  }
  ++index;
  return argv[index];
}

void parse_intrinsics(ApplicationOptions & options, const std::string & text)
{
  std::stringstream stream(text);
  std::string field;
  double values[4]{};
  for (int index = 0; index < 4; ++index) {
    if (!std::getline(stream, field, ',')) {
      throw std::invalid_argument("--intrinsics expects fx,fy,cx,cy");
    }
    values[index] = parse_double(field, "--intrinsics");
  }
  if (std::getline(stream, field, ',')) {
    throw std::invalid_argument("--intrinsics expects exactly fx,fy,cx,cy");
  }
  if (values[0] <= 0.0 || values[1] <= 0.0) {
    throw std::invalid_argument("--intrinsics requires positive fx and fy");
  }
  options.intrinsics_set = true;
  options.fx = values[0];
  options.fy = values[1];
  options.cx = values[2];
  options.cy = values[3];
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
  if (options.transport.transmit_period <= std::chrono::milliseconds::zero() ||
      options.transport.io_timeout <= std::chrono::milliseconds::zero() ||
      options.transport.reconnect_interval <= std::chrono::milliseconds::zero()) {
    throw std::invalid_argument("transport timeouts and period must be positive");
  }
}

}  // namespace

ApplicationOptions parse_arguments(int argc, const char * const argv[])
{
  if (argc <= 0 || argv == nullptr) throw std::invalid_argument("invalid argument vector");

  ApplicationOptions options;
  options.frame_source.type = io::FrameSourceType::opencv_camera;
  options.transport.device = "/dev/ttyACM0";

  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    if (argument == "--help" || argument == "-h") {
      options.help = true;
      continue;
    }
    if (argument == "--dry-run") {
      options.dry_run = true;
      continue;
    }
    if (argument == "--loop") {
      options.frame_source.loop = true;
      continue;
    }
    if (argument == "--input") {
      const auto value = require_value(index, argc, argv, "--input");
      if (value == "image") {
        options.frame_source.type = io::FrameSourceType::image_file;
      } else if (value == "video") {
        options.frame_source.type = io::FrameSourceType::video_file;
      } else if (value == "camera") {
        options.frame_source.type = io::FrameSourceType::opencv_camera;
      } else {
        throw std::invalid_argument("--input must be image, video or camera");
      }
      continue;
    }
    if (argument == "--source") {
      options.frame_source.uri = require_value(index, argc, argv, "--source");
      continue;
    }
    if (argument == "--camera-index") {
      const auto value = parse_integer(
        require_value(index, argc, argv, "--camera-index"), "--camera-index");
      if (value < 0 || value > std::numeric_limits<int>::max()) {
        throw std::invalid_argument("camera index is outside the supported range");
      }
      options.frame_source.camera_index = static_cast<int>(value);
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
    if (argument == "--max-frames") {
      const auto value = parse_integer(
        require_value(index, argc, argv, "--max-frames"), "--max-frames");
      if (value < 0) throw std::invalid_argument("--max-frames must not be negative");
      options.max_frames = static_cast<std::size_t>(value);
      continue;
    }
    if (argument == "--intrinsics") {
      parse_intrinsics(options, require_value(index, argc, argv, "--intrinsics"));
      continue;
    }
    if (argument == "--transmit-period-ms") {
      const auto value = parse_integer(
        require_value(index, argc, argv, "--transmit-period-ms"), "--transmit-period-ms");
      if (value <= 0 || value > std::numeric_limits<int>::max()) {
        throw std::invalid_argument("--transmit-period-ms must be positive");
      }
      options.transport.transmit_period = std::chrono::milliseconds(value);
      continue;
    }
    if (argument == "--io-timeout-ms") {
      const auto value = parse_integer(
        require_value(index, argc, argv, "--io-timeout-ms"), "--io-timeout-ms");
      if (value <= 0 || value > std::numeric_limits<int>::max()) {
        throw std::invalid_argument("--io-timeout-ms must be positive");
      }
      options.transport.io_timeout = std::chrono::milliseconds(value);
      continue;
    }
    if (argument == "--reconnect-ms") {
      const auto value = parse_integer(
        require_value(index, argc, argv, "--reconnect-ms"), "--reconnect-ms");
      if (value <= 0 || value > std::numeric_limits<int>::max()) {
        throw std::invalid_argument("--reconnect-ms must be positive");
      }
      options.transport.reconnect_interval = std::chrono::milliseconds(value);
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
    << "  --input image|video|camera  input mode (default: camera)\n"
    << "  --source PATH               image/video path\n"
    << "  --camera-index N            OpenCV camera index (default: 0)\n"
    << "  --device PATH               USB CDC device (default: /dev/ttyACM0)\n"
    << "  --intrinsics fx,fy,cx,cy    camera intrinsic parameters\n"
    << "  --log PATH                  write CSV runtime log\n"
    << "  --max-frames N              stop after N frames; 0 means no limit\n"
    << "  --loop                      loop image/video input\n"
    << "  --dry-run                   do not start transport; always remains safe\n"
    << "  --transmit-period-ms N      CDC transmit period (default: 10)\n"
    << "  --io-timeout-ms N           serial IO timeout (default: 20)\n"
    << "  --reconnect-ms N            reconnect interval (default: 100)\n"
    << "  --help                      show this help\n";
  return stream.str();
}

}  // namespace srm::app
