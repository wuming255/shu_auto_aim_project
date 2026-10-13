#include "io/frame_source.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include <opencv2/imgcodecs.hpp>

namespace srm::io
{


namespace
{
constexpr auto kTimestampStep = std::chrono::microseconds(1);

void validate_config(const FrameSourceConfig & config)    // 检查配置是否合法
{
  if (config.width < 0 || config.height < 0) {
    throw std::invalid_argument("frame source width and height must not be negative");
  }
  if (config.fps < 0.0 || !std::isfinite(config.fps)) {   // 有限的帧率
    throw std::invalid_argument("frame source fps must be finite and non-negative");
  }
  if (config.type == FrameSourceType::mvs_camera) validate_mvs_config(config.mvs);

  if (
    (config.type == FrameSourceType::image_file || config.type == FrameSourceType::video_file) &&       // 要有文件路径
    config.uri.empty()) {
    throw std::invalid_argument("file frame source uri must not be empty");
  }
}

bool finite_positive(double value)        // 检查数字是否有限且大于0
{
  return std::isfinite(value) && value > 0.0;
}

}  // namespace



const char * to_string(FrameSourceType type) noexcept       // 将输入源类型转换为字符串
{
  switch (type) {
    case FrameSourceType::image_file:
      return "image_file";
    case FrameSourceType::video_file:
      return "video_file";
    case FrameSourceType::opencv_camera:
      return "opencv_camera";
    case FrameSourceType::mvs_camera:
      return "mvs_camera";
    default:
      return "unknown";
  }
}

FrameSource::FrameSource(FrameSourceConfig config) : config_(std::move(config))
{
  validate_config(config_);

  switch (config_.type) {
    case FrameSourceType::image_file:
      description_ = "image:" + config_.uri;        // 写描述
      break;
    case FrameSourceType::video_file:
      description_ = "video:" + config_.uri;
      break;
    case FrameSourceType::opencv_camera:
      description_ = "opencv_camera:" +
                     (config_.uri.empty() ? std::to_string(config_.camera_index) : config_.uri);
      break;
    case FrameSourceType::mvs_camera:
      description_ = "mvs_camera";
      break;
    default:
      throw std::invalid_argument("unsupported frame source type");
  }

  open();
}

FrameSource::~FrameSource()
{
  capture_.release();
}

void FrameSource::open()
{
  if (config_.type == FrameSourceType::mvs_camera) {
    mvs_camera_ = std::make_unique<MvsCamera>(config_.mvs, config_.width, config_.height, config_.fps);
    description_ = mvs_camera_->report();
    nominal_fps_ = mvs_camera_->nominal_fps();
    open_ = true;
    return;
  }
  if (config_.type == FrameSourceType::image_file) {
    image_ = cv::imread(config_.uri, cv::IMREAD_COLOR);     // 读一帧彩色图片
    if (image_.empty()) {
      throw std::runtime_error("failed to open image frame source: " + config_.uri);
    }
    open_ = true;                                           // 标记打开成功
    nominal_fps_ = config_.fps;
    return;
  }

  if (config_.type == FrameSourceType::video_file) {        // 用OpenCV打开视频文件
    if (!capture_.open(config_.uri, config_.api_preference)) {
      throw std::runtime_error("failed to open video frame source: " + config_.uri);
    }
  } else if (config_.type == FrameSourceType::opencv_camera) {
    if (!capture_.open(config_.camera_index, config_.api_preference)) {
      throw std::runtime_error(
        "failed to open OpenCV camera index " + std::to_string(config_.camera_index));
    }
  }
  // 给 OpenCV VideoCapture 设置采集属性，只有大于0的才设置，0表示使用默认值
  if (config_.width > 0) capture_.set(cv::CAP_PROP_FRAME_WIDTH, config_.width);
  if (config_.height > 0) capture_.set(cv::CAP_PROP_FRAME_HEIGHT, config_.height);
  if (config_.fps > 0.0) capture_.set(cv::CAP_PROP_FPS, config_.fps);

  open_ = capture_.isOpened();
  if (!open_) throw std::runtime_error("frame source opened and closed immediately: " + description_);

  const double actual_fps = capture_.get(cv::CAP_PROP_FPS);
  nominal_fps_ = config_.fps > 0.0 ? config_.fps : (finite_positive(actual_fps) ? actual_fps : 0.0);
}

bool FrameSource::read(Frame & frame)
{
  if (!open_) return false;

  Frame next;
  bool success = false;
  if (config_.type == FrameSourceType::image_file) {    // 按类型读取帧
    success = read_image(next);
  } else if (mvs_camera_) {
    success = mvs_camera_->read(next.bgr, next.timestamp);
    last_error_ = mvs_camera_->last_error();
    // 实时采集失败不是文件 EOF；不输出上一次的图像。
    if (success) stamp(next, 0.0);
  } else {
    success = read_capture(next);
  }

  if (!success) return false;
  frame = std::move(next);      // 放入下一帧图像
  return true;
}

bool FrameSource::read(cv::Mat & bgr, std::chrono::steady_clock::time_point & timestamp)    // 兼容 CameraBase 的接口的直接读图
{
  Frame frame;
  if (!read(frame)) return false;
  bgr = std::move(frame.bgr);
  timestamp = frame.timestamp;
  return true;
}

bool FrameSource::read_image(Frame & frame)     // 输入源为图片时每次read()都会调用read_image()
{
  if (image_emitted_) {       // 给过调用方吗？
    if (!config_.loop) {      // 输出过吗？
      end_of_stream_ = true;
      return false;
    }
    image_emitted_ = false;
  }

  frame.bgr = image_.clone();
  frame.source_time_seconds = 0.0;
  image_emitted_ = true;
  end_of_stream_ = false;
  stamp(frame, frame.source_time_seconds);
  return true;
}

bool FrameSource::read_capture(Frame & frame)
{
  cv::Mat image;
  if (!capture_.read(image) || image.empty()) {
    if (config_.loop && config_.type == FrameSourceType::video_file) {  // 是不是循环播放的视频?
      // 循环播放只回到文件起点，不重置对外 sequence 和 monotonic timestamp。
      const bool rewound = capture_.set(cv::CAP_PROP_POS_FRAMES, 0.0);   // 请求opencv将播放位置设置为第0帧
      if (!rewound || !capture_.read(image) || image.empty()) {         // 回头设置成功就继续读
        end_of_stream_ = true;
        return false;
      }
      end_of_stream_ = false;
    } else {
      end_of_stream_ = true;
      return false;
    }
  }

  frame.bgr = image.clone();
  const double position_ms = capture_.get(cv::CAP_PROP_POS_MSEC);   // 获取当前帧在视频中的位置，单位为毫秒
  if (finite_positive(position_ms)) {       // 设置文件源的时间（这帧图像在文件中的时间位置，单位为秒）
    frame.source_time_seconds = position_ms / 1000.0;
  } else if (finite_positive(nominal_fps_)) {
    frame.source_time_seconds = static_cast<double>(next_sequence_) / nominal_fps_;
  } else {
    frame.source_time_seconds = 0.0;
  }

  end_of_stream_ = false;
  stamp(frame, frame.source_time_seconds);
  return true;
}

void FrameSource::stamp(Frame & frame, double source_time_seconds)
{
  auto timestamp = frame.timestamp == std::chrono::steady_clock::time_point{}
                     ? std::chrono::steady_clock::now() : frame.timestamp;
  if (last_timestamp_ != std::chrono::steady_clock::time_point{} && timestamp <= last_timestamp_) {     // 不是新时间且当前时间在上次时间前
    timestamp = last_timestamp_ + kTimestampStep;
  }     // 防御性保护，一般不会读到提前的，有时候不小心读到两张一样的倒是有可能
  last_timestamp_ = timestamp;

  frame.timestamp = timestamp;
  frame.sequence = next_sequence_++;      // 帧序号+1
  frame.source_time_seconds = source_time_seconds; 
}

void FrameSource::reset()
{
  image_emitted_ = false;
  end_of_stream_ = false;
  next_sequence_ = 0;
  last_timestamp_ = {};
  last_error_.clear();

  if (config_.type == FrameSourceType::image_file) return;

  if (config_.type == FrameSourceType::video_file) {
    capture_.set(cv::CAP_PROP_POS_FRAMES, 0.0);
  }
}

bool FrameSource::is_open() const noexcept
{
  return open_;
}

bool FrameSource::end_of_stream() const noexcept
{
  return end_of_stream_;
}

FrameSourceType FrameSource::type() const noexcept
{
  return config_.type;
}

const FrameSourceConfig & FrameSource::config() const noexcept
{
  return config_;
}

const std::string & FrameSource::description() const noexcept
{
  return description_;
}

double FrameSource::nominal_fps() const noexcept
{
  return nominal_fps_;
}

const std::string & FrameSource::last_error() const noexcept { return last_error_; }

}  // namespace srm::io
