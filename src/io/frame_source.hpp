#ifndef SHU_AUTO_AIM_FRAME_SOURCE_HPP
#define SHU_AUTO_AIM_FRAME_SOURCE_HPP

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

namespace srm::io
{

enum class FrameSourceType
{
  image_file,
  video_file,
  opencv_camera,
};

const char * to_string(FrameSourceType type) noexcept;

struct FrameSourceConfig
{
  FrameSourceType type = FrameSourceType::image_file;

  // image_file/video_file: 文件路径；opencv_camera: 仅用于日志描述，可为空。
  std::string uri;
  int camera_index = 0;

  // 只有大于 0 时才尝试设置对应的 OpenCV 采集属性；0 表示使用默认值。
  int width = 0;
  int height = 0;
  double fps = 0.0;

  // 对图片或视频到达末尾时是否从头开始。实时相机忽略该字段。
  bool loop = false;

  // OpenCV VideoCapture 后端选择。默认 CAP_ANY，现场适配时可配置为指定后端。
  int api_preference = cv::CAP_ANY;
};

struct Frame
{
  cv::Mat bgr;
  std::chrono::steady_clock::time_point timestamp;
  std::uint64_t sequence = 0;

  // 文件输入的源时间，单位为秒；没有可用源时间时为 0。
  double source_time_seconds = 0.0;
};

// 输入抽象的第一版：支持离线图片、离线视频和 OpenCV camera index。
// 海康 MVS 专用适配器不在本轮直接绑定，后续可在相同的 Frame 接口上接入。
class FrameSource final
{
public:
  explicit FrameSource(FrameSourceConfig config);
  ~FrameSource();

  FrameSource(const FrameSource &) = delete;
  FrameSource & operator=(const FrameSource &) = delete;

  // 返回 true 表示得到一帧；文件到达末尾或采集失败时返回 false。
  bool read(Frame & frame);

  // 与参考项目 CameraBase 兼容的便捷接口。
  bool read(cv::Mat & bgr, std::chrono::steady_clock::time_point & timestamp);

  // 将文件输入恢复到第一帧；相机输入只重置本地序号和结束状态。
  void reset();

  // 表示底层输入已经成功打开。文件输入到达末尾后仍保持 open，使用 end_of_stream() 判断结束。
  bool is_open() const noexcept;
  bool end_of_stream() const noexcept;
  FrameSourceType type() const noexcept;
  const FrameSourceConfig & config() const noexcept;
  const std::string & description() const noexcept;
  double nominal_fps() const noexcept;

private:
  void open();
  bool read_image(Frame & frame);
  bool read_capture(Frame & frame);
  void stamp(Frame & frame, double source_time_seconds);

  FrameSourceConfig config_;
  std::string description_;
  cv::Mat image_;
  cv::VideoCapture capture_;
  bool open_ = false;
  bool image_emitted_ = false;
  bool end_of_stream_ = false;
  std::uint64_t next_sequence_ = 0;
  std::chrono::steady_clock::time_point last_timestamp_{};
  double nominal_fps_ = 0.0;
};

}  // namespace srm::io

#endif  // SHU_AUTO_AIM_FRAME_SOURCE_HPP
