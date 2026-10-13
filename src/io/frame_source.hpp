#ifndef SHU_AUTO_AIM_FRAME_SOURCE_HPP
#define SHU_AUTO_AIM_FRAME_SOURCE_HPP

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include "io/mvs_camera.hpp"

namespace srm::io
{

enum class FrameSourceType      // 输入源类型(枚举类型，相当于一个类，里面的值是常量)
{
  image_file,
  video_file,
  opencv_camera,
  mvs_camera,
};

const char * to_string(FrameSourceType type) noexcept;      // 将输入源类型转换为字符串

struct FrameSourceConfig
{
  FrameSourceType type = FrameSourceType::image_file;     // 默认是输入图片

  // image_file/video_file: 文件路径；opencv_camera: 仅用于日志描述，可为空。
  std::string uri;            // 放调试时的文件路径，正式运行时可以为空
  int camera_index = 0;

  // 只有大于 0 时才尝试设置对应的 OpenCV 采集属性；0 表示使用默认值。
  int width = 0;
  int height = 0;
  double fps = 0.0;

  // 对图片或视频到达末尾时是否从头开始。实时相机忽略该字段。
  bool loop = false;

  // OpenCV VideoCapture 后端选择。默认 CAP_ANY，现场适配时可配置为指定后端。
  int api_preference = cv::CAP_ANY;
  MvsCameraConfig mvs{};
};

// 单独的读取到的一帧数据
struct Frame
{
  cv::Mat bgr;      // BGR 格式的图像数据
  std::chrono::steady_clock::time_point timestamp;      // 读取到时的帧时间戳
  std::uint64_t sequence = 0;             // 帧序号

  // 文件输入的源时间，单位为秒；没有可用源时间时为 0。
  double source_time_seconds = 0.0;     // 这一帧在源文件中的时间位置，单位为秒
};

// 图片、视频、OpenCV camera index 和可选的海康 MVS 输入共用帧接口。
class FrameSource final     // 用于打开输入源，读帧，加时间戳和序号
{
public:
  explicit FrameSource(FrameSourceConfig config);     // 读配置
  ~FrameSource();

  FrameSource(const FrameSource &) = delete;                      // 禁止拷贝构造函数
  FrameSource & operator=(const FrameSource &) = delete;          // 禁止拷贝赋值函数

  // 返回 true 表示得到一帧；文件到达末尾或采集失败时返回 false。
  bool read(Frame & frame);

  // 与参考项目 CameraBase 兼容的便捷接口。
  bool read(cv::Mat & bgr, std::chrono::steady_clock::time_point & timestamp);    
  // 为什么要和参考项目 CameraBase 兼容？因为 CameraBase 是一个通用的相机接口，很多项目都使用它，所以为了方便集成和迁移，FrameSource 提供了类似的接口。

  // 将文件输入恢复到第一帧；相机输入只重置本地序号和结束状态。
  void reset();

  // 表示底层输入已经成功打开。文件输入到达末尾后仍保持 open，使用 end_of_stream() 判断结束。
  // 这下面都是一些状态查询函数，方便用户了解当前输入源的状态。
  bool is_open() const noexcept;                          // 输入源是不是开了
  bool end_of_stream() const noexcept;                    // 输入源是不是到末尾了
  FrameSourceType type() const noexcept;                  // 输入源是什么类型
  const FrameSourceConfig & config() const noexcept;      // 返回输入源的配置
  const std::string & description() const noexcept;       // 返回输入源的描述字符串（输入源的描述字符串是拿来的？）
  double nominal_fps() const noexcept;                    // 名义帧率，文件输入时是文件的帧率，相机输入时是配置的帧率
  const std::string & last_error() const noexcept;

private:
  void open();                                            // 打开输入源
  bool read_image(Frame & frame);                         // 读取一张图片
  bool read_capture(Frame & frame);                       // 用OpenCV读取一帧视频或相机数据 
  void stamp(Frame & frame, double source_time_seconds);  // 设置时间戳和序号

  FrameSourceConfig config_;                // 配置
  std::string description_;                 // 描述字符串
  cv::Mat image_;                           // 读取到的图片数据
  cv::VideoCapture capture_;                // OpenCV 视频捕获对象
  std::unique_ptr<MvsCamera> mvs_camera_;
  std::string last_error_;
  bool open_ = false;
  bool image_emitted_ = false;              // 这张图片有没有输出过
  bool end_of_stream_ = false;
  std::uint64_t next_sequence_ = 0;         // 下一帧的序号
  std::chrono::steady_clock::time_point last_timestamp_{};      // 上一帧的时间戳
  double nominal_fps_ = 0.0;                // 输入源的名义帧率，文件输入时是文件的帧率，相机输入时是配置的帧率
};

}  // namespace srm::io

#endif  // SHU_AUTO_AIM_FRAME_SOURCE_HPP
