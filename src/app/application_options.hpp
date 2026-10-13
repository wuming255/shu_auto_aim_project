#ifndef SHU_AUTO_AIM_APPLICATION_OPTIONS_HPP
#define SHU_AUTO_AIM_APPLICATION_OPTIONS_HPP

#include <cstddef>
#include <string>

#include "io/frame_source.hpp"
#include "io/self_aim_transport.hpp"

namespace srm::app
{

struct ApplicationOptions
{
  io::FrameSourceConfig frame_source;   // 输入源相关配置文件
  io::TransportConfig transport;        // 串口通信相关配置文件

  // 不启动串口线程的文件回放/相机检查；安全控制器会拒绝瞄准命令。
  bool dry_run = false;                 // 是否为离线运行模式
  bool help = false;                    // 是否显示帮助信息
  bool list_cameras = false;  // 枚举 MVS USB 设备后退出，不打开相机/CDC。
  bool camera_info = false;   // 打开 MVS 相机并查询节点后退出，不修改节点/取流/CDC。
  std::string config_path;
  std::string log_path;
  std::string debug_directory;
  std::string summary_path;             // 摘要文件路径
  std::size_t max_frames = 0;  // 最大处理帧数，0 表示直到 EOF 或收到停止信号。
  std::size_t debug_every = 1;  // 每隔多少帧保存一张 debug 图像。

  // 窗口仅用于 dry-run；相机预览不暂停、不按 playback_fps 限速。
  bool show_debug = false;
  bool start_paused = false;
  double playback_fps = 0.0;  // 0 使用视频名义帧率，不可用时回退到 30 FPS。

  std::string record_path;  // 空值关闭；保存未经检测绘制的 BGR 帧。
  double record_fps = 0.0;  // 0 使用输入名义帧率，不可用时回退到 30 FPS。
  std::string record_codec = "MJPG";

  // 没有提供标定内参时保留 PoseSolver 的默认矩阵，并由主程序打印提示。
  bool intrinsics_set = false;      // 是否设置了相机内参
  double fx = 1.0;
  double fy = 1.0;
  double cx = 0.0;
  double cy = 0.0;
};

// 解析命令行参数；参数错误或配置文件错误抛出 std::invalid_argument。
// 若同时提供配置文件和命令行参数，命令行参数覆盖配置文件中的同名值。
ApplicationOptions parse_arguments(int argc, const char * const argv[]);

// 读取简单 key=value 配置文件并覆盖 options。配置文件不支持隐式默认值，
// 未知键或格式错误会抛出 std::invalid_argument。
void load_config_file(const std::string & path, ApplicationOptions & options);

std::string usage(const char * program_name);

}  // namespace srm::app

#endif  // SHU_AUTO_AIM_APPLICATION_OPTIONS_HPP
