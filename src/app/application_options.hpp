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
  io::FrameSourceConfig frame_source;
  io::TransportConfig transport;

  // 只用于不启动串口线程的离线运行；安全控制器仍会因为 transport 未运行而拒绝瞄准命令。
  bool dry_run = false;
  bool help = false;
  std::string log_path;
  std::size_t max_frames = 0;  // 0 表示直到 EOF 或收到停止信号。

  // 没有提供标定内参时保留 PoseSolver 的默认矩阵，并由主程序打印提示。
  bool intrinsics_set = false;
  double fx = 1.0;
  double fy = 1.0;
  double cx = 0.0;
  double cy = 0.0;
};

// 解析命令行参数；参数错误抛出 std::invalid_argument。
ApplicationOptions parse_arguments(int argc, const char * const argv[]);

std::string usage(const char * program_name);

}  // namespace srm::app

#endif  // SHU_AUTO_AIM_APPLICATION_OPTIONS_HPP
