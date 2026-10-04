#ifndef SHU_AUTO_AIM_SELF_AIM_TRANSPORT_HPP
#define SHU_AUTO_AIM_SELF_AIM_TRANSPORT_HPP

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "protocol/self_aim_protocol.hpp"
#include "serial/serial.h"

namespace srm::io
{

struct TransportConfig
{
  // USB CDC 设备在 Linux 上通常为 /dev/ttyACM*。可以使用伪终端进行测试。
  std::string device = "/dev/ttyACM0";

  // USB CDC 不依赖传统串口波特率；serial 库仍需要一个配置值，这里使用
  // 115200 作为打开设备时的兼容占位值。
  std::uint32_t nominal_baudrate = 115200;

  // 发送完整 ID1 + ID2 帧的周期。默认 100 Hz，实际可按现场状态调整。
  std::chrono::milliseconds transmit_period{10};

  // 读写单次调用的最大等待时间。必须有限，以便 stop() 能及时退出。
  std::chrono::milliseconds io_timeout{20};

  // 设备未连接或通信异常后的重连间隔。
  std::chrono::milliseconds reconnect_interval{100};
};

class SelfAimTransport final
{
public:
  explicit SelfAimTransport(TransportConfig config);
  ~SelfAimTransport();

  SelfAimTransport(const SelfAimTransport &) = delete;
  SelfAimTransport & operator=(const SelfAimTransport &) = delete;

  // 启动收发线程。启动后即使设备暂时不存在，也会按配置自动重连。
  void start();
  void stop() noexcept;

  bool running() const noexcept;
  // connected 表示设备文件已成功打开，不代表下位机已经完成应用层握手。
  bool connected() const noexcept;
  bool wait_until_connected(std::chrono::milliseconds timeout) const;

  // 默认命令中的 fire_flag 为 0。调用方必须显式设置视觉开火请求。
  void set_command(const protocol::Command & command);
  protocol::Command command() const;

  // 在当前已连接的设备上同步发送一次给定命令，同时更新周期发送命令。
  // 主要用于退出前发送确定的安全帧；未运行或未连接时返回 false。
  bool send_command_now(const protocol::Command & command);

  std::optional<protocol::IncomingState> latest_state() const;
  // 最近一次完整状态帧被解析到的单调时间；没有收到过完整帧时返回空。
  std::optional<std::chrono::steady_clock::time_point> latest_state_timestamp() const;
  std::uint64_t sent_frame_count() const noexcept;
  std::uint64_t received_frame_count() const noexcept;
  std::uint64_t communication_error_count() const noexcept;
  std::string last_error() const;

private:
  void receive_loop();
  void transmit_loop();

  bool ensure_connected();
  void mark_disconnected(const std::string & error);
  void wait_for_stop(std::chrono::milliseconds duration) const;
  void set_last_error(const std::string & error);

  TransportConfig config_;
  serial::Serial serial_;

  mutable std::mutex lifecycle_mutex_;
  mutable std::condition_variable state_changed_;
  std::atomic<bool> stop_requested_{false};
  std::atomic<bool> running_{false};
  std::atomic<bool> connected_{false};
  bool started_ = false;

  std::thread receive_thread_;
  std::thread transmit_thread_;

  mutable std::mutex serial_mutex_;
  mutable std::mutex command_mutex_;
  protocol::Command command_{};

  mutable std::mutex state_mutex_;
  std::optional<protocol::IncomingState> latest_state_;
  std::optional<std::chrono::steady_clock::time_point> latest_state_timestamp_;
  std::string last_error_;

  std::atomic<std::uint64_t> sent_frame_count_{0};
  std::atomic<std::uint64_t> received_frame_count_{0};
  std::atomic<std::uint64_t> communication_error_count_{0};
};

}  // namespace srm::io

#endif  // SHU_AUTO_AIM_SELF_AIM_TRANSPORT_HPP
