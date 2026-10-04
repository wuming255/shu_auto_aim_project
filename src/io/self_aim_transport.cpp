#include "self_aim_transport.hpp"

#include <algorithm>
#include <array>
#include <exception>
#include <stdexcept>
#include <utility>
#include <vector>

namespace srm::io
{
namespace
{
constexpr std::size_t kReadChunkSize = 64;

serial::Timeout make_serial_timeout(std::chrono::milliseconds timeout)
{
  const auto milliseconds = static_cast<std::uint32_t>(std::max<std::int64_t>(1, timeout.count()));
  return serial::Timeout::simpleTimeout(milliseconds);
}

}  // namespace

SelfAimTransport::SelfAimTransport(TransportConfig config)
: config_(std::move(config)),
  // 参考 serial 实现会在构造函数收到非空 port 时立即 open()。这里必须先用
  // 空 port 构造，再由 ensure_connected() 管理打开和重连，才能支持设备晚出现。
  serial_() {
  if (config_.device.empty()) throw std::invalid_argument("transport device must not be empty");
  if (config_.transmit_period <= std::chrono::milliseconds::zero()) {
    throw std::invalid_argument("transport transmit period must be positive");
  }
  if (config_.io_timeout <= std::chrono::milliseconds::zero()) {
    throw std::invalid_argument("transport IO timeout must be positive");
  }
  if (config_.reconnect_interval <= std::chrono::milliseconds::zero()) {
    throw std::invalid_argument("transport reconnect interval must be positive");
  }

  serial_.setPort(config_.device);
  auto timeout = make_serial_timeout(config_.io_timeout);
  serial_.setTimeout(timeout);
  serial_.setBaudrate(config_.nominal_baudrate);
  serial_.setBytesize(serial::eightbits);
  serial_.setParity(serial::parity_none);
  serial_.setStopbits(serial::stopbits_one);
  serial_.setFlowcontrol(serial::flowcontrol_none);
}

SelfAimTransport::~SelfAimTransport()
{
  stop();
}

void SelfAimTransport::start()
{
  std::lock_guard<std::mutex> lock(lifecycle_mutex_);
  if (started_) return;

  started_ = true;
  stop_requested_ = false;
  running_ = true;
  receive_thread_ = std::thread(&SelfAimTransport::receive_loop, this);
  transmit_thread_ = std::thread(&SelfAimTransport::transmit_loop, this);
}

void SelfAimTransport::stop() noexcept
{
  {
    std::lock_guard<std::mutex> lock(lifecycle_mutex_);
    if (!started_) return;
    stop_requested_ = true;
  }
  state_changed_.notify_all();

  if (receive_thread_.joinable()) receive_thread_.join();
  if (transmit_thread_.joinable()) transmit_thread_.join();

  {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    try {
      if (serial_.isOpen()) serial_.close();
    } catch (...) {
      // 析构路径不能向调用方抛出异常。
    }
  }

  connected_ = false;
  running_ = false;
  state_changed_.notify_all();
}

bool SelfAimTransport::running() const noexcept
{
  return running_.load();
}

bool SelfAimTransport::connected() const noexcept
{
  return connected_.load();
}

bool SelfAimTransport::wait_until_connected(std::chrono::milliseconds timeout) const
{
  std::unique_lock<std::mutex> lock(lifecycle_mutex_);
  return state_changed_.wait_for(lock, timeout, [this] {
    return connected_.load() || !running_.load() || stop_requested_.load();
  }) && connected_.load();
}

void SelfAimTransport::set_command(const protocol::Command & command)
{
  std::lock_guard<std::mutex> lock(command_mutex_);
  command_ = command;
}

protocol::Command SelfAimTransport::command() const
{
  std::lock_guard<std::mutex> lock(command_mutex_);
  return command_;
}

bool SelfAimTransport::send_command_now(const protocol::Command & command)
{
  set_command(command);
  if (!running() || !connected() || stop_requested_) return false;

  try {
    const auto frame = protocol::encode_command(command);
    std::size_t bytes_written = 0;
    {
      std::lock_guard<std::mutex> lock(serial_mutex_);
      if (!serial_.isOpen()) {
        connected_ = false;
        state_changed_.notify_all();
        return false;
      }
      bytes_written = serial_.write(frame);
    }
    if (bytes_written != frame.size()) {
      throw std::runtime_error("serial write timed out before a complete frame was sent");
    }
    sent_frame_count_++;
    return true;
  } catch (const std::exception & error) {
    if (!stop_requested_) mark_disconnected(error.what());
    return false;
  } catch (...) {
    if (!stop_requested_) mark_disconnected("unknown serial transmit error");
    return false;
  }
}

std::optional<protocol::IncomingState> SelfAimTransport::latest_state() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return latest_state_;
}

std::optional<std::chrono::steady_clock::time_point> SelfAimTransport::latest_state_timestamp() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return latest_state_timestamp_;
}

std::uint64_t SelfAimTransport::sent_frame_count() const noexcept
{
  return sent_frame_count_.load();
}

std::uint64_t SelfAimTransport::received_frame_count() const noexcept
{
  return received_frame_count_.load();
}

std::uint64_t SelfAimTransport::communication_error_count() const noexcept
{
  return communication_error_count_.load();
}

std::string SelfAimTransport::last_error() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return last_error_;
}

bool SelfAimTransport::ensure_connected()
{
  if (stop_requested_) return false;

  std::lock_guard<std::mutex> lock(serial_mutex_);
  if (serial_.isOpen()) {
    connected_ = true;
    state_changed_.notify_all();
    return true;
  }

  try {
    serial_.open();
    {
      std::lock_guard<std::mutex> state_lock(state_mutex_);
      // 每次重新打开设备都要求重新收到一帧完整下位机状态，不能把
      // 上一次连接的状态带入新的连接周期。
      latest_state_.reset();
      latest_state_timestamp_.reset();
    }
    connected_ = true;
    state_changed_.notify_all();
    return true;
  } catch (const std::exception & error) {
    connected_ = false;
    communication_error_count_++;
    set_last_error(error.what());
    state_changed_.notify_all();
    try {
      if (serial_.isOpen()) serial_.close();
    } catch (...) {
    }
    return false;
  }
}

void SelfAimTransport::mark_disconnected(const std::string & error)
{
  communication_error_count_++;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    // 断链后不能把断链前的下位机状态带过快速重连窗口，否则安全层
    // 可能在尚未收到新状态帧时误把旧 bullet_speed 当作当前状态。
    latest_state_.reset();
    latest_state_timestamp_.reset();
    last_error_ = error;
  }
  connected_ = false;
  state_changed_.notify_all();

  std::lock_guard<std::mutex> lock(serial_mutex_);
  try {
    if (serial_.isOpen()) serial_.close();
  } catch (const std::exception & close_error) {
    set_last_error(close_error.what());
  } catch (...) {
  }
}

void SelfAimTransport::wait_for_stop(std::chrono::milliseconds duration) const
{
  std::unique_lock<std::mutex> lock(lifecycle_mutex_);
  state_changed_.wait_for(lock, duration, [this] { return stop_requested_.load(); });
}

void SelfAimTransport::set_last_error(const std::string & error)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  last_error_ = error;
}

void SelfAimTransport::receive_loop()
{
  protocol::IncomingStreamParser parser;
  std::array<std::uint8_t, kReadChunkSize> buffer{};

  while (!stop_requested_) {
    if (!ensure_connected()) {
      wait_for_stop(config_.reconnect_interval);
      continue;
    }

    try {
      std::size_t bytes_read = 0;
      {
        std::lock_guard<std::mutex> lock(serial_mutex_);
        if (!serial_.isOpen()) {
          connected_ = false;
          continue;
        }
        bytes_read = serial_.read(buffer.data(), buffer.size());
      }

      if (bytes_read == 0) {
        // 某些伪终端/驱动在无数据时会立即返回 0；避免接收线程忙等并
        // 长时间占用串口互斥锁，导致发送线程无法按周期发送。
        wait_for_stop(std::chrono::milliseconds(1));
        continue;
      }

      parser.append(buffer.data(), bytes_read);
      const auto states = parser.parse_available();
      for (const auto & state : states) {
        {
          std::lock_guard<std::mutex> lock(state_mutex_);
          latest_state_ = state;
          latest_state_timestamp_ = std::chrono::steady_clock::now();
        }
        received_frame_count_++;
      }
    } catch (const std::exception & error) {
      parser.reset();
      if (!stop_requested_) mark_disconnected(error.what());
    } catch (...) {
      parser.reset();
      if (!stop_requested_) mark_disconnected("unknown serial receive error");
    }
  }
}

void SelfAimTransport::transmit_loop()
{
  while (!stop_requested_) {
    const auto cycle_start = std::chrono::steady_clock::now();

    if (connected_) {
      try {
        const auto frame = protocol::encode_command(command());
        std::size_t bytes_written = 0;
        {
          std::lock_guard<std::mutex> lock(serial_mutex_);
          if (!serial_.isOpen()) {
            connected_ = false;
          } else {
            bytes_written = serial_.write(frame);
          }
        }
        if (bytes_written != frame.size()) {
          throw std::runtime_error("serial write timed out before a complete frame was sent");
        }
        sent_frame_count_++;
      } catch (const std::exception & error) {
        if (!stop_requested_) mark_disconnected(error.what());
      } catch (...) {
        if (!stop_requested_) mark_disconnected("unknown serial transmit error");
      }
    }

    const auto elapsed = std::chrono::steady_clock::now() - cycle_start;
    const auto remaining = config_.transmit_period -
                           std::chrono::duration_cast<std::chrono::milliseconds>(elapsed);
    if (remaining > std::chrono::milliseconds::zero()) wait_for_stop(remaining);
  }
}

}  // namespace srm::io
