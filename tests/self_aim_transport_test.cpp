#include "io/self_aim_transport.hpp"

#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <stdexcept>
#include <algorithm>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

#include <cstdlib>

namespace
{
using namespace std::chrono_literals;
using srm::io::SelfAimTransport;
using srm::io::TransportConfig;
using srm::protocol::IncomingState;

class PtyPair final
{
public:
  PtyPair()
  {
    master_ = ::posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (master_ < 0) throw std::runtime_error("posix_openpt failed");
    if (::grantpt(master_) != 0 || ::unlockpt(master_) != 0) {
      ::close(master_);
      throw std::runtime_error("grantpt/unlockpt failed");
    }

    char slave_name[256]{};
    if (::ptsname_r(master_, slave_name, sizeof(slave_name)) != 0) {
      ::close(master_);
      throw std::runtime_error("ptsname_r failed");
    }
    slave_path_ = slave_name;
  }

  ~PtyPair()
  {
    if (master_ >= 0) ::close(master_);
  }

  PtyPair(const PtyPair &) = delete;
  PtyPair & operator=(const PtyPair &) = delete;

  const std::string & slave_path() const { return slave_path_; }

  void write_all(const std::vector<std::uint8_t> & data)
  {
    std::size_t offset = 0;
    while (offset < data.size()) {
      const auto written = ::write(master_, data.data() + offset, data.size() - offset);
      if (written < 0) throw std::runtime_error("pty write failed");
      offset += static_cast<std::size_t>(written);
    }
  }

  std::vector<std::uint8_t> read_exact(std::size_t size, std::chrono::milliseconds timeout)
  {
    std::vector<std::uint8_t> result;
    result.reserve(size);
    const auto deadline = std::chrono::steady_clock::now() + timeout;

    while (result.size() < size) {
      const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
        deadline - std::chrono::steady_clock::now());
      if (remaining <= 0ms) throw std::runtime_error("timed out reading pty");

      pollfd descriptor{master_, POLLIN, 0};
      const int ready = ::poll(&descriptor, 1, static_cast<int>(remaining.count()));
      if (ready < 0) throw std::runtime_error("poll failed");
      if (ready == 0) throw std::runtime_error("timed out polling pty");
      if ((descriptor.revents & POLLIN) == 0) throw std::runtime_error("pty not readable");

      std::uint8_t buffer[64]{};
      const auto wanted = std::min<std::size_t>(sizeof(buffer), size - result.size());
      const auto count = ::read(master_, buffer, wanted);
      if (count < 0) throw std::runtime_error("pty read failed");
      if (count == 0) throw std::runtime_error("pty closed");
      result.insert(result.end(), buffer, buffer + count);
    }
    return result;
  }

private:
  int master_ = -1;
  std::string slave_path_;
};

void write_u16(std::vector<std::uint8_t> & data, std::size_t offset, std::uint16_t value)
{
  data[offset] = static_cast<std::uint8_t>(value & 0xffU);
  data[offset + 1] = static_cast<std::uint8_t>((value >> 8U) & 0xffU);
}

void write_i16(std::vector<std::uint8_t> & data, std::size_t offset, std::int16_t value)
{
  write_u16(data, offset, static_cast<std::uint16_t>(value));
}

void write_i32(std::vector<std::uint8_t> & data, std::size_t offset, std::int32_t value)
{
  const auto bits = static_cast<std::uint32_t>(value);
  data[offset] = static_cast<std::uint8_t>(bits & 0xffU);
  data[offset + 1] = static_cast<std::uint8_t>((bits >> 8U) & 0xffU);
  data[offset + 2] = static_cast<std::uint8_t>((bits >> 16U) & 0xffU);
  data[offset + 3] = static_cast<std::uint8_t>((bits >> 24U) & 0xffU);
}

void write_f32(std::vector<std::uint8_t> & data, std::size_t offset, float value)
{
  std::uint32_t bits{};
  std::memcpy(&bits, &value, sizeof(bits));
  write_i32(data, offset, static_cast<std::int32_t>(bits));
}

std::vector<std::uint8_t> make_incoming_frame()
{
  std::vector<std::uint8_t> frame(srm::protocol::kIncomingFrameLength, 0);
  write_u16(frame, 0, 28);
  write_i16(frame, 2, 1);
  write_f32(frame, 4, 1.25F);
  write_f32(frame, 8, -2.5F);
  write_f32(frame, 12, 0.125F);
  write_i32(frame, 16, 3);
  write_i32(frame, 20, -7);
  write_i16(frame, 24, 2);
  write_f32(frame, 26, 22.5F);
  return frame;
}

void assert_state(const IncomingState & state)
{
  assert(std::fabs(state.gimbal.yaw - 1.25F) < 1e-6F);
  assert(std::fabs(state.gimbal.pitch + 2.5F) < 1e-6F);
  assert(std::fabs(state.gimbal.roll - 0.125F) < 1e-6F);
  assert(state.gimbal.mode == 3);
  assert(state.gimbal.color == -7);
  assert(std::fabs(state.shoot.bullet_speed - 22.5F) < 1e-6F);
}

void assert_safe_command(const std::vector<std::uint8_t> & frame)
{
  assert(frame.size() == srm::protocol::kOutgoingFrameLength);
  assert(frame[0] == 16 && frame[1] == 0);
  assert(frame[2] == 1 && frame[3] == 0);
  assert(frame[12] == 2 && frame[13] == 0);
  assert(frame[14] == 0 && frame[15] == 0 && frame[16] == 0 && frame[17] == 0);
}

TransportConfig test_config(const std::string & path)
{
  TransportConfig config;
  config.device = path;
  config.transmit_period = 5ms;
  config.io_timeout = 15ms;
  config.reconnect_interval = 10ms;
  return config;
}

void test_pty_round_trip_and_safe_default()
{
  PtyPair pty;
  SelfAimTransport transport(test_config(pty.slave_path()));
  transport.start();

  assert(transport.running());
  assert(transport.wait_until_connected(500ms));
  assert(transport.connected());

  const auto first_frame = pty.read_exact(srm::protocol::kOutgoingFrameLength, 1s);
  assert_safe_command(first_frame);

  transport.set_command({{1.25F, -2.5F}, {0}});
  bool received_target = false;
  for (int i = 0; i < 5 && !received_target; ++i) {
    const auto frame = pty.read_exact(srm::protocol::kOutgoingFrameLength, 1s);
    if (frame[14] == 0 && frame[15] == 0 && frame[16] == 0 && frame[17] == 0) {
      received_target = true;
    }
  }
  assert(received_target);

  const auto incoming = make_incoming_frame();
  pty.write_all(std::vector<std::uint8_t>(incoming.begin(), incoming.begin() + 7));
  std::this_thread::sleep_for(5ms);
  pty.write_all(std::vector<std::uint8_t>(incoming.begin() + 7, incoming.end()));

  const auto deadline = std::chrono::steady_clock::now() + 1s;
  while (transport.received_frame_count() == 0 && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(2ms);
  }
  assert(transport.received_frame_count() >= 1);
  const auto state = transport.latest_state();
  assert(state.has_value());
  assert_state(*state);
  const auto state_timestamp = transport.latest_state_timestamp();
  assert(state_timestamp.has_value());
  assert(std::chrono::steady_clock::now() >= state_timestamp.value());

  transport.stop();
  assert(!transport.running());
  assert(!transport.connected());
}

void test_device_can_appear_after_start()
{
  PtyPair pty;
  const auto link_path =
    std::string("/tmp/shu_auto_aim_transport_") + std::to_string(static_cast<long long>(::getpid()));
  ::unlink(link_path.c_str());

  SelfAimTransport transport(test_config(link_path));
  transport.start();
  assert(!transport.wait_until_connected(40ms));

  assert(::symlink(pty.slave_path().c_str(), link_path.c_str()) == 0);
  assert(transport.wait_until_connected(1s));
  const auto frame = pty.read_exact(srm::protocol::kOutgoingFrameLength, 1s);
  assert_safe_command(frame);

  transport.stop();
  ::unlink(link_path.c_str());
}

}  // namespace

int main()
{
  test_pty_round_trip_and_safe_default();
  test_device_can_appear_after_start();
  return 0;
}
