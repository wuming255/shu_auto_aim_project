#ifndef SHU_AUTO_AIM_SELF_AIM_PROTOCOL_HPP
#define SHU_AUTO_AIM_SELF_AIM_PROTOCOL_HPP

#include <cstddef>
#include <cstdint>
#include <vector>

namespace srm::protocol
{
constexpr std::uint16_t kOutgoingBodyLength = 16;
constexpr std::size_t kOutgoingFrameLength = 2 + kOutgoingBodyLength;
constexpr std::uint16_t kIncomingBodyLength = 28;
constexpr std::size_t kIncomingFrameLength = 2 + kIncomingBodyLength;

struct GimbalTarget
{
  float yaw{};
  float pitch{};
};

struct FireRequest
{
  std::int32_t fire_flag{};
};

struct Command
{
  GimbalTarget gimbal;
  FireRequest fire;
};

struct GimbalState
{
  float yaw{};
  float pitch{};
  float roll{};
  std::int32_t mode{};
  std::int32_t color{};
};

struct ShootState
{
  float bullet_speed{};
};

struct IncomingState
{
  GimbalState gimbal;
  ShootState shoot;
};

// 编码一帧上位机 -> 下位机报文。结果固定为 18 字节。
std::vector<std::uint8_t> encode_command(const Command & command);

// 解码一帧下位机 -> 上位机报文。输入必须恰好为 30 字节。
IncomingState decode_incoming_frame(const std::uint8_t * data, std::size_t size);

inline IncomingState decode_incoming_frame(const std::vector<std::uint8_t> & frame)
{
  return decode_incoming_frame(frame.data(), frame.size());
}

// 面向 USB CDC 读取的持久流式解析器。
class IncomingStreamParser
{
public:
  void append(const std::uint8_t * data, std::size_t size);
  void append(const std::vector<std::uint8_t> & data);

  // 解析当前缓存中所有完整帧；不完整帧会保留到下一次 append。
  std::vector<IncomingState> parse_available();

  void reset() noexcept;
  std::size_t buffered_size() const noexcept;

private:
  std::vector<std::uint8_t> buffer_;
};

}  // namespace srm::protocol

#endif  // SHU_AUTO_AIM_SELF_AIM_PROTOCOL_HPP
