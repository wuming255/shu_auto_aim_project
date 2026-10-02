#include "self_aim_protocol.hpp"

#include <cstring>
#include <stdexcept>
#include <string>

namespace srm::protocol
{
namespace
{
constexpr std::int16_t kGimbalTargetId = 1;
constexpr std::int16_t kFireRequestId = 2;
constexpr std::int16_t kGimbalStateId = 1;
constexpr std::int16_t kShootStateId = 2;

void require_size(bool condition, const char * message)
{
  if (!condition) throw std::invalid_argument(message);
}

void write_u16(std::vector<std::uint8_t> & output, std::size_t offset, std::uint16_t value)
{
  output[offset] = static_cast<std::uint8_t>(value & 0xffU);
  output[offset + 1] = static_cast<std::uint8_t>((value >> 8U) & 0xffU);
}

void write_i16(std::vector<std::uint8_t> & output, std::size_t offset, std::int16_t value)
{
  write_u16(output, offset, static_cast<std::uint16_t>(value));
}

void write_i32(std::vector<std::uint8_t> & output, std::size_t offset, std::int32_t value)
{
  const auto bits = static_cast<std::uint32_t>(value);
  output[offset] = static_cast<std::uint8_t>(bits & 0xffU);
  output[offset + 1] = static_cast<std::uint8_t>((bits >> 8U) & 0xffU);
  output[offset + 2] = static_cast<std::uint8_t>((bits >> 16U) & 0xffU);
  output[offset + 3] = static_cast<std::uint8_t>((bits >> 24U) & 0xffU);
}

void write_f32(std::vector<std::uint8_t> & output, std::size_t offset, float value)
{
  static_assert(sizeof(float) == sizeof(std::uint32_t));
  std::uint32_t bits{};
  std::memcpy(&bits, &value, sizeof(bits));
  write_i32(output, offset, static_cast<std::int32_t>(bits));
}

std::uint16_t read_u16(const std::uint8_t * data, std::size_t offset)
{
  return static_cast<std::uint16_t>(data[offset]) |
         (static_cast<std::uint16_t>(data[offset + 1]) << 8U);
}

std::int16_t read_i16(const std::uint8_t * data, std::size_t offset)
{
  return static_cast<std::int16_t>(read_u16(data, offset));
}

std::int32_t read_i32(const std::uint8_t * data, std::size_t offset)
{
  const auto bits = static_cast<std::uint32_t>(data[offset]) |
                    (static_cast<std::uint32_t>(data[offset + 1]) << 8U) |
                    (static_cast<std::uint32_t>(data[offset + 2]) << 16U) |
                    (static_cast<std::uint32_t>(data[offset + 3]) << 24U);
  return static_cast<std::int32_t>(bits);
}

float read_f32(const std::uint8_t * data, std::size_t offset)
{
  static_assert(sizeof(float) == sizeof(std::uint32_t));
  const auto bits = static_cast<std::uint32_t>(read_i32(data, offset));
  float value{};
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

IncomingState decode_incoming_frame_unchecked(const std::uint8_t * data)
{
  // body starts at offset 2. ID 1 has a 20-byte body, ID 2 has a 4-byte body.
  constexpr std::size_t gimbal_record_offset = 2;
  constexpr std::size_t gimbal_body_offset = gimbal_record_offset + 2;
  constexpr std::size_t shoot_record_offset = gimbal_body_offset + 20;
  constexpr std::size_t shoot_body_offset = shoot_record_offset + 2;

  if (read_i16(data, gimbal_record_offset) != kGimbalStateId) {
    throw std::invalid_argument("unexpected gimbal state record id");
  }
  if (read_i16(data, shoot_record_offset) != kShootStateId) {
    throw std::invalid_argument("unexpected shoot state record id");
  }

  IncomingState state;
  state.gimbal.yaw = read_f32(data, gimbal_body_offset);
  state.gimbal.pitch = read_f32(data, gimbal_body_offset + 4);
  state.gimbal.roll = read_f32(data, gimbal_body_offset + 8);
  state.gimbal.mode = read_i32(data, gimbal_body_offset + 12);
  state.gimbal.color = read_i32(data, gimbal_body_offset + 16);
  state.shoot.bullet_speed = read_f32(data, shoot_body_offset);
  return state;
}

}  // namespace

std::vector<std::uint8_t> encode_command(const Command & command)
{
  std::vector<std::uint8_t> frame(kOutgoingFrameLength, 0);
  write_u16(frame, 0, kOutgoingBodyLength);

  constexpr std::size_t gimbal_record_offset = 2;
  constexpr std::size_t gimbal_body_offset = gimbal_record_offset + 2;
  constexpr std::size_t fire_record_offset = gimbal_body_offset + 8;
  constexpr std::size_t fire_body_offset = fire_record_offset + 2;

  write_i16(frame, gimbal_record_offset, kGimbalTargetId);
  write_f32(frame, gimbal_body_offset, command.gimbal.yaw);
  write_f32(frame, gimbal_body_offset + 4, command.gimbal.pitch);
  write_i16(frame, fire_record_offset, kFireRequestId);
  write_i32(frame, fire_body_offset, command.fire.fire_flag);
  return frame;
}

IncomingState decode_incoming_frame(const std::uint8_t * data, std::size_t size)
{
  require_size(data != nullptr, "incoming frame data is null");
  require_size(size == kIncomingFrameLength, "incoming frame must be exactly 30 bytes");
  require_size(read_u16(data, 0) == kIncomingBodyLength, "unexpected incoming body length");
  return decode_incoming_frame_unchecked(data);
}

void IncomingStreamParser::append(const std::uint8_t * data, std::size_t size)
{
  if (size == 0) return;
  require_size(data != nullptr, "stream data is null");
  buffer_.insert(buffer_.end(), data, data + size);
}

void IncomingStreamParser::append(const std::vector<std::uint8_t> & data)
{
  append(data.data(), data.size());
}

std::vector<IncomingState> IncomingStreamParser::parse_available()
{
  std::vector<IncomingState> states;

  while (true) {
    if (buffer_.size() < sizeof(std::uint16_t)) break;

    const auto body_length = read_u16(buffer_.data(), 0);
    if (body_length != kIncomingBodyLength) {
      // 协议没有魔数，无法可靠地从任意字节重同步。
      buffer_.clear();
      break;
    }

    if (buffer_.size() < kIncomingFrameLength) break;

    try {
      states.push_back(decode_incoming_frame(buffer_.data(), kIncomingFrameLength));
    } catch (const std::invalid_argument &) {
      // 当前缓存无法可靠重同步，按协议丢弃当前缓存。
      buffer_.clear();
      break;
    }

    buffer_.erase(buffer_.begin(), buffer_.begin() + kIncomingFrameLength);
  }

  return states;
}

void IncomingStreamParser::reset() noexcept
{
  buffer_.clear();
}

std::size_t IncomingStreamParser::buffered_size() const noexcept
{
  return buffer_.size();
}

}  // namespace srm::protocol
