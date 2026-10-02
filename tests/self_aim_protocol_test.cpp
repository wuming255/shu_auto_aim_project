#include "protocol/self_aim_protocol.hpp"

#include <cassert>
#include <cstring>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace
{
using srm::protocol::IncomingState;
using srm::protocol::kIncomingFrameLength;

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
  static_assert(sizeof(bits) == sizeof(value));
  std::memcpy(&bits, &value, sizeof(bits));
  write_i32(data, offset, static_cast<std::int32_t>(bits));
}

std::vector<std::uint8_t> make_incoming_frame(
  float yaw, float pitch, float roll, std::int32_t mode, std::int32_t color, float bullet_speed)
{
  std::vector<std::uint8_t> frame(kIncomingFrameLength, 0);
  write_u16(frame, 0, 28);
  write_i16(frame, 2, 1);
  write_f32(frame, 4, yaw);
  write_f32(frame, 8, pitch);
  write_f32(frame, 12, roll);
  write_i32(frame, 16, mode);
  write_i32(frame, 20, color);
  write_i16(frame, 24, 2);
  write_f32(frame, 26, bullet_speed);
  return frame;
}

void assert_near(float actual, float expected)
{
  assert(std::fabs(actual - expected) < 1e-6F);
}

void assert_state(const IncomingState & state)
{
  assert_near(state.gimbal.yaw, 1.25F);
  assert_near(state.gimbal.pitch, -2.5F);
  assert_near(state.gimbal.roll, 0.125F);
  assert(state.gimbal.mode == 3);
  assert(state.gimbal.color == -7);
  assert_near(state.shoot.bullet_speed, 22.5F);
}

void test_command_encoding()
{
  const srm::protocol::Command command{{1.25F, -2.5F}, {1}};
  const auto frame = srm::protocol::encode_command(command);

  assert(frame.size() == 18);
  assert(frame[0] == 16);
  assert(frame[1] == 0);
  assert(frame[2] == 1);
  assert(frame[3] == 0);
  assert(frame[12] == 2);
  assert(frame[13] == 0);
  assert(frame[14] == 1);
  assert(frame[15] == 0);
  assert(frame[16] == 0);
  assert(frame[17] == 0);

  const srm::protocol::Command no_fire{{-0.25F, 0.75F}, {0}};
  const auto no_fire_frame = srm::protocol::encode_command(no_fire);
  assert(no_fire_frame[14] == 0);
  assert(no_fire_frame[15] == 0);
  assert(no_fire_frame[16] == 0);
  assert(no_fire_frame[17] == 0);
}

void test_incoming_decoding()
{
  const auto frame = make_incoming_frame(1.25F, -2.5F, 0.125F, 3, -7, 22.5F);
  assert_state(srm::protocol::decode_incoming_frame(frame));
}

void test_fragmented_frame()
{
  const auto frame = make_incoming_frame(1.25F, -2.5F, 0.125F, 3, -7, 22.5F);
  srm::protocol::IncomingStreamParser parser;

  for (std::size_t i = 0; i < frame.size(); ++i) {
    parser.append(&frame[i], 1);
    const auto states = parser.parse_available();
    if (i + 1 < frame.size()) {
      assert(states.empty());
    } else {
      assert(states.size() == 1);
      assert_state(states.front());
    }
  }
  assert(parser.buffered_size() == 0);
}

void test_multiple_frames_and_remainder()
{
  const auto frame = make_incoming_frame(1.25F, -2.5F, 0.125F, 3, -7, 22.5F);
  std::vector<std::uint8_t> data = frame;
  data.insert(data.end(), frame.begin(), frame.end());

  srm::protocol::IncomingStreamParser parser;
  parser.append(data.data(), 11);
  assert(parser.parse_available().empty());
  parser.append(data.data() + 11, data.size() - 11);
  const auto states = parser.parse_available();
  assert(states.size() == 2);
  assert_state(states[0]);
  assert_state(states[1]);
  assert(parser.buffered_size() == 0);
}

void test_invalid_length_discards_cache_until_next_append()
{
  const auto frame = make_incoming_frame(1.25F, -2.5F, 0.125F, 3, -7, 22.5F);
  std::vector<std::uint8_t> invalid_then_valid{0xff, 0xff};
  invalid_then_valid.insert(invalid_then_valid.end(), frame.begin(), frame.end());

  srm::protocol::IncomingStreamParser parser;
  parser.append(invalid_then_valid);
  assert(parser.parse_available().empty());
  assert(parser.buffered_size() == 0);

  parser.append(frame);
  const auto states = parser.parse_available();
  assert(states.size() == 1);
  assert_state(states.front());
}

void test_invalid_id_discards_cache()
{
  auto frame = make_incoming_frame(1.25F, -2.5F, 0.125F, 3, -7, 22.5F);
  write_i16(frame, 2, 99);

  srm::protocol::IncomingStreamParser parser;
  parser.append(frame);
  assert(parser.parse_available().empty());
  assert(parser.buffered_size() == 0);
}

void test_invalid_direct_decode()
{
  const auto frame = make_incoming_frame(1.25F, -2.5F, 0.125F, 3, -7, 22.5F);
  bool threw = false;
  try {
    (void)srm::protocol::decode_incoming_frame(frame.data(), frame.size() - 1);
  } catch (const std::invalid_argument &) {
    threw = true;
  }
  assert(threw);
}

}  // namespace

int main()
{
  test_command_encoding();
  test_incoming_decoding();
  test_fragmented_frame();
  test_multiple_frames_and_remainder();
  test_invalid_length_discards_cache_until_next_append();
  test_invalid_id_discards_cache();
  test_invalid_direct_decode();
  std::cout << "self-aim protocol tests passed\n";
  return 0;
}
