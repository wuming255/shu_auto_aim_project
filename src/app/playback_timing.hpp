#ifndef SHU_AUTO_AIM_PLAYBACK_TIMING_HPP
#define SHU_AUTO_AIM_PLAYBACK_TIMING_HPP
#include <chrono>
#include <algorithm>

namespace srm::app
{
// HighGUI waitKey(0) blocks indefinitely, so event pumping always needs a positive delay.
inline int playback_wait_ms(bool paused, std::chrono::steady_clock::time_point deadline,
                             std::chrono::steady_clock::time_point now)
{
  if (paused) return 10;
  const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count();
  return static_cast<int>(std::clamp<long long>(remaining, 1, 10));
}
}
#endif
