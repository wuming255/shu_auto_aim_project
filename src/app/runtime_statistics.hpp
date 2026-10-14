#ifndef SHU_AUTO_AIM_RUNTIME_STATISTICS_HPP
#define SHU_AUTO_AIM_RUNTIME_STATISTICS_HPP

#include <cstddef>
#include <cstdint>
#include <iosfwd>
#include <string>
#include <array>

#include "control/auto_aim_runtime.hpp"

namespace srm::app
{

struct RuntimeTransportSnapshot
{
  std::uint64_t sent_frame_count = 0;
  std::uint64_t received_frame_count = 0;
  std::uint64_t communication_error_count = 0;
};

enum class TimingStage : std::size_t
{
  acquisition, detection, pipeline, recording, debug_draw, debug_save,
  gui_display, gui_wait, logging, loop, recording_close, count
};
struct TimingStatistics
{
  std::uint64_t sample_count = 0;
  std::uint64_t total_us = 0;
  std::uint64_t max_us = 0;
};

struct RuntimeStatisticsSnapshot
{
  std::uint64_t step_count = 0;
  std::uint64_t frame_count = 0;
  std::uint64_t end_of_stream_count = 0;
  std::uint64_t error_count = 0;
  std::uint64_t candidate_count = 0;
  std::uint64_t safe_count = 0;
  std::uint64_t aiming_count = 0;
  std::uint64_t fire_ready_count = 0;
  std::uint64_t nonzero_fire_request_count = 0;
  std::uint64_t total_step_duration_us = 0;
  std::uint64_t max_step_duration_us = 0;
  RuntimeTransportSnapshot transport;
  std::array<TimingStatistics, static_cast<std::size_t>(TimingStage::count)> timings{};
};

class RuntimeStatistics final
{
public:
  void observe(
    const control::RuntimeStepResult & result,
    const RuntimeTransportSnapshot & transport = {});

  const RuntimeStatisticsSnapshot & snapshot() const noexcept;
  void observe_duration(TimingStage stage, std::chrono::microseconds duration);

  // 输出稳定的 key=value 摘要，适合写入 summary 文件或直接打印到终端。
  void write_summary(std::ostream & output) const;
  std::string summary_text() const;

private:
  RuntimeStatisticsSnapshot snapshot_;
};

}  // namespace srm::app

#endif  // SHU_AUTO_AIM_RUNTIME_STATISTICS_HPP
