#include "app/runtime_statistics.hpp"

#include <algorithm>
#include <iomanip>
#include <ostream>
#include <sstream>

namespace srm::app
{

void RuntimeStatistics::observe(
  const control::RuntimeStepResult & result, const RuntimeTransportSnapshot & transport)
{
  ++snapshot_.step_count;
  if (result.frame_read) ++snapshot_.frame_count;
  if (result.end_of_stream) ++snapshot_.end_of_stream_count;
  if (!result.error.empty() && !result.end_of_stream) ++snapshot_.error_count;
  if (result.pipeline.has_value()) {
    snapshot_.candidate_count += result.pipeline->selection.observations.size();
  }

  switch (result.safety.state) {
    case control::SafetyState::safe:
      ++snapshot_.safe_count;
      break;
    case control::SafetyState::aiming:
      ++snapshot_.aiming_count;
      break;
    case control::SafetyState::fire_ready:
      ++snapshot_.fire_ready_count;
      break;
  }

  if (result.safety.command.fire.fire_flag != 0) ++snapshot_.nonzero_fire_request_count;

  const auto duration_us = result.step_duration.count();
  if (duration_us > 0) {
    snapshot_.total_step_duration_us += static_cast<std::uint64_t>(duration_us);
    snapshot_.max_step_duration_us = std::max(
      snapshot_.max_step_duration_us, static_cast<std::uint64_t>(duration_us));
  }
  snapshot_.transport = transport;
  observe_duration(TimingStage::acquisition, result.acquisition_duration);
  if (result.frame_read) {
    observe_duration(TimingStage::detection, result.detection_duration);
    observe_duration(TimingStage::pipeline, result.pipeline_duration);
  }
}

void RuntimeStatistics::observe_duration(TimingStage stage, std::chrono::microseconds duration)
{
  auto & timing = snapshot_.timings.at(static_cast<std::size_t>(stage));
  const auto us = static_cast<std::uint64_t>(std::max<std::int64_t>(0, duration.count()));
  ++timing.sample_count;
  timing.total_us += us;
  timing.max_us = std::max(timing.max_us, us);
}

const RuntimeStatisticsSnapshot & RuntimeStatistics::snapshot() const noexcept
{
  return snapshot_;
}

void RuntimeStatistics::write_summary(std::ostream & output) const
{
  const auto average_step_duration_us =
    snapshot_.step_count == 0
      ? 0.0
      : static_cast<double>(snapshot_.total_step_duration_us) /
          static_cast<double>(snapshot_.step_count);
  output << "step_count=" << snapshot_.step_count << '\n'
         << "frame_count=" << snapshot_.frame_count << '\n'
         << "end_of_stream_count=" << snapshot_.end_of_stream_count << '\n'
         << "error_count=" << snapshot_.error_count << '\n'
         << "candidate_count=" << snapshot_.candidate_count << '\n'
         << "safe_count=" << snapshot_.safe_count << '\n'
         << "aiming_count=" << snapshot_.aiming_count << '\n'
         << "fire_ready_count=" << snapshot_.fire_ready_count << '\n'
         << "nonzero_fire_request_count=" << snapshot_.nonzero_fire_request_count << '\n'
         << "total_step_duration_us=" << snapshot_.total_step_duration_us << '\n'
         << "average_step_duration_us=" << std::setprecision(17)
         << average_step_duration_us << '\n'
         << "max_step_duration_us=" << snapshot_.max_step_duration_us << '\n'
         << "transport_sent_frame_count=" << snapshot_.transport.sent_frame_count << '\n'
         << "transport_received_frame_count=" << snapshot_.transport.received_frame_count << '\n'
         << "transport_communication_error_count="
         << snapshot_.transport.communication_error_count << '\n';
  constexpr const char * stages[] = {"acquisition", "detection", "pipeline", "recording", "debug_draw",
    "debug_save", "gui_display", "gui_wait", "logging", "loop", "recording_close"};
  for (std::size_t i = 0; i < snapshot_.timings.size(); ++i) {
    const auto & timing = snapshot_.timings[i];
    output << stages[i] << "_sample_count=" << timing.sample_count << '\n'
           << "total_" << stages[i] << "_duration_us=" << timing.total_us << '\n'
           << "average_" << stages[i] << "_duration_us="
           << (timing.sample_count ? static_cast<double>(timing.total_us) / timing.sample_count : 0.0) << '\n'
           << "max_" << stages[i] << "_duration_us=" << timing.max_us << '\n';
  }
}

std::string RuntimeStatistics::summary_text() const
{
  std::ostringstream output;
  write_summary(output);
  return output.str();
}

}  // namespace srm::app
