#include "app/application_options.hpp"
#include "app/runtime_csv_logger.hpp"
#include "app/runtime_statistics.hpp"
#include "control/auto_aim_pipeline.hpp"
#include "control/auto_aim_runtime.hpp"
#include "control/safety_controller.hpp"
#include "io/frame_source.hpp"
#include "io/self_aim_transport.hpp"
#include "vision/traditional_detector.hpp"

#include <atomic>
#include <csignal>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <optional>
#include <opencv2/imgcodecs.hpp>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>

namespace
{
volatile std::sig_atomic_t g_stop_requested = 0;

void on_signal(int)
{
  g_stop_requested = 1;
}

srm::protocol::Command safe_command()
{
  srm::protocol::Command command{};
  command.fire.fire_flag = 0;
  return command;
}

srm::control::AutoAimPipelineConfig make_pipeline_config(const srm::app::ApplicationOptions & options)
{
  srm::control::AutoAimPipelineConfig config;
  if (options.intrinsics_set) {
    config.pose_solver.camera_matrix = cv::Matx33d(
      options.fx, 0.0, options.cx, 0.0, options.fy, options.cy, 0.0, 0.0, 1.0);
  }
  return config;
}

void prepare_debug_directory(const std::string & directory)
{
  if (directory.empty()) return;

  std::error_code error;
  std::filesystem::create_directories(directory, error);
  if (error) {
    throw std::runtime_error(
      "failed to create debug directory '" + directory + "': " + error.message());
  }
  if (!std::filesystem::is_directory(directory, error) || error) {
    throw std::runtime_error("debug path is not a directory: " + directory);
  }
}

void save_debug_frame(
  const std::string & directory, std::size_t debug_every, const srm::io::Frame & frame,
  const srm::vision::DetectionResult & detection, const srm::vision::TraditionalDetector & detector)
{
  if (directory.empty() || frame.sequence % debug_every != 0) return;

  const auto drawing = detector.draw_debug(frame.bgr, detection);
  std::ostringstream filename;
  filename << "frame_" << std::setfill('0') << std::setw(6) << frame.sequence << ".png";
  const auto path = std::filesystem::path(directory) / filename.str();
  if (!cv::imwrite(path.string(), drawing)) {
    throw std::runtime_error("failed to write debug image: " + path.string());
  }
}

void write_summary(
  const std::string & path, const srm::app::RuntimeStatistics & statistics,
  std::size_t processed_frames, std::size_t log_rows, bool dry_run)
{
  if (path.empty()) return;

  std::ofstream output(path, std::ios::out | std::ios::trunc);
  if (!output.is_open()) throw std::runtime_error("failed to open summary: " + path);

  output << "processed_frames=" << processed_frames << '\n'
         << "log_rows=" << log_rows << '\n'
         << "dry_run=" << (dry_run ? 1 : 0) << '\n';
  statistics.write_summary(output);
  output.flush();
  if (!output) throw std::runtime_error("failed while writing summary: " + path);
}

}  // namespace

int main(int argc, const char * const argv[])
{
  try {
    const auto options = srm::app::parse_arguments(argc, argv);
    if (options.help) {
      std::cout << srm::app::usage(argc > 0 ? argv[0] : nullptr);
      return EXIT_SUCCESS;
    }

    if (!options.intrinsics_set) {
      std::cerr
        << "warning: camera intrinsics are not configured; use --intrinsics fx,fy,cx,cy "
           "before relying on PnP output\n";
    }

    prepare_debug_directory(options.debug_directory);

    srm::io::FrameSource frame_source(options.frame_source);
    srm::io::SelfAimTransport transport(options.transport);
    srm::vision::TraditionalDetector detector;
    srm::app::RuntimeCsvLogger logger(options.log_path);
    srm::app::RuntimeStatistics statistics;
    std::optional<srm::vision::DetectionResult> last_detection;

    srm::control::AutoAimRuntime runtime(
      frame_source, transport, srm::control::AutoAimPipeline(make_pipeline_config(options)),
      srm::control::SafetyController(),
      [&detector, &last_detection](const srm::io::Frame & frame) {
        last_detection = detector.detect(frame.bgr);
        return last_detection.value();
      });

    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    std::atomic<bool> stop_requested{false};
    if (!options.dry_run) transport.start();

    std::size_t processed_frames = 0;
    while (!stop_requested.load()) {
      if (g_stop_requested != 0) stop_requested.store(true);
      last_detection.reset();

      const auto result = runtime.step();
      const srm::app::RuntimeTransportSnapshot transport_snapshot{
        transport.sent_frame_count(), transport.received_frame_count(),
        transport.communication_error_count()};
      statistics.observe(result, transport_snapshot);
      logger.log(result, transport_snapshot);
      if (result.frame_read) {
        ++processed_frames;
        if (last_detection.has_value() && result.frame.has_value()) {
          save_debug_frame(
            options.debug_directory, options.debug_every, result.frame.value(),
            last_detection.value(), detector);
        }
      }

      if (result.end_of_stream) break;
      if (options.max_frames != 0 && processed_frames >= options.max_frames) break;
      if (!result.frame_read && (!options.dry_run && !transport.running())) break;
    }

    if (!options.dry_run) {
      (void)transport.send_command_now(safe_command());
      transport.stop();
    }

    write_summary(
      options.summary_path, statistics, processed_frames, logger.rows_written(), options.dry_run);

    std::cerr << "processed_frames=" << processed_frames
              << " log_rows=" << logger.rows_written()
              << " dry_run=" << (options.dry_run ? 1 : 0) << '\n';
    return EXIT_SUCCESS;
  } catch (const std::exception & error) {
    std::cerr << "srm_auto_aim_app: " << error.what() << '\n';
    return EXIT_FAILURE;
  }
}
