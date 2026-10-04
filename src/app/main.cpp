#include "app/application_options.hpp"
#include "app/runtime_csv_logger.hpp"
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
#include <iostream>
#include <string>

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

    srm::io::FrameSource frame_source(options.frame_source);
    srm::io::SelfAimTransport transport(options.transport);
    srm::vision::TraditionalDetector detector;
    srm::app::RuntimeCsvLogger logger(options.log_path);

    srm::control::AutoAimRuntime runtime(
      frame_source, transport, srm::control::AutoAimPipeline(make_pipeline_config(options)),
      srm::control::SafetyController(),
      [&detector](const srm::io::Frame & frame) { return detector.detect(frame.bgr); });

    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    std::atomic<bool> stop_requested{false};
    if (!options.dry_run) transport.start();

    std::size_t processed_frames = 0;
    while (!stop_requested.load()) {
      if (g_stop_requested != 0) stop_requested.store(true);

      const auto result = runtime.step();
      logger.log(result);
      if (result.frame_read) ++processed_frames;

      if (result.end_of_stream) break;
      if (options.max_frames != 0 && processed_frames >= options.max_frames) break;
      if (!result.frame_read && (!options.dry_run && !transport.running())) break;
    }

    if (!options.dry_run) {
      (void)transport.send_command_now(safe_command());
      transport.stop();
    }

    std::cerr << "processed_frames=" << processed_frames
              << " log_rows=" << logger.rows_written()
              << " dry_run=" << (options.dry_run ? 1 : 0) << '\n';
    return EXIT_SUCCESS;
  } catch (const std::exception & error) {
    std::cerr << "srm_auto_aim_app: " << error.what() << '\n';
    return EXIT_FAILURE;
  }
}
