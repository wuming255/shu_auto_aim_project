#include "app/application_options.hpp"
#include "app/debug_viewer.hpp"
#include "app/runtime_csv_logger.hpp"
#include "app/runtime_statistics.hpp"
#include "app/video_recorder.hpp"
#include "app/tracking_debug.hpp"
#include "control/auto_aim_pipeline.hpp"
#include "control/auto_aim_runtime.hpp"
#include "control/safety_controller.hpp"
#include "io/frame_source.hpp"
#include "io/self_aim_transport.hpp"
#include "vision/traditional_detector.hpp"
#include "vision/camera_calibration.hpp"

#include <atomic>
#include <cmath>
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



namespace   // 依旧匿名函数创建临时函数
{
volatile std::sig_atomic_t g_stop_requested = 0;

void on_signal(int)   // 收ctrl + C,设置变量使程序退出
{
  g_stop_requested = 1;       // 设置全局变量，表示程序需要停止运行(不是开关)
}

srm::protocol::Command safe_command()   // 生成一条安全的命令，设置fire_flag为0
{
  srm::protocol::Command command{};
  command.fire.fire_flag = 0;
  return command;
}

srm::control::AutoAimPipelineConfig make_pipeline_config(
  const srm::app::ApplicationOptions & options,
  const std::optional<srm::vision::CameraCalibration> & calibration)
{
  srm::control::AutoAimPipelineConfig config;
  config.rotation = options.rotation;
  config.fixed_guard = options.fixed_guard;
  config.follow_target = options.follow_target;
  config.pose_solver.feature_size_m = options.feature_size_m;
  if (options.intrinsics_set) {
    config.pose_solver.camera_matrix = cv::Matx33d(
      options.fx, 0.0, options.cx, 0.0, options.fy, options.cy, 0.0, 0.0, 1.0); // 创建标准内参矩阵
  }
  if (calibration) {
    config.pose_solver.camera_matrix = calibration->camera_matrix;
    config.pose_solver.distortion_coefficients = calibration->distortion_coefficients;
    if (!options.extrinsics_path.empty()) {
      srm::vision::load_camera_extrinsics(options.extrinsics_path, calibration->geometry, config.pose_solver);
    }
  }
  return config;
}

void prepare_debug_directory(const std::string & directory)     // 创建调试图像保存目录
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

void save_debug_frame(        // 保存调试图像
  const std::string & directory, const srm::io::Frame & frame, const cv::Mat & drawing)
{
  std::ostringstream filename;
  filename << "frame_" << std::setfill('0') << std::setw(6) << frame.sequence << ".png";
  const auto path = std::filesystem::path(directory) / filename.str();
  if (!cv::imwrite(path.string(), drawing)) {
    throw std::runtime_error("failed to write debug image: " + path.string());
  }
}

void write_summary(     // 写入运行摘要文件
  const std::string & path, const srm::app::RuntimeStatistics & statistics,
  std::size_t processed_frames, std::size_t log_rows, bool dry_run,
  bool calibration_loaded, bool extrinsics_loaded, const std::optional<srm::io::ImageGeometry> & geometry)
{
  if (path.empty()) return;

  std::ofstream output(path, std::ios::out | std::ios::trunc);    // 创建输出文件流
  if (!output.is_open()) throw std::runtime_error("failed to open summary: " + path);

  output << "processed_frames=" << processed_frames << '\n'
         << "log_rows=" << log_rows << '\n'
         << "dry_run=" << (dry_run ? 1 : 0) << '\n';
  output << "calibration_loaded=" << calibration_loaded << '\n'
         << "extrinsics_loaded=" << extrinsics_loaded << '\n';
  if (geometry) output << "image_width=" << geometry->size.width << '\n'
                       << "image_height=" << geometry->size.height << '\n'
                       << "orientation_deg=" << geometry->orientation_deg << '\n';
  statistics.write_summary(output);
  output.flush();   // 刷新缓冲区，确保数据写入文件
  if (!output) throw std::runtime_error("failed while writing summary: " + path);
}

}  // namespace



int main(int argc, const char * const argv[])
{
  try {
    const auto options = srm::app::parse_arguments(argc, argv);   // 解析命令行参数，返回配置数据类
    if (options.help) {
      std::cout << srm::app::usage(argc > 0 ? argv[0] : nullptr);
      return EXIT_SUCCESS;
    }
    if (options.list_cameras) {
      std::cout << srm::io::list_mvs_cameras(options.frame_source.mvs.sdk_library);
      return EXIT_SUCCESS;
    }
    if (options.camera_info) {
      srm::io::MvsCamera camera(options.frame_source.mvs, 0, 0, 0.0, false);
      std::cout << camera.report();
      return EXIT_SUCCESS;
    }

    std::optional<srm::vision::CameraCalibration> calibration;
    if (!options.calibration_path.empty()) calibration = srm::vision::load_camera_calibration(options.calibration_path);
    const auto pipeline_config = make_pipeline_config(options, calibration);
    if (!options.dry_run && (!calibration || options.extrinsics_path.empty())) {
      throw std::invalid_argument("--live requires full --calibration and --extrinsics; use --dry-run for uncalibrated input checks");
    }
    if (!options.dry_run && options.follow_target && !options.rotation.feedback_frame_confirmed) {
      throw std::invalid_argument("live follow requires rotation.feedback_frame_confirmed=true (Tongji yaw/roll right-hand, pitch down positive)");
    }
    std::cerr << "aim_mode=" << (options.follow_target ? "non_firing_follow" : "configured_pipeline")
              << " bullet_speed_source=" << (options.safety.fixed_bullet_speed_m_s ? "fixed" : "referee");
    if (options.safety.fixed_bullet_speed_m_s) std::cerr << " bullet_speed_m_s=" << *options.safety.fixed_bullet_speed_m_s;
    std::cerr << '\n';
    if (!options.intrinsics_set && !calibration) {
      std::cerr
        << "warning: camera intrinsics are not configured; use --intrinsics fx,fy,cx,cy "
           "before relying on PnP output\n";
    }
    if (options.intrinsics_set) std::cerr << "warning: legacy intrinsics have no distortion/resolution/orientation binding; dry-run only\n";
    if (options.extrinsics_path.empty()) std::cerr << "warning: camera-to-gimbal extrinsics are uncalibrated; default axis mapping is for offline diagnostics only\n";

    if (options.show_debug) srm::app::DebugViewer::check_environment();
    prepare_debug_directory(options.debug_directory);

    auto source_config = options.frame_source;
    if (calibration) source_config.expected_geometry = calibration->geometry;
    srm::io::FrameSource frame_source(source_config);          // 图像输入模块
    if (frame_source.type() == srm::io::FrameSourceType::mvs_camera) {
      std::cerr << frame_source.description();
    }
    // 在其他输出文件打开前拒绝已有录像（也包括指向同一文件的硬链接）。
    std::optional<srm::app::VideoRecorder> recorder;
    if (!options.record_path.empty()) {
      if (std::filesystem::symlink_status(options.record_path + ".metadata.yml").type() != std::filesystem::file_type::not_found) {
        throw std::invalid_argument("recording metadata already exists: " + options.record_path + ".metadata.yml");
      }
      recorder.emplace(options.record_path,
        srm::app::VideoRecorder::choose_fps(options.record_fps, frame_source.nominal_fps()), options.record_codec);
      std::cerr << "recording raw BGR: path=" << recorder->path() << " fps=" << recorder->fps()
                << " codec=" << options.record_codec << '\n';
    }
    srm::io::SelfAimTransport transport(options.transport);           // 串口通信模块
    srm::vision::TraditionalDetectorConfig detector_config;
    detector_config.require_center_pattern = options.require_center_pattern;
    detector_config.classifier = options.classifier;
    srm::vision::TraditionalDetector detector(detector_config);       // 传统视觉检测模块
    srm::app::RuntimeCsvLogger logger(options.log_path);              // 运行日志记录模块
    srm::app::RuntimeStatistics statistics;                           // 运行统计模块，记录各个功能运行了多少次，有什么报错，运行了多久等信息
    const auto timed = [&statistics](srm::app::TimingStage stage, const auto & action) {
      const auto start = std::chrono::steady_clock::now();
      action();
      statistics.observe_duration(stage, std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now() - start));
    };
    std::optional<srm::vision::DetectionResult> last_detection;       // 本帧检测结果，供保存和窗口显示使用。
    std::optional<srm::app::DebugViewer> debug_viewer;
    if (options.show_debug) {
      const bool realtime = frame_source.type() == srm::io::FrameSourceType::mvs_camera ||
                            frame_source.type() == srm::io::FrameSourceType::opencv_camera;
      const double nominal_fps = frame_source.nominal_fps();
      const double playback_fps = options.playback_fps > 0.0
                                   ? options.playback_fps
                                   : (std::isfinite(nominal_fps) && nominal_fps > 0.0
                                        ? nominal_fps : 30.0);
      debug_viewer.emplace(
        playback_fps,
        options.start_paused ||
          options.frame_source.type == srm::io::FrameSourceType::image_file, realtime);
      if (realtime) std::cerr << "camera preview: q/Esc=quit (no pause or playback pacing)\n";
      else std::cerr << "debug windows: Space=pause/resume n=next q/Esc=quit playback_fps="
                     << playback_fps << '\n';
    }

    srm::control::AutoAimRuntime runtime(                             // 一帧图像的总调度模块，安排各个模块的协作和数据流转
      frame_source, transport, srm::control::AutoAimPipeline(pipeline_config),        // 瞄准算法模块
      srm::control::SafetyController(options.safety),                 // 安全控制模块
      [&detector, &last_detection](const srm::io::Frame & frame) {    // 这个匿名函数输入一帧图像，返回检测结果
        last_detection = detector.detect(frame.bgr);
        return last_detection.value();                                // 注意这里传入的是整个匿名函数，使后面在类中可以被回调
      });

    std::signal(SIGINT, on_signal);     // SIGINT是终端的ctrl+c信号，SIGTERM是kill命令发送的终止信号，on_signal是自定义的信号处理函数
    std::signal(SIGTERM, on_signal);    // on_signal函数会将全局变量g_stop_requested设置为1，表示程序需要停止运行

    std::atomic<bool> stop_requested{false};    // 原子变量，主循环开关，控制主循环是否继续运行
    if (!options.dry_run) transport.start();

    std::size_t processed_frames = 0;     // 已处理的帧数
    std::optional<srm::io::ImageGeometry> delivered_geometry;
    bool acquisition_failed = false;
    while (!stop_requested.load()) {
      if (g_stop_requested != 0) break;
      const auto loop_start = std::chrono::steady_clock::now();
      bool viewer_quit = false;
      last_detection.reset();                                       // 清理上一帧的检测结果

      auto result = runtime.step();                           // 返回的是一次处理结果的汇总
      if (!result.frame_read && frame_source.type() == srm::io::FrameSourceType::mvs_camera) {
        result.error = frame_source.last_error();
        acquisition_failed = true;
        std::cerr << result.error << '\n';
      }
      const srm::app::RuntimeTransportSnapshot transport_snapshot{  // 记录串口通信的统计信息，用于日志记录和统计
        transport.sent_frame_count(), transport.received_frame_count(),
        transport.communication_error_count()};
      statistics.observe(result, transport_snapshot);           
      timed(srm::app::TimingStage::logging, [&] { logger.log(result, transport_snapshot); });
      if (result.frame_read) {                                      // 成功读到了吗？
        ++processed_frames;
        if (result.frame.has_value()) {
          const auto & frame = result.frame.value();
          if (!delivered_geometry) {
            delivered_geometry = srm::io::ImageGeometry{frame.bgr.size(),frame.orientation_deg};
            if (recorder) srm::io::write_image_geometry(recorder->path() + ".metadata.yml", *delivered_geometry);
          }
          if (recorder) timed(srm::app::TimingStage::recording, [&] { recorder->write(frame.bgr); });
          const bool save_debug = last_detection.has_value() && !options.debug_directory.empty() &&
                                  frame.sequence % options.debug_every == 0;
          if (save_debug || debug_viewer.has_value()) {
            cv::Mat drawing;
            timed(srm::app::TimingStage::debug_draw, [&] {
              drawing = last_detection.has_value() ? detector.draw_debug(frame.bgr, last_detection.value()) : frame.bgr;
              if (result.pipeline) drawing = srm::app::draw_tracking_debug(drawing,*result.pipeline,pipeline_config.pose_solver);
            });
            if (save_debug) timed(srm::app::TimingStage::debug_save, [&] { save_debug_frame(options.debug_directory, frame, drawing); });
            if (debug_viewer.has_value()) {
              viewer_quit = !debug_viewer->show_and_wait(
                  drawing, frame, last_detection.has_value() ? &last_detection.value() : nullptr,
                  [] { return g_stop_requested != 0; });
              statistics.observe_duration(srm::app::TimingStage::gui_display, debug_viewer->display_duration());
              statistics.observe_duration(srm::app::TimingStage::gui_wait, debug_viewer->wait_duration());
            }
          }
        }
      }
      statistics.observe_duration(srm::app::TimingStage::loop, std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now() - loop_start));
      if (viewer_quit) break;

      if (result.end_of_stream || acquisition_failed) break;                            // EOF 或实时取流失败
      if (options.max_frames != 0 && processed_frames >= options.max_frames) break;     // 读到设定的最大帧数就退出
      if (!result.frame_read && (!options.dry_run && !transport.running())) break;      // 读不到帧了，且串口线程也没在运行，就退出
    }

    // Close the preview and restore terminal input before potentially slow recording validation.
    debug_viewer.reset();
    if (!options.dry_run) {         // 如果不是离线运行模式，就发送一条安全命令，关闭串口线程
      (void)transport.send_command_now(safe_command());
      transport.stop();
    }

    if (recorder) {
      std::cerr << "finalizing recording (full decode verification): " << recorder->path() << '\n';
      timed(srm::app::TimingStage::recording_close, [&] { recorder->close(); });
      std::cerr << "recorded_frames=" << recorder->frames_written()
                << " record_path=" << recorder->path() << '\n';
    }

    write_summary(        //  写入运行摘要文件
      options.summary_path, statistics, processed_frames, logger.rows_written(), options.dry_run,
      calibration.has_value(), !options.extrinsics_path.empty(), delivered_geometry);

    std::cerr << "processed_frames=" << processed_frames
              << " log_rows=" << logger.rows_written()
              << " dry_run=" << (options.dry_run ? 1 : 0) << '\n';
    return acquisition_failed ? EXIT_FAILURE : EXIT_SUCCESS;
  } catch (const std::exception & error) {
    std::cerr << "srm_auto_aim_app: " << error.what() << '\n';
    return EXIT_FAILURE;
  }
}
