#include "app/application_options.hpp"
#include "app/debug_viewer.hpp"
#include "app/runtime_csv_logger.hpp"
#include "app/runtime_statistics.hpp"
#include "control/auto_aim_pipeline.hpp"
#include "control/auto_aim_runtime.hpp"
#include "control/safety_controller.hpp"
#include "io/frame_source.hpp"
#include "io/self_aim_transport.hpp"
#include "vision/traditional_detector.hpp"

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

srm::control::AutoAimPipelineConfig make_pipeline_config(const srm::app::ApplicationOptions & options)
{
  srm::control::AutoAimPipelineConfig config;
  if (options.intrinsics_set) {
    config.pose_solver.camera_matrix = cv::Matx33d(
      options.fx, 0.0, options.cx, 0.0, options.fy, options.cy, 0.0, 0.0, 1.0); // 创建标准内参矩阵
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
  std::size_t processed_frames, std::size_t log_rows, bool dry_run)
{
  if (path.empty()) return;

  std::ofstream output(path, std::ios::out | std::ios::trunc);    // 创建输出文件流
  if (!output.is_open()) throw std::runtime_error("failed to open summary: " + path);

  output << "processed_frames=" << processed_frames << '\n'
         << "log_rows=" << log_rows << '\n'
         << "dry_run=" << (dry_run ? 1 : 0) << '\n';
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

    if (!options.intrinsics_set) {
      std::cerr
        << "warning: camera intrinsics are not configured; use --intrinsics fx,fy,cx,cy "
           "before relying on PnP output\n";
    }

    prepare_debug_directory(options.debug_directory);

    srm::io::FrameSource frame_source(options.frame_source);          // 图像输入模块
    if (frame_source.type() == srm::io::FrameSourceType::mvs_camera) {
      std::cerr << frame_source.description();
    }
    srm::io::SelfAimTransport transport(options.transport);           // 串口通信模块
    srm::vision::TraditionalDetector detector;                        // 传统视觉检测模块
    srm::app::RuntimeCsvLogger logger(options.log_path);              // 运行日志记录模块
    srm::app::RuntimeStatistics statistics;                           // 运行统计模块，记录各个功能运行了多少次，有什么报错，运行了多久等信息
    std::optional<srm::vision::DetectionResult> last_detection;       // 本帧检测结果，供保存和窗口显示使用。
    std::optional<srm::app::DebugViewer> debug_viewer;
    if (options.show_debug) {
      const double nominal_fps = frame_source.nominal_fps();
      const double playback_fps = options.playback_fps > 0.0
                                   ? options.playback_fps
                                   : (std::isfinite(nominal_fps) && nominal_fps > 0.0
                                        ? nominal_fps : 30.0);
      debug_viewer.emplace(
        playback_fps,
        options.start_paused ||
          options.frame_source.type == srm::io::FrameSourceType::image_file);
      std::cerr << "debug windows: Space=pause/resume n=next q/Esc=quit playback_fps="
                << playback_fps << '\n';
    }

    srm::control::AutoAimRuntime runtime(                             // 一帧图像的总调度模块，安排各个模块的协作和数据流转
      frame_source, transport, srm::control::AutoAimPipeline(make_pipeline_config(options)),        // 瞄准算法模块
      srm::control::SafetyController(),                               // 安全控制模块
      [&detector, &last_detection](const srm::io::Frame & frame) {    // 这个匿名函数输入一帧图像，返回检测结果
        last_detection = detector.detect(frame.bgr);
        return last_detection.value();                                // 注意这里传入的是整个匿名函数，使后面在类中可以被回调
      });

    std::signal(SIGINT, on_signal);     // SIGINT是终端的ctrl+c信号，SIGTERM是kill命令发送的终止信号，on_signal是自定义的信号处理函数
    std::signal(SIGTERM, on_signal);    // on_signal函数会将全局变量g_stop_requested设置为1，表示程序需要停止运行

    std::atomic<bool> stop_requested{false};    // 原子变量，主循环开关，控制主循环是否继续运行
    if (!options.dry_run) transport.start();

    std::size_t processed_frames = 0;     // 已处理的帧数
    bool acquisition_failed = false;
    while (!stop_requested.load()) {
      if (g_stop_requested != 0) break;
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
      logger.log(result, transport_snapshot);                       // 记录运行日志
      if (result.frame_read) {                                      // 成功读到了吗？
        ++processed_frames;
        if (result.frame.has_value()) {
          const auto & frame = result.frame.value();
          const bool save_debug = last_detection.has_value() && !options.debug_directory.empty() &&
                                  frame.sequence % options.debug_every == 0;
          if (save_debug || debug_viewer.has_value()) {
            const auto drawing = last_detection.has_value()
                                   ? detector.draw_debug(frame.bgr, last_detection.value())
                                   : frame.bgr;
            if (save_debug) save_debug_frame(options.debug_directory, frame, drawing);
            if (debug_viewer.has_value() &&
                !debug_viewer->show_and_wait(
                  drawing, frame, last_detection.has_value() ? &last_detection.value() : nullptr,
                  [] { return g_stop_requested != 0; })) {
              break;
            }
          }
        }
      }

      if (result.end_of_stream || acquisition_failed) break;                            // EOF 或实时取流失败
      if (options.max_frames != 0 && processed_frames >= options.max_frames) break;     // 读到设定的最大帧数就退出
      if (!result.frame_read && (!options.dry_run && !transport.running())) break;      // 读不到帧了，且串口线程也没在运行，就退出
    }

    if (!options.dry_run) {         // 如果不是离线运行模式，就发送一条安全命令，关闭串口线程
      (void)transport.send_command_now(safe_command());
      transport.stop();
    }

    write_summary(        //  写入运行摘要文件
      options.summary_path, statistics, processed_frames, logger.rows_written(), options.dry_run);

    std::cerr << "processed_frames=" << processed_frames
              << " log_rows=" << logger.rows_written()
              << " dry_run=" << (options.dry_run ? 1 : 0) << '\n';
    return acquisition_failed ? EXIT_FAILURE : EXIT_SUCCESS;
  } catch (const std::exception & error) {
    std::cerr << "srm_auto_aim_app: " << error.what() << '\n';
    return EXIT_FAILURE;
  }
}
