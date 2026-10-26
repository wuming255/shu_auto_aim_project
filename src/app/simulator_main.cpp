#include "app/debug_viewer.hpp"
#include "app/tracking_debug.hpp"
#include "io/ros_image.hpp"
#include "io/simulator_control.hpp"
#include "vision/traditional_detector.hpp"

#include <chrono>
#include <cmath>
#include <csignal>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <opencv2/imgcodecs.hpp>

namespace
{
using Clock = std::chrono::steady_clock;
volatile std::sig_atomic_t stop_requested = 0;

void request_stop(int) { stop_requested = 1; }

void prepare_parent(const std::filesystem::path & path)
{
  if (!path.parent_path().empty()) std::filesystem::create_directories(path.parent_path());
}

class SimulatorDetector final : public rclcpp::Node
{
public:
  SimulatorDetector() : Node("srm_simulator_detector"), detector_(detector_config())
  {
    const auto image_topic = declare_parameter<std::string>("image_topic", "/image_raw");
    const auto camera_topic = declare_parameter<std::string>("camera_info_topic", "/camera_info");
    const auto debug_topic = declare_parameter<std::string>("debug_topic", "/srm_simulator/detection/image");
    const bool control_enabled = declare_parameter<bool>("control_enabled", true);
    const bool show_debug = declare_parameter<bool>("show_debug", true);
    max_frames_ = declare_parameter<std::int64_t>("max_frames", 0);
    debug_every_ = declare_parameter<std::int64_t>("debug_every", 20);
    idle_timeout_sec_ = declare_parameter<double>("idle_timeout_sec", 0.0);
    debug_directory_ = declare_parameter<std::string>("debug_dir", "");
    summary_path_ = declare_parameter<std::string>("summary_path", "");
    const auto log_path = declare_parameter<std::string>("log_path", "");
    if (!log_path.empty()) {
      prepare_parent(log_path);
      command_log_.open(log_path);
      if (!command_log_) throw std::runtime_error("cannot open simulator command log: " + log_path);
      command_log_ << "stamp_ns,target_state,yaw_deg,pitch_deg,delta_yaw_deg,delta_pitch_deg,range_m,track,aim_valid,safety,status\n";
    }
    if (max_frames_ < 0 || debug_every_ < 1 || !std::isfinite(idle_timeout_sec_) || idle_timeout_sec_ < 0) {
      throw std::invalid_argument("max_frames/idle_timeout_sec must be nonnegative; debug_every must be positive");
    }
    if (!debug_directory_.empty()) std::filesystem::create_directories(debug_directory_);
    if (show_debug) viewer_ = std::make_unique<srm::app::DebugViewer>(30.0, false, true);
    const auto qos = rclcpp::SensorDataQoS().keep_last(1);
    debug_publisher_ = create_publisher<sensor_msgs::msg::Image>(debug_topic, qos);
    if (control_enabled) {
      control_ = std::make_unique<srm::io::SimulatorControl>(control_config());
      const auto feedback_topic = declare_parameter<std::string>("feedback_topic", "/vision_receive_data");
      const auto command_topic = declare_parameter<std::string>("command_topic", "/vision_send_data");
      const auto tf_topic = declare_parameter<std::string>("tf_topic", "/tf");
      command_publisher_ = create_publisher<rm_interfaces::msg::VisionSendData>(command_topic, qos);
      feedback_subscription_ = create_subscription<rm_interfaces::msg::VisionReceiveData>(feedback_topic, qos,
        [this](rm_interfaces::msg::VisionReceiveData::ConstSharedPtr message) { control_->receive_feedback(*message); });
      tf_subscription_ = create_subscription<tf2_msgs::msg::TFMessage>(tf_topic, rclcpp::QoS(10),
        [this](tf2_msgs::msg::TFMessage::ConstSharedPtr message) { control_->receive_transforms(*message); });
      control_timer_ = create_wall_timer(std::chrono::milliseconds(20), [this] {
        if (!finished_) publish_command(control_->command(system_now_ns()));
      });
      RCLCPP_INFO(get_logger(), "Simulator control: feedback=%s TF=%s output=%s (50 Hz)",
        feedback_topic.c_str(), tf_topic.c_str(), command_topic.c_str());
    }
    image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic, qos, [this](sensor_msgs::msg::Image::ConstSharedPtr image) { on_image(*image); });
    camera_subscription_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_topic, qos, [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr info) {
        try {
          camera_ = srm::io::camera_from_ros_info(*info);
          if (control_) control_->set_camera(*camera_);
          RCLCPP_INFO_ONCE(get_logger(), "CameraInfo %dx%d fx=%.3f fy=%.3f cx=%.3f cy=%.3f orientation=0",
            camera_->geometry.size.width, camera_->geometry.size.height,
            camera_->camera_matrix(0, 0), camera_->camera_matrix(1, 1),
            camera_->camera_matrix(0, 2), camera_->camera_matrix(1, 2));
        } catch (const std::exception & error) {
          ++rejected_camera_info_;
          camera_.reset();
          if (control_) control_->clear_camera();
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "CameraInfo rejected: %s", error.what());
        }
      });
    last_frame_ = Clock::now();
    timer_ = create_wall_timer(std::chrono::milliseconds(200), [this] {
      if (idle_timeout_sec_ > 0 && std::chrono::duration<double>(Clock::now() - last_frame_).count() >= idle_timeout_sec_) {
        RCLCPP_ERROR(get_logger(), "No valid image for %.1f s; check simulator, ROS_DOMAIN_ID and image_topic", idle_timeout_sec_);
        failed_ = finished_ = true;
      } else if (processed_frames_ == 0) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for simulator images...");
      }
    });
    RCLCPP_INFO(get_logger(), "C++ simulator ready: image=%s camera=%s debug=%s show_debug=%d control=%d",
      image_topic.c_str(), camera_topic.c_str(), debug_topic.c_str(), show_debug, control_enabled);
  }

  bool finished() const noexcept { return finished_; }
  bool failed() const noexcept { return failed_; }

  void stop_control()
  {
    if (command_publisher_ && rclcpp::ok()) {
      publish_command(srm::io::SimulatorControl::cancellation(system_now_ns()));
    }
    if (command_log_.is_open()) command_log_.flush();
  }

  void write_summary() const
  {
    if (summary_path_.empty()) return;
    prepare_parent(summary_path_);
    std::ofstream output(summary_path_);
    if (!output) throw std::runtime_error("cannot open simulator summary: " + summary_path_);
    output << "processed_frames=" << processed_frames_ << '\n'
           << "frames_with_armors=" << frames_with_armors_ << '\n'
           << "armor_count=" << armor_count_ << '\n'
           << "rejected_images=" << rejected_images_ << '\n'
           << "rejected_camera_info=" << rejected_camera_info_ << '\n'
           << "camera_info_ready=" << camera_.has_value() << '\n'
           << "idle_timeout=" << failed_ << '\n'
           << "control_enabled=" << static_cast<bool>(control_) << '\n'
           << "command_count=" << command_count_ << '\n'
           << "aim_command_count=" << aim_command_count_ << '\n'
           << "fire_pulse_count=" << fire_pulse_count_ << '\n';
    if (control_) {
      output << "pipeline_frames=" << control_->pipeline_frames() << '\n'
             << "geometry_ready=" << control_->geometry_ready() << '\n'
             << "rejected_feedback=" << control_->rejected_feedback() << '\n'
             << "rejected_transforms=" << control_->rejected_transforms() << '\n';
    }
    if (camera_) {
      output << "image_width=" << camera_->geometry.size.width << '\n'
             << "image_height=" << camera_->geometry.size.height << '\n'
             << "fx=" << std::setprecision(16) << camera_->camera_matrix(0, 0) << '\n'
             << "fy=" << camera_->camera_matrix(1, 1) << '\n'
             << "cx=" << camera_->camera_matrix(0, 2) << '\n'
             << "cy=" << camera_->camera_matrix(1, 2) << '\n';
    }
    output.flush();
    if (!output) throw std::runtime_error("cannot write simulator summary: " + summary_path_);
  }

private:
  static std::int64_t system_now_ns()
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
  }

  srm::io::SimulatorControlConfig control_config()
  {
    srm::io::SimulatorControlConfig config;
    config.aim_mode = declare_parameter<std::string>("aim_mode", "follow");
    const auto color = declare_parameter<std::string>("enemy_color", "blue");
    if (color != "blue" && color != "red") throw std::invalid_argument("enemy_color must be blue or red");
    config.enemy_color = color == "blue" ? srm::vision::LightColor::blue : srm::vision::LightColor::red;
    config.allow_visual_fire = declare_parameter<bool>("allow_visual_fire", true);
    config.frame_max_age_sec = declare_parameter<double>("frame_max_age_sec", 0.25);
    config.feedback_max_age_sec = declare_parameter<double>("feedback_max_age_sec", 0.25);
    config.fire_period_sec = declare_parameter<double>("fire_period_sec", 0.2);
    config.pitch_limit_deg = declare_parameter<double>("pitch_limit_deg", 45.0);
    config.small_feature_width_m = declare_parameter<double>("small_feature_width_m", 0.1258);
    config.large_feature_width_m = declare_parameter<double>("large_feature_width_m", 0.2208);
    config.feature_height_m = declare_parameter<double>("feature_height_m", 0.05961);
    config.mounting_tilt_rad = declare_parameter<double>("mounting_tilt_rad", 0.2617993877991494);
    config.additional_latency_sec = declare_parameter<double>("additional_latency_sec", 0.0);
    return config;
  }

  void publish_command(const rm_interfaces::msg::VisionSendData & command)
  {
    command_publisher_->publish(command);
    ++command_count_;
    if (command.target_state) ++aim_command_count_;
    if (command.target_state == 2 && previous_target_state_ != 2) ++fire_pulse_count_;
    previous_target_state_ = command.target_state;
    if (command_log_.is_open()) {
      const auto & result = control_->result();
      command_log_ << system_now_ns() << ',' << static_cast<int>(command.target_state) << ','
        << command.yaw << ',' << command.pitch << ',' << command.delta_yaw << ',' << command.delta_pitch << ','
        << command.target_distance << ',' << (result ? srm::vision::to_string(result->tracking.state) : "lost") << ','
        << (result && result->aim.valid) << ',' << std::quoted(control_->safety().summary) << ','
        << std::quoted(control_->status()) << '\n';
      if (!command_log_) throw std::runtime_error("cannot write simulator command log");
    }
  }

  srm::vision::TraditionalDetectorConfig detector_config()
  {
    srm::vision::TraditionalDetectorConfig config;
    const auto method = declare_parameter<std::string>("brightness_method", "max_channel");
    if (method != "max_channel" && method != "grayscale") {
      throw std::invalid_argument("brightness_method must be max_channel or grayscale");
    }
    config.brightness_method = method == "max_channel" ? srm::vision::BrightnessMethod::max_channel
                                                       : srm::vision::BrightnessMethod::grayscale;
    config.brightness_threshold = declare_parameter<double>("brightness_threshold", 160.0);
    config.classifier.enabled = declare_parameter<bool>("classifier_enabled", true);
    config.classifier.model_path = declare_parameter<std::string>(
      "classifier_model", std::string(SRM_DEFAULT_NUMBER_MODEL_DIR) + "/mlp.onnx");
    config.classifier.label_path = declare_parameter<std::string>(
      "classifier_labels", std::string(SRM_DEFAULT_NUMBER_MODEL_DIR) + "/label.txt");
    config.classifier.min_confidence = declare_parameter<double>("classifier_confidence", 0.7);
    return config;
  }

  void on_image(const sensor_msgs::msg::Image & message)
  {
    if (finished_) return;
    srm::io::Frame frame;
    frame.timestamp = Clock::now();
    srm::vision::DetectionResult detection;
    try {
      frame.bgr = srm::io::decode_ros_image(message);
      if (camera_) srm::vision::validate_geometry(camera_->geometry, {frame.bgr.size(), 0});
      detection = detector_.detect(frame.bgr);
    } catch (const std::exception & error) {
      ++rejected_images_;
      if (control_) control_->invalidate_frame();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Image rejected: %s", error.what());
      return;
    }
    last_frame_ = Clock::now();
    frame.sequence = processed_frames_++;
    frame.source_time_seconds = rclcpp::Time(message.header.stamp).seconds();
    armor_count_ += detection.armors.size();
    if (!detection.armors.empty()) ++frames_with_armors_;
    if (control_) {
      try {
        control_->process(frame, detection, rclcpp::Time(message.header.stamp).nanoseconds());
      } catch (const std::exception & error) {
        control_->invalidate_frame();
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Simulator pipeline rejected: %s", error.what());
      }
    }
    auto drawing = detector_.draw_debug(frame.bgr, detection);
    if (control_) {
      if (control_->result() && control_->pose_config()) {
        drawing = srm::app::draw_tracking_debug(drawing, *control_->result(), *control_->pose_config());
      }
      cv::putText(drawing, "sim: " + control_->status(), {8, 80}, cv::FONT_HERSHEY_SIMPLEX,
        0.5, {0, 255, 255}, 1, cv::LINE_AA);
    }
    if (debug_publisher_->get_subscription_count() > 0) {
      sensor_msgs::msg::Image debug;
      debug.header = message.header;
      debug.width = static_cast<std::uint32_t>(drawing.cols);
      debug.height = static_cast<std::uint32_t>(drawing.rows);
      debug.encoding = "bgr8";
      debug.step = debug.width * 3;
      debug.data.resize(static_cast<std::size_t>(debug.step) * debug.height);
      for (int row = 0; row < drawing.rows; ++row) {
        std::memcpy(debug.data.data() + static_cast<std::size_t>(row) * debug.step, drawing.ptr(row), debug.step);
      }
      debug_publisher_->publish(std::move(debug));
    }
    if (!debug_directory_.empty() && frame.sequence % static_cast<std::uint64_t>(debug_every_) == 0) {
      std::ostringstream name;
      name << "frame_" << std::setfill('0') << std::setw(6) << frame.sequence << ".png";
      const auto path = std::filesystem::path(debug_directory_) / name.str();
      if (!cv::imwrite(path.string(), drawing)) throw std::runtime_error("cannot save simulator debug image: " + path.string());
    }
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
      "frames=%llu lights=%zu armors=%zu frames_with_armors=%llu control=%s",
      static_cast<unsigned long long>(processed_frames_), detection.lightbars.size(), detection.armors.size(),
      static_cast<unsigned long long>(frames_with_armors_), control_ ? control_->status().c_str() : "disabled");
    if (viewer_ && !viewer_->show_and_wait(drawing, frame, &detection,
        [] { return !rclcpp::ok() || stop_requested; })) finished_ = true;
    if (max_frames_ > 0 && processed_frames_ >= static_cast<std::uint64_t>(max_frames_)) finished_ = true;
  }

  srm::vision::TraditionalDetector detector_;
  std::unique_ptr<srm::app::DebugViewer> viewer_;
  std::optional<srm::vision::CameraCalibration> camera_;
  std::unique_ptr<srm::io::SimulatorControl> control_;
  rclcpp::Publisher<rm_interfaces::msg::VisionSendData>::SharedPtr command_publisher_;
  rclcpp::Subscription<rm_interfaces::msg::VisionReceiveData>::SharedPtr feedback_subscription_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_subscription_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  std::ofstream command_log_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::int64_t max_frames_ = 0, debug_every_ = 20;
  double idle_timeout_sec_ = 0;
  std::string debug_directory_, summary_path_;
  Clock::time_point last_frame_;
  std::uint64_t processed_frames_ = 0, frames_with_armors_ = 0, armor_count_ = 0;
  std::uint64_t rejected_images_ = 0, rejected_camera_info_ = 0;
  std::uint64_t command_count_ = 0, aim_command_count_ = 0, fire_pulse_count_ = 0;
  std::uint8_t previous_target_state_ = 0;
  bool finished_ = false, failed_ = false;
};

void usage()
{
  std::cout << "Usage: bash run.sh simulator [--ros-args -p NAME:=VALUE ...]\n"
    "C++ ROS 2 auto aim using the existing detector, pose solver, follow/guard pipeline and safety gate.\n"
    "  image_topic          /image_raw\n"
    "  camera_info_topic    /camera_info (native upright CameraInfo; no calibration file needed)\n"
    "  debug_topic          /srm_simulator/detection/image (BGR8, original header)\n"
    "  show_debug           true (q/Esc/Ctrl+C to quit; set false for headless)\n"
    "  brightness_method    max_channel (or grayscale)\n"
    "  brightness_threshold 160.0\n"
    "  classifier_enabled   true\n"
    "  classifier_model / classifier_labels / classifier_confidence (default 0.7)\n"
    "  control_enabled      true (false restores recognition-only mode)\n"
    "  feedback_topic       /vision_receive_data; tf_topic=/tf\n"
    "  command_topic        /vision_send_data (50 Hz; world gimbal angles in degrees)\n"
    "  aim_mode             follow (current armor + ballistics) or guard (rotation prediction/fixed guard)\n"
    "  enemy_color          blue (or red)\n"
    "  allow_visual_fire    false (true enables simulated fire after safety/settle gates)\n"
    "  frame_max_age_sec / feedback_max_age_sec 0.25; fire_period_sec=0.2; pitch_limit_deg=45.0\n"
    "  small_feature_width_m=0.1258; large_feature_width_m=0.2208; feature_height_m=0.05961\n"
    "  mounting_tilt_rad=0.2617993877991494; additional_latency_sec=0.0\n"
    "  log_path             empty (optional actual command CSV)\n"
    "  max_frames           0 (unlimited)\n"
    "  idle_timeout_sec     0.0 (wait indefinitely; positive value exits with failure on input timeout)\n"
    "  debug_dir            empty (optional saved detections); debug_every=20\n"
    "  summary_path         empty (optional key=value run summary)\n"
    "Use the same ROS_DOMAIN_ID as the simulator, robot camera mode (F3), and enable auto-aim input (F5).\n";
}
}  // namespace

int main(int argc, char ** argv)
{
  for (int index = 1; index < argc; ++index) {
    if (std::string(argv[index]) == "--help" || std::string(argv[index]) == "-h") { usage(); return 0; }
  }
  int result = 0;
  std::shared_ptr<SimulatorDetector> node;
  try {
    rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
    std::signal(SIGINT, request_stop);
    std::signal(SIGTERM, request_stop);
    node = std::make_shared<SimulatorDetector>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    while (rclcpp::ok() && !stop_requested && !node->finished()) executor.spin_once(std::chrono::milliseconds(100));
    result = node->failed() ? 1 : 0;
  } catch (const std::exception & error) {
    std::cerr << "srm_simulator_app: " << error.what() << '\n';
    result = 1;
  }
  if (node) {
    try {
      node->stop_control();
      node->write_summary();
    } catch (const std::exception & error) {
      std::cerr << "srm_simulator_app shutdown: " << error.what() << '\n';
      result = 1;
    }
  }
  if (rclcpp::ok()) rclcpp::shutdown();
  return result;
}
