#ifndef SHU_AUTO_AIM_SIMULATOR_CONTROL_HPP
#define SHU_AUTO_AIM_SIMULATOR_CONTROL_HPP

#include <array>
#include <cstdint>
#include <deque>
#include <memory>
#include <optional>
#include <string>

#include <rm_interfaces/msg/vision_receive_data.hpp>
#include <rm_interfaces/msg/vision_send_data.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include "control/safety_controller.hpp"
#include "io/frame_source.hpp"
#include "vision/camera_calibration.hpp"

namespace srm::io
{
struct SimulatorControlConfig
{
  std::string aim_mode = "follow";
  vision::LightColor enemy_color = vision::LightColor::blue;
  bool allow_visual_fire = false;
  double frame_max_age_sec = 0.25;
  double feedback_max_age_sec = 0.25;
  double fire_period_sec = 0.2;
  double pitch_limit_deg = 45.0;
  double small_feature_width_m = 0.1258;
  double large_feature_width_m = 0.2208;
  double feature_height_m = 0.05961;
  double mounting_tilt_rad = 0.2617993877991494;
  double additional_latency_sec = 0.0;
};

// Bevy's ROS boundary only: the original C++ pipelines and safety gate remain authoritative.
// All callbacks and command() must run on the same executor thread.
class SimulatorControl final
{
public:
  explicit SimulatorControl(SimulatorControlConfig config);
  void set_camera(const vision::CameraCalibration & camera);
  void clear_camera();
  void receive_feedback(const rm_interfaces::msg::VisionReceiveData & message);
  void receive_transforms(const tf2_msgs::msg::TFMessage & message);
  void process(const Frame & frame, const vision::DetectionResult & detection, std::int64_t source_stamp_ns);
  void invalidate_frame();
  rm_interfaces::msg::VisionSendData command(std::int64_t now_ros_ns);
  static rm_interfaces::msg::VisionSendData cancellation(std::int64_t now_ros_ns);

  const std::optional<control::AutoAimFrameResult> & result() const noexcept { return result_; }
  const control::SafetyOutput & safety() const noexcept { return last_safety_; }
  const std::optional<vision::PoseSolverConfig> & pose_config() const noexcept { return selected_pose_; }
  const std::string & status() const noexcept { return status_; }
  std::uint64_t pipeline_frames() const noexcept { return pipeline_frames_; }
  std::uint64_t rejected_feedback() const noexcept { return rejected_feedback_; }
  std::uint64_t rejected_transforms() const noexcept { return rejected_transforms_; }
  bool geometry_ready() const noexcept { return mount_.has_value(); }

private:
  struct Geometry
  {
    std::int64_t stamp_ns = 0;
    cv::Matx33d barrel_to_world = cv::Matx33d::eye();
    cv::Vec3d camera_local{}, muzzle_local{}, origin_world{};
  };
  void reset_pipelines();
  void prepare_pipelines(const Geometry & geometry, const rm_interfaces::msg::VisionReceiveData & feedback);
  cv::Vec3d muzzle_world(const Geometry & geometry,
    const rm_interfaces::msg::VisionReceiveData & feedback) const;
  std::optional<rm_interfaces::msg::VisionReceiveData> feedback_at(std::int64_t stamp_ns) const;
  SimulatorControlConfig config_;
  control::SafetyController safety_;
  std::optional<vision::CameraCalibration> camera_;
  std::deque<rm_interfaces::msg::VisionReceiveData> feedback_;
  std::deque<Geometry> geometry_;
  std::array<std::unique_ptr<control::AutoAimPipeline>, 2> pipelines_;
  std::optional<cv::Matx33d> mount_;
  std::optional<cv::Vec3d> camera_from_muzzle_, guard_origin_;
  std::optional<control::AutoAimFrameResult> result_;
  std::optional<vision::PoseSolverConfig> selected_pose_;
  control::SafetyOutput last_safety_;
  std::chrono::steady_clock::time_point frame_time_{};
  std::int64_t source_stamp_ns_ = 0, pulse_until_ns_ = 0, next_fire_ns_ = 0;
  std::size_t selected_type_ = 0;
  std::uint64_t pipeline_frames_ = 0, rejected_feedback_ = 0, rejected_transforms_ = 0;
  std::string status_ = "waiting_for_feedback_and_tf";
};
}  // namespace srm::io
#endif
