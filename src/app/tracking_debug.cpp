#include "app/tracking_debug.hpp"

#include <cmath>
#include <iomanip>
#include <sstream>

namespace srm::app
{
namespace
{
std::optional<cv::Point> project(const cv::Vec3d & world,
  const vision::RotationEstimate & rotation, const vision::PoseSolverConfig & pose)
{
  const cv::Vec3d camera = pose.rotation_camera_to_gimbal.t() *
    (rotation.rotation_gimbal_to_world.t()*world-pose.translation_camera_to_gimbal);
  if (!cv::checkRange(cv::Mat(camera)) || camera[2] <= 1e-6) return std::nullopt;
  std::vector<cv::Point2d> pixels;
  cv::projectPoints(std::vector<cv::Point3d>{{camera[0],camera[1],camera[2]}},
    cv::Vec3d{},cv::Vec3d{},pose.camera_matrix,pose.distortion_coefficients,pixels);
  const auto & pixel = pixels.front();
  if (!std::isfinite(pixel.x) || !std::isfinite(pixel.y) ||
      std::abs(pixel.x) > 1e6 || std::abs(pixel.y) > 1e6) return std::nullopt;
  return cv::Point(cvRound(pixel.x),cvRound(pixel.y));
}
}

cv::Mat draw_tracking_debug(const cv::Mat & drawing,
  const control::AutoAimFrameResult & result, const vision::PoseSolverConfig & pose)
{
  cv::Mat output = drawing.clone();
  const auto & track = result.tracking;
  const auto & rotation = result.rotation;
  std::ostringstream observation;
  observation << "track=" << vision::to_string(track.state);
  if (track.has_current_measurement && track.current_observation) {
    const auto & current = *track.current_observation;
    std::vector<cv::Point> corners;
    for (const auto & point : current.armor.points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
          std::abs(point.x) > 1e6 || std::abs(point.y) > 1e6) { corners.clear(); break; }
      corners.emplace_back(cvRound(point.x),cvRound(point.y));
    }
    if (corners.size() == 4) cv::polylines(output,corners,true,{255,0,255},3,cv::LINE_AA);
    observation << " visible id=" << current.detection_index
                << " label=" << current.armor.classification.label;
  } else observation << " no current observation";

  std::ostringstream model;
  model << "rotation=" << rotation.status;
  std::ostringstream quality;
  quality << std::fixed << std::setprecision(3);
  if (rotation.initialized) {
    quality << "r=" << rotation.radius_m << "m phase=" << rotation.phase_rad
            << " omega=" << rotation.angular_velocity_rad_s << "rad/s "
            << (rotation.reliable ? "RELIABLE" : "DIAGNOSTIC") << " ";
  }
  quality << "prediction=" << ((result.follow_target ? track.prediction_valid : rotation.prediction_valid) ? "VALID" : "INVALID");
  if (result.follow_target && result.aim.valid) {
    quality << " lead=" << result.aim.total_prediction_lead_s << "s";
  }
  if (rotation.prediction_valid && rotation.initialized && rotation.stable_frame_valid) {
    for (int plate = 0; plate < 4; ++plate) {
      const auto pixel = project(rotation.armor_position(0,plate),rotation,pose);
      if (!pixel) continue;
      cv::drawMarker(output,*pixel,{255,255,0},cv::MARKER_CROSS,
        plate == rotation.associated_plate ? 16 : 10,2,cv::LINE_AA);
    }
    const auto future = project(rotation.armor_position(0.1,rotation.associated_plate),rotation,pose);
    if (future) cv::circle(output,*future,7,{255,255,0},2,cv::LINE_AA);
    quality << (rotation.current_visible ? " [cyan: model, circle +0.10s]" : " PREDICTION ONLY");
  }
  const std::array<std::string,3> lines{observation.str(),model.str(),quality.str()};
  for (std::size_t row = 0; row < lines.size(); ++row) {
    const cv::Point origin(8,18+static_cast<int>(row)*20);
    cv::putText(output,lines[row],origin,cv::FONT_HERSHEY_SIMPLEX,0.48,{0,0,0},3,cv::LINE_AA);
    cv::putText(output,lines[row],origin,cv::FONT_HERSHEY_SIMPLEX,0.48,{255,255,255},1,cv::LINE_AA);
  }
  return output;
}
}
