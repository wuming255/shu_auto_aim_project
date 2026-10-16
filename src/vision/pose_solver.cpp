#include "vision/pose_solver.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace srm::vision
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

bool finite(double value)
{
  return std::isfinite(value);
}

bool finite_point(const cv::Point2f & point)
{
  return finite(point.x) && finite(point.y);
}

double determinant(const cv::Matx33d & matrix)
{
  return cv::determinant(cv::Mat(matrix));
}

cv::Vec3d read_vec3(const cv::Mat & matrix)
{
  const auto values = matrix.reshape(1, 3);
  cv::Mat converted;
  values.convertTo(converted, CV_64F);
  return {
    converted.at<double>(0, 0), converted.at<double>(1, 0), converted.at<double>(2, 0)};
}

bool all_positive_depth(const cv::Vec3d & translation, const cv::Vec3d & rotation_vector,
                        const std::array<cv::Point3f, 4> & points)
{
  cv::Mat rotation_matrix;
  cv::Rodrigues(rotation_vector, rotation_matrix);
  cv::Mat converted;
  rotation_matrix.convertTo(converted, CV_64F);
  const cv::Matx33d rotation(
    converted.at<double>(0, 0), converted.at<double>(0, 1), converted.at<double>(0, 2),
    converted.at<double>(1, 0), converted.at<double>(1, 1), converted.at<double>(1, 2),
    converted.at<double>(2, 0), converted.at<double>(2, 1), converted.at<double>(2, 2));
  for (const auto & point : points) {
    const cv::Vec3d camera_point = rotation * cv::Vec3d(point.x, point.y, point.z) + translation;
    if (!finite(camera_point[2]) || camera_point[2] <= 0.0) return false;
  }
  return true;
}

}  // namespace

ArmorPoseSolver::ArmorPoseSolver(PoseSolverConfig config)
: config_(std::move(config)),
  object_points_(object_points(config_.feature_size_m ? config_.feature_size_m->width : config_.armor_width_m,
                              config_.feature_size_m ? config_.feature_size_m->height : config_.armor_height_m))
{
  validate_config();
}

PoseSolution ArmorPoseSolver::solve(const std::array<cv::Point2f, 4> & image_points) const
{
  const auto candidates = solve_candidates(image_points);
  if (!candidates.empty()) return candidates.front();
  PoseSolution failure;
  failure.error = "PnP produced no positive-depth pose within reprojection RMS limit";
  return failure;
}

std::vector<PoseSolution> ArmorPoseSolver::solve_candidates(
  const std::array<cv::Point2f, 4> & image_points) const
{
  for (const auto & point : image_points) {
    if (!finite_point(point)) throw std::invalid_argument("armor image point is not finite");
  }

  const std::vector<cv::Point3f> object_points(object_points_.begin(), object_points_.end());
  const std::vector<cv::Point2f> observed_points(image_points.begin(), image_points.end());

  std::vector<cv::Mat> rotation_vectors;
  std::vector<cv::Mat> translation_vectors;
  const int solution_count = cv::solvePnPGeneric(
    object_points, observed_points, cv::Mat(config_.camera_matrix),
    cv::Mat(config_.distortion_coefficients), rotation_vectors, translation_vectors, false,
    cv::SOLVEPNP_IPPE);

  std::vector<PoseSolution> candidates;
  if (solution_count <= 0 || rotation_vectors.empty() || translation_vectors.empty()) {
    return candidates;
  }

  for (std::size_t i = 0; i < rotation_vectors.size() && i < translation_vectors.size(); ++i) {
    const cv::Vec3d rotation_vector = read_vec3(rotation_vectors[i]);
    const cv::Vec3d translation = read_vec3(translation_vectors[i]);

    if (!all_positive_depth(translation, rotation_vector, object_points_)) continue;

    std::vector<cv::Point2f> projected_points;
    cv::projectPoints(
      object_points, rotation_vector, translation, cv::Mat(config_.camera_matrix),
      cv::Mat(config_.distortion_coefficients), projected_points);
    if (projected_points.size() != image_points.size()) continue;

    double squared_error = 0.0;
    for (std::size_t point_index = 0; point_index < image_points.size(); ++point_index) {
      const auto dx = static_cast<double>(projected_points[point_index].x - image_points[point_index].x);
      const auto dy = static_cast<double>(projected_points[point_index].y - image_points[point_index].y);
      squared_error += dx * dx + dy * dy;
    }
    const double rms = std::sqrt(squared_error / image_points.size());
    if (!finite(rms) || rms > config_.max_reprojection_rms_px) continue;

    const cv::Vec3d translation_gimbal =
      config_.rotation_camera_to_gimbal * translation + config_.translation_camera_to_gimbal;
    if (!finite(translation_gimbal[0]) || !finite(translation_gimbal[1]) ||
        !finite(translation_gimbal[2])) {
      continue;
    }

    PoseSolution best;
    best.valid = true;
    best.error.clear();
    best.translation_camera_m = translation;
    best.translation_gimbal_m = translation_gimbal;
    best.rotation_vector_camera = rotation_vector;
    best.range_m = cv::norm(translation_gimbal);
    best.yaw_rad = std::atan2(translation_gimbal[1], translation_gimbal[0]);
    best.pitch_rad = std::atan2(
      translation_gimbal[2], std::hypot(translation_gimbal[0], translation_gimbal[1]));
    best.yaw_deg = best.yaw_rad * 180.0 / kPi;
    best.pitch_deg = best.pitch_rad * 180.0 / kPi;
    best.reprojection_rms_px = rms;
    best.feature_geometry_confirmed = config_.feature_size_m.has_value();
    candidates.push_back(best);
  }

  std::stable_sort(candidates.begin(), candidates.end(), [](const auto & a, const auto & b) {
    return a.reprojection_rms_px < b.reprojection_rms_px;
  });
  return candidates;
}

PoseSolution ArmorPoseSolver::solve(const ArmorObservation & armor) const
{
  return solve(armor.points);
}

const PoseSolverConfig & ArmorPoseSolver::config() const noexcept
{
  return config_;
}

std::array<cv::Point3f, 4> ArmorPoseSolver::object_points(double width_m, double height_m)
{
  if (!finite(width_m) || !finite(height_m) || width_m <= 0.0 || height_m <= 0.0) {
    throw std::invalid_argument("armor object dimensions must be finite and positive");
  }

  const float half_width = static_cast<float>(width_m / 2.0);
  const float half_height = static_cast<float>(height_m / 2.0);
  // 物点顺序匹配检测器的 TL, TR, BR, BL 像素顺序；目标局部坐标系为 X 右、Y 上。
  return {{{-half_width, half_height, 0.0F},
           {half_width, half_height, 0.0F},
           {half_width, -half_height, 0.0F},
           {-half_width, -half_height, 0.0F}}};
}

void ArmorPoseSolver::validate_config() const
{
  if (!finite(config_.armor_width_m) || config_.armor_width_m <= 0 ||
      !finite(config_.armor_height_m) || config_.armor_height_m <= 0) {
    throw std::invalid_argument("invalid armor surface dimensions");
  }
  const auto & matrix = config_.camera_matrix;
  if (!finite(matrix(0, 0)) || !finite(matrix(1, 1)) || matrix(0, 0) <= 0.0 ||
      matrix(1, 1) <= 0.0 || !finite(matrix(0, 1)) || !finite(matrix(0, 2)) ||
      !finite(matrix(1, 2)) || !finite(matrix(2, 0)) || !finite(matrix(2, 1)) ||
      !finite(matrix(2, 2)) || std::abs(matrix(2, 0)) > 1e-9 ||
      std::abs(matrix(2, 1)) > 1e-9 || std::abs(matrix(2, 2) - 1.0) > 1e-9) {
    throw std::invalid_argument("invalid camera intrinsic matrix");
  }

  for (const auto coefficient : config_.distortion_coefficients) {
    if (!finite(coefficient)) throw std::invalid_argument("distortion coefficient is not finite");
  }
  const auto count = config_.distortion_coefficients.size();
  if (!(count == 0 || count == 4 || count == 5 || count == 8 || count == 12 || count == 14)) {
    throw std::invalid_argument("unsupported OpenCV distortion coefficient count");
  }

  const auto & rotation = config_.rotation_camera_to_gimbal;
  for (const auto value : rotation.val) {
    if (!finite(value)) throw std::invalid_argument("extrinsic rotation is not finite");
  }
  const cv::Matx33d orthogonality_error = rotation * rotation.t() - cv::Matx33d::eye();
  if (cv::norm(cv::Mat(orthogonality_error), cv::NORM_INF) >
      config_.rotation_orthogonality_tolerance) {
    throw std::invalid_argument("camera-to-gimbal rotation is not orthonormal");
  }
  if (std::abs(determinant(rotation) - 1.0) > config_.rotation_orthogonality_tolerance) {
    throw std::invalid_argument("camera-to-gimbal rotation determinant must be +1");
  }

  for (const auto value : config_.translation_camera_to_gimbal.val) {
    if (!finite(value)) throw std::invalid_argument("extrinsic translation is not finite");
  }
  if (!finite(config_.max_reprojection_rms_px) || config_.max_reprojection_rms_px <= 0.0) {
    throw std::invalid_argument("maximum reprojection RMS must be finite and positive");
  }
  if (!finite(config_.rotation_orthogonality_tolerance) ||
      config_.rotation_orthogonality_tolerance <= 0.0) {
    throw std::invalid_argument("rotation tolerance must be finite and positive");
  }
}

}  // namespace srm::vision
