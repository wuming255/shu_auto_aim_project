#include "vision/pose_solver.hpp"

#include <cassert>
#include <cmath>
#include <algorithm>
#include <limits>
#include <stdexcept>
#include <vector>

namespace
{
using srm::vision::ArmorPoseSolver;
using srm::vision::PoseSolverConfig;

constexpr double kEpsilon = 1e-3;

void assert_near(double actual, double expected, double tolerance = kEpsilon)
{
  assert(std::abs(actual - expected) < tolerance);
}

PoseSolverConfig make_config()
{
  PoseSolverConfig config;
  config.camera_matrix = cv::Matx33d(800.0, 0.0, 320.0, 0.0, 800.0, 240.0, 0.0, 0.0, 1.0);
  config.distortion_coefficients = {0.0, 0.0, 0.0, 0.0, 0.0};
  config.rotation_camera_to_gimbal = cv::Matx33d::eye();
  config.translation_camera_to_gimbal = cv::Vec3d(0.0, 0.0, 0.0);
  config.max_reprojection_rms_px = 0.1;
  return config;
}

void test_static_pose_round_trip()
{
  auto config = make_config();
  ArmorPoseSolver solver(config);

  const auto object_points = ArmorPoseSolver::object_points();
  const cv::Vec3d rotation_vector(0.0, 0.0, 0.0);
  const cv::Vec3d translation(0.2, -0.1, 5.0);
  std::vector<cv::Point3f> object_vector(object_points.begin(), object_points.end());
  std::vector<cv::Point2f> projected;
  cv::projectPoints(
    object_vector, rotation_vector, translation, cv::Mat(config.camera_matrix),
    cv::Mat(config.distortion_coefficients), projected);

  std::array<cv::Point2f, 4> image_points{};
  std::copy(projected.begin(), projected.end(), image_points.begin());
  const auto solution = solver.solve(image_points);

  assert(solution.valid);
  assert(solution.error.empty());
  assert_near(solution.translation_camera_m[0], translation[0]);
  assert_near(solution.translation_camera_m[1], translation[1]);
  assert_near(solution.translation_camera_m[2], translation[2]);
  assert_near(solution.translation_gimbal_m[0], translation[0]);
  assert_near(solution.translation_gimbal_m[1], translation[1]);
  assert_near(solution.translation_gimbal_m[2], translation[2]);
  assert_near(solution.range_m, cv::norm(translation));
  assert_near(solution.yaw_rad, std::atan2(translation[1], translation[0]));
  assert_near(
    solution.pitch_rad,
    std::atan2(translation[2], std::hypot(translation[0], translation[1])));
  assert(solution.reprojection_rms_px < 0.1);
}

void test_extrinsic_translation_and_rotation()
{
  auto config = make_config();
  config.rotation_camera_to_gimbal = cv::Matx33d(
    0.0, 0.0, 1.0,
    -1.0, 0.0, 0.0,
    0.0, -1.0, 0.0);
  config.translation_camera_to_gimbal = cv::Vec3d(0.1, 0.2, 0.3);
  config.max_reprojection_rms_px = 0.1;
  ArmorPoseSolver solver(config);

  const auto object_points = ArmorPoseSolver::object_points();
  std::vector<cv::Point3f> object_vector(object_points.begin(), object_points.end());
  std::vector<cv::Point2f> projected;
  const cv::Vec3d camera_translation(0.2, -0.1, 5.0);
  cv::projectPoints(
    object_vector, cv::Vec3d(0.0, 0.0, 0.0), camera_translation,
    cv::Mat(config.camera_matrix), cv::Mat(config.distortion_coefficients), projected);

  std::array<cv::Point2f, 4> image_points{};
  std::copy(projected.begin(), projected.end(), image_points.begin());
  const auto solution = solver.solve(image_points);
  assert(solution.valid);

  const auto expected = config.rotation_camera_to_gimbal * camera_translation +
                        config.translation_camera_to_gimbal;
  assert_near(solution.translation_gimbal_m[0], expected[0]);
  assert_near(solution.translation_gimbal_m[1], expected[1]);
  assert_near(solution.translation_gimbal_m[2], expected[2]);
}

void test_invalid_inputs()
{
  bool bad_camera_threw = false;
  try {
    auto config = make_config();
    config.camera_matrix(0, 0) = 0.0;
    ArmorPoseSolver solver(config);
  } catch (const std::invalid_argument &) {
    bad_camera_threw = true;
  }
  assert(bad_camera_threw);

  bool bad_rotation_threw = false;
  try {
    auto config = make_config();
    config.rotation_camera_to_gimbal(0, 0) = 2.0;
    ArmorPoseSolver solver(config);
  } catch (const std::invalid_argument &) {
    bad_rotation_threw = true;
  }
  assert(bad_rotation_threw);

  bool bad_point_threw = false;
  try {
    auto config = make_config();
    ArmorPoseSolver solver(config);
    std::array<cv::Point2f, 4> points{
      cv::Point2f(0.0F, 0.0F), cv::Point2f(1.0F, 0.0F), cv::Point2f(1.0F, 1.0F),
      cv::Point2f(std::numeric_limits<float>::quiet_NaN(), 1.0F)};
    (void)solver.solve(points);
  } catch (const std::invalid_argument &) {
    bad_point_threw = true;
  }
  assert(bad_point_threw);
}

}  // namespace

int main()
{
  test_static_pose_round_trip();
  test_extrinsic_translation_and_rotation();
  test_invalid_inputs();
  return 0;
}
