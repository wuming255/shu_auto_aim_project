#include "vision/target_selector.hpp"

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace
{
using srm::vision::ArmorObservation;
using srm::vision::ArmorPoseSolver;
using srm::vision::LightColor;
using srm::vision::PoseSolverConfig;
using srm::vision::TargetSelector;
using srm::vision::TargetSelectorConfig;

PoseSolverConfig make_solver_config()
{
  PoseSolverConfig config;
  config.camera_matrix = cv::Matx33d(800.0, 0.0, 320.0, 0.0, 800.0, 240.0, 0.0, 0.0, 1.0);
  config.distortion_coefficients = {0.0, 0.0, 0.0, 0.0, 0.0};
  config.rotation_camera_to_gimbal = cv::Matx33d(
    0.0, 0.0, 1.0,
    -1.0, 0.0, 0.0,
    0.0, -1.0, 0.0);
  config.translation_camera_to_gimbal = cv::Vec3d(0.0, 0.0, 0.0);
  config.max_reprojection_rms_px = 0.1;
  return config;
}

ArmorObservation make_observation(
  const ArmorPoseSolver & solver, const cv::Vec3d & translation, LightColor color)
{
  const auto object_points = ArmorPoseSolver::object_points();
  const std::vector<cv::Point3f> object_vector(object_points.begin(), object_points.end());
  std::vector<cv::Point2f> projected;
  cv::projectPoints(
    object_vector, cv::Vec3d(0.0, 0.0, 0.0), translation,
    cv::Mat(solver.config().camera_matrix), cv::Mat(solver.config().distortion_coefficients),
    projected);

  ArmorObservation observation;
  observation.color = color;
  std::copy(projected.begin(), projected.end(), observation.points.begin());
  observation.center = (observation.points[0] + observation.points[1] + observation.points[2] +
                        observation.points[3]) /
                       4.0F;
  return observation;
}

void test_selects_nearest_valid_target_and_preserves_frame_metadata()
{
  ArmorPoseSolver solver(make_solver_config());
  srm::vision::DetectionResult detections;
  detections.armors.push_back(
    make_observation(solver, cv::Vec3d(0.8, 0.0, 5.0), LightColor::blue));
  detections.armors.push_back(
    make_observation(solver, cv::Vec3d(0.1, 0.0, 5.0), LightColor::blue));

  TargetSelector selector;
  const auto timestamp = std::chrono::steady_clock::now();
  const auto result = selector.select(detections, solver, timestamp, 42);

  assert(result.observations.size() == 2);
  assert(result.has_target());
  assert(result.selected() != nullptr);
  assert(result.selected()->detection_index == 1);
  assert(result.selected()->frame_sequence == 42);
  assert(result.selected()->timestamp == timestamp);
  assert(result.selected()->pose.valid);
  assert(result.selected()->aim_error_rad < result.observations[0].aim_error_rad);
}

void test_selection_tie_break_is_stable()
{
  ArmorPoseSolver solver(make_solver_config());
  srm::vision::DetectionResult detections;
  const auto observation = make_observation(
    solver, cv::Vec3d(0.2, 0.0, 5.0), LightColor::blue);
  detections.armors.push_back(observation);
  detections.armors.push_back(observation);

  TargetSelector selector;
  const auto result = selector.select(detections, solver, std::chrono::steady_clock::now(), 43);
  assert(result.has_target());
  assert(result.selected()->detection_index == 0);
}

void test_color_and_quality_filters()
{
  ArmorPoseSolver solver(make_solver_config());
  srm::vision::DetectionResult detections;
  detections.armors.push_back(
    make_observation(solver, cv::Vec3d(0.0, 0.0, 5.0), LightColor::red));
  detections.armors.push_back(
    make_observation(solver, cv::Vec3d(0.2, 0.0, 5.0), LightColor::blue));

  TargetSelectorConfig config;
  config.filter_color = true;
  config.required_color = LightColor::blue;
  config.max_aim_error_rad = 0.1;
  TargetSelector selector(config);
  const auto result = selector.select(detections, solver, std::chrono::steady_clock::now(), 7);

  assert(result.has_target());
  assert(result.selected()->detection_index == 1);
  assert(result.observations[0].pose.valid);
  assert(result.observations[0].armor.color == LightColor::red);

  TargetSelectorConfig too_strict;
  too_strict.max_range_m = 1.0;
  TargetSelector strict_selector(too_strict);
  const auto rejected =
    strict_selector.select(detections, solver, std::chrono::steady_clock::now(), 8);
  assert(!rejected.has_target());
  assert(rejected.observations.size() == detections.armors.size());
}

void test_bad_candidate_isolated_and_empty_result()
{
  ArmorPoseSolver solver(make_solver_config());
  srm::vision::DetectionResult detections;
  ArmorObservation bad;
  bad.color = LightColor::blue;
  bad.points[0] = cv::Point2f(std::numeric_limits<float>::quiet_NaN(), 0.0F);
  bad.points[1] = cv::Point2f(1.0F, 0.0F);
  bad.points[2] = cv::Point2f(1.0F, 1.0F);
  bad.points[3] = cv::Point2f(0.0F, 1.0F);
  detections.armors.push_back(bad);

  TargetSelector selector;
  const auto result = selector.select(detections, solver, std::chrono::steady_clock::now(), 9);
  assert(result.observations.size() == 1);
  assert(!result.has_target());
  assert(result.selected() == nullptr);
  assert(!result.observations.front().pose.valid);
  assert(!result.observations.front().pose.error.empty());

  const srm::vision::DetectionResult empty;
  const auto empty_result = selector.select(empty, solver, std::chrono::steady_clock::now(), 10);
  assert(empty_result.observations.empty());
  assert(!empty_result.has_target());
}

void test_invalid_configuration()
{
  bool bad_color_threw = false;
  try {
    TargetSelectorConfig config;
    config.filter_color = true;
    TargetSelector invalid(config);
  } catch (const std::invalid_argument &) {
    bad_color_threw = true;
  }
  assert(bad_color_threw);

  bool bad_limit_threw = false;
  try {
    TargetSelectorConfig config;
    config.max_range_m = -1.0;
    TargetSelector invalid(config);
  } catch (const std::invalid_argument &) {
    bad_limit_threw = true;
  }
  assert(bad_limit_threw);
}

}  // namespace

int main()
{
  test_selects_nearest_valid_target_and_preserves_frame_metadata();
  test_selection_tie_break_is_stable();
  test_color_and_quality_filters();
  test_bad_candidate_isolated_and_empty_result();
  test_invalid_configuration();
  return 0;
}
