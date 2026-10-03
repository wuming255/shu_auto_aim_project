#include "vision/traditional_detector.hpp"

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
constexpr double kArmorRoiExtension = 1.125;

bool finite_non_negative(double value)
{
  return std::isfinite(value) && value >= 0.0;
}

double line_angle_error(double line_angle, double target_angle)
{
  // 直线方向按 pi 周期处理，避免向左倾斜的近竖直灯条被误判为 180 度错误。
  auto error = std::abs(std::atan2(std::sin(line_angle - target_angle),
                                   std::cos(line_angle - target_angle)));
  if (error > kPi / 2.0) error = kPi - error;
  return error;
}

cv::Rect clipped_rect(const cv::Rect & rect, const cv::Size & size)
{
  return rect & cv::Rect(0, 0, size.width, size.height);
}

}  // namespace

const char * to_string(LightColor color) noexcept
{
  switch (color) {
    case LightColor::red:
      return "red";
    case LightColor::blue:
      return "blue";
    case LightColor::unknown:
      return "unknown";
    default:
      return "unknown";
  }
}

TraditionalDetector::TraditionalDetector(TraditionalDetectorConfig config) : config_(config)
{
  if (!finite_non_negative(config_.brightness_threshold) || config_.brightness_threshold > 255.0) {
    throw std::invalid_argument("brightness threshold must be within [0, 255]");
  }
  if (!finite_non_negative(config_.max_lightbar_angle_error_deg) ||
      config_.max_lightbar_angle_error_deg > 180.0) {
    throw std::invalid_argument("lightbar angle error must be within [0, 180] degrees");
  }
  if (!finite_non_negative(config_.min_lightbar_ratio) ||
      config_.max_lightbar_ratio <= config_.min_lightbar_ratio) {
    throw std::invalid_argument("invalid lightbar ratio range");
  }
  if (!finite_non_negative(config_.min_lightbar_length) ||
      !finite_non_negative(config_.min_armor_ratio) ||
      config_.max_armor_ratio <= config_.min_armor_ratio ||
      !finite_non_negative(config_.max_side_ratio) ||
      !finite_non_negative(config_.max_rectangular_error_deg) ||
      config_.max_rectangular_error_deg > 180.0 ||
      !finite_non_negative(config_.min_color_difference) ||
      config_.min_color_difference > 255.0) {
    throw std::invalid_argument("invalid traditional detector geometry configuration");
  }

  max_lightbar_angle_error_rad_ = config_.max_lightbar_angle_error_deg * kPi / 180.0;
  max_rectangular_error_rad_ = config_.max_rectangular_error_deg * kPi / 180.0;
}

DetectionResult TraditionalDetector::detect(const cv::Mat & bgr_image) const
{
  if (bgr_image.empty()) throw std::invalid_argument("traditional detector received an empty image");
  if (bgr_image.type() != CV_8UC3) {
    throw std::invalid_argument("traditional detector expects an 8-bit BGR image");
  }

  // 参考项目使用灰度阈值。本实现改用三个 BGR 通道的最大值，保留红/蓝灯条的高亮信息。
  std::vector<cv::Mat> channels;
  cv::split(bgr_image, channels);
  cv::Mat brightness = cv::max(channels[0], channels[1]);
  brightness = cv::max(brightness, channels[2]);

  DetectionResult result;
  cv::threshold(
    brightness, result.brightness_mask, config_.brightness_threshold, 255, cv::THRESH_BINARY);

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(
    result.brightness_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

  std::size_t lightbar_id = 0;
  for (const auto & contour : contours) {
    if (contour.size() < 5) continue;

    const auto rotated_rect = cv::minAreaRect(contour);
    auto lightbar = make_lightbar(rotated_rect, lightbar_id);
    if (!valid_lightbar(lightbar)) continue;

    lightbar.color = classify_color(bgr_image, contour);
    if (lightbar.color == LightColor::unknown) continue;

    result.lightbars.push_back(std::move(lightbar));
    ++lightbar_id;
  }

  std::sort(
    result.lightbars.begin(), result.lightbars.end(),
    [](const LightbarObservation & lhs, const LightbarObservation & rhs) {
      return lhs.center.x < rhs.center.x;
    });

  for (std::size_t left_index = 0; left_index < result.lightbars.size(); ++left_index) {
    for (std::size_t right_index = left_index + 1; right_index < result.lightbars.size();
         ++right_index) {
      const auto & left = result.lightbars[left_index];
      const auto & right = result.lightbars[right_index];
      if (left.color != right.color) continue;

      ArmorObservation armor;
      armor.left = left;
      armor.right = right;
      armor.color = left.color;
      armor.center = (left.center + right.center) / 2.0F;
      armor.points = {left.top, right.top, right.bottom, left.bottom};

      const auto center_distance = cv::norm(right.center - left.center);
      const auto max_length = std::max(left.length, right.length);
      const auto min_length = std::min(left.length, right.length);
      if (max_length <= std::numeric_limits<double>::epsilon() ||
          min_length <= std::numeric_limits<double>::epsilon()) {
        continue;
      }

      armor.ratio = center_distance / max_length;
      armor.side_ratio = max_length / min_length;
      const auto left_to_right = right.center - left.center;
      const auto roll = std::atan2(left_to_right.y, left_to_right.x);
      const auto left_error = line_angle_error(left.angle, roll + kPi / 2.0);
      const auto right_error = line_angle_error(right.angle, roll + kPi / 2.0);
      armor.rectangular_error = std::max(left_error, right_error);

      if (!valid_armor(armor)) continue;
      armor.roi = make_roi(armor, bgr_image.size());
      if (armor.roi.empty()) continue;
      result.armors.push_back(std::move(armor));
    }
  }

  return result;
}

cv::Mat TraditionalDetector::draw_debug(
  const cv::Mat & bgr_image, const DetectionResult & result) const
{
  if (bgr_image.empty() || bgr_image.type() != CV_8UC3) {
    throw std::invalid_argument("draw_debug expects an 8-bit BGR image");
  }

  auto drawing = bgr_image.clone();
  for (const auto & lightbar : result.lightbars) {
    const cv::Scalar color = lightbar.color == LightColor::red
                               ? cv::Scalar(0, 0, 255)
                               : cv::Scalar(255, 0, 0);
    cv::line(drawing, lightbar.top, lightbar.bottom, color, 2, cv::LINE_AA);
    cv::putText(
      drawing, to_string(lightbar.color), lightbar.top, cv::FONT_HERSHEY_SIMPLEX, 0.45, color, 1,
      cv::LINE_AA);
  }

  for (const auto & armor : result.armors) {
    const std::vector<cv::Point> points{
      armor.points[0], armor.points[1], armor.points[2], armor.points[3]};
    cv::polylines(drawing, points, true, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
    cv::rectangle(drawing, armor.roi, cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
  }
  return drawing;
}

const TraditionalDetectorConfig & TraditionalDetector::config() const noexcept
{
  return config_;
}

LightbarObservation TraditionalDetector::make_lightbar(
  const cv::RotatedRect & rotated_rect, std::size_t id)
{
  std::array<cv::Point2f, 4> corners{};
  rotated_rect.points(corners.data());
  std::sort(corners.begin(), corners.end(), [](const cv::Point2f & lhs, const cv::Point2f & rhs) {
    if (lhs.y == rhs.y) return lhs.x < rhs.x;
    return lhs.y < rhs.y;
  });

  LightbarObservation lightbar;
  lightbar.id = id;
  lightbar.rotated_rect = rotated_rect;
  lightbar.center = rotated_rect.center;
  lightbar.top = (corners[0] + corners[1]) / 2.0F;
  lightbar.bottom = (corners[2] + corners[3]) / 2.0F;
  lightbar.top2bottom = lightbar.bottom - lightbar.top;
  lightbar.width = cv::norm(corners[0] - corners[1]);
  lightbar.length = cv::norm(lightbar.top2bottom);
  if (lightbar.width > std::numeric_limits<double>::epsilon()) {
    lightbar.ratio = lightbar.length / lightbar.width;
  }
  lightbar.angle = std::atan2(lightbar.top2bottom.y, lightbar.top2bottom.x);
  lightbar.angle_error = std::abs(std::abs(lightbar.angle) - kPi / 2.0);
  return lightbar;
}

bool TraditionalDetector::valid_lightbar(const LightbarObservation & lightbar) const
{
  return lightbar.angle_error < max_lightbar_angle_error_rad_ &&
         lightbar.ratio > config_.min_lightbar_ratio &&
         lightbar.ratio < config_.max_lightbar_ratio &&
         lightbar.length > config_.min_lightbar_length;
}

bool TraditionalDetector::valid_armor(const ArmorObservation & armor) const
{
  return armor.ratio > config_.min_armor_ratio &&
         armor.ratio < config_.max_armor_ratio &&
         armor.side_ratio < config_.max_side_ratio &&
         armor.rectangular_error < max_rectangular_error_rad_;
}

LightColor TraditionalDetector::classify_color(
  const cv::Mat & bgr_image, const std::vector<cv::Point> & contour) const
{
  double red_sum = 0.0;
  double blue_sum = 0.0;
  std::size_t count = 0;
  for (const auto & point : contour) {
    if (point.x < 0 || point.x >= bgr_image.cols || point.y < 0 || point.y >= bgr_image.rows) {
      continue;
    }
    const auto pixel = bgr_image.at<cv::Vec3b>(point);
    blue_sum += pixel[0];
    red_sum += pixel[2];
    ++count;
  }

  if (count == 0 || std::abs(blue_sum - red_sum) < config_.min_color_difference * count) {
    return LightColor::unknown;
  }
  return blue_sum > red_sum ? LightColor::blue : LightColor::red;
}

cv::Rect TraditionalDetector::make_roi(
  const ArmorObservation & armor, const cv::Size & image_size) const
{
  const auto left_axis = armor.left.top2bottom * static_cast<float>(kArmorRoiExtension);
  const auto right_axis = armor.right.top2bottom * static_cast<float>(kArmorRoiExtension);
  const std::array<cv::Point2f, 4> extended_points{
    armor.left.center - left_axis, armor.right.center - right_axis,
    armor.right.center + right_axis, armor.left.center + left_axis};

  float min_x = extended_points[0].x;
  float max_x = extended_points[0].x;
  float min_y = extended_points[0].y;
  float max_y = extended_points[0].y;
  for (const auto & point : extended_points) {
    min_x = std::min(min_x, point.x);
    max_x = std::max(max_x, point.x);
    min_y = std::min(min_y, point.y);
    max_y = std::max(max_y, point.y);
  }

  const auto left = static_cast<int>(std::floor(min_x));
  const auto top = static_cast<int>(std::floor(min_y));
  const auto right = static_cast<int>(std::ceil(max_x));
  const auto bottom = static_cast<int>(std::ceil(max_y));
  return clipped_rect(cv::Rect(left, top, right - left, bottom - top), image_size);
}

}  // namespace srm::vision
