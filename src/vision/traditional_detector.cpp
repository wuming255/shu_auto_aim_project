#include "vision/traditional_detector.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>
#include <functional>
#include <set>

// 本轮灯条筛选、跨灯排除和中央图案提取适配自 rm_auto_aim 参考实现。
// Copyright (c) 2022 ChenJun
// Copyright (C) 2024 Zheng Yu
// 上游 MIT 许可及本项目适配范围见 docs/traditional_detector.md。

namespace srm::vision
{

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kArmorRoiExtension = 1.125;

bool finite_non_negative(double value)      // 检查数据有限且不小于
{
  return std::isfinite(value) && value >= 0.0;
}

double line_angle_error(double line_angle, double target_angle)
{   // 算方向误差
  // 直线方向按 pi 周期处理，避免向左倾斜的近竖直灯条被误判为 180 度错误。
  auto error = std::abs(std::atan2(std::sin(line_angle - target_angle),
                                   std::cos(line_angle - target_angle)));
  if (error > kPi / 2.0) error = kPi - error;
  return error;
}

cv::Rect clipped_rect(const cv::Rect & rect, const cv::Size & size) // 把矩形画到图像里面去
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


TraditionalDetector::TraditionalDetector(TraditionalDetectorConfig config) : config_(config)    // 构造函数加载配置
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
  if (!finite_non_negative(config_.min_lightbar_fill_ratio) ||
      config_.min_lightbar_fill_ratio > 1.0 ||
      !finite_non_negative(config_.min_pattern_contrast) || config_.min_pattern_contrast > 255.0 ||
      !std::isfinite(config_.min_pattern_height_ratio) || config_.min_pattern_height_ratio <= 0.0 ||
      config_.min_pattern_height_ratio > 2.0) {
    throw std::invalid_argument("invalid traditional detector appearance configuration");
  }

  max_lightbar_angle_error_rad_ = config_.max_lightbar_angle_error_deg * kPi / 180.0;   // 灯条和竖向差角的最大值，弧度表示
  max_rectangular_error_rad_ = config_.max_rectangular_error_deg * kPi / 180.0;         // 灯条和横向差角的最大值，弧度表示
  if (config_.classifier.enabled) classifier_.emplace(config_.classifier);
}


DetectionResult TraditionalDetector::detect(const cv::Mat & bgr_image) const
{
  if (bgr_image.empty()) throw std::invalid_argument("traditional detector received an empty image");   // 收到了空图像
  if (bgr_image.type() != CV_8UC3) {
    throw std::invalid_argument("traditional detector expects an 8-bit BGR image");
  }

  // 参考项目使用灰度阈值。本实现改用三个 BGR 通道的最大值，保留红/蓝灯条的高亮信息。
  std::vector<cv::Mat> channels;        // 几个容器用来放拆分的图像
  cv::split(bgr_image, channels);       // 拆分通道放到容器中
  cv::Mat brightness = cv::max(channels[0], channels[1]);
  brightness = cv::max(brightness, channels[2]);      // 每个像素取三通道中的最大值

  DetectionResult result;   
  cv::threshold(    // 获取二值图
    brightness, result.brightness_mask, config_.brightness_threshold, 255, cv::THRESH_BINARY);

  std::vector<std::vector<cv::Point>> contours;         // 二位数组，其中每个一维数组表示一个轮廓，每个轮廓一维数组又是由一堆点组成的
  cv::findContours(                                     // 找轮廓
    result.brightness_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);      // contours是接收轮廓的

  std::size_t lightbar_id = 0;
  for (const auto & contour : contours) {
    if (contour.size() < 5) continue;     // 轮廓点少于5个跳过

    const auto rotated_rect = cv::minAreaRect(contour);
    auto lightbar = make_lightbar(rotated_rect, lightbar_id);     // 识别灯条的
    if (!valid_lightbar(lightbar)) continue;        // 判断灯条合理性的
    // 参考 FaterYU 2024 检测器的填充率筛选，使用连续轮廓面积避免小灯条像素计数偏大。
    const auto rect_area = static_cast<double>(rotated_rect.size.area());
    if (rect_area <= 0.0 || cv::contourArea(contour) / rect_area < config_.min_lightbar_fill_ratio) {
      continue;
    }

    lightbar.color = classify_color(bgr_image, contour);      // 判断灯条颜色
    if (lightbar.color == LightColor::unknown) continue;

    result.lightbars.push_back(std::move(lightbar));      // 判断ok了加入检测结果中的灯条识别结果
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
      const auto & left = result.lightbars[left_index];         // 左灯条放左
      const auto & right = result.lightbars[right_index];       // 右灯条放右
      if (left.color != right.color) continue;

      ArmorObservation armor;     // 构建装甲板并输入各个数据
      armor.left = left;
      armor.right = right;
      armor.color = left.color;
      armor.center = (left.center + right.center) / 2.0F;
      armor.points = {left.top, right.top, right.bottom, left.bottom};

      const auto center_distance = cv::norm(right.center - left.center);      // 算灯条中心距离（norm是范数的意思）
      const auto max_length = std::max(left.length, right.length);
      const auto min_length = std::min(left.length, right.length);
      if (max_length <= std::numeric_limits<double>::epsilon() ||
          min_length <= std::numeric_limits<double>::epsilon()) {     // std::numeric_limits<double>::epsilon()是获取double的机器精度，
        continue;                                                     // 小于机器精度的话说明这个数值过小接近于0了
      }

      armor.ratio = center_distance / max_length;                    // 中心距离和长度距离的比值
      armor.side_ratio = max_length / min_length;                    // 最长和最短的比值
      const auto left_to_right = right.center - left.center;         // 方向向量，用于下面算方向角的
      const auto roll = std::atan2(left_to_right.y, left_to_right.x);                  // 算中心连线的方向角
      const auto left_error = line_angle_error(left.angle, roll + kPi / 2.0);          // 比较两条直线的夹角，返回其中较小的那个方向角
      const auto right_error = line_angle_error(right.angle, roll + kPi / 2.0);
      armor.rectangular_error = std::max(left_error, right_error);

      if (!valid_armor(armor)) continue;                  // 这里面写了对方向角的要求
      if (contains_light(armor, result.lightbars)) continue;
      // 分类器直接处理几何候选，不能先被旧亮图案阈值排除。
      if (!classifier_ && config_.require_center_pattern && !valid_center_pattern(bgr_image, armor)) continue;
      armor.roi = make_roi(armor, bgr_image.size());      // 装甲板合格就生成关注区域
      if (armor.roi.empty()) continue;
      if (classifier_) {
        armor.classification = classifier_->classify(bgr_image, armor);
        if (!armor.classification.rejection.empty()) {
          result.rejected_armors.push_back(std::move(armor));
          continue;
        }
      }
      result.armors.push_back(std::move(armor));        // 成功生成关注区域后加入装甲板列表
    }
  }

  if (classifier_) resolve_shared_light_conflicts(result, config_.classifier.conflict_margin);
  return result;
}

void resolve_shared_light_conflicts(DetectionResult & result, double margin)
{
  if (!std::isfinite(margin) || margin < 0 || margin > 1) {
    throw std::invalid_argument("invalid shared light confidence margin");
  }
    // 共享灯条候选按连通分量求最大总置信度的互斥配对，保留多个独立装甲。
    // 近似等分的冲突整组拒绝；大分量限额防止噪声触发指数搜索。
    const auto shares = [](const ArmorObservation & a, const ArmorObservation & b) {
      return a.left.id == b.left.id || a.left.id == b.right.id ||
             a.right.id == b.left.id || a.right.id == b.right.id;
    };
    std::vector<bool> visited(result.armors.size(), false), keep(result.armors.size(), false);
    for (std::size_t start = 0; start < result.armors.size(); ++start) {
      if (visited[start]) continue;
      std::vector<std::size_t> component{start};
      visited[start] = true;
      for (std::size_t i = 0; i < component.size(); ++i) {
        for (std::size_t j = 0; j < result.armors.size(); ++j) {
          if (!visited[j] && shares(result.armors[component[i]], result.armors[j])) {
            visited[j] = true; component.push_back(j);
          }
        }
      }
      if (component.size() == 1) { keep[start] = true; continue; }
      double best = -1, second = -1;
      std::vector<std::size_t> chosen, winner;
      std::set<std::size_t> used;
      std::function<void(std::size_t, double)> search = [&](std::size_t i, double score) {
        if (i == component.size()) {
          if (score > best) { second = best; best = score; winner = chosen; }
          else second = std::max(second, score);
          return;
        }
        search(i+1, score);
        const auto index = component[i];
        const auto & a = result.armors[index];
        if (used.count(a.left.id) || used.count(a.right.id)) return;
        used.insert(a.left.id); used.insert(a.right.id); chosen.push_back(index);
        search(i+1, score+a.classification.confidence);
        chosen.pop_back(); used.erase(a.left.id); used.erase(a.right.id);
      };
      if (component.size() <= 18) search(0,0);
       const bool ambiguous = best < 0 || best-second <= margin;
      if (!ambiguous) for (const auto index : winner) keep[index] = true;
      for (const auto index : component) if (!keep[index]) {
        result.armors[index].classification.rejection = ambiguous ? "ambiguous_shared_light" : "shared_light";
      }
    }
    std::vector<ArmorObservation> accepted;
    for (std::size_t i = 0; i < result.armors.size(); ++i) {
      if (keep[i]) accepted.push_back(std::move(result.armors[i]));
      else result.rejected_armors.push_back(std::move(result.armors[i]));
    }
    result.armors = std::move(accepted);
}


cv::Mat TraditionalDetector::draw_debug(
  const cv::Mat & bgr_image, const DetectionResult & result) const
{
  if (bgr_image.empty() || bgr_image.type() != CV_8UC3) {
    throw std::invalid_argument("draw_debug expects an 8-bit BGR image");
  }

  auto drawing = bgr_image.clone();         // 创建图片副本
  for (const auto & lightbar : result.lightbars) {      // 遍历检测结果中的灯条
    const cv::Scalar color = lightbar.color == LightColor::red      // 选颜色
                               ? cv::Scalar(0, 0, 255)
                               : cv::Scalar(255, 0, 0);
    cv::line(drawing, lightbar.top, lightbar.bottom, color, 2, cv::LINE_AA);    // 画线
    cv::putText(                                                                // 写字
      drawing, to_string(lightbar.color), lightbar.top, cv::FONT_HERSHEY_SIMPLEX, 0.45, color, 1,
      cv::LINE_AA);
  }

  for (const auto & armor : result.armors) {        // 遍历装甲板
    const std::vector<cv::Point> points{            // 拿点
      armor.points[0], armor.points[1], armor.points[2], armor.points[3]};
    cv::polylines(drawing, points, true, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);        // 画装甲板矩形
    cv::rectangle(drawing, armor.roi, cv::Scalar(0, 255, 255), 1, cv::LINE_AA);         // 画注意区矩形
    if (!armor.classification.label.empty()) cv::putText(drawing,
      armor.classification.label + " " + std::to_string(armor.classification.confidence).substr(0,4),
      armor.center, cv::FONT_HERSHEY_SIMPLEX, 0.5, {0,255,0}, 1, cv::LINE_AA);
  }
  return drawing;
}

const TraditionalDetectorConfig & TraditionalDetector::config() const noexcept
{
  return config_;
}

LightbarObservation TraditionalDetector::make_lightbar(       // 识别灯条
  const cv::RotatedRect & rotated_rect, std::size_t id)
{
  // 取点并排序
  std::array<cv::Point2f, 4> corners{};
  rotated_rect.points(corners.data());            // 取点放入corners
  std::sort(corners.begin(), corners.end(), [](const cv::Point2f & lhs, const cv::Point2f & rhs) {
    if (lhs.y == rhs.y) return lhs.x < rhs.x;
    return lhs.y < rhs.y;
  });

  // 记录一堆数据
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

bool TraditionalDetector::contains_light(
  const ArmorObservation & armor, const std::vector<LightbarObservation> & lightbars)
{
  // 参考两个 rm_auto_aim 的 containLight；用实际四边形避免倾斜目标的 AABB 误排除。
  const std::vector<cv::Point2f> polygon(armor.points.begin(), armor.points.end());
  for (const auto & light : lightbars) {
    if (light.id == armor.left.id || light.id == armor.right.id) continue;
    for (const auto & point : {light.top, light.center, light.bottom}) {
      if (cv::pointPolygonTest(polygon, point, false) >= 0.0) return true;
    }
  }
  return false;
}

bool TraditionalDetector::valid_center_pattern(
  const cv::Mat & bgr_image, const ArmorObservation & armor) const
{
  // 沿用参考项目 extractNumbers 的透视矫正/向上下扩展思路；适配本项目 TL/TR/BR/BL。
  // 保留中央 60%，避开两侧发光灯条，数字可能高于灯条，故不只裁灯条之间的高度。
  constexpr int kWidth = 64;
  constexpr int kHeight = 56;
  constexpr int kLightLength = 24;
  const std::array<cv::Point2f, 4> target{{{0, 16}, {63, 16}, {63, 40}, {0, 40}}};
  const auto transform = cv::getPerspectiveTransform(armor.points.data(), target.data());
  cv::Mat warped;
  cv::warpPerspective(bgr_image, warped, transform, {kWidth, kHeight});
  cv::Mat gray;
  cv::cvtColor(warped(cv::Rect(13, 0, 38, kHeight)), gray, cv::COLOR_BGR2GRAY);
  cv::GaussianBlur(gray, gray, {3, 3}, 0);
  // 从灯条附近的中央高度估计阈值，避免上下扩展区的亮背景淹没暗数字。
  // 二值化和连通域检查仍使用完整高度，保留数字的上下笔画和边界排除依据。
  const auto threshold_region = gray(cv::Rect(0, 16, gray.cols, kLightLength));
  double minimum = 0.0;
  double maximum = 0.0;
  cv::minMaxLoc(threshold_region, &minimum, &maximum);
  if (maximum - minimum < config_.min_pattern_contrast) return false;

  cv::Mat binary;
  // 对比度门限用于拒绝低幅噪声；二值化仍由 Otsu 自适应，保留亮度不均的暗数字。
  const auto threshold = cv::threshold(
    threshold_region, binary, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
  cv::threshold(gray, binary, threshold, 255, cv::THRESH_BINARY);
  cv::Mat labels;
  cv::Mat stats;
  cv::Mat centroids;
  const auto count = cv::connectedComponentsWithStats(binary, labels, stats, centroids);
  for (int index = 1; index < count; ++index) {
    const auto width = stats.at<int>(index, cv::CC_STAT_WIDTH);
    const auto height = stats.at<int>(index, cv::CC_STAT_HEIGHT);
    const auto top = stats.at<int>(index, cv::CC_STAT_TOP);
    const auto area = stats.at<int>(index, cv::CC_STAT_AREA);
    const auto x = centroids.at<double>(index, 0);
    const auto y = centroids.at<double>(index, 1);
    // 至少有一块居中的连通亮图案；细小螺丝、满幅亮背景、上下贯穿纹理不构成证据。
    if (width >= 2 && height >= config_.min_pattern_height_ratio * kLightLength &&
        area >= 0.02 * binary.total() && area <= 0.6 * binary.total() &&
        top > 0 && top + height < kHeight &&
        x >= 0.25 * binary.cols && x <= 0.75 * binary.cols &&
        top <= kHeight / 2 && top + height > kHeight / 2 &&
        y >= 0.35 * kHeight && y <= 0.65 * kHeight) {
      return true;
    }
  }
  return false;
}

LightColor TraditionalDetector::classify_color(
  const cv::Mat & bgr_image, const std::vector<cv::Point> & contour) const
{
  // 在轮廓内部统计 BGR，包含颜色核心而不混入外接矩形的背景。
  const auto rect = cv::boundingRect(contour);
  cv::Mat mask = cv::Mat::zeros(rect.size(), CV_8UC1);
  cv::drawContours(mask, std::vector<std::vector<cv::Point>>{contour}, 0, cv::Scalar(255),
                   cv::FILLED, cv::LINE_8, cv::noArray(), 0, -rect.tl());
  const auto mean = cv::mean(bgr_image(rect), mask);
  if (std::abs(mean[0] - mean[2]) < config_.min_color_difference || mean[0] == mean[2]) {
    return LightColor::unknown;
  }
  return mean[0] > mean[2] ? LightColor::blue : LightColor::red;
}

cv::Rect TraditionalDetector::make_roi(
  const ArmorObservation & armor, const cv::Size & image_size) const
{
  // kArmorRoiExtension的值为1.25，这里就是按1.25倍缩放矩形罢了
  const auto left_axis = armor.left.top2bottom * static_cast<float>(kArmorRoiExtension);      
  const auto right_axis = armor.right.top2bottom * static_cast<float>(kArmorRoiExtension);
  const std::array<cv::Point2f, 4> extended_points{                   // 四个拓展点，也就是拓展出去的矩形的四个角点
    armor.left.center - left_axis, armor.right.center - right_axis,
    armor.right.center + right_axis, armor.left.center + left_axis};

  // 找最左和最右的，最上和最下的边界
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
  return clipped_rect(cv::Rect(left, top, right - left, bottom - top), image_size);     // 矩形和图像取交集，防止画到图像外面
}

}  // namespace srm::vision
