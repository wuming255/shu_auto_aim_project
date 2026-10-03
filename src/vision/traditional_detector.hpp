#ifndef SHU_AUTO_AIM_TRADITIONAL_DETECTOR_HPP
#define SHU_AUTO_AIM_TRADITIONAL_DETECTOR_HPP

#include <array>
#include <cstddef>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <string>
#include <vector>

namespace srm::vision
{

enum class LightColor
{
  red,
  blue,
  unknown,
};

const char * to_string(LightColor color) noexcept;

struct TraditionalDetectorConfig
{
  // 对 BGR 三个通道的最大值做阈值，避免纯红/纯蓝灯条被灰度化后亮度过低。
  double brightness_threshold = 150.0;

  // 以下几何参数与参考项目传统检测器的含义保持一致，角度单位为度。
  double max_lightbar_angle_error_deg = 45.0;
  double min_lightbar_ratio = 1.5;
  double max_lightbar_ratio = 20.0;
  double min_lightbar_length = 8.0;
  double min_armor_ratio = 1.0;
  double max_armor_ratio = 5.0;
  double max_side_ratio = 1.5;
  double max_rectangular_error_deg = 25.0;

  // 红蓝通道差小于该值时不把灯条归类为任一颜色。
  double min_color_difference = 10.0;
};

struct LightbarObservation
{
  std::size_t id = 0;
  LightColor color = LightColor::unknown;
  cv::RotatedRect rotated_rect;
  cv::Point2f center;
  cv::Point2f top;
  cv::Point2f bottom;
  cv::Point2f top2bottom;
  double angle = 0.0;
  double angle_error = 0.0;
  double length = 0.0;
  double width = 0.0;
  double ratio = 0.0;
};

struct ArmorObservation
{
  LightbarObservation left;
  LightbarObservation right;
  LightColor color = LightColor::unknown;

  // 顺序固定为：左上、右上、右下、左下；与后续 PnP 输入约定一致。
  std::array<cv::Point2f, 4> points{};
  cv::Point2f center;
  cv::Rect roi;

  // 两灯条中点距离 / 较长灯条长度。
  double ratio = 0.0;
  // 较长灯条长度 / 较短灯条长度。
  double side_ratio = 0.0;
  // 灯条方向与装甲板横向方向的矩形误差，单位为弧度。
  double rectangular_error = 0.0;
};

struct DetectionResult
{
  // 本轮保留二值图，便于离线调试和后续可视化；生产主循环可忽略它。
  cv::Mat brightness_mask;
  std::vector<LightbarObservation> lightbars;
  std::vector<ArmorObservation> armors;
};

class TraditionalDetector final
{
public:
  explicit TraditionalDetector(TraditionalDetectorConfig config = {});

  DetectionResult detect(const cv::Mat & bgr_image) const;

  // 只绘制检测结果，不打开窗口，适合离线测试和无桌面环境。
  cv::Mat draw_debug(const cv::Mat & bgr_image, const DetectionResult & result) const;

  const TraditionalDetectorConfig & config() const noexcept;

private:
  static LightbarObservation make_lightbar(const cv::RotatedRect & rotated_rect, std::size_t id);

  bool valid_lightbar(const LightbarObservation & lightbar) const;
  bool valid_armor(const ArmorObservation & armor) const;
  LightColor classify_color(
    const cv::Mat & bgr_image, const std::vector<cv::Point> & contour) const;
  cv::Rect make_roi(const ArmorObservation & armor, const cv::Size & image_size) const;

  TraditionalDetectorConfig config_;
  double max_lightbar_angle_error_rad_ = 0.0;
  double max_rectangular_error_rad_ = 0.0;
};

}  // namespace srm::vision

#endif  // SHU_AUTO_AIM_TRADITIONAL_DETECTOR_HPP
