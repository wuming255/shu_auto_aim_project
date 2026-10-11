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

struct TraditionalDetectorConfig        // 检测器配置文件
{
  // 对 BGR 三个通道的最大值做阈值，避免纯红/纯蓝灯条被灰度化后亮度过低。
  double brightness_threshold = 150.0;

  // 以下几何参数与参考项目传统检测器的含义保持一致，角度单位为度。
  double max_lightbar_angle_error_deg = 45.0;       // 灯条允许偏离竖直方向多少
  double min_lightbar_ratio = 1.5;                  // 灯条的最小长宽比
  double max_lightbar_ratio = 20.0;                 // 灯条的最大长宽比
  double min_lightbar_length = 8.0;                 // 灯条最小长度
  double min_armor_ratio = 1.0;                     // 装甲板的最小长宽比
  double max_armor_ratio = 5.0;                     // 装甲板的最大长宽比
  double max_side_ratio = 1.5;                      // 两根灯条长度的最大比值
  double max_rectangular_error_deg = 25.0;

  // 红蓝通道差小于该值时不把灯条归类为任一颜色。
  double min_color_difference = 10.0;
};

struct LightbarObservation      // 一根灯条的检测结果
{
  std::size_t id = 0;
  LightColor color = LightColor::unknown;
  cv::RotatedRect rotated_rect;                 // 旋转矩形
  cv::Point2f center;
  cv::Point2f top;
  cv::Point2f bottom;
  cv::Point2f top2bottom;
  double angle = 0.0;
  double angle_error = 0.0;                   // 偏离竖直方向多少角度
  double length = 0.0;
  double width = 0.0;
  double ratio = 0.0;                         // 长宽比
};

struct ArmorObservation                       // 装甲板识别结果
{
  LightbarObservation left;                   // 左灯条
  LightbarObservation right;                  // 右灯条
  LightColor color = LightColor::unknown;

  // 顺序固定为：左上、右上、右下、左下；与后续 PnP 输入约定一致。
  std::array<cv::Point2f, 4> points{};        // 四个角点的坐标
  cv::Point2f center;
  cv::Rect roi;                               // 装甲板及周围的一块“感兴趣”矩形区域

  // 两灯条中点距离 / 较长灯条长度。
  double ratio = 0.0;
  // 较长灯条长度 / 较短灯条长度。
  double side_ratio = 0.0;
  // 灯条方向与装甲板横向方向的矩形误差，单位为弧度。
  double rectangular_error = 0.0;
};

struct DetectionResult                // 单次检测结果
{
  // 本轮保留二值图，便于离线调试和后续可视化；生产主循环可忽略它。
  cv::Mat brightness_mask;            // 二值图
  std::vector<LightbarObservation> lightbars;     // 灯条列表
  std::vector<ArmorObservation> armors;           // 装甲板列表
};

class TraditionalDetector final
{
public:
  explicit TraditionalDetector(TraditionalDetectorConfig config = {});      // 禁止隐形调用构造函数

  DetectionResult detect(const cv::Mat & bgr_image) const;        // 检测函数

  // 只绘制检测结果，不打开窗口，适合离线测试和无桌面环境。
  cv::Mat draw_debug(const cv::Mat & bgr_image, const DetectionResult & result) const;      // 画检测结果

  const TraditionalDetectorConfig & config() const noexcept;      // 读配置

private:
  static LightbarObservation make_lightbar(const cv::RotatedRect & rotated_rect, std::size_t id);     // 识别灯条

  bool valid_lightbar(const LightbarObservation & lightbar) const;      // 判断灯条合理性
  bool valid_armor(const ArmorObservation & armor) const;               // 判断装甲板合理性
  LightColor classify_color(                                            // 灯条颜色分类
    const cv::Mat & bgr_image, const std::vector<cv::Point> & contour) const;
  cv::Rect make_roi(const ArmorObservation & armor, const cv::Size & image_size) const;       // 识别关注区域

  TraditionalDetectorConfig config_;
  double max_lightbar_angle_error_rad_ = 0.0;
  double max_rectangular_error_rad_ = 0.0;
};

}  // namespace srm::vision

#endif  // SHU_AUTO_AIM_TRADITIONAL_DETECTOR_HPP
