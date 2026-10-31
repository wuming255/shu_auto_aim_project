#ifndef SHU_AUTO_AIM_TRADITIONAL_DETECTOR_HPP
#define SHU_AUTO_AIM_TRADITIONAL_DETECTOR_HPP

#include <array>
#include <cstddef>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <string>
#include <vector>
#include <optional>
#include "vision/number_classifier.hpp"

namespace srm::vision
{

enum class LightColor
{
  red,
  blue,
  unknown,
};

const char * to_string(LightColor color) noexcept;

enum class BrightnessMethod
{
  max_channel,
  grayscale,
};

struct TraditionalDetectorConfig        // 检测器配置文件
{
  // 灰度法可分离有高亮核心的灯条与彩色光晕；最大通道法保留暗的纯色灯条。
  BrightnessMethod brightness_method = BrightnessMethod::max_channel;
  double brightness_threshold = 150.0;

  // 以下几何参数与参考项目传统检测器的含义保持一致，角度单位为度。
  double max_lightbar_angle_error_deg = 45.0;       // 灯条允许偏离竖直方向多少
  double min_lightbar_ratio = 1.5;                  // 灯条的最小长宽比
  double max_lightbar_ratio = 20.0;                 // 灯条的最大长宽比
  double min_lightbar_length = 8.0;                 // 灯条最小长度
  // A pixel occupies an area, whereas a contour traces pixel centers. Enabled
  // only for the simulator's small rasterized lamps; real-camera baseline is opt-in.
  bool compensate_lightbar_pixel_extent = false;
  // 中心距 / 平均灯长；含下界、不含上界，表示图像几何而非物理靶板类型。
  double min_armor_ratio = 0.8;
  double max_armor_ratio = 5.5;
  double max_side_ratio = 1.5;                      // 两根灯条长度的最大比值
  double max_rectangular_error_deg = 25.0;

  // 红蓝通道差小于该值时不把灯条归类为任一颜色。
  double min_color_difference = 10.0;

  // 轮廓面积 / 最小外接旋转矩形面积，排除稀疏的弯折亮区域。
  double min_lightbar_fill_ratio = 0.35;

  // 对带有中央亮色标记的靶板检查图案证据；不是数字身份分类器。
  bool require_center_pattern = true;
  double min_pattern_contrast = 6.0;
  // 连通图案的最小高度 / 矫正后的灯条长度。
  double min_pattern_height_ratio = 0.5;
  NumberClassifierConfig classifier;
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

  // 两灯条中点距离 / 平均灯条长度；同时用于几何筛选和分类裁剪宽度。
  double ratio = 0.0;
  // 较长灯条长度 / 较短灯条长度。
  double side_ratio = 0.0;
  // 灯条方向与装甲板横向方向的矩形误差，单位为弧度。
  double rectangular_error = 0.0;
  NumberClassification classification;
};

// 每个灯条对只记首次失败的几何阶段；内容/共享灯条拒绝另见 rejected_armors。
struct PairingRejectionCounts
{
  std::size_t color = 0;
  std::size_t degenerate = 0;
  std::size_t distance = 0;
  std::size_t side_ratio = 0;
  std::size_t angle = 0;
  std::size_t intervening_light = 0;
};

struct DetectionResult                // 单次检测结果
{
  // 本轮保留二值图，便于离线调试和后续可视化；生产主循环可忽略它。
  cv::Mat brightness_mask;            // 二值图
  std::vector<LightbarObservation> lightbars;     // 灯条列表
  std::vector<ArmorObservation> armors;           // 装甲板列表
  std::vector<ArmorObservation> rejected_armors;  // 内容或共享灯条消歧失败，保留证据。
  PairingRejectionCounts pairing_rejections;
};

// 按内容置信度选择互不共享灯条的候选；近似等分或过大的冲突组保留拒绝证据。
void resolve_shared_light_conflicts(DetectionResult & result, double margin);

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
  bool valid_armor(const ArmorObservation & armor, PairingRejectionCounts & rejections) const;
  static bool contains_light(
    const ArmorObservation & armor, const std::vector<LightbarObservation> & lightbars);
  bool valid_center_pattern(const cv::Mat & bgr_image, const ArmorObservation & armor) const;
  LightColor classify_color(                                            // 灯条颜色分类
    const cv::Mat & bgr_image, const std::vector<cv::Point> & contour) const;
  cv::Rect make_roi(const ArmorObservation & armor, const cv::Size & image_size) const;       // 识别关注区域

  TraditionalDetectorConfig config_;
  std::optional<NumberClassifier> classifier_;
  double max_lightbar_angle_error_rad_ = 0.0;
  double max_rectangular_error_rad_ = 0.0;
};

}  // namespace srm::vision

#endif  // SHU_AUTO_AIM_TRADITIONAL_DETECTOR_HPP
