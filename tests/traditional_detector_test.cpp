#include "vision/traditional_detector.hpp"

#include <cassert>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <utility>

namespace
{
using srm::vision::LightColor;
using srm::vision::TraditionalDetector;
using srm::vision::TraditionalDetectorConfig;

cv::Mat make_blue_armor_image()
{
  cv::Mat image(240, 320, CV_8UC3, cv::Scalar(0, 0, 0));
  cv::rectangle(image, cv::Rect(92, 90, 10, 60), cv::Scalar(255, 0, 0), cv::FILLED);
  cv::rectangle(image, cv::Rect(158, 90, 10, 60), cv::Scalar(255, 0, 0), cv::FILLED);
  cv::putText(image, "2", {112, 144}, cv::FONT_HERSHEY_SIMPLEX, 1.2, {90, 90, 90}, 3);
  return image;
}

void test_single_blue_armor()
{
  TraditionalDetectorConfig config;
  config.brightness_threshold = 100;
  config.min_lightbar_length = 20;
  config.min_armor_ratio = 0.5;
  config.max_armor_ratio = 3.0;
  config.max_rectangular_error_deg = 15;
  TraditionalDetector detector(config);

  const auto image = make_blue_armor_image();
  const auto result = detector.detect(image);

  assert(result.brightness_mask.type() == CV_8UC1);
  assert(result.lightbars.size() == 2);
  assert(result.armors.size() == 1);

  const auto & armor = result.armors.front();
  assert(armor.color == LightColor::blue);
  assert(std::fabs(armor.center.x - 130.0F) < 1.0F);
  assert(std::fabs(armor.center.y - 120.0F) < 1.0F);
  assert(armor.points[0].x < armor.points[1].x);
  assert(armor.roi.x >= 0 && armor.roi.y >= 0);
  assert(armor.roi.br().x <= image.cols && armor.roi.br().y <= image.rows);

  const auto drawing = detector.draw_debug(image, result);
  assert(!drawing.empty());
  assert(drawing.size() == image.size());
}

void test_different_colors_do_not_pair()
{
  TraditionalDetectorConfig config;
  config.brightness_threshold = 100;
  config.min_lightbar_length = 20;
  config.min_armor_ratio = 0.5;
  config.max_armor_ratio = 3.0;
  TraditionalDetector detector(config);

  cv::Mat image(240, 320, CV_8UC3, cv::Scalar(0, 0, 0));
  cv::rectangle(image, cv::Rect(92, 90, 10, 60), cv::Scalar(255, 0, 0), cv::FILLED);
  cv::rectangle(image, cv::Rect(158, 90, 10, 60), cv::Scalar(0, 0, 255), cv::FILLED);
  const auto result = detector.detect(image);

  assert(result.lightbars.size() == 2);
  assert(result.armors.empty());
}

void test_invalid_image_and_configuration()
{
  TraditionalDetector detector;
  bool empty_threw = false;
  try {
    (void)detector.detect(cv::Mat());
  } catch (const std::invalid_argument &) {
    empty_threw = true;
  }
  assert(empty_threw);

  bool gray_threw = false;
  try {
    (void)detector.detect(cv::Mat(10, 10, CV_8UC1, cv::Scalar(0)));
  } catch (const std::invalid_argument &) {
    gray_threw = true;
  }
  assert(gray_threw);

  bool config_threw = false;
  try {
    TraditionalDetectorConfig config;
    config.brightness_threshold = 256;
    TraditionalDetector invalid_detector(config);
  } catch (const std::invalid_argument &) {
    config_threw = true;
  }
  assert(config_threw);
}

void test_adjacent_armors_do_not_create_gap_target()
{
  cv::Mat image(240, 360, CV_8UC3, cv::Scalar(8, 8, 8));
  for (int x : {50, 115, 205, 270}) {
    cv::rectangle(image, cv::Rect(x, 90, 8, 50), {255, 0, 0}, cv::FILLED);
  }
  cv::putText(image, "2", {69, 136}, cv::FONT_HERSHEY_SIMPLEX, 1.1, {90, 90, 90}, 3);
  cv::putText(image, "2", {224, 136}, cv::FONT_HERSHEY_SIMPLEX, 1.1, {90, 90, 90}, 3);
  const auto result = TraditionalDetector().detect(image);
  assert(result.lightbars.size() == 4);
  assert(result.armors.size() == 2);
  assert(std::abs(result.armors[0].center.x - 86.0F) < 1.0F);
  assert(std::abs(result.armors[1].center.x - 241.0F) < 1.0F);
}

void test_white_edges_do_not_hide_light_color()
{
  auto image = make_blue_armor_image();
  for (int x : {92, 158}) {
    cv::rectangle(image, cv::Rect(x, 90, 10, 60), {255, 255, 255}, 1);
  }
  const auto result = TraditionalDetector().detect(image);
  assert(result.lightbars.size() == 2);
  assert(result.lightbars[0].color == LightColor::blue);
  assert(result.lightbars[1].color == LightColor::blue);
  assert(result.armors.size() == 1);
}

void test_sparse_bright_shapes_are_not_lightbars()
{
  cv::Mat image(240, 320, CV_8UC3, cv::Scalar(0, 0, 0));
  for (int x : {90, 170}) {
    cv::rectangle(image, cv::Rect(x, 90, 4, 60), {255, 0, 0}, cv::FILLED);
    cv::rectangle(image, cv::Rect(x, 146, 18, 4), {255, 0, 0}, cv::FILLED);
  }
  assert(TraditionalDetector().detect(image).lightbars.empty());
}

void test_pair_must_not_span_another_light()
{
  cv::Mat image(240, 320, CV_8UC3, cv::Scalar(0, 0, 0));
  for (int x : {60, 125, 190}) {
    cv::rectangle(image, cv::Rect(x, 90, 8, 50), {255, 0, 0}, cv::FILLED);
  }
  for (int x : {78, 143}) {
    cv::putText(image, "2", {x, 136}, cv::FONT_HERSHEY_SIMPLEX, 1.1, {90, 90, 90}, 3);
  }
  const auto result = TraditionalDetector().detect(image);
  assert(result.lightbars.size() == 3);
  assert(result.armors.size() == 2);
  for (const auto & armor : result.armors) {
    assert(armor.right.center.x - armor.left.center.x < 100.0F);
  }
}

void test_tapered_lightbars_are_retained()
{
  cv::Mat image(240, 320, CV_8UC3, cv::Scalar(0, 0, 0));
  for (int x : {90, 170}) {
    cv::fillConvexPoly(image, std::vector<cv::Point>{{x, 90}, {x + 8, 149}, {x - 4, 149}},
                      {255, 0, 0});
  }
  TraditionalDetectorConfig config;
  config.require_center_pattern = false;
  const auto result = TraditionalDetector(config).detect(image);
  assert(result.lightbars.size() == 2);
  assert(result.armors.size() == 1);
}

void test_background_above_pair_is_not_center_pattern()
{
  auto image = make_blue_armor_image();
  cv::rectangle(image, cv::Rect(103, 55, 50, 100), {0, 0, 0}, cv::FILLED);
  cv::rectangle(image, cv::Rect(115, 63, 25, 30), {60, 60, 60}, cv::FILLED);
  const auto result = TraditionalDetector().detect(image);
  assert(result.lightbars.size() == 2);
  assert(result.armors.empty());
}

void test_dim_uneven_marking_is_retained()
{
  cv::Mat image(240, 320, CV_8UC3, cv::Scalar(0, 0, 0));
  for (int x : {92, 178}) {
    cv::rectangle(image, {x, 90, 8, 40}, {255, 0, 0}, cv::FILLED);
  }
  cv::putText(image, "2", {112, 136}, cv::FONT_HERSHEY_SIMPLEX, 2.4, {24, 24, 24}, 4);
  // 模拟实际录像中同一数字下半部更暗；全图仍有足够对比度。
  for (int y = 115; y < image.rows; ++y) {
    for (int x = 105; x < 175; ++x) {
      auto & pixel = image.at<cv::Vec3b>(y, x);
      if (pixel == cv::Vec3b(24, 24, 24)) pixel = cv::Vec3b(12, 12, 12);
    }
  }
  const auto result = TraditionalDetector().detect(image);
  assert(result.lightbars.size() == 2);
  assert(result.armors.size() == 1);
  for (int y = 0; y < image.rows; ++y) {
    for (int x = 105; x < 175; ++x) {
      auto & pixel = image.at<cv::Vec3b>(y, x);
      pixel /= 6;
    }
  }
  assert(TraditionalDetector().detect(image).armors.empty());
}

void test_thin_sampled_lightbar_is_retained()
{
  // 来自本地录像 01 第 547 帧的细灯条轮廓，平移后生成独立的合成配对。
  const std::vector<cv::Point> contour{{18,10}, {17,11}, {17,13}, {16,14}, {16,19},
    {14,22}, {14,27}, {13,28}, {13,30}, {12,31}, {12,36}, {10,39}, {10,44},
    {10,42}, {11,41}, {11,39}, {12,38}, {12,34}, {13,33}, {13,31}, {14,30},
    {14,26}, {15,25}, {15,23}, {16,22}, {16,18}, {17,17}, {17,14}, {18,13}};
  cv::Mat image(240, 320, CV_8UC3, cv::Scalar(0, 0, 0));
  for (int x : {80, 140}) {
    auto translated = contour;
    for (auto & point : translated) point += cv::Point(x, 80);
    cv::fillPoly(image, std::vector<std::vector<cv::Point>>{translated}, {255, 0, 0});
  }
  TraditionalDetectorConfig config;
  config.require_center_pattern = false;
  const auto result = TraditionalDetector(config).detect(image);
  assert(result.lightbars.size() == 2);
  assert(result.armors.size() == 1);
}

void test_dim_uniform_marking_is_retained()
{
  for (int level : {9, 12}) {
    auto image = make_blue_armor_image();
    for (int y = 0; y < image.rows; ++y) {
      for (int x = 103; x < 158; ++x) {
        auto & pixel = image.at<cv::Vec3b>(y, x);
        if (pixel == cv::Vec3b(90, 90, 90)) pixel = cv::Vec3b(level, level, level);
      }
    }
    assert(TraditionalDetector().detect(image).armors.size() == 1);
  }
}

void test_bright_border_does_not_hide_center_marking()
{
  for (int border_height : {12, 30}) {
    auto image = make_blue_armor_image();
    for (int y = 0; y < image.rows; ++y) {
      for (int x = 103; x < 158; ++x) {
        auto & pixel = image.at<cv::Vec3b>(y, x);
        if (pixel == cv::Vec3b(90, 90, 90)) pixel = cv::Vec3b(12, 12, 12);
      }
    }
    // 矫正图上边缘的亮背景不应使中央暗数字在 Otsu 二值化时消失。
    cv::rectangle(image, {103, 50, 50, border_height}, {60, 60, 60}, cv::FILLED);
    assert(TraditionalDetector().detect(image).armors.size() == 1);
  }
}

void test_blank_pair_and_uniform_background_are_rejected()
{
  for (int background : {0, 40, 90}) {
    cv::Mat image(240, 320, CV_8UC3, cv::Scalar(background, background, background));
    for (int x : {92, 158}) {
      cv::rectangle(image, cv::Rect(x, 90, 10, 60), {255, 0, 0}, cv::FILLED);
    }
    assert(TraditionalDetector().detect(image).armors.empty());
    TraditionalDetectorConfig geometry_only;
    geometry_only.require_center_pattern = false;
    assert(TraditionalDetector(geometry_only).detect(image).armors.size() == 1);
  }
}

void test_tilted_red_armor_keeps_corner_order()
{
  auto image = make_blue_armor_image();
  cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
  const auto rotation = cv::getRotationMatrix2D({130, 120}, 12, 1);
  cv::warpAffine(image, image, rotation, image.size());
  const auto result = TraditionalDetector().detect(image);
  assert(result.armors.size() == 1);
  const auto & armor = result.armors.front();
  assert(armor.color == LightColor::red);
  const std::vector<cv::Point2f> original{{96.5F, 90}, {162.5F, 90},
                                        {162.5F, 149}, {96.5F, 149}};
  std::vector<cv::Point2f> expected;
  cv::transform(original, expected, rotation);
  for (std::size_t i = 0; i < armor.points.size(); ++i) {
    assert(cv::norm(armor.points[i] - expected[i]) < 2.0);
  }
}

void test_intervening_light_uses_polygon_and_all_colors()
{
  TraditionalDetectorConfig config;
  config.require_center_pattern = false;
  cv::Mat image(240, 320, CV_8UC3, cv::Scalar(0, 0, 0));
  cv::rectangle(image, {66, 55, 8, 50}, {255, 0, 0}, cv::FILLED);
  cv::rectangle(image, {186, 95, 8, 50}, {255, 0, 0}, cv::FILLED);
  // 位于轴对齐包围框内、实际倾斜四边形外的小红灯条不应排除蓝色目标。
  cv::rectangle(image, {138, 55, 4, 11}, {0, 0, 255}, cv::FILLED);
  assert(TraditionalDetector(config).detect(image).armors.size() == 1);
  // 同一灯条移到候选四边形内部，即使颜色不同也应阻止跨灯配对。
  cv::rectangle(image, {138, 55, 4, 11}, {0, 0, 0}, cv::FILLED);
  cv::rectangle(image, {138, 95, 4, 11}, {0, 0, 255}, cv::FILLED);
  const auto result = TraditionalDetector(config).detect(image);
  assert(result.lightbars.size() == 3);
  assert(result.armors.empty());
}

void test_invalid_appearance_configuration()
{
  for (double value : {-0.1, 1.1, std::numeric_limits<double>::quiet_NaN()}) {
    TraditionalDetectorConfig config;
    config.min_lightbar_fill_ratio = value;
    bool rejected = false;
    try { (void)TraditionalDetector(config); }
    catch (const std::invalid_argument &) { rejected = true; }
    assert(rejected);
  }
  for (double value : {-1.0, 256.0, std::numeric_limits<double>::infinity()}) {
    TraditionalDetectorConfig config;
    config.min_pattern_contrast = value;
    bool rejected = false;
    try { (void)TraditionalDetector(config); }
    catch (const std::invalid_argument &) { rejected = true; }
    assert(rejected);
  }
  for (double value : {0.0, 2.1, std::numeric_limits<double>::quiet_NaN()}) {
    TraditionalDetectorConfig config;
    config.min_pattern_height_ratio = value;
    bool rejected = false;
    try { (void)TraditionalDetector(config); }
    catch (const std::invalid_argument &) { rejected = true; }
    assert(rejected);
  }
}

void test_shared_lights_keep_multiple_independent_armors()
{
  const auto armor = [](std::size_t left, std::size_t right, double confidence) {
    srm::vision::ArmorObservation result;
    result.left.id = left; result.right.id = right;
    result.classification = {"2",confidence,{}};
    return result;
  };
  srm::vision::DetectionResult result;
  result.armors = {armor(0,1,0.90),armor(1,2,0.99),armor(2,3,0.92),armor(4,5,0.8)};
  srm::vision::resolve_shared_light_conflicts(result,0.05);
  assert(result.armors.size() == 3 && result.rejected_armors.size() == 1);
  assert(result.armors[0].left.id == 0 && result.armors[1].left.id == 2 && result.armors[2].left.id == 4);
  assert(result.rejected_armors[0].classification.rejection == "shared_light");
  for (double margin : {0.0,0.05}) {
    result = {};
    result.armors = {armor(0,1,0.9),armor(1,2,0.9),armor(4,5,0.8)};
    srm::vision::resolve_shared_light_conflicts(result,margin);
    assert(result.armors.size() == 1 && result.armors[0].left.id == 4);
    assert(result.rejected_armors.size() == 2);
    assert(result.rejected_armors[0].classification.rejection == "ambiguous_shared_light");
  }
}

void test_classifier_replaces_brightness_gate_and_rejects_blank()
{
  TraditionalDetectorConfig config;
  config.classifier.enabled = true;
  config.classifier.model_path = std::string(SRM_TEST_NUMBER_MODEL_DIR)+"/mlp.onnx";
  config.classifier.label_path = std::string(SRM_TEST_NUMBER_MODEL_DIR)+"/label.txt";
  config.min_pattern_contrast = 255; // 旧图案门限必不通过，但不能抢先排除模型候选。
  const auto image = make_blue_armor_image();
  const auto result = TraditionalDetector(config).detect(image);
  // 手绘字体不作为模型精度真值；确认几何候选确实经过分类器而非旧图案门限。
  assert(result.armors.size() + result.rejected_armors.size() == 1);
  const auto & classified = result.armors.empty() ? result.rejected_armors[0] : result.armors[0];
  assert(!classified.classification.label.empty());
  assert(classified.classification.confidence > 0);
  auto blank = image.clone();
  cv::rectangle(blank,{103,0,55,240},{0,0,0},cv::FILLED);
  const auto rejected = TraditionalDetector(config).detect(blank);
  assert(rejected.armors.empty() && rejected.rejected_armors.size() == 1);
  assert(!rejected.rejected_armors[0].classification.rejection.empty());
}

}  // namespace

int main(int argc, char ** argv)
{
  const std::pair<const char *, void (*)()> tests[] = {
    {"single_blue_armor", test_single_blue_armor},
    {"different_colors_do_not_pair", test_different_colors_do_not_pair},
    {"invalid_image_and_configuration", test_invalid_image_and_configuration},
    {"adjacent_armors", test_adjacent_armors_do_not_create_gap_target},
    {"white_edges", test_white_edges_do_not_hide_light_color},
    {"sparse_shapes", test_sparse_bright_shapes_are_not_lightbars},
    {"intervening_light", test_pair_must_not_span_another_light},
    {"tapered_lights", test_tapered_lightbars_are_retained},
    {"offcenter_background", test_background_above_pair_is_not_center_pattern},
    {"dim_uneven_marking", test_dim_uneven_marking_is_retained},
    {"thin_sampled_lightbar", test_thin_sampled_lightbar_is_retained},
    {"dim_uniform_marking", test_dim_uniform_marking_is_retained},
    {"bright_border", test_bright_border_does_not_hide_center_marking},
    {"blank_background", test_blank_pair_and_uniform_background_are_rejected},
    {"tilted_red_armor", test_tilted_red_armor_keeps_corner_order},
    {"intervening_color_and_polygon", test_intervening_light_uses_polygon_and_all_colors},
    {"appearance_configuration", test_invalid_appearance_configuration},
    {"shared_lights", test_shared_lights_keep_multiple_independent_armors},
    {"classifier", test_classifier_replaces_brightness_gate_and_rejects_blank},
  };
  bool ran = false;
  for (const auto & test : tests) {
    if (argc == 1 || std::strcmp(argv[1], test.first) == 0) {
      test.second();
      ran = true;
    }
  }
  assert(ran);
  return 0;
}
