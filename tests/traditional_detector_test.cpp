#include "vision/traditional_detector.hpp"

#include <cassert>
#include <cmath>
#include <stdexcept>

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

}  // namespace

int main()
{
  test_single_blue_armor();
  test_different_colors_do_not_pair();
  test_invalid_image_and_configuration();
  return 0;
}
