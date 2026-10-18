// ROI extraction and ONNX softmax adapted from rm_auto_aim, Copyright 2022 Chen Jun.
// MIT license: models/armor_number/LICENSE; source: models/armor_number/README.md.
#include "vision/number_classifier.hpp"
#include "vision/traditional_detector.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <stdexcept>
#include <utility>

namespace srm::vision
{
NumberClassifier::NumberClassifier(NumberClassifierConfig config) : config_(std::move(config))
{
  if (!std::isfinite(config_.min_confidence) || config_.min_confidence <= 0.5 ||
      config_.min_confidence > 1.0 || !std::isfinite(config_.conflict_margin) ||
      config_.conflict_margin < 0.0 || config_.conflict_margin > 1.0) {
    throw std::invalid_argument("invalid number classifier thresholds");
  }
  std::ifstream input(config_.label_path);
  if (!input) throw std::invalid_argument("cannot read classifier labels: " + config_.label_path);
  std::string line;
  while (std::getline(input, line)) {
    if (!line.empty() && line.back() == '\r') line.pop_back();
    if (line.empty()) throw std::invalid_argument("empty classifier label");
    labels_.push_back(line);
  }
  if (input.bad() || std::count(labels_.begin(), labels_.end(), "negative") != 1) {
    throw std::invalid_argument("classifier labels require exactly one negative class");
  }
  net_ = cv::dnn::readNetFromONNX(config_.model_path);
  if (net_.empty()) throw std::invalid_argument("empty classifier network");
  net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
  net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
}

NumberClassification NumberClassifier::classify(
  const cv::Mat & bgr, const ArmorObservation & armor) const
{
  // 与配对一致，ratio 为中心距 / 平均灯长；3.2仅决定图像裁剪宽度。
  const int width = armor.ratio > 3.2 ? 54 : 32;
  const std::array<cv::Point2f, 4> destination{{{0,7}, {static_cast<float>(width-1),7},
    {static_cast<float>(width-1),19}, {0,19}}};
  const auto transform = cv::getPerspectiveTransform(armor.points.data(), destination.data());
  cv::Mat warped, gray, binary, normalized;
  cv::warpPerspective(bgr, warped, transform, {width,28});
  // 上游 ROS 输入已转成 RGB；本项目输入始终为 BGR，须使用对应的灰度权重。
  cv::cvtColor(warped(cv::Rect((width-20)/2,0,20,28)), gray, cv::COLOR_BGR2GRAY);
  cv::threshold(gray, binary, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
  binary.convertTo(normalized, CV_32F, 1.0/255.0);
  net_.setInput(cv::dnn::blobFromImage(normalized));
  cv::Mat logits = net_.forward().reshape(1,1);
  if (logits.total() != labels_.size() || logits.type() != CV_32F || !cv::checkRange(logits)) {
    throw std::runtime_error("classifier output/labels mismatch or nonfinite logits");
  }
  double maximum;
  cv::minMaxLoc(logits, nullptr, &maximum);
  cv::Mat probabilities;
  cv::exp(logits - maximum, probabilities);
  probabilities /= cv::sum(probabilities)[0];
  cv::Point index;
  NumberClassification result;
  cv::minMaxLoc(probabilities, nullptr, &result.confidence, nullptr, &index);
  result.label = labels_.at(static_cast<std::size_t>(index.x));
  if (result.label == "negative") result.rejection = "negative";
  else if (result.confidence < config_.min_confidence) result.rejection = "low_confidence";
  return result;
}
}  // namespace srm::vision
