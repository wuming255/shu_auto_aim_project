#ifndef SHU_AUTO_AIM_NUMBER_CLASSIFIER_HPP
#define SHU_AUTO_AIM_NUMBER_CLASSIFIER_HPP

#include <string>
#include <vector>
#include <opencv2/dnn.hpp>

namespace srm::vision
{
struct ArmorObservation;

struct NumberClassifierConfig
{
  bool enabled = false;  // 库调用者可仅使用几何；正式应用默认开启。
  std::string model_path;
  std::string label_path;
  double min_confidence = 0.7;
  double conflict_margin = 0.05;
};

struct NumberClassification
{
  std::string label;
  double confidence = 0.0;
  std::string rejection;
};

// 与检测器一样每实例由单个处理线程调用；OpenCV Net 的 forward 不可并发。
class NumberClassifier final
{
public:
  explicit NumberClassifier(NumberClassifierConfig config);
  NumberClassification classify(const cv::Mat & bgr, const ArmorObservation & armor) const;
private:
  NumberClassifierConfig config_;
  mutable cv::dnn::Net net_;
  std::vector<std::string> labels_;
};
}  // namespace srm::vision
#endif
