#ifndef SHU_AUTO_AIM_POSE_SOLVER_HPP
#define SHU_AUTO_AIM_POSE_SOLVER_HPP

#include <array>
#include <string>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#include "vision/traditional_detector.hpp"

namespace srm::vision
{

struct PoseSolverConfig
{
  // 相机内参。必须使用和输入图像相同的分辨率/缩放关系。
  cv::Matx33d camera_matrix = cv::Matx33d::eye();
  std::vector<double> distortion_coefficients;

  // 相机坐标系 -> 云台坐标系：p_gimbal = R_camera_to_gimbal * p_camera + t_camera_to_gimbal。
  // 默认轴映射沿用参考项目常见约定：相机 +Z -> 云台 +X，+X -> 云台 -Y，+Y -> 云台 -Z。
  cv::Matx33d rotation_camera_to_gimbal{
    0.0, 0.0, 1.0,
    -1.0, 0.0, 0.0,
    0.0, -1.0, 0.0};
  cv::Vec3d translation_camera_to_gimbal{0.0, 0.0, 0.0};

  // 校内赛规则给出的装甲板外形尺寸，单位 m；与参考项目旧版 SMALL_ARMOR_WIDTH 不同，按规则为准。
  double armor_width_m = 0.140;
  double armor_height_m = 0.125;

  // IPPE 对平面目标返回两个可能姿态时，拒绝重投影 RMS 大于此值的解。
  double max_reprojection_rms_px = 5.0;
  double rotation_orthogonality_tolerance = 1e-3;
};

struct PoseSolution
{
  bool valid = false;
  std::string error;

  cv::Vec3d translation_camera_m{};
  cv::Vec3d translation_gimbal_m{};
  cv::Vec3d rotation_vector_camera{};

  // 云台轴约定参考 sp_vision_25 的 xyz2ypd：x 前、y 左、z 上；角度为弧度。
  double yaw_rad = 0.0;
  double pitch_rad = 0.0;
  double yaw_deg = 0.0;
  double pitch_deg = 0.0;
  double range_m = 0.0;
  double reprojection_rms_px = 0.0;
};

class ArmorPoseSolver final
{
public:
  explicit ArmorPoseSolver(PoseSolverConfig config);

  // 输入角点顺序为检测器约定的 TL, TR, BR, BL。
  // 参数无效时抛 std::invalid_argument；PnP 无有效正深度解时返回 valid=false。
  PoseSolution solve(const std::array<cv::Point2f, 4> & image_points) const;
  PoseSolution solve(const ArmorObservation & armor) const;

  const PoseSolverConfig & config() const noexcept;
  static std::array<cv::Point3f, 4> object_points(
    double width_m = 0.140, double height_m = 0.125);

private:
  void validate_config() const;
  PoseSolverConfig config_;
  std::array<cv::Point3f, 4> object_points_;
};

}  // namespace srm::vision

#endif  // SHU_AUTO_AIM_POSE_SOLVER_HPP
