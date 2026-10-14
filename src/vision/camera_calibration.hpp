#ifndef SHU_AUTO_AIM_CAMERA_CALIBRATION_HPP
#define SHU_AUTO_AIM_CAMERA_CALIBRATION_HPP

#include "io/image_geometry.hpp"
#include "vision/pose_solver.hpp"

namespace srm::vision
{
struct CameraCalibration
{
  io::ImageGeometry geometry;
  cv::Matx33d camera_matrix = cv::Matx33d::eye();
  std::vector<double> distortion_coefficients;
};
CameraCalibration load_camera_calibration(const std::string & path);
void validate_geometry(const io::ImageGeometry & expected, const io::ImageGeometry & actual);
void load_camera_extrinsics(const std::string & path, const io::ImageGeometry & geometry,
                            PoseSolverConfig & config);

struct ChessboardSpec
{
  cv::Size inner_corners;
  double square_size_m = 0.0;
};
struct ChessboardResult
{
  CameraCalibration camera;
  double rms_px = 0.0;
  std::vector<double> view_rms_px;
};
// Headless API: detection and calibration can be reviewed independently of the CLI.
class ChessboardCalibration final
{
public:
  ChessboardCalibration(ChessboardSpec board, int orientation_deg);
  bool add_image(const cv::Mat & bgr, std::vector<cv::Point2f> & corners);
  ChessboardResult solve() const;
  std::size_t view_count() const noexcept { return image_points_.size(); }
  const ChessboardSpec & board() const noexcept { return board_; }
private:
  ChessboardSpec board_;
  io::ImageGeometry geometry_;
  std::vector<cv::Point3f> object_points_;
  std::vector<std::vector<cv::Point2f>> image_points_;
};
void save_camera_calibration(const std::string & path, const ChessboardSpec & board,
                             const ChessboardResult & result);
}  // namespace srm::vision
#endif
