#include "vision/camera_calibration.hpp"

#include <cmath>
#include <stdexcept>
#include <opencv2/imgproc.hpp>

namespace srm::vision
{
namespace
{
cv::Matx33d matrix33(const cv::FileNode & node)
{
  cv::Mat matrix;
  node >> matrix;
  if (matrix.rows != 3 || matrix.cols != 3 || matrix.channels() != 1 || !cv::checkRange(matrix)) {
    throw std::invalid_argument("expected finite 3x3 matrix: " + node.name());
  }
  matrix.convertTo(matrix, CV_64F);
  return cv::Matx33d(matrix);
}

std::vector<double> vector_values(const cv::FileNode & node)
{
  if (!node.isSeq()) throw std::invalid_argument("expected explicit coefficient sequence: " + node.name());
  std::vector<double> result;
  for (const auto & value : node) {
    if (!value.isInt() && !value.isReal()) throw std::invalid_argument("non-numeric coefficient: " + node.name());
    const double number = static_cast<double>(value);
    if (!std::isfinite(number)) throw std::invalid_argument("non-finite coefficient: " + node.name());
    result.push_back(number);
  }
  return result;
}

void validate_camera(const CameraCalibration & camera)
{
  const auto & k = camera.camera_matrix;
  if (std::abs(k(0,1)) > 1e-9 || std::abs(k(1,0)) > 1e-9) {
    throw std::invalid_argument("camera matrix must use the OpenCV pinhole convention (zero skew)");
  }
  PoseSolverConfig config;
  config.camera_matrix = k;
  config.distortion_coefficients = camera.distortion_coefficients;
  (void)ArmorPoseSolver(config);
}
}  // namespace

CameraCalibration load_camera_calibration(const std::string & path)
{
  CameraCalibration camera;
  camera.geometry = io::read_image_geometry(path);
  cv::FileStorage file(path, cv::FileStorage::READ);
  camera.camera_matrix = matrix33(file["camera_matrix"]);
  camera.distortion_coefficients = vector_values(file["distortion_coefficients"]);
  validate_camera(camera);
  return camera;
}

void validate_geometry(const io::ImageGeometry & expected, const io::ImageGeometry & actual)
{
  if (expected.size != actual.size) throw std::invalid_argument("calibration resolution does not match delivered frame resolution");
  if (expected.orientation_deg != actual.orientation_deg) throw std::invalid_argument("calibration orientation does not match delivered frame orientation");
}

void load_camera_extrinsics(const std::string & path, const io::ImageGeometry & geometry, PoseSolverConfig & config)
{
  validate_geometry(io::read_image_geometry(path), geometry);
  cv::FileStorage file(path, cv::FileStorage::READ);
  if (static_cast<std::string>(file["camera_frame"]) != "opencv_processed" ||
      static_cast<std::string>(file["gimbal_frame"]) != "x_forward_y_left_z_up" ||
      static_cast<std::string>(file["translation_unit"]) != "m") {
    throw std::invalid_argument("extrinsics require camera_frame=opencv_processed, gimbal_frame=x_forward_y_left_z_up, translation_unit=m");
  }
  auto candidate = config;
  candidate.rotation_camera_to_gimbal = matrix33(file["rotation_camera_to_gimbal"]);
  const auto translation = vector_values(file["translation_camera_to_gimbal"]);
  if (translation.size() != 3) throw std::invalid_argument("extrinsic translation must contain 3 values in meters");
  candidate.translation_camera_to_gimbal = {translation[0],translation[1],translation[2]};
  (void)ArmorPoseSolver(candidate); // Existing orthonormality, determinant and finite checks.
  config = candidate;
}

ChessboardCalibration::ChessboardCalibration(ChessboardSpec board, int orientation_deg)
  : board_(board), geometry_{{},orientation_deg}
{
  io::validate_rotation(orientation_deg);
  if (board.inner_corners.width < 2 || board.inner_corners.height < 2 ||
      board.inner_corners.width > 100 || board.inner_corners.height > 100 ||
      !std::isfinite(board.square_size_m) || board.square_size_m <= 0.0 || board.square_size_m > 1.0) {
    throw std::invalid_argument("explicit chessboard inner corners [2,100] and square size (0,1] meters required");
  }
  // Same planar row/column object-point model as the reference calibration tool.
  for (int row = 0; row < board.inner_corners.height; ++row) {
    for (int col = 0; col < board.inner_corners.width; ++col) {
      object_points_.push_back({static_cast<float>(col*board.square_size_m),
        static_cast<float>(row*board.square_size_m),0.0F});
    }
  }
}

bool ChessboardCalibration::add_image(const cv::Mat & bgr, std::vector<cv::Point2f> & corners)
{
  corners.clear();
  if (bgr.empty() || bgr.type() != CV_8UC3) throw std::invalid_argument("chessboard input must be nonempty BGR8");
  if (geometry_.size.empty()) geometry_.size = bgr.size();
  if (geometry_.size != bgr.size()) throw std::invalid_argument("mixed calibration image resolutions");
  cv::Mat gray;
  cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
  const bool found = cv::findChessboardCorners(gray, board_.inner_corners, corners,
    cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK);
  if (!found) return false;
  cv::cornerSubPix(gray, corners, {5,5}, {-1,-1}, {cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 40, .001});
  image_points_.push_back(corners);
  return true;
}

ChessboardResult ChessboardCalibration::solve() const
{
  if (image_points_.size() < 8) throw std::invalid_argument("at least 8 detected chessboard views required; capture diverse positions/tilts");
  // Reject effectively identical observations; a low RMS alone cannot detect this degeneracy.
  double max_displacement = 0.0;
  for (const auto & points : image_points_) {
    max_displacement = std::max(max_displacement, cv::norm(points, image_points_.front(), cv::NORM_L2) / std::sqrt(points.size()));
  }
  if (max_displacement < 2.0) throw std::invalid_argument("calibration views are effectively identical; capture varied board poses");
  std::vector<std::vector<cv::Point3f>> objects(image_points_.size(), object_points_);
  cv::Mat k, d;
  std::vector<cv::Mat> rvecs, tvecs;
  ChessboardResult result;
  result.camera.geometry = geometry_;
  result.rms_px = cv::calibrateCamera(objects, image_points_, geometry_.size, k, d, rvecs, tvecs, 0,
    {cv::TermCriteria::COUNT | cv::TermCriteria::EPS,100,1e-9});
  if (!std::isfinite(result.rms_px)) throw std::runtime_error("calibration did not produce finite RMS");
  result.camera.camera_matrix = cv::Matx33d(k);
  result.camera.distortion_coefficients.assign(d.ptr<double>(), d.ptr<double>() + d.total());
  validate_camera(result.camera);
  for (std::size_t view = 0; view < image_points_.size(); ++view) {
    std::vector<cv::Point2f> projected;
    cv::projectPoints(object_points_, rvecs[view], tvecs[view], k, d, projected);
    result.view_rms_px.push_back(cv::norm(projected, image_points_[view], cv::NORM_L2) / std::sqrt(projected.size()));
  }
  return result;
}

void save_camera_calibration(const std::string & path, const ChessboardSpec & board, const ChessboardResult & result)
{
  validate_camera(result.camera);
  cv::FileStorage file(".yml", cv::FileStorage::WRITE | cv::FileStorage::MEMORY);
  file << "schema_version" << 1 << "image_width" << result.camera.geometry.size.width
       << "image_height" << result.camera.geometry.size.height << "orientation_deg" << result.camera.geometry.orientation_deg
       << "camera_matrix" << cv::Mat(result.camera.camera_matrix)
       << "distortion_coefficients" << result.camera.distortion_coefficients
       << "board_type" << "chessboard" << "inner_columns" << board.inner_corners.width
       << "inner_rows" << board.inner_corners.height << "square_size_m" << board.square_size_m
       << "rms_px" << result.rms_px << "view_rms_px" << result.view_rms_px;
  io::write_new_text_file(path, file.releaseAndGetString());
}
}  // namespace srm::vision
