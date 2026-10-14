#include "vision/camera_calibration.hpp"
#include "io/frame_source.hpp"
#include <cassert>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <unistd.h>

namespace
{
template<class F> void rejects(F action)
{
  bool rejected = false;
  try { action(); } catch (const std::exception &) { rejected = true; }
  assert(rejected);
}

// Render a physical planar checkerboard through a known pinhole camera.
cv::Mat chessboard(int view)
{
  cv::Mat board(350, 450, CV_8UC3, cv::Scalar(255,255,255));
  for (int y = 0; y < 7; ++y) for (int x = 0; x < 9; ++x) {
    if ((x+y)%2 == 0) cv::rectangle(board, {x*50,y*50,50,50}, {0,0,0}, cv::FILLED);
  }
  std::vector<cv::Point3f> outer{{-.03F,-.03F,0}, {.24F,-.03F,0}, {.24F,.18F,0}, {-.03F,.18F,0}};
  std::vector<cv::Point2f> projected;
  const cv::Matx33d k(800,0,320,0,810,240,0,0,1);
  cv::projectPoints(outer, cv::Vec3d(-.3+.06*view, .24*std::sin(view), .08*std::cos(view)),
    cv::Vec3d(-.12+.02*std::sin(view), -.08+.02*std::cos(view), .8+.02*view), k, cv::noArray(), projected);
  const std::vector<cv::Point2f> pixels{{0,0},{450,0},{450,350},{0,350}};
  cv::Mat image;
  cv::warpPerspective(board, image, cv::getPerspectiveTransform(pixels, projected), {640,480},
    cv::INTER_LINEAR, cv::BORDER_CONSTANT, {180,180,180});
  return image;
}

void test_directions(const std::string & root)
{
  const auto path = root + "/asymmetric.png";
  cv::Mat image(4,6,CV_8UC3,cv::Scalar(0,0,0));
  image.at<cv::Vec3b>(0,0) = {11,22,33};
  assert(cv::imwrite(path, image));
  srm::io::FrameSourceConfig config;
  config.uri = path;
  config.camera_rotation_deg = 180; // Must not affect files.
  srm::io::Frame frame;
  srm::io::FrameSource native(config);
  assert(native.read(frame));
  assert(frame.bgr.at<cv::Vec3b>(0,0) == cv::Vec3b(11,22,33));
  config.file_rotation_deg = 90;
  srm::io::FrameSource rotated(config);
  assert(rotated.read(frame));
  assert(frame.bgr.size() == cv::Size(4,6) && frame.orientation_deg == 90);
  assert(frame.bgr.at<cv::Vec3b>(0,3) == cv::Vec3b(11,22,33));
  const auto recorded = root + "/recorded.png";
  assert(cv::imwrite(recorded, frame.bgr));
  srm::io::write_image_geometry(recorded + ".metadata.yml", {frame.bgr.size(),frame.orientation_deg});
  config.uri = recorded;
  config.file_rotation_deg = 0;
  srm::io::FrameSource replay(config);
  assert(replay.read(frame));
  assert(frame.orientation_deg == 90 && frame.bgr.size() == cv::Size(4,6));
  assert(frame.bgr.at<cv::Vec3b>(0,3) == cv::Vec3b(11,22,33));
  rejects([&] { srm::io::write_image_geometry(recorded + ".metadata.yml", {{4,6},0}); });
}

void test_chessboard(const std::string & root)
{
  srm::vision::ChessboardCalibration calibration({{8,6},.03}, 0);
  cv::VideoWriter video(root + "/boards.avi", cv::CAP_FFMPEG,
    cv::VideoWriter::fourcc('F','F','V','1'), 30, {640,480});
  assert(video.isOpened());
  std::vector<cv::Point2f> corners;
  for (int view = 0; view < 12; ++view) {
    const auto image = chessboard(view);
    assert(cv::imwrite(root + "/board_" + std::to_string(view) + ".png", image));
    video.write(image);
    assert(calibration.add_image(image, corners));
    assert(corners.size() == 48);
  }
  video.release();
  const auto result = calibration.solve();
  assert(result.view_rms_px.size() == 12 && result.rms_px < .6);
  assert(std::abs(result.camera.camera_matrix(0,0) - 800) < 45);
  assert(std::abs(result.camera.camera_matrix(1,1) - 810) < 45);
  const auto path = root + "/synthetic.yml";
  srm::vision::save_camera_calibration(path, calibration.board(), result);
  const auto loaded = srm::vision::load_camera_calibration(path);
  assert(loaded.geometry.size == cv::Size(640,480));
  assert(cv::norm(cv::Mat(loaded.camera_matrix - result.camera.camera_matrix)) < 1e-9);
  assert(loaded.distortion_coefficients == result.camera.distortion_coefficients);
  rejects([&] { srm::vision::save_camera_calibration(path, calibration.board(), result); });
  rejects([&] { srm::vision::validate_geometry(loaded.geometry, {{480,640},0}); });
  rejects([&] { srm::vision::validate_geometry(loaded.geometry, {{640,480},180}); });
  rejects([&] { calibration.add_image(cv::Mat::zeros(240,320,CV_8UC3), corners); });
  srm::vision::ChessboardCalibration empty({{8,6},.03},0);
  assert(!empty.add_image(cv::Mat::zeros(480,640,CV_8UC3), corners));
  rejects([&] { (void)empty.solve(); });
  rejects([] { srm::vision::ChessboardCalibration invalid({{8,6},0},0); });
  srm::vision::ChessboardCalibration duplicate({{8,6},.03},0);
  for (int view = 0; view < 8; ++view) assert(duplicate.add_image(chessboard(0), corners));
  rejects([&] { (void)duplicate.solve(); });
}

void test_invalid_files(const std::string & root)
{
  const auto path = root + "/bad.yml";
  const auto write = [&](const std::string & text) { std::ofstream file(path); file << "%YAML:1.0\n---\n" << text; };
  write("schema_version: 1\n");
  rejects([&] { (void)srm::vision::load_camera_calibration(path); });
  const std::string geometry = "schema_version: 1\nimage_width: 640\nimage_height: 480\norientation_deg: 0\n";
  const std::string k = "camera_matrix: !!opencv-matrix\n   rows: 3\n   cols: 3\n   dt: d\n   data: [800.,0.,320.,0.,810.,240.,0.,0.,1.]\n";
  write(geometry + k + "distortion_coefficients: [0.,0.,0.]\n");
  rejects([&] { (void)srm::vision::load_camera_calibration(path); });
  write(geometry + k); // Missing distortion must not become a silent zero vector.
  rejects([&] { (void)srm::vision::load_camera_calibration(path); });
  const std::string fields = geometry + "camera_frame: opencv_processed\ngimbal_frame: x_forward_y_left_z_up\ntranslation_unit: m\n";
  const std::string rotation = "rotation_camera_to_gimbal: !!opencv-matrix\n   rows: 3\n   cols: 3\n   dt: d\n   data: [0.,0.,1.,-1.,0.,0.,0.,-1.,0.]\n";
  write(fields + rotation + "translation_camera_to_gimbal: [0.01,0.02,0.03]\n");
  srm::vision::PoseSolverConfig config;
  srm::vision::load_camera_extrinsics(path, {{640,480},0}, config);
  assert(config.translation_camera_to_gimbal == cv::Vec3d(.01,.02,.03));
  rejects([&] { srm::vision::load_camera_extrinsics(path, {{640,480},180}, config); });
  write(fields + "rotation_camera_to_gimbal: !!opencv-matrix\n   rows: 3\n   cols: 3\n   dt: d\n   data: [-1.,0.,0.,0.,1.,0.,0.,0.,1.]\ntranslation_camera_to_gimbal: [0.,0.,0.]\n");
  rejects([&] { srm::vision::load_camera_extrinsics(path, {{640,480},0}, config); });
}
}

int main(int argc, char ** argv)
{
  const auto root = argc > 1 ? std::string(argv[1]) : "/tmp/opencode/srm_calibration_" + std::to_string(::getpid());
  std::filesystem::create_directories(root);
  test_directions(root);
  test_chessboard(root);
  test_invalid_files(root);
  std::cout << "synthetic calibration and geometry checks passed: " << root << '\n';
}
