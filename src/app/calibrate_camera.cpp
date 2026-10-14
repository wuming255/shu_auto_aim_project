#include "vision/camera_calibration.hpp"
#include "io/frame_source.hpp"

#include <algorithm>
#include <cmath>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace
{
const char * help =
  "Usage: srm_calibrate_camera --input images|video --source PATH --output NEW_DIR\n"
  "  --board chessboard --columns N --rows N --square-size-mm MM   required board specification\n"
  "  columns/rows are INNER corners, not squares. Source images: directory of PNG/JPG/BMP/TIF/PPM.\n"
  "  --file-rotation 0|90|180|270   additional clockwise rotation (default: 0)\n"
  "  --file-orientation DEGREES    orientation already stored in legacy pixels (default: 0 or sidecar)\n"
  "  --every N                    sample every Nth decoded frame/image (default: 1)\n"
  "  --max-views N                stop after N accepted views (default: 60, minimum: 8)\n"
  "  --max-frames N               limit decoded inputs (default: 3000)\n"
  "  --max-rms PX                 optional maximum RMS; reject result above it\n"
  "  --help                       no camera/CDC access; all operation is offline/headless\n"
  "Output: camera.yml, views.csv, detected_*.png, undistorted_first.png. Existing output is rejected.\n"
  "Result is a calibration candidate; inspect views/errors and verify with independent images.\n";

double number(const std::string & text)
{
  std::size_t used = 0;
  const double value = std::stod(text, &used);
  if (used != text.size() || !std::isfinite(value)) throw std::invalid_argument("invalid numeric argument: " + text);
  return value;
}
int integer(const std::string & text)
{
  const double value = number(text);
  if (value != std::floor(value) || value < 0 || value > 1000000) throw std::invalid_argument("invalid integer: " + text);
  return static_cast<int>(value);
}
}

int main(int argc, char ** argv)
{
  try {
    std::string input, source, output, board_type;
    srm::vision::ChessboardSpec board;
    srm::io::FrameSourceConfig config;
    int every = 1, max_views = 60, max_frames = 3000;
    double max_rms = 0.0;
    for (int i = 1; i < argc; ++i) {
      const std::string key = argv[i];
      if (key == "--help" || key == "-h") { std::cout << help; return 0; }
      if (++i == argc) throw std::invalid_argument("missing value for " + key);
      const std::string value = argv[i];
      if (key == "--input") input = value;
      else if (key == "--source") source = value;
      else if (key == "--output") output = value;
      else if (key == "--board") board_type = value;
      else if (key == "--columns") board.inner_corners.width = integer(value);
      else if (key == "--rows") board.inner_corners.height = integer(value);
      else if (key == "--square-size-mm") board.square_size_m = number(value) / 1000.0;
      else if (key == "--file-rotation") config.file_rotation_deg = integer(value);
      else if (key == "--file-orientation") config.file_orientation_deg = integer(value);
      else if (key == "--every") every = integer(value);
      else if (key == "--max-views") max_views = integer(value);
      else if (key == "--max-frames") max_frames = integer(value);
      else if (key == "--max-rms") max_rms = number(value);
      else throw std::invalid_argument("unknown option: " + key);
    }
    if ((input != "images" && input != "video") || source.empty() || output.empty() || board_type != "chessboard") {
      throw std::invalid_argument("explicit --input images|video, --source, --output and --board chessboard required; see --help");
    }
    if (every < 1 || max_frames < 1 || max_views < 8 || max_rms < 0) throw std::invalid_argument("invalid sampling/RMS limit");
    // Validate board specification before creating output or decoding input.
    (void)srm::vision::ChessboardCalibration(board, config.file_rotation_deg);
    if (config.file_orientation_deg) srm::io::validate_rotation(*config.file_orientation_deg);
    std::vector<std::string> images;
    if (input == "images") {
      for (const auto & entry : std::filesystem::directory_iterator(source)) {
        auto ext = entry.path().extension().string();
        std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) { return std::tolower(c); });
        if (entry.is_regular_file() && (ext == ".png" || ext == ".jpg" || ext == ".jpeg" || ext == ".bmp" ||
            ext == ".tif" || ext == ".tiff" || ext == ".ppm")) images.push_back(entry.path().string());
      }
      std::sort(images.begin(), images.end());
      if (images.empty()) throw std::invalid_argument("no supported images in input directory");
    }
    std::unique_ptr<srm::io::FrameSource> video;
    if (input == "video") {
      config.type = srm::io::FrameSourceType::video_file;
      config.uri = source;
      video = std::make_unique<srm::io::FrameSource>(config);
    }
    if (!std::filesystem::create_directory(output)) throw std::invalid_argument("output directory already exists: " + output);
    std::ofstream report(output + "/views.csv");
    if (!report) throw std::runtime_error("cannot create views report");
    report << "sample_index,source_frame,source_time_seconds,found,orientation_deg,view_index\n";
    std::unique_ptr<srm::vision::ChessboardCalibration> calibration;
    std::optional<srm::io::ImageGeometry> sample_geometry;
    std::vector<int> accepted_indices;
    cv::Mat first_accepted;
    int scanned = 0;
    for (; scanned < max_frames; ++scanned) {
      srm::io::Frame frame;
      if (video) {
        if (!video->read(frame)) break;
      } else {
        if (static_cast<std::size_t>(scanned) >= images.size()) break;
        if (scanned % every != 0) continue;
        config.type = srm::io::FrameSourceType::image_file;
        config.uri = images[scanned];
        srm::io::FrameSource image(config);
        if (!image.read(frame)) throw std::runtime_error("failed to read input image");
        frame.sequence = static_cast<std::uint64_t>(scanned);
      }
      if (scanned % every != 0) continue;
      if (!calibration) {
        calibration = std::make_unique<srm::vision::ChessboardCalibration>(board, frame.orientation_deg);
        sample_geometry = srm::io::ImageGeometry{frame.bgr.size(),frame.orientation_deg};
        srm::io::write_image_geometry(output + "/sample_geometry.yml", *sample_geometry);
      }
      // For image directories, sidecars may differ; do not mix coordinate conventions.
      srm::vision::validate_geometry(*sample_geometry, {frame.bgr.size(),frame.orientation_deg});
      std::vector<cv::Point2f> corners;
      const bool found = calibration->add_image(frame.bgr, corners);
      if (found && first_accepted.empty()) {
        first_accepted = frame.bgr.clone();
      }
      const auto view_index = found ? static_cast<int>(calibration->view_count()) - 1 : -1;
      report << scanned << ',' << frame.sequence << ',' << frame.source_time_seconds << ',' << found
             << ',' << frame.orientation_deg << ',' << view_index << '\n';
      auto drawing = frame.bgr.clone();
      cv::drawChessboardCorners(drawing, board.inner_corners, corners, found);
      if (!cv::imwrite(output + "/detected_" + std::to_string(scanned) + ".png", drawing)) throw std::runtime_error("cannot write detection image");
      if (found) accepted_indices.push_back(scanned);
      std::cout << "sample=" << scanned << " found=" << found << " views=" << calibration->view_count() << '\n';
      if (calibration->view_count() >= static_cast<std::size_t>(max_views)) { ++scanned; break; }
    }
    if (!calibration) throw std::runtime_error("no sampled input frames");
    const auto result = calibration->solve();
    report << "\nview_index,sample_index,reprojection_rms_px\n";
    for (std::size_t i = 0; i < accepted_indices.size(); ++i) report << i << ',' << accepted_indices[i] << ',' << result.view_rms_px[i] << '\n';
    report.flush();
    if (!report) throw std::runtime_error("failed writing views report");
    if (max_rms > 0 && result.rms_px > max_rms) throw std::runtime_error("calibration RMS exceeds --max-rms; inspect views.csv; camera.yml not published");
    cv::Mat undistorted;
    cv::undistort(first_accepted, undistorted, result.camera.camera_matrix, result.camera.distortion_coefficients);
    if (!cv::imwrite(output + "/undistorted_first.png", undistorted)) throw std::runtime_error("cannot write undistorted image");
    srm::vision::save_camera_calibration(output + "/camera.yml", board, result);
    std::cout << "decoded_inputs=" << scanned << " accepted_views=" << calibration->view_count()
              << " rms_px=" << result.rms_px << " calibration=" << output << "/camera.yml\n"
              << "Inspect detected images, per-view RMS, coverage and independent validation before deployment.\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "srm_calibrate_camera: " << error.what() << '\n';
    return 1;
  }
}
