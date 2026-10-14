#ifndef SHU_AUTO_AIM_IMAGE_GEOMETRY_HPP
#define SHU_AUTO_AIM_IMAGE_GEOMETRY_HPP

#include <string>
#include <opencv2/core.hpp>

namespace srm::io
{
void validate_rotation(int degrees);
cv::Mat rotate_image(const cv::Mat & image, int degrees);

// Orientation is clockwise from native sensor pixels. Stored pixels already have this rotation.
struct ImageGeometry
{
  cv::Size size;
  int orientation_deg = 0;
};
ImageGeometry read_image_geometry(const std::string & path);
void write_image_geometry(const std::string & path, const ImageGeometry & geometry);
// Publish a complete text file without replacing an existing path, including symlinks.
void write_new_text_file(const std::string & path, const std::string & text);
}  // namespace srm::io
#endif
