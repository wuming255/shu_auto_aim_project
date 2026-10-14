#include "io/image_geometry.hpp"

#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <fcntl.h>
#include <unistd.h>

namespace srm::io
{
void validate_rotation(int degrees)
{
  if (degrees != 0 && degrees != 90 && degrees != 180 && degrees != 270) {
    throw std::invalid_argument("rotation/orientation must be 0, 90, 180 or 270 degrees clockwise");
  }
}

cv::Mat rotate_image(const cv::Mat & image, int degrees)
{
  validate_rotation(degrees);
  if (degrees == 0) return image;
  cv::Mat result;
  cv::rotate(image, result, degrees == 90 ? cv::ROTATE_90_CLOCKWISE :
    degrees == 180 ? cv::ROTATE_180 : cv::ROTATE_90_COUNTERCLOCKWISE);
  return result;
}

ImageGeometry read_image_geometry(const std::string & path)
{
  cv::FileStorage file(path, cv::FileStorage::READ);
  if (!file.isOpened()) throw std::invalid_argument("cannot open image metadata: " + path);
  for (const auto * key : {"schema_version", "image_width", "image_height", "orientation_deg"}) {
    if (!file[key].isInt()) throw std::invalid_argument(std::string("missing/integer metadata field: ") + key);
  }
  ImageGeometry result{{static_cast<int>(file["image_width"]), static_cast<int>(file["image_height"])},
    static_cast<int>(file["orientation_deg"])};
  if (static_cast<int>(file["schema_version"]) != 1 || result.size.width <= 0 || result.size.height <= 0) {
    throw std::invalid_argument("invalid image metadata schema/resolution: " + path);
  }
  validate_rotation(result.orientation_deg);
  return result;
}

void write_new_text_file(const std::string & path, const std::string & text)
{
  const int fd = ::open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0666);
  if (fd < 0) throw std::runtime_error("cannot create new output '" + path + "': " + std::strerror(errno));
  std::size_t offset = 0;
  while (offset < text.size()) {
    const auto written = ::write(fd, text.data() + offset, text.size() - offset);
    if (written < 0 && errno == EINTR) continue;
    if (written <= 0) {
      ::close(fd);
      ::unlink(path.c_str());
      throw std::runtime_error("failed to write output: " + path);
    }
    offset += static_cast<std::size_t>(written);
  }
  if (::close(fd) != 0) {
    ::unlink(path.c_str());
    throw std::runtime_error("failed to close output: " + path);
  }
}

void write_image_geometry(const std::string & path, const ImageGeometry & geometry)
{
  validate_rotation(geometry.orientation_deg);
  if (geometry.size.width <= 0 || geometry.size.height <= 0) throw std::invalid_argument("invalid metadata resolution");
  cv::FileStorage file(".yml", cv::FileStorage::WRITE | cv::FileStorage::MEMORY);
  file << "schema_version" << 1 << "image_width" << geometry.size.width
       << "image_height" << geometry.size.height << "orientation_deg" << geometry.orientation_deg;
  write_new_text_file(path, file.releaseAndGetString());
}
}  // namespace srm::io
