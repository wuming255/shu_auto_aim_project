#ifndef SHU_AUTO_AIM_ROS_IMAGE_HPP
#define SHU_AUTO_AIM_ROS_IMAGE_HPP

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "vision/camera_calibration.hpp"

namespace srm::io
{
// Return owned, unrotated BGR8 pixels; reject unsupported encodings and short rows/buffers.
cv::Mat decode_ros_image(const sensor_msgs::msg::Image & image);

// CameraInfo describes the simulator's native upright pinhole image, not the real camera.
vision::CameraCalibration camera_from_ros_info(const sensor_msgs::msg::CameraInfo & info);
}  // namespace srm::io

#endif
