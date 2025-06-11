#ifndef USB_CAM__WEBCAM_DRIVER_HPP_
#define USB_CAM__WEBCAM_DRIVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <camera_info_manager/camera_info_manager.hpp>

class WebcamDriver : public rclcpp::Node
{
public:
  explicit WebcamDriver(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void capture_frame();
  rclcpp::TimerBase::SharedPtr timer_;
  image_transport::Publisher image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
  cv::VideoCapture cap_;
  std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;
};

#endif  // USB_CAM__WEBCAM_DRIVER_HPP_