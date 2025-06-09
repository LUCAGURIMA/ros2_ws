#ifndef USB_CAM__WEBCAM_DRIVER_HPP_
#define USB_CAM__WEBCAM_DRIVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

class WebcamDriver : public rclcpp::Node
{
public:
  WebcamDriver();

private:
  void capture_frame();
  rclcpp::TimerBase::SharedPtr timer_;
  image_transport::Publisher image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
  cv::VideoCapture cap_;
};

#endif  // USB_CAM__WEBCAM_DRIVER_HPP_