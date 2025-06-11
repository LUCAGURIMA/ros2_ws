#include "usb_cam/webcam_driver.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WebcamDriver>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}