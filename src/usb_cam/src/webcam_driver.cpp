#include "usb_cam/webcam_driver.hpp"
#include <camera_info_manager/camera_info_manager.hpp>

using namespace std::chrono_literals;

WebcamDriver::WebcamDriver(const rclcpp::NodeOptions & options)
: Node("camera", options)
{
  this->declare_parameter("video_device", "/dev/video2");
  this->declare_parameter("frame_width", 1920);
  this->declare_parameter("frame_height", 1080);
  this->declare_parameter("fps", 30);

  std::string video_device = this->get_parameter("video_device").as_string();
  int frame_width = this->get_parameter("frame_width").as_int();
  int frame_height = this->get_parameter("frame_height").as_int();
  int fps = this->get_parameter("fps").as_int();

  cap_.open(video_device, cv::CAP_V4L2);

  if (!cap_.isOpened()) {
    RCLCPP_ERROR(get_logger(), "Failed to open webcam at %s", video_device.c_str());
    throw std::runtime_error("Webcam open failed");
  }

  cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
  cap_.set(cv::CAP_PROP_FRAME_WIDTH, frame_width);
  cap_.set(cv::CAP_PROP_FRAME_HEIGHT, frame_height);
  cap_.set(cv::CAP_PROP_FPS, fps);

  // CameraInfoManager no namespace do node ("camera")
  cinfo_ = std::make_shared<camera_info_manager::CameraInfoManager>(this, "camera");

  // Publisher SEM barra, para ficar em /camera/camera_info
  image_pub_ = image_transport::create_publisher(this, "image_raw");
  camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 10);

  int interval_ms = 1000 / fps;
  timer_ = create_wall_timer(
    std::chrono::milliseconds(interval_ms),
    std::bind(&WebcamDriver::capture_frame, this));

  RCLCPP_INFO(get_logger(), "Webcam driver started on %s", video_device.c_str());
}

void WebcamDriver::capture_frame()
{
  cv::Mat frame;
  cap_ >> frame;

  if (frame.empty()) {
    RCLCPP_WARN(get_logger(), "Empty frame captured");
    return;
  }

  auto image_msg = cv_bridge::CvImage(
    std_msgs::msg::Header(),
    "bgr8",
    frame
  ).toImageMsg();

  rclcpp::Time stamp = now();
  image_msg->header.stamp = stamp;
  image_msg->header.frame_id = "camera_frame";

  auto camera_info_msg = cinfo_->getCameraInfo();
  camera_info_msg.header.stamp = stamp;
  camera_info_msg.header.frame_id = "camera_frame";
  camera_info_msg.width = frame.cols;
  camera_info_msg.height = frame.rows;

  image_pub_.publish(image_msg);
  camera_info_pub_->publish(camera_info_msg);

  RCLCPP_DEBUG(get_logger(), "Frame publicado: %dx%d", frame.cols, frame.rows);
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(WebcamDriver)