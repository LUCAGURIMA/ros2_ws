#include "../include/usb_cam/webcam_driver.hpp"

using namespace std::chrono_literals;

WebcamDriver::WebcamDriver() : Node("webcam_driver")
{
  // Abre a webcam com backend V4L2
  cap_.open(2, cv::CAP_V4L2);
  
  if (!cap_.isOpened()) {
    RCLCPP_ERROR(get_logger(), "Failed to open webcam!");
    throw std::runtime_error("Webcam open failed");
  }
  // Após abrir a câmera, adicione:
  cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));  // Tente MJPG primeiro
  // Configurações de resolução e FPS
  cap_.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
  cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
  cap_.set(cv::CAP_PROP_FPS, 30);

  // Cria publisher diretamente sem ImageTransport
  image_pub_ = image_transport::create_publisher(this, "image_raw");
  
  // Timer para captura (30 FPS)
  timer_ = create_wall_timer(
    33ms,
    std::bind(&WebcamDriver::capture_frame, this));
    
  RCLCPP_INFO(get_logger(), "Webcam driver started");
  // Exibe configurações atuais
  RCLCPP_INFO(get_logger(), "Configurações atuais:");
  RCLCPP_INFO(get_logger(), "  Resolução: %dx%d", 
              static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_WIDTH)),
              static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT)));
  RCLCPP_INFO(get_logger(), "  FPS: %.2f", cap_.get(cv::CAP_PROP_FPS));
  RCLCPP_INFO(get_logger(), "  Brilho: %.2f", cap_.get(cv::CAP_PROP_BRIGHTNESS));
  RCLCPP_INFO(get_logger(), "  Contraste: %.2f", cap_.get(cv::CAP_PROP_CONTRAST));
  RCLCPP_INFO(get_logger(), "  Saturação: %.2f", cap_.get(cv::CAP_PROP_SATURATION));
  RCLCPP_INFO(get_logger(), "  Matiz (Hue): %.2f", cap_.get(cv::CAP_PROP_HUE));

}

void WebcamDriver::capture_frame()
{
  cv::Mat frame;
  cap_ >> frame;
  
  if (frame.empty()) {
    RCLCPP_WARN(get_logger(), "Empty frame captured");
    return;
  }

  // Converte para mensagem ROS
  auto msg = cv_bridge::CvImage(
    std_msgs::msg::Header(),
    "bgr8",
    frame
  ).toImageMsg();

  msg->header.stamp = now();
  msg->header.frame_id = "camera_frame";
  
  image_pub_.publish(msg);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WebcamDriver>());
  rclcpp::shutdown();
  return 0;
}