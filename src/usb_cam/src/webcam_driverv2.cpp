#include "usb_cam/webcam_driver.hpp"

using namespace std::chrono_literals;

WebcamDriver::WebcamDriver() : Node("webcam_driver")
{
  // Declaração de parâmetros configuráveis
  this->declare_parameter("device", "/dev/video2");  // Caminho padrão para webcam USB
  this->declare_parameter("width", 1280);
  this->declare_parameter("height", 720);
  this->declare_parameter("fps", 30);
  this->declare_parameter("brightness", -1.0);     // -1 = valor padrão do driver
  this->declare_parameter("contrast", -1.0);       // -1 = valor padrão do driver
  this->declare_parameter("saturation", -1.0);     // -1 = valor padrão do driver
  this->declare_parameter("hue", -1.0);            // -1 = valor padrão do driver

  // Obtém os valores dos parâmetros
  std::string device = this->get_parameter("device").as_string();
  int width = this->get_parameter("width").as_int();
  int height = this->get_parameter("height").as_int();
  double fps = this->get_parameter("fps").as_int();
  double brightness = this->get_parameter("brightness").as_double();
  double contrast = this->get_parameter("contrast").as_double();
  double saturation = this->get_parameter("saturation").as_double();
  double hue = this->get_parameter("hue").as_double();

  // Tenta abrir a câmera (USB ou embutida)
  RCLCPP_INFO(get_logger(), "Tentando abrir dispositivo: %s", device.c_str());
  cap_.open(device, cv::CAP_V4L2);  // Usa backend V4L2 para Linux
  
  if (!cap_.isOpened()) {
    RCLCPP_ERROR(get_logger(), "Falha ao abrir a webcam no dispositivo %s!", device.c_str());
    throw std::runtime_error("Webcam open failed");
  }

  // Configura resolução e FPS
  cap_.set(cv::CAP_PROP_FRAME_WIDTH, width);
  cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height);
  cap_.set(cv::CAP_PROP_FPS, fps);

  // Configura parâmetros da imagem (se valores != -1)
  auto set_param = [&](const std::string& name, int prop_id, double value) {
    if (value >= 0.0) {
      if (cap_.set(prop_id, value)) {
        RCLCPP_INFO(get_logger(), "%s configurado para: %.2f", name.c_str(), value);
      } else {
        RCLCPP_WARN(get_logger(), "Falha ao configurar %s", name.c_str());
      }
    }
  };

  set_param("Brilho", cv::CAP_PROP_BRIGHTNESS, brightness);
  set_param("Contraste", cv::CAP_PROP_CONTRAST, contrast);
  set_param("Saturação", cv::CAP_PROP_SATURATION, saturation);
  set_param("Matiz (Hue)", cv::CAP_PROP_HUE, hue);

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

  // Publisher de imagens
  image_pub_ = image_transport::create_publisher(this, "image_raw");
  
  // Timer baseado no FPS configurado
  auto timer_period = std::chrono::milliseconds(static_cast<int>(1000 / fps));
  timer_ = create_wall_timer(
    timer_period,
    std::bind(&WebcamDriver::capture_frame, this));
    
  RCLCPP_INFO(get_logger(), "Driver de webcam inicializado");
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