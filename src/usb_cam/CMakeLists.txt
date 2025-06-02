cmake_minimum_required(VERSION 3.12)
project(usb_cam)

# Configuração C++
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Busca dependências
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

# Configuração de includes
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# Executável
add_executable(webcam_driver
  src/webcam_driver.cpp
)

# Linkagem
target_link_libraries(webcam_driver
  ${rclcpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${cv_bridge_LIBRARIES}
  ${image_transport_LIBRARIES}
  ${OpenCV_LIBS}
)

# Instalação
install(TARGETS webcam_driver
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  launch 
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY 
  include/
  DESTINATION include
)

# Exporta dependências
ament_export_dependencies(
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
)

ament_package()