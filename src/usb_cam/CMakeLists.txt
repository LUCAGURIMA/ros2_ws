cmake_minimum_required(VERSION 3.12)
project(usb_cam)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(class_loader REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

add_library(webcam_driver SHARED
  src/webcam_driver.cpp
)
ament_target_dependencies(webcam_driver
  rclcpp
  rclcpp_components
  sensor_msgs
  cv_bridge
  image_transport
  camera_info_manager
  class_loader
  yaml-cpp
  OpenCV
)
target_link_libraries(webcam_driver
  ${OpenCV_LIBS}
  yaml-cpp
)

rclcpp_components_register_nodes(webcam_driver "WebcamDriver")

install(TARGETS webcam_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_executable(webcam_driver_node src/webcam_driver_main.cpp)
ament_target_dependencies(webcam_driver_node
  rclcpp
  rclcpp_components
  sensor_msgs
  cv_bridge
  image_transport
  camera_info_manager
  class_loader
  yaml-cpp
  OpenCV
)
target_link_libraries(webcam_driver_node
  webcam_driver
  ${OpenCV_LIBS}
  yaml-cpp
)

install(TARGETS webcam_driver_node
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

ament_export_dependencies(
  rclcpp
  rclcpp_components
  sensor_msgs
  cv_bridge
  image_transport
  camera_info_manager
  class_loader
  yaml-cpp
  OpenCV
)

ament_package()