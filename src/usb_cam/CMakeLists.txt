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
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${image_transport_INCLUDE_DIRS}
  ${camera_info_manager_INCLUDE_DIRS}
  ${ament_index_cpp_INCLUDE_DIRS}
  ${yaml-cpp_INCLUDE_DIRS}
)

add_library(webcam_driver SHARED
  src/webcam_driver.cpp
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
  sensor_msgs
  cv_bridge
  image_transport
  camera_info_manager
  yaml-cpp
)

ament_package()