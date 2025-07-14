#pragma once

#include <apriltag/apriltag.h>
#include <functional>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <unordered_map>
#include <array>
#include <opencv2/core.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>

typedef std::function<geometry_msgs::msg::Transform(apriltag_detection_t* const, const std::array<double, 4>&, const double&)> pose_estimation_f;

extern const std::unordered_map<std::string, pose_estimation_f> pose_estimation_methods;

// Função utilitária para logar pose
void log_pose(const cv::Mat& tvec, const cv::Mat& rvec);

// Função para publicar pose
void publish_pose(
    const cv::Mat& tvec,
    const cv::Mat& rvec,
    const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr& pose_pub,
    const std_msgs::msg::Header& header);