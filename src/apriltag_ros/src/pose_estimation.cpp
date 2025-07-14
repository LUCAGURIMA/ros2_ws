#include "pose_estimation.hpp"
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
#include <apriltag/apriltag_pose.h>
#include <apriltag/common/homography.h>
#include <opencv2/calib3d.hpp>
#include <tf2/convert.h>
#include <rclcpp/rclcpp.hpp>
#include <cmath>

void publish_pose(
    const cv::Mat& tvec,
    const cv::Mat& rvec,
    const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr& pose_pub,
    const std_msgs::msg::Header& header)
{
    double x = tvec.at<double>(0);
    double y = tvec.at<double>(1);
    double z = tvec.at<double>(2);

    cv::Mat R;
    cv::Rodrigues(rvec, R);

    Eigen::Matrix3d eig_R;
    cv::cv2eigen(R, eig_R);
    Eigen::Quaterniond q(eig_R);

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header = header;
    pose_msg.pose.position.x = x;
    pose_msg.pose.position.y = y;
    pose_msg.pose.position.z = z;
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();

    pose_pub->publish(pose_msg);
}

geometry_msgs::msg::Transform
homography(apriltag_detection_t* const detection, const std::array<double, 4>& intr, const double& tagsize)
{
    apriltag_detection_info_t info = {detection, tagsize, intr[0], intr[1], intr[2], intr[3]};
    apriltag_pose_t pose;
    estimate_pose_for_tag_homography(&info, &pose);

    // rotate frame such that z points in the opposite direction towards the camera
    for(int i = 0; i < 3; i++) {
        // swap x and y axes
        std::swap(MATD_EL(pose.R, 0, i), MATD_EL(pose.R, 1, i));
        // invert z axis
        MATD_EL(pose.R, 2, i) *= -1;
    }

    return tf2::toMsg<apriltag_pose_t, geometry_msgs::msg::Transform>(const_cast<const apriltag_pose_t&>(pose));
}

geometry_msgs::msg::Transform
pnp(apriltag_detection_t* const detection, const std::array<double, 4>& intr, const double& tagsize)
{
    const std::vector<cv::Point3d> objectPoints{
        {-tagsize / 2, -tagsize / 2, 0},
        {+tagsize / 2, -tagsize / 2, 0},
        {+tagsize / 2, +tagsize / 2, 0},
        {-tagsize / 2, +tagsize / 2, 0},
    };

    const std::vector<cv::Point2d> imagePoints{
        {detection->p[0][0], detection->p[0][1]},
        {detection->p[1][0], detection->p[1][1]},
        {detection->p[2][0], detection->p[2][1]},
        {detection->p[3][0], detection->p[3][1]},
    };

    cv::Matx33d cameraMatrix = cv::Matx33d::eye();
    cameraMatrix(0, 0) = intr[0];// fx
    cameraMatrix(1, 1) = intr[1];// fy
    cameraMatrix(0, 2) = intr[2];// cx
    cameraMatrix(1, 2) = intr[3];// cy

    cv::Mat rvec, tvec;
    cv::solvePnP(objectPoints, imagePoints, cameraMatrix, {}, rvec, tvec);

    log_pose(tvec, rvec);

    // Não publica aqui! Publique no callback do nó principal, pois só lá você tem acesso ao publisher e ao header correto.

    return tf2::toMsg<std::pair<cv::Mat_<double>, cv::Mat_<double>>, geometry_msgs::msg::Transform>(std::make_pair(tvec, rvec));
}

void log_pose(const cv::Mat& tvec, const cv::Mat& rvec)
{
    double x = tvec.at<double>(0);
    double y = tvec.at<double>(1);
    double z = tvec.at<double>(2);

    cv::Mat R;
    cv::Rodrigues(rvec, R);

    double roll, pitch, yaw;
    pitch = std::asin(-R.at<double>(2, 0));
    if (std::cos(pitch) > 1e-6) {
        roll  = std::atan2(R.at<double>(2, 1), R.at<double>(2, 2));
        yaw   = std::atan2(R.at<double>(1, 0), R.at<double>(0, 0));
    } else {
        roll  = std::atan2(-R.at<double>(1, 2), R.at<double>(1, 1));
        yaw   = 0;
    }

    RCLCPP_INFO(
        rclcpp::get_logger("pose_estimation"),
        "Exported pose: X=%.3f Y=%.3f Z=%.3f | Roll=%.3f Pitch=%.3f Yaw=%.3f (rad)",
        x, y, z, roll, pitch, yaw
    );
}

const std::unordered_map<std::string, pose_estimation_f> pose_estimation_methods{
    {"homography", homography},
    {"pnp", pnp},
};