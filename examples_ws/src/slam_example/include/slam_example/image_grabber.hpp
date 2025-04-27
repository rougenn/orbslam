#ifndef SLAM_EXAMPLE_IMAGE_GRABBER_HPP
#define SLAM_EXAMPLE_IMAGE_GRABBER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <queue>
#include <deque>
#include <mutex>
#include <memory>

#include "include/System.h"  // ORB_SLAM3

using ImuMsg = sensor_msgs::msg::Imu;
using TS = ORB_SLAM3::Tracking;

class ImageGrabber : public std::enable_shared_from_this<ImageGrabber>
{
public:
    ImageGrabber();
    ImageGrabber(
      std::shared_ptr<ORB_SLAM3::System> pSLAM,
      bool bClahe,
      rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr rospub,
      std::shared_ptr<rclcpp::Node> ros_node,
      const std::string camera_frame_name
    );

    void grabImage(const sensor_msgs::msg::Image::SharedPtr msg);
    cv::Mat getImage(const sensor_msgs::msg::Image::SharedPtr &img_msg);
    void processImages();
    void publishSE3fToOdom(const Sophus::SE3f& se3);
    void grabImu(const ImuMsg::SharedPtr msg);

private:
    // Буфер видео
    std::queue<sensor_msgs::msg::Image::SharedPtr> img0Buf;
    std::mutex mBufMutex;

    std::queue<ImuMsg::SharedPtr> mImuBuf;
    std::mutex mBufMutexImu;

    // SLAM и паблишер
    std::shared_ptr<ORB_SLAM3::System> mpSLAM;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    nav_msgs::msg::Odometry odom_msg_;
    std::shared_ptr<rclcpp::Node> rosNode_;
    std::string tf_frame;
    bool mbClahe;
    bool first_pose;
    cv::Ptr<cv::CLAHE> mClahe;

    vector<ORB_SLAM3::IMU::Point> imuToSlam(double tImg);
    void FpsLog(double tIm);
    void LogResult(const Sophus::SE3f&);

};

#endif // SLAM_EXAMPLE_IMAGE_GRABBER_HPP
