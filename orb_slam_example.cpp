#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <rclcpp/qos.hpp>
#include <rmw/types.h>  

#include "include/System.h"  // Include the SLAM system header

#include "slam_example/image_grabber.hpp"

#include <queue>
#include <mutex>
#include <thread>


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("slam_example");

    //std::string vocab_path = node->get_parameter("vocab_path").as_string();
    std::string vocab_path = "/home/examples_ws/src/slam_example/config/ORBvoc.txt";
    std::cout << "vocab_path " << vocab_path  << std::endl;

    //std::string config_path = node->get_parameter("config_path").as_string();
    std::string config_path = "/home/examples_ws/src/slam_example/config/camera_and_slam_settings.yaml";
    std::cout << "config_path " << config_path << std::endl;
    bool showPangolin = false ; // true If you want to spone the Pangolin window with pose estimation drawed
    bool bEqual = false;

    // Publish odom message from SE3
    auto odom_publ = node->create_publisher<nav_msgs::msg::Odometry>("/odometry/slam", 10);
    std::cout << "create_publisher " << std::endl;

    // Create SLAM system and ImageGrabber
    auto SLAM = std::make_shared<ORB_SLAM3::System>(vocab_path, config_path, ORB_SLAM3::System::MONOCULAR, showPangolin);
    std::cout << "ORB_SLAM3 " << std::endl;
    auto igb = std::make_shared<ImageGrabber>(SLAM, bEqual,  odom_publ, node, "oak-d_frame");
    std::cout << "ImageGrabber " << std::endl;

    // Creating Image subscription
    std::string imgTopicName = "/rover_camera/image_raw" ;
    // Subscribe to the camera image topic
    auto sub_img0 = node->create_subscription<sensor_msgs::msg::Image>(
        imgTopicName, 5, [igb](const sensor_msgs::msg::Image::SharedPtr msg) { igb->grabImage(msg); });
    std::cout << "create_subscription " << std::endl;

    // Start processing images in a separate thread
    std::thread image_thread(&ImageGrabber::processImages, igb);

    // Run the ROS node
    rclcpp::spin(node);
    std::cout << "Node stop to spinning!" << std::endl;

    // Shutdown the node and wait for the thread to complete
    rclcpp::shutdown();
    image_thread.join();

    return 0;
}
