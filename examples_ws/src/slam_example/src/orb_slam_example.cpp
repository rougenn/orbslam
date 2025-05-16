#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <queue>
#include <deque>
#include <mutex>
#include <thread>

#include "include/System.h"            // ORB_SLAM3 System
#include "slam_example/image_grabber.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("slam_example");

    // Параметры словаря и конфига
    std::string vocab_path = "/home/examples_ws/src/slam_example/config/ORBvoc.txt";
    std::string config_path = "/home/examples_ws/src/slam_example/config/camera_and_slam_settings.yaml";
    // std::string config_path = "/home/examples_ws/src/slam_example/cfg.yaml";


    // Паблишер SLAM-одометрии
    auto odom_pub = node->create_publisher<nav_msgs::msg::Odometry>("/odometry/slam", 10);

    // Запускаем ORB‑SLAM3 в режиме Monocular‑Inertial
    auto SLAM = std::make_shared<ORB_SLAM3::System>(
        vocab_path,
        config_path,
        ORB_SLAM3::System::MONOCULAR,
        false
    );

    // Создаём ImageGrabber и передаём ему указатели на imu_buf и imu_mtx
    auto igb = std::make_shared<ImageGrabber>(
        SLAM,
        false,              // bClahe
        odom_pub,
        node,
        "oak-d_frame"
    );

    // Подписка на изображения (не забудьте захват igb!)
    auto img_sub = node->create_subscription<sensor_msgs::msg::Image>(
        "/rover_camera/image_raw",
        5,
        [igb](sensor_msgs::msg::Image::SharedPtr msg) {
            igb->grabImage(msg);
        }
    );

    // Подписка на IMU (SensorDataQoS напрямую)
    auto imu_sub = node->create_subscription<sensor_msgs::msg::Imu>(
        "/imu/data_raw",
        rclcpp::SensorDataQoS(),
        [igb](sensor_msgs::msg::Imu::SharedPtr msg) {
            igb->grabImu(msg);
        }
    );

    // Запускаем фоновый поток обработки
    std::thread img_thread(&ImageGrabber::processImages, igb);

    rclcpp::spin(node);
    rclcpp::shutdown();
    img_thread.join();
    return 0;
}
