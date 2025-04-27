#include <memory>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "imu_usb_driver/imu_reader.hpp"

using namespace std::chrono_literals;

class ImuNode : public rclcpp::Node {
public:
  ImuNode()
  : Node("imu_usb_node")
  {
    auto port = declare_parameter("port",  std::string("/dev/ttyUSB0"));
    auto baud = declare_parameter("baud",  115200);
    RCLCPP_INFO(
      get_logger(),
      "Starting IMU node on port '%s' at %u baud",
      port.c_str(), baud);
    reader_ = std::make_unique<ImuReader>(port, static_cast<unsigned>(baud));

    frame_id_ = declare_parameter("frame", "imu_link");
    pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);

    timer_ = create_wall_timer(
      5ms,
      [this]() {
        RCLCPP_DEBUG(get_logger(), "Timer tick");
        onTimer();
      }
    );
  }

private:
  void onTimer() {
    TINSData data;
    if (!reader_->readNext(data)) {
      RCLCPP_DEBUG(get_logger(), "No complete IMU packet yet");
      return;
    }
    RCLCPP_INFO(
      get_logger(),
      "Got IMU packet: roll=%.2f°, pitch=%.2f°, yaw=%.2f°",
      data.Roll_deg, data.Pitch_deg, data.GyroMagnHeading_deg);

    auto msg = sensor_msgs::msg::Imu();
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;

    tf2::Quaternion q;
    q.setRPY(
      data.Roll_deg  * M_PI/180.0,
      data.Pitch_deg * M_PI/180.0,
      data.GyroMagnHeading_deg * M_PI/180.0
    );
    msg.orientation = tf2::toMsg(q);

    msg.angular_velocity.x = data.OmegaB_deg_s[0] * M_PI/180.0;
    msg.angular_velocity.y = data.OmegaB_deg_s[1] * M_PI/180.0;
    msg.angular_velocity.z = data.OmegaB_deg_s[2] * M_PI/180.0;

    msg.linear_acceleration.x = data.fB_m_s_s[0];
    msg.linear_acceleration.y = data.fB_m_s_s[1];
    msg.linear_acceleration.z = data.fB_m_s_s[2];

    pub_->publish(msg);
    RCLCPP_DEBUG(get_logger(), "Published IMU message");
  }

  std::unique_ptr<ImuReader> reader_;
  std::string frame_id_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuNode>());
  rclcpp::shutdown();
  return 0;
}
