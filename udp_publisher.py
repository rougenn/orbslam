#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import socket
import json
import math
import tf_transformations

class UdpBridgeNode(Node):
    def __init__(self):
        super().__init__('udp_bridge_node')
        self.declare_parameter('udp_ip', '10.0.0.2')
        self.declare_parameter('udp_port', 5005)
        self.udp_ip = self.get_parameter('udp_ip').value
        self.udp_port = self.get_parameter('udp_port').value
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.server_addr = (self.udp_ip, self.udp_port)
        self.last_imu = None
        self.idx = 0
        self.create_subscription(Imu, 'imu/data_raw', self.imu_callback, 10)
        self.create_subscription(Odometry, '/odometry/slam', self.odom_callback, 10)

    def imu_callback(self, msg: Imu):
        self.last_imu = msg

    def odom_callback(self, msg: Odometry):
        if not self.last_imu:
            return
        self.idx += 1
        Pn = msg.pose.pose.position.x
        Pe = msg.pose.pose.position.y
        Pd = msg.pose.pose.position.z
        Vn = msg.twist.twist.linear.x
        Ve = msg.twist.twist.linear.y
        Vd = msg.twist.twist.linear.z
        q = msg.pose.pose.orientation
        roll, pitch, yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        ax = self.last_imu.linear_acceleration.x
        ay = self.last_imu.linear_acceleration.y
        az = self.last_imu.linear_acceleration.z
        Am = math.sqrt(ax*ax + ay*ay + az*az)
        packet = {
            "idx": self.idx,
            "p": round(pitch, 3),
            "r": round(roll,   3),
            "y": round(yaw,    3),
            "Pn": round(Pn,    3),
            "Pe": round(Pe,    3),
            "Pd": round(Pd,    3),
            "ax": round(ax,    3),
            "ay": round(ay,    3),
            "az": round(az,    3),
            "Vn": round(Vn,    3),
            "Ve": round(Ve,    3),
            "Vd": round(Vd,    3),
            "Am": round(Am,    3)
        }
        self.sock.sendto(json.dumps(packet).encode('utf-8'), self.server_addr)
        self.get_logger().info(f"Sent UDP #{self.idx}")

def main():
    rclpy.init()
    node = UdpBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()