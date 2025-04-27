cmake_minimum_required(VERSION 3.5)
project(imu_usb_driver)

find_package(ament_cmake REQUIRED)
find_package(rclcpp             REQUIRED)
find_package(sensor_msgs        REQUIRED)
find_package(tf2                REQUIRED)
find_package(tf2_geometry_msgs  REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
)

add_library(ins_packets STATIC src/INS_packets.c)
target_include_directories(ins_packets PUBLIC include)

add_library(imu_reader STATIC src/imu_reader.cpp)
ament_target_dependencies(imu_reader rclcpp)
target_link_libraries(imu_reader ins_packets Boost::system)

add_executable(imu_node src/imu_node.cpp)
ament_target_dependencies(imu_node
  rclcpp
  sensor_msgs
  tf2
  tf2_geometry_msgs
)
target_link_libraries(imu_node imu_reader)

install(TARGETS
  ins_packets
  imu_reader
  imu_node
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
