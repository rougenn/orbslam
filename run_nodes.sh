#!/usr/bin/env bash
set -e

CAMERA_WIDTH="${CAMERA_WIDTH:-640}"
CAMERA_HEIGHT="${CAMERA_HEIGHT:-480}"

PIDS=() 

cleanup() {
  echo -e "\n[INFO] Caught Ctrl+C or termination. Stopping all background processes..."
  for pid in "${PIDS[@]}"; do
    echo "[INFO] Killing PID $pid"
    kill "$pid" 2>/dev/null || true
  done
  exit 0
}

trap cleanup SIGINT SIGTERM

launch_camera() {
  echo "[INFO] Launching camera node..."
  (
    source /opt/ros/humble/setup.bash
    source /camera_ws/install/setup.bash
    ros2 run camera_ros camera_node \
      --ros-args \
      -p width:=$CAMERA_WIDTH \
      -p height:=$CAMERA_HEIGHT \
      -p camera_name:=imx219 \
      -p format:=BGR888 \
      -p orientation:=180 \
      -p role:=still \
      -r /camera/image_raw:=/rover_camera/image_raw
  ) > camera.log 2>&1 &
  PIDS+=($!)
}

launch_imu() {
  echo "[INFO] Launching IMU node..."
  (
    source /opt/ros/humble/setup.bash
    source /home/orbslam/ros2_ws/install/setup.bash
    ros2 run imu_usb_driver imu_node \
      --ros-args \
      -p port:=/dev/ttyUSB0 \
      -p baud:=115200 \
      -p frame:=imu_link
  ) > imu.log 2>&1 &
  PIDS+=($!)
}

launch_foxglove_and_traj() {
  echo "[INFO] Launching Foxglove Bridge and trajectory draw..."
  (
    source /opt/ros/humble/setup.bash
    ros2 launch foxglove_bridge foxglove_bridge_launch.xml \
      port:=8765 address:=0.0.0.0 &
    FOX_PID=$!
    sleep 2 
    python3 /home/trajectory_draw.py
    kill $FOX_PID 2>/dev/null || true
  ) > foxglove.log 2>&1 &
  PIDS+=($!)
}

launch_slam() {
  echo "[INFO] Launching SLAM node..."
  (
    source /opt/ros/humble/setup.bash
    source /home/examples_ws/install/setup.bash
    export LD_LIBRARY_PATH=/home/ORB_SLAM3/lib:/usr/local/lib:$LD_LIBRARY_PATH
    ros2 launch slam_example slam_example.launch.py
  ) > slam.log 2>&1 &
  PIDS+=($!)
}

show_help() {
  cat <<EOF
Usage: $0 [--camera] [--imu] [--fox] [--slam] [--all]
  --camera  start camera node
  --imu     start IMU node
  --fox     start Foxglove + trajectory
  --slam    start SLAM node
  --all     start all nodes
  -h, --help
EOF
  exit 0
}

if [[ $# -eq 0 ]]; then
  show_help
fi

DO_CAMERA=false; DO_IMU=false; DO_FOX=false; DO_SLAM=false

while [[ $# -gt 0 ]]; do
  case "$1" in
    --camera) DO_CAMERA=true; shift ;;
    --imu)    DO_IMU=true;    shift ;;
    --fox)    DO_FOX=true;    shift ;;
    --slam)   DO_SLAM=true;   shift ;;
    --all)    DO_CAMERA=true; DO_IMU=true; DO_FOX=true; DO_SLAM=true; shift ;;
    -h|--help) show_help ;;
    *) echo "Unknown flag: $1"; show_help ;;
  esac
done

$DO_CAMERA && launch_camera
$DO_IMU    && launch_imu
$DO_FOX    && launch_foxglove_and_traj
$DO_SLAM   && launch_slam

echo "[INFO] All requested nodes launched in background."
echo "[INFO] Press Ctrl+C to stop them all."

wait