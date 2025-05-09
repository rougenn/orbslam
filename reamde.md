# Docker installation

### Set locale
```bash
echo 'LANG=en_GB.UTF-8'    | sudo tee -a /etc/environment
echo 'LC_ALL=en_GB.UTF-8'  | sudo tee -a /etc/environment
```

### Install docker and add it to privilleged group
```bash
curl -fsSL https://get.docker.com | sh
sudo usermod -aG docker $USER
# помогло вот это
sudo chmod 666 /var/run/docker.sock
```

### Add the downloaded image to docker
```bash
docker load -i rp_v3_slam_image.tar
```

# ORB_SLAM3 installation
```bash
git clone -b c++14_comp https://github.com/UZ-SLAMLab/ORB_SLAM3.git ORB_SLAM3
cd ORB_SLAM3
```
*Install the boost library*
```bash
sudo apt install libboost-all-dev
```
*Execute this file in orb slam directory*
```bash
#!/bin/bash

echo "Configuring and building Thirdparty/DBoW2 ..."

cd Thirdparty/DBoW2
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4

cd ../../g2o

echo "Configuring and building Thirdparty/g2o ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4

cd ../../Sophus

echo "Configuring and building Thirdparty/Sophus ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4

cd ../../../

echo "Uncompress vocabulary ..."

cd Vocabulary
tar -xf ORBvoc.txt.tar.gz
cd ..

echo "Configuring and building ORB_SLAM3 ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j2
```

# Connect the camera (imx219)
### Config changes
*Firstly change the config to manually activate the camera driver for libcamera*
```bash
sudo nano /boot/firmware/config.txt
```
*Add these strings to the end of the file*
```bash
dtoverlay=bcm2835-v4l2
dtoverlay=imx219,cam0
start_x=1
gpu_mem=256

# !!! make sure that this string is commented !!!
# display_auto_detect=1
```


# Libcamera installation
### install the dependencies and clone the repo
```bash
sudo apt update
sudo apt install -y python-pip git python3-jinja2
sudo apt install -y python3-ply

sudo apt install -y \
  libboost-dev \
  libgnutls28-dev openssl libtiff-dev pybind11-dev \
  qtbase5-dev libqt5core5a libqt5widgets5 \
  meson cmake \
  python3-yaml python3-ply \
  libglib2.0-dev \
  build-essential ninja-build libdrm-dev libexpat1-dev python3-packaging

sudo apt install -y libunwind-dev \ 
  libgstreamer-plugins-base1.0-dev

git clone https://github.com/raspberrypi/libcamera.git
cd libcamera
```
# мы его поставили выше зачем новый не очень понятно! 

### meson 
```bash
sudo apt install python3-pip
sudo pip3 install --upgrade meson 
export PATH=/usr/local/bin:$PATH
```
### build it
```bash
meson setup build --buildtype=release \
  -Dpipelines=rpi/vc4,rpi/pisp \
  -Dipas=rpi/vc4,rpi/pisp \
  -Dv4l2=true \
  -Dgstreamer=disabled \
  -Dtest=false \
  -Dlc-compliance=disabled \
  -Dcam=enabled \
  -Dqcam=disabled \
  -Ddocumentation=disabled \
  -Dpycamera=enabled

ninja -C build
sudo ninja -C build install
sudo ldconfig
```

*check the camera availibillity:*
```bash 
cam -l

# output example:
# [0:49:22.288367528] [5074]  INFO Camera camera_manager.cpp:326 libcamera v0.5.0+59-d83ff0a4
# [0:49:22.289695146] [5075]  INFO RPI pisp.cpp:720 libpisp version v1.2.0 50426319aa1a 02-05-2025 (17:31:55)
# [0:49:22.305555585] [5075]  INFO RPI pisp.cpp:1179 Registered camera /base/axi/pcie@120000/rp1/i2c@88000/imx219@10 to CFE device /dev/media0 and ISP device /dev/media3 using PiSP variant BCM2712_D0
# Available cameras:
# 1: 'imx219' (/base/axi/pcie@120000/rp1/i2c@88000/imx219@10)
```

# Install the camera node 
```bash
sudo pip3 install --upgrade colcon-meson colcon-common-extensions
sudo apt install -y python3-colcon-core python3-colcon-meson python3-colcon-common-extensions
```

```bash
sudo apt update
sudo apt install -y libcamera-dev pkg-config
export PKG_CONFIG_PATH=/usr/local/lib/pkgconfig:$PKG_CONFIG_PATH
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
```

```bash
mkdir -p ~/camera_ws/src
cd ~/camera_ws/src
git clone https://github.com/christianrauch/camera_ros.git
cd ~/camera_ws
source /opt/ros/$ROS_DISTRO/setup.bash
```

```bash
sudo rosdep init
rosdep update

rosdep install -y \
    --from-paths src \
    --ignore-src \
    --rosdistro $ROS_DISTRO \
    --skip-keys=libcamera

colcon build \
    --packages-select camera_ros \
    --event-handlers=console_direct+
```
### to run camera node:
```bash
cd ~/camera_ws/src
source install/setup.bash

ros2 run camera_ros camera_node \
    --ros-args \
    -p width:=640 \
    -p height:=360 \
    -p camera_name:=imx219 -p format:=BGR888 \
    -p role:=still \
    -r /camera/image_raw:=/rover_camera/image_raw
```

# Foxglove installation
```bash
source /opt/ros/humble/setup.bash
sudo apt install ros-$ROS_DISTRO-foxglove-bridge
```
### to run it:
```bash
source /opt/ros/humble/setup.bash

ros2 launch foxglove_bridge foxglove_bridge_launch.xml \
  port:=8765 address:=0.0.0.0
```

# ROS odometry node installation
```bash
cd /home
mkdir examples_ws
cd examples_ws
mkdir src
cd src
ros2 pkg create --build-type ament_cmake slam_example
```

# // !!!!! переделать от сюда (клонировать с другой ветки)
### create cpp files
```bash
cd slam_example
git clone https://github.com/rougenn/orbslam.git

mv orbslam/image_grabber.hpp include/slam_example/
mv orbslam/image_grabber.cpp src/
mv orbslam/orb_slam_example.cpp src/
```
### move the vocab
```bash
mkdir config
cp /home/ORB_SLAM3/Vocabulary/ORBvoc.txt config/
```
### create the config file
```bash
touch config/camera_and_slam_settings.yaml
```

```bash
mkdir launch
touch launch/slam_example.launch.py

touch package.xml
touch CMakeLists.txt

```
# добавить еще ноду иму клонирование и перенос в корень
```bash
cd /ros2_ws/src
colcol build 
source install/setup.bash
```
### to launch imu node 
```bash
ros2 run imu_usb_driver imu_node \
  --ros-args -p port:=/dev/ttyUSB0 -p baud:=115200 -p frame:=imu_link
```
# // !!!!! переделать до сюда

```bash
colcon build
. install/setup.bash
export LD_LIBRARY_PATH=/home/ORB_SLAM3/lib:$LD_LIBRARY_PATH
```
# Run the node
```bash
ros2 launch slam_example slam_example.launch.py
```

```
