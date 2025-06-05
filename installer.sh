cd /home

# add a new ros repo key
source /opt/ros/humble/setup.bash

sudo apt update && sudo apt install curl gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

source /opt/ros/humble/setup.bash

sudo apt update

sudo apt install -y python-pip git python3-jinja2

sudo apt install -y \
  libboost-dev \
  libgnutls28-dev openssl libtiff-dev pybind11-dev \
  qtbase5-dev libqt5core5a libqt5widgets5 \
  meson cmake \
  python3-yaml python3-ply \
  libglib2.0-dev libgstreamer-plugins-base1.0-dev \
  build-essential ninja-build libdrm-dev libexpat1-dev python3-packaging

git clone https://github.com/raspberrypi/libcamera.git
cd libcamera

sudo apt install -y python3-pip
sudo pip3 install --upgrade meson 
export PATH=/usr/local/bin:$PATH

sudo apt install -y pkg-config libevent-dev
sudo apt install -y python3-ply

meson setup build --buildtype=release \
  -Dpipelines=rpi/vc4,rpi/pisp \
  -Dipas=rpi/vc4,rpi/pisp \
  -Dv4l2=enabled \
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
# ---------- here cam -l works!!!! ----------


mkdir -p ~/camera_ws/src
cd ~/camera_ws/src
git clone https://github.com/christianrauch/camera_ros.git
cd ~/camera_ws
source /opt/ros/$ROS_DISTRO/setup.bash

sudo rosdep init
source /opt/ros/humble/setup.bash
rosdep update
source /opt/ros/humble/setup.bash
rosdep install -y \
    --from-paths src \
    --ignore-src \
    --rosdistro $ROS_DISTRO \
    --skip-keys=libcamera

colcon build \
    --packages-select camera_ros \
    --event-handlers=console_direct+

# now camera node installed!!!

cd /home
git clone -b c++14_comp https://github.com/UZ-SLAMLab/ORB_SLAM3.git ORB_SLAM3
cd ORB_SLAM3
sudo apt install -y libboost-all-dev

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

export LD_LIBRARY_PATH=/home/ORB_SLAM3/lib:/usr/local/lib:$LD_LIBRARY_PATH

# orbslam have built
sudo apt update

source /opt/ros/humble/setup.bash
sudo apt install ros-$ROS_DISTRO-foxglove-bridge -y

sudo apt install ros-humble-tf-transformations -y

cd /home
git clone https://github.com/rougenn/orbslam.git -b master
mv orbslam/examples_ws/ .
mv /home/ORB_SLAM3/Vocabulary/ORBvoc.txt examples_ws/src/slam_example/config/

cd examples_ws
colcon build
cd ..

mv orbslam/run_nodes.sh .
chmod +x ./run_nodes.sh

cd orbslam/ros2_ws/
colcon build


cd /home
mv orbslam/trajectory_draw.py .
mv orbslam/udp_publisher.py .
