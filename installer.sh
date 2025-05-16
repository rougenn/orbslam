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

sudo apt install python3-pip
sudo apt install pkg-config libevent-dev
sudo apt install python3-ply
sudo apt install libboost-all-dev
sudo apt install ros-humble-tf-transformations -y # for udp_publisher