## Installation DIR; TODO: Configure based on your needs
INSTALL_DIR=ros2_realsense

## Register the server's public key
sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE

## Add the server to the list of repositories
sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo bionic main" -u

## Install the libraries
sudo apt-get install -y librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev

## Install ROS dependencies
sudo apt-get install -y ros-${ROS_DISTRO}-cv-bridge ros-${ROS_DISTRO}-message-filters

## Create working dir
mkdir -p ${INSTALL_DIR}/src && cd ${INSTALL_DIR}/src

## Clone repo
git clone https://github.com/intel/ros2_intel_realsense.git -b ${ROS_DISTRO}

## Return to ${INSTALL_DIR}
cd ..

## Build
colcon build --symlink-install

## Add source prototype to '~/.bashrc'
echo -e "\n### ROS2 (${ROS_DISTRO}) RealSense" >> ~/.bashrc
echo -e "# source ${PWD}/install/local_setup.bash" >> ~/.bashrc
