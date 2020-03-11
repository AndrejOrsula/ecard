## Specify installation dir
INSTALL_DIR=./moveit2 # TODO: change me!

## Create installation dir
mkdir -p ${INSTALL_DIR}/src

## Enter installation dir
cd ${INSTALL_DIR}

## Get absolute path to installation dir
INSTALL_DIR=${PWD}

## Enter source dir
cd src

## Clone source code
git clone https://github.com/ros-planning/moveit2.git -b master
vcs import < moveit2/moveit2.repos
vcs import < moveit2/moveit_demo_nodes/run_moveit_cpp/run_moveit_cpp.repos

## Install dependencies
rosdep install -r --from-paths . --ignore-src --rosdistro eloquent -y

## Enter installation dir again
cd ${INSTALL_DIR}

## Source ROS 2 Eloquent
source /opt/ros/eloquent/setup.bash

## Build with colcon
colcon build --event-handlers desktop_notification- status- --cmake-args -DCMAKE_BUILD_TYPE=Release

## Build
colcon build --symlink-install

## Add source prototype to '~/.bashrc'
echo -e "\n### ROS2 (${ROS_DISTRO}) MoveIt 2" >> ~/.bashrc
echo -e "# source ${INSTALL_DIR}/install/local_setup.bash" >> ~/.bashrc
