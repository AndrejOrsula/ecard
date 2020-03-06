## Setup Sources
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

## Set up keys
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

## Install ROS melodic packages
sudo apt-get update
sudo apt-get install ros-melodic-desktop

## Initialize rosdep
sudo rosdep init
rosdep update

## Add source prototype to '~/.bashrc'
echo -e "\n### ROS1 (melodic)" >> ~/.bashrc
echo -e "# source /opt/ros/melodic/setup.bash" >> ~/.bashrc
