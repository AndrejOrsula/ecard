# Eye Controlled Assistive Robotic Device (ECARD)

```
git clone https://github.com/AndrejOrsula/ecard.git && cd ecard
vcs import < ecard.repos 
rosdep install --from-paths . --ignore-src --rosdistro ${ROS_DISTRO}
cd ..
colcon build --symlink-install --cmake-args "-DCMAKE_BUILD_TYPE=Release"
```
