PKG=${1}
colcon build
source install/local_setup.bash
ros2 run $PKG $PKG