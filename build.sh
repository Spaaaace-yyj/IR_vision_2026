source /opt/ros/humble/setup.bash
export OpenCV_DIR=/usr/local/lib/cmake/opencv4
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

colcon build --symlink-install