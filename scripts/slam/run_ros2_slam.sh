#!/bin/bash
# Launch ROS 2 SLAM (SLAM Toolbox)
# Usage: ./run_ros2_slam.sh
#
# Note: This script must be run INSIDE the Docker container.

# Source ROS 2 using the container's ros2_env script
if [ -f /usr/local/bin/ros2_env ]; then
    source /usr/local/bin/ros2_env
elif [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    source /root/ros2_ws/install/setup.bash 2>/dev/null || true
else
    echo "ERROR: ROS 2 environment not found!"
    echo "Please run 'ros2_env' or 'source /opt/ros/humble/setup.bash' first."
    exit 1
fi

# Check if slam_toolbox is installed
if ! ros2 pkg list 2>/dev/null | grep -q slam_toolbox; then
    echo "WARNING: slam_toolbox not found."
    echo "Install it with: apt-get update && apt-get install -y ros-humble-slam-toolbox"
    echo "Or use cartographer: apt-get install -y ros-humble-cartographer ros-humble-cartographer-ros"
    exit 1
fi

# Launch SLAM Toolbox with asynchronous mode (good for large maps)
echo "Starting SLAM Toolbox..."
ros2 launch slam_toolbox online_async_launch.py \
    slam_params_file:=/opt/ros/humble/share/slam_toolbox/config/mapper_params_online_async.yaml \
    use_sim_time:=true
