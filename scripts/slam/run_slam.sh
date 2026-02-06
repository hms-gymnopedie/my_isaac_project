#!/bin/bash
# Wrapper script for Drone SLAM Simulation
# Sets up ROS 2 environment for Isaac Sim's internal ROS 2 bridge

set -e

# Isaac Sim ROS 2 Bridge environment
export ROS_DISTRO=humble
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:/isaacsim/exts/isaacsim.ros2.bridge/humble/lib"

echo "=== Drone SLAM Simulation ==="
echo "ROS_DISTRO: $ROS_DISTRO"
echo "RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"
echo "==="

# Default values
MAP="${1:-warehouse}"
EXTRA_ARGS="${@:2}"

# Run the simulation
cd /isaacsim
./python.sh /workspace/scripts/slam/run_slam_simulation.py --map "$MAP" --headless --webrtc $EXTRA_ARGS
