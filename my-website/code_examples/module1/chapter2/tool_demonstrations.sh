#!/bin/bash

# This script demonstrates basic ROS 2 tooling commands.

echo "--- Demonstrating ros2 topic list ---"
ros2 topic list

echo ""
echo "--- Demonstrating ros2 node list ---"
ros2 node list

echo ""
echo "--- Demonstrating ros2 run ---"
# You need a package and node to run, assuming "turtlesim" is installed
# ros2 run turtlesim turtlesim_node &
# TURTLESIM_PID=$!
# echo "Turtlesim node started with PID $TURTLESIM_PID"

echo ""
echo "--- Demonstrating ros2 topic echo /parameter_events ---"
# This topic is always available
ros2 topic echo /parameter_events --once

echo ""
echo "--- Demonstrating ros2 service list ---"
ros2 service list

echo ""
echo "--- Demonstrating ros2 param list ---"
# You need a node with parameters for this, e.g., turtlesim
# ros2 param list

# Kill turtlesim if it was started
# if [ -n "$TURTLESIM_PID" ]; then
#   kill "$TURTLESIM_PID"
#   echo "Turtlesim node killed."
# fi
