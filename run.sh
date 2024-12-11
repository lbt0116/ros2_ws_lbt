#!/bin/bash

# Navigate to the workspace
cd ~/ros2_ws

# Build the workspace
echo "Building the workspace..."
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -G Ninja  --event-handlers console_cohesion+

# Source the setup file
if [ $? -eq 0 ]; then
    echo "Sourcing the setup file..."
    source install/setup.bash

    # Launch the nodes
    echo "Launching the nodes..."
    ros2 launch launch/unified_launch.py launch_subscriber:=true
else
    echo "Build failed. Please check the error messages above."
fi
