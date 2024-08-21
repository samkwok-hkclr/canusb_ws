#!/bin/bash 
# Basic entrypoint for ROS / Colcon Docker containers

# Source ROS 2
if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]
then
  source /opt/ros/${ROS_DISTRO}/setup.bash
fi

# Source the base workspace, if built
if [ -f /plc_ws/install/setup.bash ]
then
  source /plc_ws/install/setup.bash
fi

# Execute the command passed into this entrypoint
exec "$@"