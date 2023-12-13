#!/bin/bash

# Load the ROS environment
source /opt/ros/foxy/setup.bash

# Load local env
source ~/capstone_welding_analysis/ros2_ws/install/setup.bash

# Set the ROS_DOMAIN_ID
export ROS_DOMAIN_ID=20

# Set the RMW_IMPLEMENTATION
export RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"

# Source the virtual python environment
source ~/capstone_welding_analysis/env/bin/activate

# Add the site-packages to the PYTHONPATH
export PYTHONPATH=$PYTHONPATH:~/capstone_welding_analysis/env/lib/site-packages