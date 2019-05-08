#!/bin/bash

# Run the point cloud generator in the ROS environment.  Assumes you have
# already successfully run catkin_make to build the node.

# Put the hostname or IP address of your ROS master here:
export ROS_MASTER_URI=http://rosmaster5000:11311

# Find path to this script
MY_PATH=`dirname $0`

# Import ROS environment variables
source /opt/ros/kinetic/setup.bash
source $MY_PATH/devel/setup.bash

# Publish approximate transforms so that RViz will work by default
DOWN_TILT_RADS=0.34906585039886591538
ROLL_FRD_TO_FLU=3.141592654
static_transform_publisher 0 0 0 0 $DOWN_TILT_RADS $ROLL_FRD_TO_FLU og_platform map&
# static_transform_publisher 0 0 0 0 $DOWN_TILT_RADS $ROLL_FRD_TO_FLU stereo_optical map&

# Run node
rosrun optical_guide pcg $MY_PATH/config/pcgConfig.json

# End the transform publishers you started
echo "Killing all TF publishers."
killall static_transform_publisher
