#!/bin/bash

# Play the specified ROS bag.
ROSBAGFILE=$1
echo Playing rosbag $ROSBAGFILE
rosbag play --delay 7 $ROSBAGFILE

# Sleep to ensure the other nodes have completed before exiting.
sleep 10

