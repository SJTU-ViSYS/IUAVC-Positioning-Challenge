#! /bin/bash
output_path=$1
team=$2
rosbag_path=$3
echo ${rosbag_path}
export ROS_IP='172.17.0.1' 
export ROS_MASTER_URI='http://172.17.0.1:11311'
sleep 1s
roslaunch stage4_refree refree.launch output_path_base:=${output_path} team:=${team} rosbag_path:=${rosbag_path};
rosnode kill -a
