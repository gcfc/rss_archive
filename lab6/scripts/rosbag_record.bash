#!/bin/bash

rosbag record \
/built_trajectory/end_pose \
/built_trajectory/path \
/built_trajectory/start_point \
/clicked_point \
/drive \
/followed_trajectory/end_pose \
/followed_trajectory/path \
/followed_trajectory/start_point \
/initialpose \
/joint_states \
/map \
/map_metadata \
/move_base_simple/goal \
/odom \
/planned_trajectory/end_pose \
/planned_trajectory/path \
/planned_trajectory/path_array \
/planned_trajectory/start_point \
/pose \
/posearray \
/tf \
/tf_static \
/trajectory/current 
