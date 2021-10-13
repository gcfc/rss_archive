#!/bin/bash

rosbag record \
  /avg_marker \
  /clicked_point \
  /drive \
  /initialpose \
  /joint_states \
  /map \
  /map_metadata \
  /map_updates \
  /odom \
  /particle_marker \
  /particle_marker_array \
  /pf/pose/odom \
  /pose \
  /posearray \
  /scan \
  /tf \
  /tf_static 